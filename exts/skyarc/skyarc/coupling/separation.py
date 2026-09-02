# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Passive separation actuator and signed envelope-gap monitoring."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Tuple

from ..components.contract import Component, ComponentDescriptor, Determinism, ScenarioContext, StepOutput
from ..configuration.schema import IgnitionConfig
from ..effects.types import EffectBatch
from ..events import EVENT_ABORT, EVENT_RECONTACT, EVENT_SEPARATION_CONFIRMED, Event
from ..launcher.geometry import TubePath, path_pose
from ..linalg import dot, is_finite, sub
from ..names import (
    BODY_CART,
    BODY_ROCKET,
    MARKER_CART_CRADLE_FRONT,
    MARKER_ROCKET_AFT,
    PAIR_ROCKET_CRADLE,
    SLOT_SEPARATION_ACTUATOR,
)
from ..state import MarkerSpec, Observation, SimulationState, marker_world_position


@dataclass(frozen=True)
class SeparationMeasurement:
    """Minimum signed clearance and separating speed along the local exit tangent."""

    gap_m: float
    relative_speed_mps: float


@dataclass(frozen=True)
class SeparationStatus:
    active: bool
    confirmed: bool
    failed: bool
    gap_m: float
    relative_speed_mps: float
    maximum_contact_impulse_ns: float
    last_contact_time_s: float | None
    release_time_s: float | None


def measure_separation(
    state: SimulationState,
    layout: TubePath,
    markers: Mapping[str, MarkerSpec],
) -> SeparationMeasurement:
    """Measure the named rocket-aft to cradle-front envelope gap.

    The aft marker controls tube exit elsewhere, but the clearance calculation uses both
    named envelope markers and therefore is not an aft-marker position surrogate.
    """
    try:
        rocket_marker = markers[MARKER_ROCKET_AFT]
        cradle_marker = markers[MARKER_CART_CRADLE_FRONT]
    except KeyError as exc:
        raise ValueError(f"separation marker {exc.args[0]!r} is missing") from None
    rocket_point = marker_world_position(rocket_marker, state)
    cradle_point = marker_world_position(cradle_marker, state)
    cart_s = layout.axial_position(state.body(BODY_CART).position)
    tangent = path_pose(layout, cart_s).tangent
    gap = dot(sub(rocket_point, cradle_point), tangent)
    relative_speed = dot(
        sub(
            state.body(BODY_ROCKET).linear_velocity,
            state.body(BODY_CART).linear_velocity,
        ),
        tangent,
    )
    if not is_finite((gap, relative_speed)):
        raise ValueError("separation measurement must be finite")
    return SeparationMeasurement(gap_m=gap, relative_speed_mps=relative_speed)


class NoneSeparationActuator(Component):
    """Baseline passive mechanism: braking creates separation, the actuator adds no impulse."""

    def __init__(self, *, code_hash: str) -> None:
        if not code_hash:
            raise ValueError("separation-actuator code hash may not be empty")
        self._code_hash = code_hash

    @property
    def descriptor(self) -> ComponentDescriptor:
        return ComponentDescriptor(
            slot=SLOT_SEPARATION_ACTUATOR,
            model_id="none_v1",
            model_version="1.0.0",
            parameter_schema_version="1",
            code_hash=self._code_hash,
            determinism=Determinism.DETERMINISTIC,
        )

    def prepare(self, context: ScenarioContext) -> None:
        pass

    def reset(self, initial_state: SimulationState) -> None:
        pass

    def pre_step(self, observation: Observation) -> StepOutput:
        return StepOutput.empty(SLOT_SEPARATION_ACTUATOR)

    def post_step(self, state: SimulationState) -> StepOutput:
        return StepOutput.empty(SLOT_SEPARATION_ACTUATOR)

    def snapshot_state(self) -> Mapping[str, Any]:
        return {"impulse_enabled": False}


class SeparationMonitor:
    """Confirm passive separation and fail closed on timeout, impulse, or recontact."""

    def __init__(self, criteria: IgnitionConfig) -> None:
        self._criteria = criteria
        self.reset()

    @property
    def status(self) -> SeparationStatus:
        return SeparationStatus(
            active=self._release_time_s is not None,
            confirmed=self._confirmed,
            failed=self._failed,
            gap_m=self._gap_m,
            relative_speed_mps=self._relative_speed_mps,
            maximum_contact_impulse_ns=self._maximum_impulse_ns,
            last_contact_time_s=self._last_contact_time_s,
            release_time_s=self._release_time_s,
        )

    def reset(self) -> None:
        self._release_time_s: float | None = None
        self._last_contact_time_s: float | None = None
        self._maximum_impulse_ns = 0.0
        self._gap_m = 0.0
        self._relative_speed_mps = 0.0
        self._confirmed = False
        self._failed = False
        self._failure_emitted = False

    def begin(self, state: SimulationState) -> None:
        if self._release_time_s is None:
            self._release_time_s = state.time_s
            # Even a contact-free release must survive the configured dwell before it can
            # be confirmed.  The release instant is the start of that clean interval.
            self._last_contact_time_s = state.time_s

    def _abort(self, state: SimulationState, reason: str, **data: object) -> Tuple[Event, ...]:
        self._failed = True
        if self._failure_emitted:
            return ()
        self._failure_emitted = True
        return (
            Event(
                name=EVENT_ABORT,
                time_s=state.time_s,
                step_index=state.step_index,
                source=SLOT_SEPARATION_ACTUATOR,
                data={"reason": reason, **data},
            ),
        )

    def update(
        self,
        state: SimulationState,
        measurement: SeparationMeasurement,
    ) -> Tuple[Event, ...]:
        if self._release_time_s is None or self._failed:
            return ()
        self._gap_m = measurement.gap_m
        self._relative_speed_mps = measurement.relative_speed_mps
        contact = state.contact(PAIR_ROCKET_CRADLE)
        impulse = contact.magnitude_ns
        self._maximum_impulse_ns = max(self._maximum_impulse_ns, impulse)
        if contact.time_scaling != "impulse":
            return self._abort(state, "contact_quantity_not_impulse")
        if contact.active or impulse > 0.0:
            self._last_contact_time_s = state.time_s
            if self._confirmed:
                self._failed = True
                return (
                    Event(
                        name=EVENT_RECONTACT,
                        time_s=state.time_s,
                        step_index=state.step_index,
                        source=SLOT_SEPARATION_ACTUATOR,
                        data={"impulse_ns": impulse},
                    ),
                    *self._abort(state, "rocket_cradle_recontact", impulse_ns=impulse),
                )
        if impulse > self._criteria.maximum_contact_impulse_ns:
            return self._abort(
                state,
                "separation_contact_impulse_exceeded",
                impulse_ns=impulse,
            )
        elapsed = state.time_s - self._release_time_s
        if not self._confirmed and elapsed > self._criteria.separation_timeout_s + 1e-12:
            return self._abort(
                state,
                "separation_timeout",
                gap_m=measurement.gap_m,
                relative_speed_mps=measurement.relative_speed_mps,
            )
        assert self._last_contact_time_s is not None
        dwell = state.time_s - self._last_contact_time_s
        if (
            not self._confirmed
            and measurement.gap_m >= self._criteria.minimum_cart_clearance_m
            and measurement.relative_speed_mps >= self._criteria.minimum_relative_speed_mps
            and dwell + 1e-12 >= self._criteria.no_recontact_dwell_s
        ):
            self._confirmed = True
            return (
                Event(
                    name=EVENT_SEPARATION_CONFIRMED,
                    time_s=state.time_s,
                    step_index=state.step_index,
                    source=SLOT_SEPARATION_ACTUATOR,
                    data={
                        "gap_m": measurement.gap_m,
                        "relative_speed_mps": measurement.relative_speed_mps,
                        "no_recontact_dwell_s": dwell,
                        "maximum_contact_impulse_ns": self._maximum_impulse_ns,
                    },
                ),
            )
        return ()
