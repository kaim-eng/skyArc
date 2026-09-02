# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Baseline interlock-gated constant-mass rocket motor."""

from __future__ import annotations

from typing import Any, Mapping, Tuple

from ..components.contract import Component, ComponentDescriptor, Determinism, ScenarioContext, StepOutput
from ..configuration.schema import MotorConfig
from ..coupling.interlock import InterlockDecision
from ..effects.types import EffectBatch, Frame, Wrench
from ..events import EVENT_BURNOUT, EVENT_IGNITION, EVENT_INTERLOCK_BLOCKED, Event
from ..names import BODY_ROCKET, SLOT_ROCKET_MOTOR
from ..state import Observation, SimulationState


class ConstantMassThrustMotor(Component):
    """Apply body-axial thrust for a fixed duration without changing rocket mass."""

    def __init__(
        self,
        parameters: MotorConfig,
        *,
        application_point_body_m: tuple[float, float, float] = (0.0, 0.0, 0.0),
        code_hash: str,
    ) -> None:
        if not code_hash:
            raise ValueError("motor code hash may not be empty")
        self._parameters = parameters
        self._application_point = application_point_body_m
        self._code_hash = code_hash
        self.reset_state()

    @property
    def descriptor(self) -> ComponentDescriptor:
        return ComponentDescriptor(
            slot=SLOT_ROCKET_MOTOR,
            model_id="constant_mass_thrust_v1",
            model_version="1.0.0",
            parameter_schema_version="1",
            code_hash=self._code_hash,
            determinism=Determinism.DETERMINISTIC,
        )

    @property
    def ignited(self) -> bool:
        return self._ignition_time_s is not None

    @property
    def burned_out(self) -> bool:
        return self._burnout_emitted

    def prepare(self, context: ScenarioContext) -> None:
        if self._parameters.model != "constant":
            raise ValueError(
                f"constant_mass_thrust_v1 cannot implement motor model {self._parameters.model!r}"
            )

    def reset_state(self) -> None:
        self._ignition_time_s: float | None = None
        self._burnout_emitted = False
        self._last_thrust_n = 0.0

    def reset(self, initial_state: SimulationState) -> None:
        self.reset_state()

    def command_ignition(
        self,
        observation: Observation,
        decision: InterlockDecision,
    ) -> Tuple[Event, ...]:
        if self.ignited:
            return ()
        if not decision.allowed:
            return (
                Event(
                    name=EVENT_INTERLOCK_BLOCKED,
                    time_s=observation.time_s,
                    step_index=observation.step_index,
                    source=SLOT_ROCKET_MOTOR,
                    data={"blocked_gates": decision.blocked_gates},
                ),
            )
        self._ignition_time_s = observation.time_s
        return (
            Event(
                name=EVENT_IGNITION,
                time_s=observation.time_s,
                step_index=observation.step_index,
                source=SLOT_ROCKET_MOTOR,
                data={"thrust_n": self._parameters.thrust_n},
            ),
        )

    def pre_step(self, observation: Observation) -> StepOutput:
        if self._ignition_time_s is None or observation.coupled:
            self._last_thrust_n = 0.0
            return StepOutput.empty(SLOT_ROCKET_MOTOR)
        elapsed = observation.time_s - self._ignition_time_s
        if elapsed + 1e-12 >= self._parameters.burn_duration_s:
            self._last_thrust_n = 0.0
            events = ()
            if not self._burnout_emitted:
                self._burnout_emitted = True
                events = (
                    Event(
                        name=EVENT_BURNOUT,
                        time_s=observation.time_s,
                        step_index=observation.step_index,
                        source=SLOT_ROCKET_MOTOR,
                        data={"burn_duration_s": self._parameters.burn_duration_s},
                    ),
                )
            return StepOutput(effects=EffectBatch.empty(SLOT_ROCKET_MOTOR), events=events)
        self._last_thrust_n = self._parameters.thrust_n
        return StepOutput(
            effects=EffectBatch(
                source=SLOT_ROCKET_MOTOR,
                wrenches=(
                    Wrench(
                        source=SLOT_ROCKET_MOTOR,
                        body=BODY_ROCKET,
                        force_n=(self._parameters.thrust_n, 0.0, 0.0),
                        application_point_m=self._application_point,
                        frame=Frame.BODY,
                    ),
                ),
            )
        )

    def post_step(self, state: SimulationState) -> StepOutput:
        return StepOutput.empty(SLOT_ROCKET_MOTOR)

    def snapshot_state(self) -> Mapping[str, Any]:
        return {
            "ignited": self.ignited,
            "burned_out": self.burned_out,
            "ignition_time_s": self._ignition_time_s,
            "last_thrust_n": self._last_thrust_n,
        }
