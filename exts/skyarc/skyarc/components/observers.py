# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Baseline zero-noise, zero-latency ground-truth observer."""

from __future__ import annotations

from typing import Any, Mapping

from ..effects.types import EffectBatch
from ..launcher.geometry import TubePath, axial_velocity, path_pose
from ..names import (
    BODY_CART,
    BODY_ROCKET,
    MARKER_ROCKET_STAGNATION,
    SLOT_OBSERVER,
)
from ..state import (
    AxialQuantities,
    MarkerSpec,
    Observation,
    SimulationState,
    combined_mass,
    marker_world_position,
)
from .contract import Component, ComponentDescriptor, Determinism, ScenarioContext, StepOutput


class GroundTruthObserver(Component):
    """Copy selected latent state without noise, delay, or rate conversion."""

    def __init__(self, layout: TubePath, markers: Mapping[str, MarkerSpec], *, code_hash: str) -> None:
        if not code_hash:
            raise ValueError("observer code hash may not be empty")
        self._layout = layout
        self._markers = dict(markers)
        self._code_hash = code_hash
        self._last_observation: Observation | None = None

    @property
    def descriptor(self) -> ComponentDescriptor:
        return ComponentDescriptor(
            slot=SLOT_OBSERVER,
            model_id="ground_truth_v1",
            model_version="1.0.0",
            parameter_schema_version="1",
            code_hash=self._code_hash,
            determinism=Determinism.DETERMINISTIC,
        )

    def prepare(self, context: ScenarioContext) -> None:
        # Unconditional: guarding this on a non-empty context.markers would disable the
        # check in the one case where every marker is missing.
        unknown = sorted(set(self._markers) - set(context.markers))
        if unknown:
            raise ValueError(f"observer markers are absent from scenario context: {unknown}")
        mismatched = sorted(
            name for name, spec in self._markers.items() if context.markers[name] != spec
        )
        if mismatched:
            raise ValueError(
                f"observer marker definitions disagree with scenario context: {mismatched}"
            )

    def reset(self, initial_state: SimulationState) -> None:
        self._last_observation = None

    def observe(
        self,
        state: SimulationState,
        *,
        coupled: bool,
        separation_gap_m: float,
        separation_rate_mps: float,
    ) -> Observation:
        """Create the controller-visible packet from one immutable backend state."""
        frozen = state.frozen()
        cart = frozen.body(BODY_CART)
        rocket = frozen.body(BODY_ROCKET)
        marker_s = {
            name: self._layout.axial_position(marker_world_position(spec, frozen))
            for name, spec in self._markers.items()
        }
        s_cart_m = self._layout.axial_position(cart.position)
        s_rocket_m = self._layout.axial_position(rocket.position)
        density_sample_s = marker_s.get(MARKER_ROCKET_STAGNATION, s_rocket_m)
        active_stage = self._layout.stage_index(density_sample_s)
        stage_name = "exterior" if active_stage is None else self._layout.stages[active_stage].name
        assembly_mass = (
            combined_mass(frozen, (BODY_CART, BODY_ROCKET)) if coupled else cart.mass_kg
        )
        # A curved layout has no single global axis; the axial direction is the local
        # centerline tangent at each body's own arc coordinate. For a straight layout
        # path_pose returns exactly layout.axis, so straight behavior is unchanged.
        cart_tangent = path_pose(self._layout, s_cart_m).tangent
        rocket_tangent = path_pose(self._layout, s_rocket_m).tangent
        axial = AxialQuantities(
            s_cart_m=s_cart_m,
            s_rocket_m=s_rocket_m,
            marker_s_m=marker_s,
            cart_axial_velocity_mps=axial_velocity(cart.linear_velocity, cart_tangent),
            rocket_axial_velocity_mps=axial_velocity(rocket.linear_velocity, rocket_tangent),
            assembly_mass_kg=assembly_mass,
            stage_index=-1 if active_stage is None else active_stage,
            stage_name=stage_name,
            effective_density_ratio=self._layout.density_ratio(density_sample_s),
            separation_gap_m=separation_gap_m,
            separation_rate_mps=separation_rate_mps,
        )
        observation = Observation(
            source_model=self.descriptor.model_id,
            time_s=frozen.time_s,
            step_index=frozen.step_index,
            dt_s=frozen.dt_s,
            state=frozen,
            axial=axial,
            coupled=coupled,
            latency_steps=0,
        )
        self._last_observation = observation
        return observation

    def pre_step(self, observation: Observation) -> StepOutput:
        # Observations never enter the physical EffectBatch. This lifecycle hook remains
        # empty so the observer still participates in reset/snapshot orchestration.
        return StepOutput(effects=EffectBatch.empty(SLOT_OBSERVER))

    def post_step(self, state: SimulationState) -> StepOutput:
        return StepOutput(effects=EffectBatch.empty(SLOT_OBSERVER))

    def snapshot_state(self) -> Mapping[str, Any]:
        return {
            "has_observation": self._last_observation is not None,
            "last_step_index": None if self._last_observation is None else self._last_observation.step_index,
        }
