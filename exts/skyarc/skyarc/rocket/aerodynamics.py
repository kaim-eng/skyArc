# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Baseline point-drag model for the detached rocket."""

from __future__ import annotations

import math
from typing import Any, Mapping

from ..components.contract import Component, ComponentDescriptor, Determinism, ScenarioContext, StepOutput
from ..configuration.schema import ExteriorAtmosphereConfig, RocketConfig
from ..effects.types import EffectBatch, Frame, Wrench
from ..linalg import norm, scale
from ..names import BODY_ROCKET, SLOT_ROCKET_AERODYNAMICS
from ..state import Observation, SimulationState, body_com_world


class QuadraticPointDrag(Component):
    """Apply stationary-air quadratic drag at the rocket centre of mass."""

    def __init__(
        self,
        rocket: RocketConfig,
        *,
        reference_density_kg_m3: float,
        exterior_density_ratio: float,
        exterior_atmosphere: ExteriorAtmosphereConfig | None,
        code_hash: str,
    ) -> None:
        if not code_hash:
            raise ValueError("rocket-aerodynamics code hash may not be empty")
        if not math.isfinite(reference_density_kg_m3) or reference_density_kg_m3 <= 0.0:
            raise ValueError("reference density must be finite and positive")
        self._rocket = rocket
        self._reference_density = reference_density_kg_m3
        self._exterior_ratio = exterior_density_ratio
        self._exterior_atmosphere = exterior_atmosphere
        self._code_hash = code_hash
        self._last_drag_n = 0.0

    @property
    def descriptor(self) -> ComponentDescriptor:
        return ComponentDescriptor(
            slot=SLOT_ROCKET_AERODYNAMICS,
            model_id="quadratic_point_drag_v1",
            model_version="1.0.0",
            parameter_schema_version="1",
            code_hash=self._code_hash,
            determinism=Determinism.DETERMINISTIC,
        )

    def prepare(self, context: ScenarioContext) -> None:
        pass

    def reset(self, initial_state: SimulationState) -> None:
        self._last_drag_n = 0.0

    def pre_step(self, observation: Observation) -> StepOutput:
        if observation.coupled:
            self._last_drag_n = 0.0
            return StepOutput.empty(SLOT_ROCKET_AERODYNAMICS)
        velocity = observation.rocket.linear_velocity
        speed = norm(velocity)
        if speed <= 1e-12:
            self._last_drag_n = 0.0
            return StepOutput.empty(SLOT_ROCKET_AERODYNAMICS)
        density_ratio = (
            self._exterior_ratio
            if self._exterior_atmosphere is None
            else self._exterior_atmosphere.density_ratio(observation.rocket.position[2])
        )
        magnitude = (
            0.5
            * self._reference_density
            * density_ratio
            * self._rocket.drag_coefficient
            * self._rocket.reference_area_m2
            * speed
            * speed
        )
        self._last_drag_n = magnitude
        return StepOutput(
            effects=EffectBatch(
                source=SLOT_ROCKET_AERODYNAMICS,
                wrenches=(
                    Wrench(
                        source=SLOT_ROCKET_AERODYNAMICS,
                        body=BODY_ROCKET,
                        force_n=scale(velocity, -magnitude / speed),
                        application_point_m=body_com_world(observation.rocket),
                        frame=Frame.WORLD,
                    ),
                ),
            )
        )

    def post_step(self, state: SimulationState) -> StepOutput:
        return StepOutput.empty(SLOT_ROCKET_AERODYNAMICS)

    def snapshot_state(self) -> Mapping[str, Any]:
        return {"last_drag_n": self._last_drag_n}
