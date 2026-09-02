# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Baseline effective-density guided drag model."""

from __future__ import annotations

import math
from typing import Any, Mapping

from ..components.contract import Component, ComponentDescriptor, Determinism, ScenarioContext, StepOutput
from ..configuration.schema import CartConfig, ExteriorAtmosphereConfig, GuidedAerodynamicsConfig
from ..effects.types import EffectBatch, Frame, Wrench
from .geometry import TubePath, path_pose
from ..linalg import dot, is_finite, scale
from ..names import BODY_CART, SLOT_ATMOSPHERE
from ..state import Observation, SimulationState, body_com_world


def quadratic_axial_drag_force_n(
    density_kg_m3: float,
    drag_coefficient: float,
    reference_area_m2: float,
    vehicle_speed_mps: float,
    air_speed_mps: float = 0.0,
) -> float:
    """Signed scalar drag along the path tangent.

    The sign is opposite the air-relative velocity.  A sufficiently strong tailwind can
    therefore produce a positive scalar force without any special-case sign convention.
    """
    values = (
        density_kg_m3,
        drag_coefficient,
        reference_area_m2,
        vehicle_speed_mps,
        air_speed_mps,
    )
    if not is_finite(values):
        raise ValueError("drag inputs must be finite")
    if density_kg_m3 < 0.0 or drag_coefficient < 0.0 or reference_area_m2 < 0.0:
        raise ValueError("drag density, coefficient, and area must be nonnegative")
    relative_speed = vehicle_speed_mps - air_speed_mps
    return (
        -0.5
        * density_kg_m3
        * drag_coefficient
        * reference_area_m2
        * relative_speed
        * abs(relative_speed)
    )


class DensityDragModel(Component):
    """Apply one equivalent guided-phase drag wrench to the attached assembly."""

    def __init__(
        self,
        layout: TubePath,
        parameters: GuidedAerodynamicsConfig,
        *,
        reference_density_kg_m3: float,
        cart: CartConfig | None = None,
        exterior_atmosphere: ExteriorAtmosphereConfig | None = None,
        code_hash: str,
    ) -> None:
        if not code_hash:
            raise ValueError("atmosphere code hash may not be empty")
        if not math.isfinite(reference_density_kg_m3) or reference_density_kg_m3 <= 0.0:
            raise ValueError("reference density must be finite and positive")
        self._layout = layout
        self._parameters = parameters
        self._reference_density = reference_density_kg_m3
        self._cart = cart
        self._exterior_atmosphere = exterior_atmosphere
        self._code_hash = code_hash
        self._last_force_n = 0.0

    @property
    def descriptor(self) -> ComponentDescriptor:
        return ComponentDescriptor(
            slot=SLOT_ATMOSPHERE,
            model_id="density_drag_v1",
            model_version="1.0.0",
            parameter_schema_version="1",
            code_hash=self._code_hash,
            determinism=Determinism.DETERMINISTIC,
        )

    def prepare(self, context: ScenarioContext) -> None:
        if self._parameters.force_model != "density_drag":
            raise ValueError(
                f"density_drag_v1 cannot implement force model {self._parameters.force_model!r}"
            )

    def reset(self, initial_state: SimulationState) -> None:
        self._last_force_n = 0.0

    def pre_step(self, observation: Observation) -> StepOutput:
        pose = path_pose(self._layout, observation.axial.s_cart_m)
        speed = dot(observation.cart.linear_velocity, pose.tangent)
        if observation.coupled:
            density_ratio = observation.axial.effective_density_ratio
            drag_coefficient = self._parameters.drag_coefficient
            reference_area = self._parameters.reference_area_m2
            air_speed = self._parameters.axial_air_velocity_mps
        elif self._cart is not None:
            density_ratio = (
                self._layout.exterior_effective_density_ratio
                if self._exterior_atmosphere is None
                else self._exterior_atmosphere.density_ratio(observation.cart.position[2])
            )
            drag_coefficient = self._cart.drag_coefficient
            reference_area = self._cart.frontal_area_m2
            air_speed = 0.0
        else:
            self._last_force_n = 0.0
            return StepOutput.empty(SLOT_ATMOSPHERE)
        force_n = quadratic_axial_drag_force_n(
            self._reference_density * density_ratio,
            drag_coefficient,
            reference_area,
            speed,
            air_speed,
        )
        self._last_force_n = force_n
        wrench = Wrench(
            source=SLOT_ATMOSPHERE,
            body=BODY_CART,
            force_n=scale(pose.tangent, force_n),
            application_point_m=body_com_world(observation.cart),
            frame=Frame.WORLD,
        )
        return StepOutput(
            effects=EffectBatch(source=SLOT_ATMOSPHERE, wrenches=(wrench,))
        )

    def post_step(self, state: SimulationState) -> StepOutput:
        return StepOutput.empty(SLOT_ATMOSPHERE)

    def snapshot_state(self) -> Mapping[str, Any]:
        return {"last_axial_force_n": self._last_force_n}
