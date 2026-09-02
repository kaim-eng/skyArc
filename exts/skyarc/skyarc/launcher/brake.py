# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Jerk-limited, force-limited cart braking controller."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

from ..components.contract import Component, ComponentDescriptor, Determinism, ScenarioContext, StepOutput
from ..configuration.schema import CartConfig, ExteriorAtmosphereConfig
from ..effects.types import EffectBatch, Frame, Wrench
from ..events import EVENT_ABORT, EVENT_CART_STOPPED, Event
from ..linalg import clamp, dot, is_finite, scale
from ..names import BODY_CART, SLOT_CART_BRAKE
from ..state import Observation, SimulationState, body_com_world
from .atmosphere import quadratic_axial_drag_force_n
from .geometry import TubePath, guide_normal_bound_mps2, path_pose
from .launch_force import STANDARD_GRAVITY_MPS2, guide_resistance_force_n


@dataclass(frozen=True)
class BrakeCommand:
    force_n: float
    brake_acceleration_mps2: float
    required_deceleration_mps2: float
    remaining_control_distance_m: float
    resultant_load_g: float
    held: bool
    reversed: bool
    hold_force_n: float = 0.0


def compute_brake_command(
    *,
    speed_mps: float,
    remaining_control_distance_m: float,
    dt_s: float,
    mass_kg: float,
    force_limit_n: float,
    jerk_limit_mps3: float,
    previous_brake_acceleration_mps2: float,
    stopped_speed_threshold_mps: float,
    gravity_tangent_mps2: float,
    external_tangent_force_n: float,
    guide_normal_load_mps2: float,
    maximum_resultant_load_g: float | None,
) -> BrakeCommand:
    values = (
        speed_mps,
        remaining_control_distance_m,
        dt_s,
        mass_kg,
        force_limit_n,
        jerk_limit_mps3,
        previous_brake_acceleration_mps2,
        stopped_speed_threshold_mps,
        gravity_tangent_mps2,
        external_tangent_force_n,
        guide_normal_load_mps2,
    )
    if not is_finite(values):
        raise ValueError("brake-command inputs must be finite")
    if dt_s <= 0.0 or mass_kg <= 0.0 or force_limit_n < 0.0 or jerk_limit_mps3 <= 0.0:
        raise ValueError("brake timestep/mass must be positive and limits nonnegative")
    if stopped_speed_threshold_mps < 0.0 or previous_brake_acceleration_mps2 < 0.0:
        raise ValueError("brake threshold and prior acceleration must be nonnegative")
    if speed_mps < -stopped_speed_threshold_mps:
        return BrakeCommand(0.0, 0.0, 0.0, remaining_control_distance_m, 0.0, False, True)
    if speed_mps <= stopped_speed_threshold_mps:
        # A hold latch is a physical command, not merely a state label.  Remove the
        # residual sub-threshold velocity in one step and then cancel grade/external force.
        hold_force = mass_kg * (-speed_mps / dt_s - gravity_tangent_mps2) - external_tangent_force_n
        low = -force_limit_n
        high = force_limit_n
        if maximum_resultant_load_g is not None:
            resultant_limit = STANDARD_GRAVITY_MPS2 * maximum_resultant_load_g
            tangent_limit = mass_kg * math.sqrt(
                max(0.0, resultant_limit * resultant_limit - guide_normal_load_mps2**2)
            )
            low = max(low, -tangent_limit - external_tangent_force_n)
            high = min(high, tangent_limit - external_tangent_force_n)
        if low > high:
            hold_force = 0.0
        else:
            hold_force = clamp(hold_force, low, high)
        resultant_load = math.hypot(
            (external_tangent_force_n + hold_force) / mass_kg,
            guide_normal_load_mps2,
        ) / STANDARD_GRAVITY_MPS2
        return BrakeCommand(
            0.0,
            0.0,
            0.0,
            remaining_control_distance_m,
            resultant_load,
            True,
            False,
            hold_force,
        )

    required_deceleration = (
        math.inf
        if remaining_control_distance_m <= 0.0
        else speed_mps * speed_mps / (2.0 * remaining_control_distance_m)
    )
    desired_force = mass_kg * (required_deceleration + gravity_tangent_mps2)
    desired_force += external_tangent_force_n
    force_ceiling = force_limit_n
    if maximum_resultant_load_g is not None:
        resultant_limit = STANDARD_GRAVITY_MPS2 * maximum_resultant_load_g
        tangent_limit = mass_kg * math.sqrt(
            max(0.0, resultant_limit * resultant_limit - guide_normal_load_mps2**2)
        )
        force_ceiling = min(force_ceiling, max(0.0, tangent_limit + external_tangent_force_n))

    no_reverse_ceiling = (
        mass_kg * speed_mps / dt_s
        + mass_kg * gravity_tangent_mps2
        + external_tangent_force_n
    )
    force_ceiling = max(0.0, min(force_ceiling, no_reverse_ceiling))
    desired_force = clamp(desired_force, 0.0, force_ceiling)
    desired_acceleration = desired_force / mass_kg
    maximum_change = jerk_limit_mps3 * dt_s
    brake_acceleration = clamp(
        desired_acceleration,
        max(0.0, previous_brake_acceleration_mps2 - maximum_change),
        previous_brake_acceleration_mps2 + maximum_change,
    )
    force = min(force_ceiling, mass_kg * brake_acceleration)
    brake_acceleration = force / mass_kg
    tangential_non_gravitational = (external_tangent_force_n - force) / mass_kg
    resultant_load = math.hypot(tangential_non_gravitational, guide_normal_load_mps2) / STANDARD_GRAVITY_MPS2
    return BrakeCommand(
        force_n=force,
        brake_acceleration_mps2=brake_acceleration,
        required_deceleration_mps2=required_deceleration,
        remaining_control_distance_m=remaining_control_distance_m,
        resultant_load_g=resultant_load,
        held=False,
        reversed=False,
    )


class ForceLimitedCartBrake(Component):
    """Apply post-release braking while respecting one shared cart load budget."""

    def __init__(
        self,
        layout: TubePath,
        cart: CartConfig,
        *,
        exit_track_length_m: float,
        reference_density_kg_m3: float,
        exterior_density_ratio: float,
        exterior_atmosphere: ExteriorAtmosphereConfig | None = None,
        gravity_mps2: tuple[float, float, float] = (0.0, 0.0, -STANDARD_GRAVITY_MPS2),
        code_hash: str,
    ) -> None:
        if not code_hash:
            raise ValueError("brake code hash may not be empty")
        if not is_finite((exit_track_length_m, reference_density_kg_m3, exterior_density_ratio)):
            raise ValueError("brake environment values must be finite")
        if exit_track_length_m <= 0.0 or reference_density_kg_m3 <= 0.0 or exterior_density_ratio < 0.0:
            raise ValueError("brake track/density values are outside their valid range")
        self._layout = layout
        self._cart = cart
        self._track_length = exit_track_length_m
        self._reference_density = reference_density_kg_m3
        self._exterior_ratio = exterior_density_ratio
        self._exterior_atmosphere = exterior_atmosphere
        self._gravity = gravity_mps2
        self._code_hash = code_hash
        self._previous_acceleration = 0.0
        self._last_command: BrakeCommand | None = None
        self._stop_event_emitted = False
        self._abort_emitted = False

    @property
    def descriptor(self) -> ComponentDescriptor:
        return ComponentDescriptor(
            slot=SLOT_CART_BRAKE,
            model_id="force_limited_v1",
            model_version="1.0.0",
            parameter_schema_version="1",
            code_hash=self._code_hash,
            determinism=Determinism.DETERMINISTIC,
        )

    def prepare(self, context: ScenarioContext) -> None:
        pass

    def reset(self, initial_state: SimulationState) -> None:
        self._previous_acceleration = 0.0
        self._last_command = None
        self._stop_event_emitted = False
        self._abort_emitted = False

    def _density_ratio(self, altitude_m: float) -> float:
        if self._exterior_atmosphere is None:
            return self._exterior_ratio
        return self._exterior_atmosphere.density_ratio(altitude_m)

    def pre_step(self, observation: Observation) -> StepOutput:
        if observation.coupled or observation.axial.s_cart_m < self._layout.length_m:
            self._last_command = None
            return StepOutput.empty(SLOT_CART_BRAKE)
        pose = path_pose(self._layout, observation.axial.s_cart_m)
        speed = dot(observation.cart.linear_velocity, pose.tangent)
        progress = observation.axial.s_cart_m - self._layout.length_m
        remaining = self._track_length - self._cart.brake_stop_margin_m - progress
        drag = quadratic_axial_drag_force_n(
            self._reference_density * self._density_ratio(observation.cart.position[2]),
            self._cart.drag_coefficient,
            self._cart.frontal_area_m2,
            speed,
        )
        resistance = guide_resistance_force_n(speed, self._cart.guide_resistance_n)
        normal_load = guide_normal_bound_mps2(
            abs(speed),
            pose.signed_curvature_per_m,
            self._gravity,
            pose.normal,
        )
        command = compute_brake_command(
            speed_mps=speed,
            remaining_control_distance_m=remaining,
            dt_s=observation.dt_s,
            mass_kg=observation.cart.mass_kg,
            force_limit_n=self._cart.brake_force_limit_n,
            jerk_limit_mps3=self._cart.brake_jerk_limit_mps3,
            previous_brake_acceleration_mps2=self._previous_acceleration,
            stopped_speed_threshold_mps=self._cart.stopped_speed_threshold_mps,
            gravity_tangent_mps2=dot(self._gravity, pose.tangent),
            external_tangent_force_n=drag + resistance,
            guide_normal_load_mps2=normal_load,
            maximum_resultant_load_g=self._cart.maximum_resultant_load_g,
        )
        self._last_command = command
        self._previous_acceleration = command.brake_acceleration_mps2
        if command.held:
            events = ()
            if not self._stop_event_emitted:
                self._stop_event_emitted = True
                events = (
                    Event(
                        name=EVENT_CART_STOPPED,
                        time_s=observation.time_s,
                        step_index=observation.step_index,
                        source=SLOT_CART_BRAKE,
                        data={"speed_mps": speed, "track_progress_m": progress},
                    ),
                )
            wrench = Wrench(
                source=SLOT_CART_BRAKE,
                body=BODY_CART,
                force_n=scale(pose.tangent, command.hold_force_n),
                application_point_m=body_com_world(observation.cart),
                frame=Frame.WORLD,
            )
            return StepOutput(
                effects=EffectBatch(source=SLOT_CART_BRAKE, wrenches=(wrench,)),
                events=events,
            )
        if command.reversed:
            events = ()
            if not self._abort_emitted:
                self._abort_emitted = True
                events = (
                    Event(
                        name=EVENT_ABORT,
                        time_s=observation.time_s,
                        step_index=observation.step_index,
                        source=SLOT_CART_BRAKE,
                        data={"reason": "cart_reversed", "speed_mps": speed},
                    ),
                )
            return StepOutput(effects=EffectBatch.empty(SLOT_CART_BRAKE), events=events)
        wrench = Wrench(
            source=SLOT_CART_BRAKE,
            body=BODY_CART,
            force_n=scale(pose.tangent, -command.force_n),
            application_point_m=body_com_world(observation.cart),
            frame=Frame.WORLD,
        )
        return StepOutput(
            effects=EffectBatch(source=SLOT_CART_BRAKE, wrenches=(wrench,))
        )

    def post_step(self, state: SimulationState) -> StepOutput:
        return StepOutput.empty(SLOT_CART_BRAKE)

    def snapshot_state(self) -> Mapping[str, Any]:
        if self._last_command is None:
            return {"active": False, "held": False}
        return {
            "active": not self._last_command.held and not self._last_command.reversed,
            "held": self._last_command.held,
            "reversed": self._last_command.reversed,
            "last_force_n": self._last_command.force_n,
            "last_brake_acceleration_mps2": self._last_command.brake_acceleration_mps2,
            "last_resultant_load_g": self._last_command.resultant_load_g,
            "hold_force_n": self._last_command.hold_force_n,
            "remaining_control_distance_m": self._last_command.remaining_control_distance_m,
        }
