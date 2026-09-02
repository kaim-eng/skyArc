# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Abstract axial electromagnetic launch-force controller."""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from ..components.contract import Component, ComponentDescriptor, Determinism, ScenarioContext, StepOutput
from ..configuration.schema import (
    ForcePositionPointConfig,
    GuidedAerodynamicsConfig,
    LaunchControlConfig,
)
from ..effects.types import EffectBatch, Frame, Wrench
from ..events import EVENT_ABORT, Event
from ..linalg import add, clamp, dot, is_finite, scale
from ..names import BODY_CART, BODY_ROCKET, SLOT_LAUNCH_FORCE
from ..state import Observation, SimulationState, body_com_world
from .atmosphere import quadratic_axial_drag_force_n
from .geometry import TubePath, guide_normal_bound_mps2, path_pose


STANDARD_GRAVITY_MPS2 = 9.81

ENTRANCE_PROJECTION_TOLERANCE_M = 1e-6
"""Numerical slack applied to the assembly coordinate at the tube entrance.

An assembly seated exactly on the entrance does not project to exactly zero: the
nearest-point search returns its own residual, measured at 2.0e-8 m for the reference
curve. The sign of that residual is numerical, not physical, and a strict ``0.0 <= s``
test therefore decides whether the launcher engages at all on a coin-flip. Getting it
wrong is not a small error -- on the reference curve's 45-degree entrance the assembly
rolls backwards at ``-g sin 45`` and the coordinate only grows more negative, so the
mission never launches. This tolerance is fifty times the observed residual and five
orders of magnitude below the 0.05 m guide clearance, so it cannot absorb a physical
displacement.
"""


@dataclass(frozen=True)
class LaunchCommand:
    force_n: float
    target_acceleration_mps2: float
    acceleration_command_mps2: float
    ramp_factor: float
    distance_to_ramp_down_m: float
    normal_load_mps2: float
    resultant_limit_force_n: float


def guide_resistance_force_n(speed_mps: float, magnitude_n: float) -> float:
    if not is_finite((speed_mps, magnitude_n)) or magnitude_n < 0.0:
        raise ValueError("guide resistance inputs must be finite and magnitude nonnegative")
    if speed_mps > 0.0:
        return -magnitude_n
    if speed_mps < 0.0:
        return magnitude_n
    return 0.0


def _interpolate_force(
    points: Sequence[ForcePositionPointConfig], position_m: float
) -> float:
    if len(points) < 2:
        raise ValueError("force-versus-position mode requires at least two points")
    positions = tuple(point.position_m for point in points)
    if position_m <= positions[0]:
        return points[0].force_n
    if position_m >= positions[-1]:
        return points[-1].force_n
    right = bisect.bisect_right(positions, position_m)
    left_point = points[right - 1]
    right_point = points[right]
    fraction = (position_m - left_point.position_m) / (
        right_point.position_m - left_point.position_m
    )
    return left_point.force_n + fraction * (right_point.force_n - left_point.force_n)


def _launch_ramp_components(
    *,
    position_m: float,
    elapsed_s: float,
    path_length_m: float,
    ramp_up_distance_m: float,
    ramp_down_distance_m: float,
    maximum_acceleration_mps2: float,
) -> tuple[float, float]:
    """Return the ramp-up and ramp-down envelopes.

    A strictly position-only ramp has an equilibrium at ``s=0, v=0``.  The time envelope
    removes that singularity.  Its duration ``sqrt(6 d/a_max)`` is the constant-jerk time
    that traverses the configured distance while acceleration rises from zero to ``a_max``.
    The larger of time and spatial progress governs ramp-up; ramp-down remains spatial so
    force is exactly zero at the named exit.
    """
    values = (
        position_m,
        elapsed_s,
        path_length_m,
        ramp_up_distance_m,
        ramp_down_distance_m,
        maximum_acceleration_mps2,
    )
    if not is_finite(values):
        raise ValueError("launch ramp inputs must be finite")
    if elapsed_s < 0.0 or path_length_m <= 0.0 or maximum_acceleration_mps2 <= 0.0:
        raise ValueError("launch ramp time must be nonnegative and length/acceleration positive")
    if ramp_up_distance_m < 0.0 or ramp_down_distance_m < 0.0:
        raise ValueError("launch ramp distances must be nonnegative")

    if ramp_up_distance_m == 0.0:
        up = 1.0
    else:
        ramp_time = math.sqrt(6.0 * ramp_up_distance_m / maximum_acceleration_mps2)
        up = max(position_m / ramp_up_distance_m, elapsed_s / ramp_time)
    if ramp_down_distance_m == 0.0:
        down = 1.0 if position_m < path_length_m else 0.0
    else:
        down = (path_length_m - position_m) / ramp_down_distance_m
    return clamp(up, 0.0, 1.0), clamp(down, 0.0, 1.0)


def launch_ramp_factor(
    *,
    position_m: float,
    elapsed_s: float,
    path_length_m: float,
    ramp_up_distance_m: float,
    ramp_down_distance_m: float,
    maximum_acceleration_mps2: float,
) -> float:
    """Resolved commanded-acceleration envelope with a rest-state bootstrap.

    Gravity/drag hold bias is retained while this factor ramps the positive acceleration
    request, so an inclined body does not roll backward before the controller can start.
    Ramp-down applies to the complete launcher force and reaches zero at the exit.
    """
    up, down = _launch_ramp_components(
        position_m=position_m,
        elapsed_s=elapsed_s,
        path_length_m=path_length_m,
        ramp_up_distance_m=ramp_up_distance_m,
        ramp_down_distance_m=ramp_down_distance_m,
        maximum_acceleration_mps2=maximum_acceleration_mps2,
    )
    return min(up, down)


def compute_launch_command(
    control: LaunchControlConfig,
    *,
    position_m: float,
    speed_mps: float,
    elapsed_s: float,
    path_length_m: float,
    assembly_mass_kg: float,
    gravity_tangent_mps2: float,
    drag_force_tangent_n: float,
    resistance_force_tangent_n: float,
    guide_normal_load_mps2: float = 0.0,
) -> LaunchCommand:
    """Resolve the four baseline modes and both control ceilings in declared order."""
    values = (
        position_m,
        speed_mps,
        elapsed_s,
        path_length_m,
        assembly_mass_kg,
        gravity_tangent_mps2,
        drag_force_tangent_n,
        resistance_force_tangent_n,
        guide_normal_load_mps2,
    )
    if not is_finite(values):
        raise ValueError("launch command inputs must be finite")
    if path_length_m <= 0.0 or assembly_mass_kg <= 0.0:
        raise ValueError("launch path length and assembly mass must be positive")
    if not 0.0 <= position_m < path_length_m:
        return LaunchCommand(0.0, 0.0, 0.0, 0.0, 0.0, guide_normal_load_mps2, 0.0)

    ramp_down_start = path_length_m - control.force_ramp_down_distance_m
    distance_to_ramp_down = max(0.0, ramp_down_start - position_m)
    environmental_force = drag_force_tangent_n + resistance_force_tangent_n
    # Modes 1 and 4 name a launcher force directly; modes 2 and 3 name a motion and let the
    # controller find the force. The hold bias below exists to serve the motion-specified
    # modes, so it may not raise the delivered force above what a force-specified mode asked
    # for -- an authored coast or low-force region would otherwise become a hold-station
    # region without any diagnostic.
    requested_force_n: float | None = None

    if control.mode == "constant_acceleration":
        target_acceleration = control.maximum_acceleration_mps2
    elif control.mode == "target_exit_speed":
        if distance_to_ramp_down <= 0.0:
            target_acceleration = 0.0
        else:
            target_acceleration = max(
                0.0,
                (control.target_exit_speed_mps**2 - speed_mps**2)
                / (2.0 * distance_to_ramp_down),
            )
    elif control.mode in {"constant_force", "force_vs_position"}:
        raw_force = (
            control.maximum_force_n
            if control.mode == "constant_force"
            else _interpolate_force(control.force_vs_position, position_m)
        )
        requested_force_n = raw_force
        target_acceleration = max(
            0.0,
            raw_force / assembly_mass_kg
            + gravity_tangent_mps2
            + environmental_force / assembly_mass_kg,
        )
    else:
        raise ValueError(f"unsupported launch-control mode {control.mode!r}")

    acceleration_command = min(target_acceleration, control.maximum_acceleration_mps2)
    up_ramp, down_ramp = _launch_ramp_components(
        position_m=position_m,
        elapsed_s=elapsed_s,
        path_length_m=path_length_m,
        ramp_up_distance_m=control.force_ramp_up_distance_m,
        ramp_down_distance_m=control.force_ramp_down_distance_m,
        maximum_acceleration_mps2=control.maximum_acceleration_mps2,
    )
    hold_force = max(
        0.0,
        -assembly_mass_kg * gravity_tangent_mps2 - environmental_force,
    )
    force = hold_force + up_ramp * assembly_mass_kg * acceleration_command
    if requested_force_n is not None:
        # Never above the authored request. When the request exceeds the hold requirement the
        # sum already equals it exactly, so this only binds where the hold bias would have
        # overridden the table; the acceleration ceiling still binds first when it is lower.
        force = min(force, requested_force_n)
    force = clamp(force, 0.0, control.maximum_force_n)

    resultant_limit_force = control.maximum_force_n
    if control.maximum_resultant_load_g is not None:
        resultant_limit = STANDARD_GRAVITY_MPS2 * control.maximum_resultant_load_g
        available_squared = resultant_limit * resultant_limit - guide_normal_load_mps2**2
        resultant_limit_force = assembly_mass_kg * math.sqrt(max(0.0, available_squared))
        force = min(force, resultant_limit_force)

    force *= down_ramp
    ramp = min(up_ramp, down_ramp)
    return LaunchCommand(
        force_n=force,
        target_acceleration_mps2=target_acceleration,
        acceleration_command_mps2=acceleration_command,
        ramp_factor=ramp,
        distance_to_ramp_down_m=distance_to_ramp_down,
        normal_load_mps2=guide_normal_load_mps2,
        resultant_limit_force_n=resultant_limit_force,
    )


class AbstractAxialLaunchForce(Component):
    """Return a zero-torque world-frame launch wrench on the guided cart."""

    def __init__(
        self,
        layout: TubePath,
        control: LaunchControlConfig,
        aerodynamics: GuidedAerodynamicsConfig,
        *,
        reference_density_kg_m3: float,
        guide_resistance_n: float,
        gravity_mps2: tuple[float, float, float] = (0.0, 0.0, -STANDARD_GRAVITY_MPS2),
        code_hash: str,
    ) -> None:
        if not code_hash:
            raise ValueError("launch-force code hash may not be empty")
        if not is_finite((*gravity_mps2, reference_density_kg_m3, guide_resistance_n)):
            raise ValueError("launch-force environment values must be finite")
        if reference_density_kg_m3 <= 0.0 or guide_resistance_n < 0.0:
            raise ValueError("reference density must be positive and resistance nonnegative")
        self._layout = layout
        self._control = control
        self._aerodynamics = aerodynamics
        self._reference_density = reference_density_kg_m3
        self._guide_resistance = guide_resistance_n
        self._gravity = gravity_mps2
        self._code_hash = code_hash
        self._start_time_s = 0.0
        self._last_command: LaunchCommand | None = None
        self._behind_entrance_reported = False

    @property
    def descriptor(self) -> ComponentDescriptor:
        return ComponentDescriptor(
            slot=SLOT_LAUNCH_FORCE,
            model_id="abstract_axial_v1",
            model_version="1.0.0",
            parameter_schema_version="1",
            code_hash=self._code_hash,
            determinism=Determinism.DETERMINISTIC,
        )

    def prepare(self, context: ScenarioContext) -> None:
        if self._control.mode == "force_vs_position" and len(self._control.force_vs_position) < 2:
            raise ValueError("force_vs_position mode requires an authored force table")

    def reset(self, initial_state: SimulationState) -> None:
        self._start_time_s = initial_state.time_s
        self._last_command = None
        self._behind_entrance_reported = False

    def pre_step(self, observation: Observation) -> StepOutput:
        assembly_mass = observation.axial.assembly_mass_kg
        assembly_com = scale(
            add(
                scale(body_com_world(observation.cart), observation.cart.mass_kg),
                scale(body_com_world(observation.rocket), observation.rocket.mass_kg),
            ),
            1.0 / assembly_mass,
        )
        assembly_velocity = scale(
            add(
                scale(observation.cart.linear_velocity, observation.cart.mass_kg),
                scale(observation.rocket.linear_velocity, observation.rocket.mass_kg),
            ),
            1.0 / assembly_mass,
        )
        position = self._layout.axial_position(assembly_com)
        if -ENTRANCE_PROJECTION_TOLERANCE_M <= position < 0.0:
            # Seated on the entrance; the residual's sign is the projection's, not the
            # assembly's. See ENTRANCE_PROJECTION_TOLERANCE_M.
            position = 0.0
        if not observation.coupled:
            # Correct and quiet: after release the launcher has nothing to push.
            self._last_command = None
            return StepOutput.empty(SLOT_LAUNCH_FORCE)
        if not 0.0 <= position < self._layout.length_m:
            self._last_command = None
            events: tuple[Event, ...] = ()
            # Past the exit is how every launch ends. Behind the entrance while still
            # coupled is a stalled mission: this component is the only thing that can
            # move the assembly, and returning an empty command forever looks exactly
            # like a launcher that is merely between stages. Say so once, loudly.
            if position < 0.0 and not self._behind_entrance_reported:
                self._behind_entrance_reported = True
                events = (
                    Event(
                        name=EVENT_ABORT,
                        time_s=observation.time_s,
                        step_index=observation.step_index,
                        source=SLOT_LAUNCH_FORCE,
                        data={
                            "reason": "assembly_behind_tube_entrance",
                            "assembly_s_m": position,
                            "tolerance_m": ENTRANCE_PROJECTION_TOLERANCE_M,
                        },
                    ),
                )
            return StepOutput(
                effects=EffectBatch.empty(SLOT_LAUNCH_FORCE), events=events
            )
        pose = path_pose(self._layout, position)
        speed = dot(assembly_velocity, pose.tangent)
        density = self._reference_density * observation.axial.effective_density_ratio
        drag = quadratic_axial_drag_force_n(
            density,
            self._aerodynamics.drag_coefficient,
            self._aerodynamics.reference_area_m2,
            speed,
            self._aerodynamics.axial_air_velocity_mps,
        )
        resistance = guide_resistance_force_n(speed, self._guide_resistance)
        normal_load = guide_normal_bound_mps2(
            abs(speed),
            pose.signed_curvature_per_m,
            self._gravity,
            pose.normal,
        )
        command = compute_launch_command(
            self._control,
            position_m=position,
            speed_mps=speed,
            elapsed_s=max(0.0, observation.time_s - self._start_time_s),
            path_length_m=self._layout.length_m,
            assembly_mass_kg=observation.axial.assembly_mass_kg,
            gravity_tangent_mps2=dot(self._gravity, pose.tangent),
            drag_force_tangent_n=drag,
            resistance_force_tangent_n=resistance,
            guide_normal_load_mps2=normal_load,
        )
        self._last_command = command
        wrench = Wrench(
            source=SLOT_LAUNCH_FORCE,
            body=BODY_CART,
            force_n=scale(pose.tangent, command.force_n),
            application_point_m=body_com_world(observation.cart),
            frame=Frame.WORLD,
        )
        return StepOutput(
            effects=EffectBatch(source=SLOT_LAUNCH_FORCE, wrenches=(wrench,))
        )

    def post_step(self, state: SimulationState) -> StepOutput:
        return StepOutput.empty(SLOT_LAUNCH_FORCE)

    def snapshot_state(self) -> Mapping[str, Any]:
        if self._last_command is None:
            return {"active": False}
        return {
            "active": True,
            "last_force_n": self._last_command.force_n,
            "last_acceleration_command_mps2": self._last_command.acceleration_command_mps2,
            "last_ramp_factor": self._last_command.ramp_factor,
            "last_normal_load_mps2": self._last_command.normal_load_mps2,
        }
