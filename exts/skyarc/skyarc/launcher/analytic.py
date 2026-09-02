# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Controlled guided-launch trajectory using the common effect boundary."""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..configuration.schema import ScenarioConfig
from ..effects.aggregator import aggregate, axial_force, axial_slot_force
from ..effects.backends.analytic import AnalyticBackend
from ..linalg import dot
from ..names import (
    BODY_CART,
    BODY_ROCKET,
    JOINT_COUPLING,
    JOINT_GUIDE,
    MARKER_ASSEMBLY_EXIT,
    MARKER_ROCKET_STAGNATION,
    PAIR_ROCKET_CRADLE,
    SLOT_ATMOSPHERE,
    SLOT_CART_BRAKE,
    SLOT_GUIDE,
    SLOT_LAUNCH_FORCE,
)
from ..state import AxialQuantities, BodyState, Observation, SimulationState
from .atmosphere import DensityDragModel
from .brake import ForceLimitedCartBrake
from .geometry import TubePath, guide_normal_bound_mps2, normal_jerk_mps3, path_pose
from .guide import IdealPathGuide
from .launch_force import AbstractAxialLaunchForce


@dataclass(frozen=True)
class GuidedTrajectoryResult:
    exit_speed_mps: float
    elapsed_s: float
    step_count: int
    peak_resultant_load_g: float
    peak_normal_jerk_mps3: float
    launch_work_j: float
    drag_work_j: float
    resistance_work_j: float
    final_state: SimulationState


@dataclass(frozen=True)
class CartBrakingResult:
    stop_distance_m: float
    elapsed_s: float
    step_count: int
    final_speed_mps: float
    peak_resultant_load_g: float
    brake_work_j: float
    drag_work_j: float
    resistance_work_j: float
    reversed: bool
    final_state: SimulationState


def _marker_axial_offset(config: ScenarioConfig, name: str) -> float:
    marker = config.markers[name]
    if abs(marker.offset_m[1]) > 1e-12 or abs(marker.offset_m[2]) > 1e-12:
        raise ValueError(
            f"analytic guided trajectory requires axial marker {name!r}; got {marker.offset_m}"
        )
    return marker.offset_m[0]


def _observation(
    state: SimulationState,
    layout: TubePath,
    *,
    s_cart_m: float | None = None,
    s_rocket_m: float | None = None,
    exit_offset_m: float,
    stagnation_offset_m: float,
) -> Observation:
    cart = state.body(BODY_CART)
    rocket = state.body(BODY_ROCKET)
    s_cart = layout.axial_position(cart.position) if s_cart_m is None else s_cart_m
    s_rocket = layout.axial_position(rocket.position) if s_rocket_m is None else s_rocket_m
    pose = path_pose(layout, s_cart)
    exit_s = s_rocket + exit_offset_m
    stagnation_s = s_rocket + stagnation_offset_m
    stage = layout.stage_index(stagnation_s)
    return Observation(
        source_model="analytic_ground_truth_v1",
        time_s=state.time_s,
        step_index=state.step_index,
        dt_s=state.dt_s,
        state=state.frozen(),
        axial=AxialQuantities(
            s_cart_m=s_cart,
            s_rocket_m=s_rocket,
            marker_s_m={
                MARKER_ASSEMBLY_EXIT: exit_s,
                MARKER_ROCKET_STAGNATION: stagnation_s,
            },
            cart_axial_velocity_mps=dot(cart.linear_velocity, pose.tangent),
            rocket_axial_velocity_mps=dot(rocket.linear_velocity, pose.tangent),
            assembly_mass_kg=cart.mass_kg + rocket.mass_kg,
            stage_index=-1 if stage is None else stage,
            stage_name="exterior" if stage is None else layout.stages[stage].name,
            effective_density_ratio=layout.density_ratio(stagnation_s),
            separation_gap_m=0.0,
            separation_rate_mps=0.0,
        ),
        coupled=True,
    )


def simulate_guided_launch(
    config: ScenarioConfig,
    layout: TubePath,
    *,
    dt_s: float | None = None,
    gravity_mps2: tuple[float, float, float] = (0.0, 0.0, -9.81),
) -> GuidedTrajectoryResult:
    """Run the attached phase until the named assembly marker crosses the exit.

    The runner intentionally stops before release.  Separation, braking, and free flight
    belong to later components and state-machine phases; keeping this case contact-free is
    what makes it useful for timestep and energy convergence.
    """
    resolved_dt = config.simulation.physics_dt_s if dt_s is None else dt_s
    if not math.isfinite(resolved_dt) or resolved_dt <= 0.0:
        raise ValueError("analytic trajectory timestep must be finite and positive")
    initial_pose = path_pose(layout, 0.0)
    initial_state = SimulationState(
        time_s=0.0,
        step_index=0,
        dt_s=resolved_dt,
        bodies={
            BODY_CART: BodyState(
                name=BODY_CART,
                position=initial_pose.position_m,
                mass_kg=config.cart.mass_kg,
            ),
            BODY_ROCKET: BodyState(
                name=BODY_ROCKET,
                position=initial_pose.position_m,
                mass_kg=config.rocket.initial_mass_kg,
            ),
        },
        joint_active={JOINT_COUPLING: True, JOINT_GUIDE: True},
        collision_pair_active={PAIR_ROCKET_CRADLE: True},
    ).frozen()
    backend = AnalyticBackend(initial_state, layout, gravity_mps2=gravity_mps2)
    atmosphere = DensityDragModel(
        layout,
        config.guided_phase_aerodynamics,
        reference_density_kg_m3=config.simulation.reference_density_kg_m3,
        cart=config.cart,
        exterior_atmosphere=config.tube.exterior_atmosphere,
        code_hash="analytic-density-drag-v1",
    )
    launcher = AbstractAxialLaunchForce(
        layout,
        config.launch_control,
        config.guided_phase_aerodynamics,
        reference_density_kg_m3=config.simulation.reference_density_kg_m3,
        guide_resistance_n=config.cart.guide_resistance_n,
        gravity_mps2=gravity_mps2,
        code_hash="analytic-launch-force-v1",
    )
    guide = IdealPathGuide(
        layout,
        model_id=config.models.guide,
        resistance_n=config.cart.guide_resistance_n,
        maximum_tracking_error_m=config.tube.guide_clearance_m,
        gravity_mps2=gravity_mps2,
        code_hash="analytic-guide-v1",
    )
    from ..components.contract import ScenarioContext

    context = ScenarioContext(
        scenario_id=config.experiment.condition_id,
        backend_capabilities=backend.capabilities.features,
    )
    atmosphere.prepare(context)
    launcher.prepare(context)
    guide.prepare(context)
    atmosphere.reset(initial_state)
    launcher.reset(initial_state)
    guide.reset(initial_state)

    exit_offset = _marker_axial_offset(config, MARKER_ASSEMBLY_EXIT)
    stagnation_offset = _marker_axial_offset(config, MARKER_ROCKET_STAGNATION)
    maximum_steps = math.ceil(config.simulation.maximum_run_time_s / resolved_dt)
    peak_load_g = 0.0
    peak_normal_jerk = 0.0
    launch_work = 0.0
    drag_work = 0.0
    resistance_work = 0.0

    for _ in range(maximum_steps):
        pre_state = backend.read_state()
        observation = _observation(
            pre_state,
            layout,
            s_cart_m=backend.path_coordinate(BODY_CART),
            s_rocket_m=backend.path_coordinate(BODY_ROCKET),
            exit_offset_m=exit_offset,
            stagnation_offset_m=stagnation_offset,
        )
        if observation.axial.marker(MARKER_ASSEMBLY_EXIT) >= layout.length_m:
            break
        pose = path_pose(layout, observation.axial.s_cart_m)
        speed = observation.axial.cart_axial_velocity_mps
        launch_output = launcher.pre_step(observation)
        atmosphere_output = atmosphere.pre_step(observation)
        guide_output = guide.pre_step(observation)
        accepted = aggregate(
            (launch_output.effects, atmosphere_output.effects, guide_output.effects),
            pre_state,
        )
        backend.apply(accepted)
        backend.step()
        post_state = backend.read_state()
        post_observation = _observation(
            post_state,
            layout,
            s_cart_m=backend.path_coordinate(BODY_CART),
            s_rocket_m=backend.path_coordinate(BODY_ROCKET),
            exit_offset_m=exit_offset,
            stagnation_offset_m=stagnation_offset,
        )
        delta_s = post_observation.axial.s_cart_m - observation.axial.s_cart_m
        launch_work += axial_slot_force(
            accepted.load(BODY_CART), SLOT_LAUNCH_FORCE, pose.tangent
        ) * delta_s
        drag_work += axial_slot_force(
            accepted.load(BODY_CART), SLOT_ATMOSPHERE, pose.tangent
        ) * delta_s
        resistance_work += axial_slot_force(
            accepted.load(BODY_CART), SLOT_GUIDE, pose.tangent
        ) * delta_s

        launch_force = axial_slot_force(
            accepted.load(BODY_CART), SLOT_LAUNCH_FORCE, pose.tangent
        )
        normal_acceleration = guide_normal_bound_mps2(
            abs(speed),
            pose.signed_curvature_per_m,
            gravity_mps2,
            pose.normal,
        )
        resultant = math.hypot(
            launch_force / observation.axial.assembly_mass_kg,
            normal_acceleration,
        ) / 9.81
        peak_load_g = max(peak_load_g, resultant)
        # Section 7's normal jerk, evaluated from the resolved tangential acceleration.
        # Differencing `normal_acceleration` would measure something else: that value is the
        # two-sided guide-normal *bound*, a max() of two branches which is not differentiable
        # where they swap, so its time derivative coincides with normal jerk only while the
        # curvature branch dominates.
        tangential_force = axial_force(
            accepted.load(BODY_CART), pose.tangent
        ) + axial_force(accepted.load(BODY_ROCKET), pose.tangent)
        tangential_acceleration = tangential_force / observation.axial.assembly_mass_kg + dot(
            gravity_mps2, pose.tangent
        )
        peak_normal_jerk = max(
            peak_normal_jerk,
            abs(
                normal_jerk_mps3(
                    speed,
                    tangential_acceleration,
                    pose.signed_curvature_per_m,
                    pose.curvature_rate_per_m2,
                )
            ),
        )
    else:
        raise RuntimeError(
            f"analytic launch did not reach the exit within {config.simulation.maximum_run_time_s}s"
        )

    final_state = backend.read_state()
    final_observation = _observation(
        final_state,
        layout,
        s_cart_m=backend.path_coordinate(BODY_CART),
        s_rocket_m=backend.path_coordinate(BODY_ROCKET),
        exit_offset_m=exit_offset,
        stagnation_offset_m=stagnation_offset,
    )
    return GuidedTrajectoryResult(
        exit_speed_mps=final_observation.axial.cart_axial_velocity_mps,
        elapsed_s=final_state.time_s,
        step_count=final_state.step_index,
        peak_resultant_load_g=peak_load_g,
        peak_normal_jerk_mps3=peak_normal_jerk,
        launch_work_j=launch_work,
        drag_work_j=drag_work,
        resistance_work_j=resistance_work,
        final_state=final_state,
    )


def _cart_observation(
    state: SimulationState,
    layout: TubePath,
    *,
    s_cart_m: float,
) -> Observation:
    cart = state.body(BODY_CART)
    pose = path_pose(layout, s_cart_m)
    speed = dot(cart.linear_velocity, pose.tangent)
    return Observation(
        source_model="analytic_ground_truth_v1",
        time_s=state.time_s,
        step_index=state.step_index,
        dt_s=state.dt_s,
        state=state.frozen(),
        axial=AxialQuantities(
            s_cart_m=s_cart_m,
            s_rocket_m=s_cart_m,
            marker_s_m={},
            cart_axial_velocity_mps=speed,
            rocket_axial_velocity_mps=0.0,
            assembly_mass_kg=cart.mass_kg,
            stage_index=-1,
            stage_name="exterior",
            effective_density_ratio=layout.density_ratio(s_cart_m),
            separation_gap_m=0.0,
            separation_rate_mps=0.0,
        ),
        coupled=False,
    )


def simulate_cart_braking(
    config: ScenarioConfig,
    layout: TubePath,
    *,
    entry_speed_mps: float | None = None,
    dt_s: float | None = None,
    gravity_mps2: tuple[float, float, float] = (0.0, 0.0, -9.81),
) -> CartBrakingResult:
    """Run the released cart from the tube exit until the brake hold threshold."""
    resolved_dt = config.simulation.physics_dt_s if dt_s is None else dt_s
    resolved_speed = (
        config.launch_control.target_exit_speed_mps
        if entry_speed_mps is None
        else entry_speed_mps
    )
    if not math.isfinite(resolved_dt) or resolved_dt <= 0.0:
        raise ValueError("analytic braking timestep must be finite and positive")
    if not math.isfinite(resolved_speed) or resolved_speed < 0.0:
        raise ValueError("analytic braking entry speed must be finite and nonnegative")
    exit_pose = path_pose(layout, layout.length_m)
    initial_state = SimulationState(
        time_s=0.0,
        step_index=0,
        dt_s=resolved_dt,
        bodies={
            BODY_CART: BodyState(
                name=BODY_CART,
                position=exit_pose.position_m,
                orientation=(
                    math.cos(-0.5 * math.radians(exit_pose.inclination_deg)),
                    0.0,
                    math.sin(-0.5 * math.radians(exit_pose.inclination_deg)),
                    0.0,
                ),
                linear_velocity=tuple(
                    value * resolved_speed for value in exit_pose.tangent
                ),
                mass_kg=config.cart.mass_kg,
            ),
        },
        joint_active={JOINT_COUPLING: False, JOINT_GUIDE: True},
        collision_pair_active={PAIR_ROCKET_CRADLE: True},
    ).frozen()
    backend = AnalyticBackend(initial_state, layout, gravity_mps2=gravity_mps2)
    atmosphere = DensityDragModel(
        layout,
        config.guided_phase_aerodynamics,
        reference_density_kg_m3=config.simulation.reference_density_kg_m3,
        cart=config.cart,
        exterior_atmosphere=config.tube.exterior_atmosphere,
        code_hash="analytic-density-drag-v1",
    )
    guide = IdealPathGuide(
        layout,
        model_id=config.models.guide,
        resistance_n=config.cart.guide_resistance_n,
        maximum_tracking_error_m=config.tube.guide_clearance_m,
        gravity_mps2=gravity_mps2,
        code_hash="analytic-guide-v1",
    )
    track_length = (
        config.tube.exit_track.length_m
        if config.tube.exit_track is not None
        else config.tube.exit_brake_track_length_m
    )
    brake = ForceLimitedCartBrake(
        layout,
        config.cart,
        exit_track_length_m=track_length,
        reference_density_kg_m3=config.simulation.reference_density_kg_m3,
        exterior_density_ratio=config.tube.exterior_effective_density_ratio,
        exterior_atmosphere=config.tube.exterior_atmosphere,
        gravity_mps2=gravity_mps2,
        code_hash="analytic-cart-brake-v1",
    )
    from ..components.contract import ScenarioContext

    context = ScenarioContext(
        scenario_id=config.experiment.condition_id + ".cart_braking",
        backend_capabilities=backend.capabilities.features,
    )
    for component in (atmosphere, guide, brake):
        component.prepare(context)
        component.reset(initial_state)

    maximum_steps = math.ceil(config.simulation.maximum_run_time_s / resolved_dt)
    peak_load = 0.0
    brake_work = 0.0
    drag_work = 0.0
    resistance_work = 0.0
    reversed_motion = False
    for _ in range(maximum_steps):
        pre_state = backend.read_state()
        pre_s = backend.path_coordinate(BODY_CART)
        observation = _cart_observation(pre_state, layout, s_cart_m=pre_s)
        brake_output = brake.pre_step(observation)
        if observation.axial.cart_axial_velocity_mps <= config.cart.stopped_speed_threshold_mps:
            break
        pose = path_pose(layout, pre_s)
        outputs = (
            brake_output,
            atmosphere.pre_step(observation),
            guide.pre_step(observation),
        )
        accepted = aggregate((output.effects for output in outputs), pre_state)
        backend.apply(accepted)
        backend.step()
        post_state = backend.read_state()
        post_s = backend.path_coordinate(BODY_CART)
        delta_s = post_s - pre_s
        brake_work += axial_slot_force(
            accepted.load(BODY_CART), SLOT_CART_BRAKE, pose.tangent
        ) * delta_s
        drag_work += axial_slot_force(
            accepted.load(BODY_CART), SLOT_ATMOSPHERE, pose.tangent
        ) * delta_s
        resistance_work += axial_slot_force(
            accepted.load(BODY_CART), SLOT_GUIDE, pose.tangent
        ) * delta_s
        snapshot = brake.snapshot_state()
        peak_load = max(peak_load, float(snapshot.get("last_resultant_load_g", 0.0)))
        post_speed = dot(post_state.body(BODY_CART).linear_velocity, path_pose(layout, post_s).tangent)
        if post_speed < -1e-9:
            reversed_motion = True
            break
    else:
        raise RuntimeError("analytic cart did not stop within the configured run time")

    final_state = backend.read_state()
    final_s = backend.path_coordinate(BODY_CART)
    final_pose = path_pose(layout, final_s)
    final_speed = dot(final_state.body(BODY_CART).linear_velocity, final_pose.tangent)
    return CartBrakingResult(
        stop_distance_m=final_s - layout.length_m,
        elapsed_s=final_state.time_s,
        step_count=final_state.step_index,
        final_speed_mps=final_speed,
        peak_resultant_load_g=peak_load,
        brake_work_j=brake_work,
        drag_work_j=drag_work,
        resistance_work_j=resistance_work,
        reversed=reversed_motion,
        final_state=final_state,
    )
