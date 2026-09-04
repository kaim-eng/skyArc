# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Cross-field scenario feasibility checks performed before scene creation."""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..launcher.geometry import (
    CircularCenterlineSegment,
    ClothoidCenterlineSegment,
    CurvedTubeLayout,
    PlanarCenterline,
    RigidBodyEnvelope,
    StraightCenterlineSegment,
    SweptEnvelopeReport,
    TubeLayout,
    TubeStage,
    guide_normal_bound_mps2,
    normal_jerk_mps3,
    swept_envelope_clearance,
)
from ..names import ALL_BODIES, ALL_MARKERS, MARKER_BODY_OWNERSHIP
from .errors import ConfigurationError
from .schema import EXECUTION_PROFILES, ScenarioConfig


@dataclass(frozen=True)
class BrakingReport:
    release_latency_distance_m: float
    jerk_ramp_distance_m: float
    constant_force_distance_m: float
    stop_margin_m: float
    required_distance_m: float
    available_distance_m: float
    remaining_margin_m: float
    grade_assist_mps2: float
    resistance_assist_mps2: float
    stop_time_s: float


@dataclass(frozen=True)
class CenterlineReport:
    length_m: float
    exit_altitude_m: float
    exit_downrange_m: float
    exit_inclination_deg: float
    maximum_absolute_curvature_per_m: float
    peak_normal_acceleration_mps2: float
    peak_normal_jerk_mps3: float


@dataclass(frozen=True)
class PreflightReport:
    braking: BrakingReport
    physics_rate_hz: float
    render_rate_hz: float
    launch_distance_required_m: float
    launch_distance_available_m: float
    centerline: CenterlineReport | None = None
    swept_envelope: SweptEnvelopeReport | None = None
    minimum_run_time_s: float | None = None


def _finite_positive(label: str, value: float, *, allow_zero: bool = False) -> None:
    valid = math.isfinite(value) and (value >= 0.0 if allow_zero else value > 0.0)
    if not valid:
        qualifier = "nonnegative" if allow_zero else "positive"
        raise ConfigurationError(f"{label} must be finite and {qualifier}, got {value!r}")


def _finite(label: str, value: float) -> None:
    """Require a finite value whose sign carries meaning and is therefore unconstrained.

    Signed quantities cannot use :func:`_finite_positive`, so without this check they reach
    the trigonometry and comparisons below unguarded. A NaN then defeats every subsequent
    ``<`` test, because a comparison against NaN is false, and an infinity raises a bare
    ``math domain error`` that names no field and is not a :class:`ConfigurationError`.
    """
    if not math.isfinite(value):
        raise ConfigurationError(f"{label} must be finite, got {value!r}")


def resolve_centerline(config: ScenarioConfig) -> PlanarCenterline:
    """Build the single resolved geometry service consumed by schema-v3 subsystems."""
    if config.schema_version != 3 or config.tube.geometry_mode != "planar_centerline":
        raise ConfigurationError("a planar centerline requires schema_version 3 and planar_centerline mode")
    definitions = []
    for index, segment in enumerate(config.tube.centerline):
        if segment.type == "straight":
            definitions.append(StraightCenterlineSegment(segment.length_m))
        elif segment.type == "clothoid":
            if segment.start_curvature_per_m is None or segment.end_curvature_per_m is None:
                raise ConfigurationError(f"tube.centerline[{index}] clothoid curvatures are required")
            definitions.append(
                ClothoidCenterlineSegment(
                    segment.length_m,
                    segment.start_curvature_per_m,
                    segment.end_curvature_per_m,
                )
            )
        elif segment.type == "circular_arc":
            if segment.radius_m is None or segment.signed_turn_deg is None:
                raise ConfigurationError(f"tube.centerline[{index}] circular parameters are required")
            definitions.append(CircularCenterlineSegment(segment.radius_m, segment.signed_turn_deg))
        else:
            raise ConfigurationError(f"unsupported tube.centerline[{index}].type {segment.type!r}")
    try:
        return PlanarCenterline(
            origin_m=(0.0, 0.0, 0.0),
            initial_angle_deg=config.tube.angle_deg,
            segments=definitions,
        )
    except ValueError as exc:
        raise ConfigurationError(f"invalid tube.centerline: {exc}") from exc


def resolve_tube_layout(config: ScenarioConfig) -> TubeLayout | CurvedTubeLayout:
    """Resolve either schema to the geometry/stage service used by launcher consumers."""
    stages = tuple(
        TubeStage(stage.name, stage.length_m, stage.effective_density_ratio)
        for stage in config.tube.stages
    )
    if config.schema_version == 3:
        return CurvedTubeLayout(
            centerline=resolve_centerline(config),
            stages=stages,
            exterior_effective_density_ratio=config.tube.exterior_effective_density_ratio,
            boundary_blend_distance_m=config.guided_phase_aerodynamics.boundary_blend_distance_m,
        )
    return TubeLayout(
        origin_m=(0.0, 0.0, 0.0),
        angle_deg=config.tube.angle_deg,
        stages=stages,
        exterior_effective_density_ratio=config.tube.exterior_effective_density_ratio,
        boundary_blend_distance_m=config.guided_phase_aerodynamics.boundary_blend_distance_m,
    )


def braking_preflight(config: ScenarioConfig) -> BrakingReport:
    """Conservative jerk-limited cart stopping-distance calculation from section 6.7."""
    cart = config.cart
    simulation = config.simulation
    speed = config.launch_control.target_exit_speed_mps
    # Checked here as well as in validate_scenario because this function is exported and
    # called directly. A non-finite grade would otherwise flow through every comparison
    # below and produce a report whose distances are NaN but whose shortfall test passes.
    _finite("tube.exit_track_grade_deg", config.tube.exit_track_grade_deg)
    brake_acceleration = cart.brake_force_limit_n / cart.mass_kg
    grade_assist = 9.81 * math.sin(math.radians(config.tube.exit_track_grade_deg))
    resistance_assist = cart.guide_resistance_n / cart.mass_kg
    constant_assist = grade_assist + resistance_assist
    release_latency_s = (
        simulation.release_command_latency_s
        + simulation.release_confirmation_steps * simulation.physics_dt_s
    )
    if constant_assist > 0.0 and constant_assist * release_latency_s >= speed:
        coast_stop_time = speed / constant_assist
        latency_distance = speed * coast_stop_time - 0.5 * constant_assist * coast_stop_time**2
        ramp_distance = 0.0
        constant_distance = 0.0
        active_braking_time = 0.0
        total_stop_time = coast_stop_time
    else:
        latency_distance = (
            speed * release_latency_s
            - 0.5 * constant_assist * release_latency_s * release_latency_s
        )
        speed_at_brake = speed - constant_assist * release_latency_s
        ramp_time = brake_acceleration / cart.brake_jerk_limit_mps3
        speed_after_ramp = (
            speed_at_brake
            - constant_assist * ramp_time
            - 0.5 * cart.brake_jerk_limit_mps3 * ramp_time * ramp_time
        )
        if speed_after_ramp <= 0.0:
            # Solve v(t) = v0 - a0*t - 0.5*j*t^2 for the positive stopping root.
            discriminant = (
                constant_assist * constant_assist
                + 2.0 * cart.brake_jerk_limit_mps3 * speed_at_brake
            )
            stop_time = (
                -constant_assist + math.sqrt(discriminant)
            ) / cart.brake_jerk_limit_mps3
            ramp_distance = (
                speed_at_brake * stop_time
                - 0.5 * constant_assist * stop_time * stop_time
                - cart.brake_jerk_limit_mps3 * stop_time**3 / 6.0
            )
            constant_distance = 0.0
            active_braking_time = stop_time
        else:
            ramp_distance = (
                speed_at_brake * ramp_time
                - 0.5 * constant_assist * ramp_time * ramp_time
                - cart.brake_jerk_limit_mps3 * ramp_time**3 / 6.0
            )
            total_constant_deceleration = brake_acceleration + constant_assist
            if total_constant_deceleration <= 0.0:
                raise ConfigurationError(
                    "brake force, grade, and resistance do not produce positive post-ramp deceleration"
                )
            constant_distance = speed_after_ramp * speed_after_ramp / (2.0 * total_constant_deceleration)
            active_braking_time = ramp_time + speed_after_ramp / total_constant_deceleration
        total_stop_time = release_latency_s + active_braking_time

    required = latency_distance + ramp_distance + constant_distance + cart.brake_stop_margin_m
    available = config.tube.exit_brake_track_length_m
    report = BrakingReport(
        release_latency_distance_m=latency_distance,
        jerk_ramp_distance_m=ramp_distance,
        constant_force_distance_m=constant_distance,
        stop_margin_m=cart.brake_stop_margin_m,
        required_distance_m=required,
        available_distance_m=available,
        remaining_margin_m=available - required,
        grade_assist_mps2=grade_assist,
        resistance_assist_mps2=resistance_assist,
        stop_time_s=total_stop_time,
    )
    # Written as a negated non-strict comparison so that a NaN margin fails. `nan < 0.0` is
    # false, which would let an unquantifiable stopping distance pass as feasible.
    if not report.remaining_margin_m >= 0.0:
        raise ConfigurationError(
            f"cart braking requires {required:.3f} m but only {available:.3f} m is available "
            f"(shortfall {-report.remaining_margin_m:.3f} m)"
        )
    return report


def validate_scenario(config: ScenarioConfig) -> PreflightReport:
    """Validate schema values and coupled physical/numerical constraints."""
    if config.schema_version not in (2, 3):
        raise ConfigurationError(
            f"unsupported schema_version {config.schema_version!r}; expected 2 or 3"
        )
    if config.experiment.replicate_id < 0:
        raise ConfigurationError("experiment.replicate_id must be nonnegative")
    sim = config.simulation
    _finite_positive("simulation.physics_dt_s", sim.physics_dt_s)
    _finite_positive("simulation.render_dt_s", sim.render_dt_s)
    _finite_positive("simulation.reference_density_kg_m3", sim.reference_density_kg_m3)
    _finite_positive("simulation.maximum_run_time_s", sim.maximum_run_time_s)
    if sim.substeps <= 0 or sim.release_confirmation_steps <= 0:
        raise ConfigurationError("simulation substeps and release_confirmation_steps must be positive")
    _finite_positive("simulation.release_command_latency_s", sim.release_command_latency_s, allow_zero=True)
    backend_id = sim.backend.strip().lower()
    device_id = sim.device.strip().lower()
    if sim.ccd_enabled:
        # Section 12: on the target build continuous collision detection is a PhysX scene
        # property only, and requesting it while the physics device is CUDA is ignored with
        # a warning rather than refused. Every path that could reach that silent ignore has
        # to be a preflight error, or the archived configuration records a setting that was
        # never in force.
        if backend_id != "physx":
            raise ConfigurationError(
                "CCD requires an explicitly selected PhysX backend; it is a PhysX scene property "
                f"and cannot be requested on backend {sim.backend!r}"
            )
        if device_id == "auto":
            raise ConfigurationError(
                "CCD requires an explicitly pinned device; 'auto' may resolve to CUDA, where the "
                "setting is ignored with a warning rather than refused"
            )
        if "cuda" in device_id or "gpu" in device_id:
            raise ConfigurationError(
                f"CCD requires a non-CUDA device; it is ignored on device {sim.device!r}"
            )

    profile = EXECUTION_PROFILES.get(sim.profile)
    if profile is None:
        raise ConfigurationError(
            f"unknown simulation.profile {sim.profile!r}; known profiles: {sorted(EXECUTION_PROFILES)}"
        )
    if profile.is_evidence and (backend_id == "auto" or device_id == "auto"):
        raise ConfigurationError(
            f"evidence profile {sim.profile!r} must pin backend and device; 'auto' is exploration-only"
        )
    if profile.is_evidence and config.schema_version == 3:
        # Phase 0 selected one exact numerical condition for curved evidence. Merely
        # rejecting ``auto`` still allowed an explicitly pinned but rejected backend or
        # device to produce an artifact that looked admissible. Interactive exploration
        # remains free to request another target; evidence must reproduce the selection.
        if backend_id != "physx" or device_id != "cpu":
            raise ConfigurationError(
                "schema-version-3 evidence must use the Phase 0 selected target "
                "backend='physx', device='cpu'"
            )

    tube = config.tube
    _finite("tube.angle_deg", tube.angle_deg)
    _finite("tube.exit_track_grade_deg", tube.exit_track_grade_deg)
    if config.schema_version == 2 and tube.geometry_mode != "straight":
        raise ConfigurationError("schema_version 2 requires straight tube geometry")
    if config.schema_version == 3 and tube.geometry_mode != "planar_centerline":
        raise ConfigurationError("schema_version 3 requires tube.geometry_mode 'planar_centerline'")
    if config.schema_version == 3 and config.models.guide != "tangent_following_v1":
        raise ConfigurationError(
            "schema_version 3 requires models.guide 'tangent_following_v1'"
        )
    if not tube.stages:
        raise ConfigurationError("tube.stages must contain at least one stage")
    _finite_positive("tube.inner_diameter_m", tube.inner_diameter_m)
    _finite_positive("tube.exit_brake_track_length_m", tube.exit_brake_track_length_m)
    _finite_positive("tube.guide_clearance_m", tube.guide_clearance_m, allow_zero=True)
    _finite_positive(
        "tube.exterior_effective_density_ratio",
        tube.exterior_effective_density_ratio,
        allow_zero=True,
    )
    stage_names = set()
    for index, stage in enumerate(tube.stages):
        if not stage.name or stage.name in stage_names:
            raise ConfigurationError(f"tube stage {index} has an empty or duplicate name {stage.name!r}")
        stage_names.add(stage.name)
        _finite_positive(f"tube.stages[{index}].length_m", stage.length_m)
        _finite_positive(
            f"tube.stages[{index}].effective_density_ratio",
            stage.effective_density_ratio,
            allow_zero=True,
        )
        if any(not math.isfinite(channel) or channel < 0.0 or channel > 1.0 for channel in stage.color_rgb):
            raise ConfigurationError(f"tube.stages[{index}].color_rgb channels must be in [0, 1]")
        if not math.isfinite(stage.opacity) or not 0.0 <= stage.opacity <= 1.0:
            raise ConfigurationError(f"tube.stages[{index}].opacity must be in [0, 1]")
    pair_names = set()
    for index, pair in enumerate(tube.anti_tunneling_pairs):
        if not pair.name or pair.name in pair_names:
            raise ConfigurationError(
                f"tube.anti_tunneling_pairs[{index}] has an empty or duplicate name {pair.name!r}"
            )
        pair_names.add(pair.name)
        _finite_positive(
            f"tube.anti_tunneling_pairs[{index}].test_relative_speed_mps",
            pair.test_relative_speed_mps,
        )

    centerline = None
    centerline_report = None
    if config.schema_version == 3:
        if not tube.centerline:
            raise ConfigurationError("schema_version 3 requires tube.centerline")
        if tube.centerline[0].type != "straight" or tube.centerline[0].initial_angle_deg is None:
            raise ConfigurationError("tube.centerline must begin with a straight segment and initial angle")
        if any(
            segment.initial_angle_deg is not None
            for segment in tube.centerline[1:]
        ):
            raise ConfigurationError("only tube.centerline[0] may declare initial_angle_deg")
        centerline = resolve_centerline(config)
        if not math.isclose(
            sum(stage.length_m for stage in tube.stages),
            centerline.length_m,
            rel_tol=1e-9,
            abs_tol=1e-3,
        ):
            raise ConfigurationError(
                "tube stage arc lengths must sum to the resolved centerline length"
            )
        exit_pose = centerline.exit_pose
        if abs(exit_pose.signed_curvature_per_m) > 1e-12:
            raise ConfigurationError("tube centerline curvature must be zero at release")
        if tube.exit_track is None:
            raise ConfigurationError("schema_version 3 requires tube.exit_track")
        if tube.exit_track.type != "tangent_straight":
            raise ConfigurationError("schema_version 3 exit track must use type 'tangent_straight'")
        if not math.isclose(tube.exit_track.curvature_per_m, 0.0, rel_tol=0.0, abs_tol=1e-12):
            raise ConfigurationError("schema_version 3 exit track must have zero curvature")
        if not math.isclose(
            tube.exit_track.inclination_deg,
            exit_pose.inclination_deg,
            rel_tol=0.0,
            abs_tol=1e-5,
        ):
            raise ConfigurationError(
                "tube.exit_track inclination must equal the resolved centerline exit tangent"
            )
        if centerline.minimum_downrange_tangent_component <= 1e-12:
            raise ConfigurationError("tube centerline reverses downrange direction")

    cart = config.cart
    rocket = config.rocket
    for label, value in (
        ("cart.mass_kg", cart.mass_kg),
        ("cart.length_m", cart.length_m),
        ("cart.width_m", cart.width_m),
        ("cart.height_m", cart.height_m),
        ("cart.frontal_area_m2", cart.frontal_area_m2),
        ("cart.brake_force_limit_n", cart.brake_force_limit_n),
        ("cart.brake_jerk_limit_mps3", cart.brake_jerk_limit_mps3),
        ("cart.stopped_speed_threshold_mps", cart.stopped_speed_threshold_mps),
        ("rocket.initial_mass_kg", rocket.initial_mass_kg),
        ("rocket.length_m", rocket.length_m),
        ("rocket.diameter_m", rocket.diameter_m),
        ("rocket.reference_area_m2", rocket.reference_area_m2),
    ):
        _finite_positive(label, value)
    for label, value in (
        ("cart.drag_coefficient", cart.drag_coefficient),
        ("cart.brake_stop_margin_m", cart.brake_stop_margin_m),
        ("cart.guide_resistance_n", cart.guide_resistance_n),
        ("rocket.drag_coefficient", rocket.drag_coefficient),
    ):
        _finite_positive(label, value, allow_zero=True)
    if config.schema_version == 3:
        if cart.maximum_resultant_load_g is None:
            raise ConfigurationError("schema_version 3 requires cart.maximum_resultant_load_g")
        _finite_positive("cart.maximum_resultant_load_g", cart.maximum_resultant_load_g)
        guide_normal_g = abs(math.cos(math.radians(tube.exit_track_grade_deg)))
        tangential_ceiling_g = (
            cart.brake_force_limit_n + cart.guide_resistance_n
        ) / (cart.mass_kg * 9.81)
        configured_resultant_g = math.hypot(tangential_ceiling_g, guide_normal_g)
        if configured_resultant_g > cart.maximum_resultant_load_g * (1.0 + 1e-9):
            raise ConfigurationError(
                "cart brake and guide-normal reaction can exceed cart.maximum_resultant_load_g"
            )

    tube_radius = 0.5 * tube.inner_diameter_m
    cart_envelope_radius = math.hypot(0.5 * cart.width_m, 0.5 * cart.height_m)
    if cart_envelope_radius + tube.guide_clearance_m > tube_radius:
        raise ConfigurationError("cart cross-section plus guide clearance does not fit inside the tube")
    if 0.5 * rocket.diameter_m + tube.guide_clearance_m > tube_radius:
        raise ConfigurationError("rocket diameter plus guide clearance does not fit inside the tube")
    clearance_centerline = centerline
    if clearance_centerline is None:
        clearance_centerline = PlanarCenterline(
            origin_m=(0.0, 0.0, 0.0),
            initial_angle_deg=tube.angle_deg,
            segments=(StraightCenterlineSegment(tube.length_m),),
        )
    try:
        swept_envelope = swept_envelope_clearance(
            clearance_centerline,
            tube_inner_diameter_m=tube.inner_diameter_m,
            guide_clearance_m=tube.guide_clearance_m,
            bodies=(
                RigidBodyEnvelope(
                    "cart",
                    half_length_m=0.5 * cart.length_m,
                    cross_section_radius_m=cart_envelope_radius,
                ),
                RigidBodyEnvelope(
                    "rocket",
                    half_length_m=0.5 * rocket.length_m,
                    cross_section_radius_m=0.5 * rocket.diameter_m,
                ),
            ),
        )
    except ValueError as exc:
        raise ConfigurationError(f"invalid swept-envelope clearance: {exc}") from exc

    guided = config.guided_phase_aerodynamics
    # Signed: the baseline is stationary air, but a configured headwind or tailwind is valid.
    _finite("guided_phase_aerodynamics.axial_air_velocity_mps", guided.axial_air_velocity_mps)
    for label, value in (
        ("guided_phase_aerodynamics.drag_coefficient", guided.drag_coefficient),
        ("guided_phase_aerodynamics.reference_area_m2", guided.reference_area_m2),
        ("guided_phase_aerodynamics.boundary_blend_distance_m", guided.boundary_blend_distance_m),
    ):
        _finite_positive(label, value, allow_zero=label.endswith("distance_m") or label.endswith("coefficient"))
    if guided.boundary_blend_distance_m > min(stage.length_m for stage in tube.stages):
        raise ConfigurationError("boundary blend distance exceeds the shortest stage length")

    if config.stage2_constraint is not None:
        # Delegated rather than restated: the screen owns what is admissible, so a config
        # cannot be accepted here and then rejected when it is scored.
        from ..launcher.feasibility import Stage2Constraint

        constraint = config.stage2_constraint
        try:
            Stage2Constraint(
                model=constraint.model,
                specific_impulse_s=constraint.specific_impulse_s,
                propellant_mass_fraction=constraint.propellant_mass_fraction,
                target_orbit_altitude_m=constraint.target_orbit_altitude_m,
                assumed_unmodeled_loss_mps=constraint.assumed_unmodeled_loss_mps,
            )
        except ValueError as exc:
            raise ConfigurationError(f"invalid stage2_constraint: {exc}") from exc

    if config.schema_version == 3:
        atmosphere = tube.exterior_atmosphere
        if atmosphere is None:
            raise ConfigurationError("schema_version 3 requires tube.exterior_atmosphere")
        if atmosphere.model not in {"constant_v1", "exponential_v1"}:
            raise ConfigurationError(
                "tube.exterior_atmosphere.model must be 'constant_v1' or 'exponential_v1'"
            )
        _finite_positive(
            "tube.exterior_atmosphere.reference_ratio", atmosphere.reference_ratio, allow_zero=True
        )
        _finite("tube.exterior_atmosphere.reference_altitude_m", atmosphere.reference_altitude_m)
        if atmosphere.model == "exponential_v1":
            if atmosphere.scale_height_m is None:
                raise ConfigurationError("exponential_v1 requires scale_height_m")
            _finite_positive("tube.exterior_atmosphere.scale_height_m", atmosphere.scale_height_m)
        elif atmosphere.scale_height_m is not None:
            raise ConfigurationError("constant_v1 may not declare scale_height_m")
        assert centerline is not None
        try:
            exit_ratio = atmosphere.density_ratio(centerline.exit_pose.position_m[2])
        except (OverflowError, ValueError) as exc:
            raise ConfigurationError(f"invalid exterior atmosphere evaluation: {exc}") from exc
        if not math.isfinite(exit_ratio):
            raise ConfigurationError("exterior atmosphere evaluation must be finite")
        if not math.isclose(
            exit_ratio,
            tube.stages[-1].effective_density_ratio,
            rel_tol=1e-3,
            abs_tol=1e-9,
        ):
            raise ConfigurationError(
                "exterior atmosphere density at the exit must match the final tube stage"
            )

    # Zero thrust is admissible and means "no upper stage is simulated": the ignition event
    # still fires and still marks the handoff, but the vehicle coasts. Feasibility sweeps
    # want this, because the parametric screen in `launcher.feasibility` is then the sole
    # authority on stage-2 delta-v and cannot double-count a demonstrator burn. The burn
    # duration stays strictly positive, so a zero-thrust motor is an explicit choice rather
    # than the accident of an unset duration.
    _finite_positive("rocket.motor.thrust_n", rocket.motor.thrust_n, allow_zero=True)
    _finite_positive("rocket.motor.burn_duration_s", rocket.motor.burn_duration_s)

    launch = config.launch_control
    allowed_modes = {"constant_force", "constant_acceleration", "target_exit_speed", "force_vs_position"}
    if launch.mode not in allowed_modes:
        raise ConfigurationError(
            f"launch_control.mode must be one of {sorted(allowed_modes)}, got {launch.mode!r}"
        )
    if launch.mode == "force_vs_position" and len(launch.force_vs_position) < 2:
        raise ConfigurationError(
            "launch_control.force_vs_position mode requires at least two table points"
        )
    previous_position = -math.inf
    for index, point in enumerate(launch.force_vs_position):
        _finite_positive(
            f"launch_control.force_vs_position[{index}].position_m",
            point.position_m,
            allow_zero=True,
        )
        _finite_positive(
            f"launch_control.force_vs_position[{index}].force_n",
            point.force_n,
            allow_zero=True,
        )
        if point.position_m <= previous_position:
            raise ConfigurationError(
                "launch_control.force_vs_position positions must be strictly increasing"
            )
        previous_position = point.position_m
    if launch.force_vs_position and launch.force_vs_position[-1].position_m > tube.length_m:
        raise ConfigurationError("launch_control.force_vs_position extends beyond the tube")
    for label, value in (
        ("launch_control.target_exit_speed_mps", launch.target_exit_speed_mps),
        ("launch_control.maximum_force_n", launch.maximum_force_n),
        ("launch_control.maximum_acceleration_mps2", launch.maximum_acceleration_mps2),
    ):
        _finite_positive(label, value)
    _finite_positive("launch_control.force_ramp_up_distance_m", launch.force_ramp_up_distance_m, allow_zero=True)
    _finite_positive(
        "launch_control.force_ramp_down_distance_m", launch.force_ramp_down_distance_m, allow_zero=True
    )
    if config.schema_version == 3:
        if launch.maximum_resultant_load_g is None:
            raise ConfigurationError(
                "schema_version 3 requires launch_control.maximum_resultant_load_g"
            )
        if launch.maximum_normal_jerk_mps3 is None:
            raise ConfigurationError(
                "schema_version 3 requires launch_control.maximum_normal_jerk_mps3"
            )
        _finite_positive(
            "launch_control.maximum_resultant_load_g", launch.maximum_resultant_load_g
        )
        _finite_positive(
            "launch_control.maximum_normal_jerk_mps3", launch.maximum_normal_jerk_mps3
        )
    available_launch_distance = (
        tube.length_m - launch.force_ramp_up_distance_m - launch.force_ramp_down_distance_m
    )
    if available_launch_distance <= 0.0:
        raise ConfigurationError("force ramp-down start must lie inside the tube")
    combined_mass = cart.mass_kg + rocket.initial_mass_kg
    target_relative_speed = launch.target_exit_speed_mps - guided.axial_air_velocity_mps
    # q(v) = (v-u)|v-u| is monotonic in vehicle speed, so its largest value over the
    # launch interval occurs at the target. For an opposing flow (q >= 0), the densest
    # stage is limiting. For a tailwind (q < 0), drag assists launch and the *least* dense
    # stage is limiting. Always taking the maximum ratio would credit a vacuum stage with
    # assistance that cannot exist and can let a force ceiling below the grade load pass.
    stage_density_ratios = tuple(stage.effective_density_ratio for stage in tube.stages)
    limiting_density_ratio = (
        max(stage_density_ratios) if target_relative_speed >= 0.0 else min(stage_density_ratios)
    )
    peak_drag = (
        0.5
        * sim.reference_density_kg_m3
        * limiting_density_ratio
        * guided.drag_coefficient
        * guided.reference_area_m2
        * target_relative_speed
        * abs(target_relative_speed)
    )
    required_peak_force = (
        combined_mass * launch.maximum_acceleration_mps2
        + combined_mass * 9.81 * math.sin(math.radians(tube.angle_deg))
        + peak_drag
        + cart.guide_resistance_n
    )
    mode_force_ceiling = launch.maximum_force_n
    if launch.mode == "force_vs_position":
        mode_force_ceiling = min(
            mode_force_ceiling,
            max(point.force_n for point in launch.force_vs_position),
        )
    achievable_acceleration = min(
        launch.maximum_acceleration_mps2,
        max(
            0.0,
            (
                mode_force_ceiling
                - combined_mass * 9.81 * math.sin(math.radians(tube.angle_deg))
                - peak_drag
                - cart.guide_resistance_n
            )
            / combined_mass,
        ),
    )
    if achievable_acceleration <= 0.0:
        raise ConfigurationError(
            "selected launch-force mode cannot overcome grade, peak drag, and resistance"
        )
    launch_distance_required = launch.target_exit_speed_mps**2 / (2.0 * achievable_acceleration)
    if launch_distance_required > available_launch_distance:
        raise ConfigurationError(
            f"target speed requires at least {launch_distance_required:.3f} m under the configured ceilings, "
            f"but only {available_launch_distance:.3f} m is available; peak force for the acceleration "
            f"ceiling would be {required_peak_force:.1f} N"
        )

    if config.schema_version == 3:
        assert centerline is not None
        sample_spacing = max(1.0, centerline.length_m / 4096.0)
        path_samples = centerline.sample(sample_spacing)
        maximum_curvature = max(abs(pose.signed_curvature_per_m) for pose in path_samples)
        peak_normal_acceleration = max(
            guide_normal_bound_mps2(
                launch.target_exit_speed_mps,
                pose.signed_curvature_per_m,
                (0.0, 0.0, -9.81),
                pose.normal,
            )
            for pose in path_samples
        )
        tangential_load_ceiling = launch.maximum_force_n / combined_mass
        configured_resultant_g = math.hypot(
            tangential_load_ceiling, peak_normal_acceleration
        ) / 9.81
        assert launch.maximum_resultant_load_g is not None
        if configured_resultant_g > launch.maximum_resultant_load_g * (1.0 + 1e-9):
            raise ConfigurationError(
                "launch force ceiling and peak curvature can exceed maximum_resultant_load_g"
            )
        constant_net_acceleration = (
            launch.target_exit_speed_mps**2 / (2.0 * centerline.length_m)
        )
        peak_normal_jerk = 0.0
        for pose in path_samples:
            speed = launch.target_exit_speed_mps * math.sqrt(pose.s_m / centerline.length_m)
            peak_normal_jerk = max(
                peak_normal_jerk,
                abs(
                    normal_jerk_mps3(
                        speed,
                        constant_net_acceleration,
                        pose.signed_curvature_per_m,
                        pose.curvature_rate_per_m2,
                    )
                ),
            )
        assert launch.maximum_normal_jerk_mps3 is not None
        if peak_normal_jerk > launch.maximum_normal_jerk_mps3 * (1.0 + 1e-9):
            raise ConfigurationError(
                f"resolved normal jerk {peak_normal_jerk:.3f} m/s^3 exceeds "
                f"launch_control.maximum_normal_jerk_mps3 {launch.maximum_normal_jerk_mps3:.3f}"
            )
        minimum_blend_distance = (
            10.0 * launch.target_exit_speed_mps * sim.physics_dt_s
        )
        if guided.boundary_blend_distance_m < minimum_blend_distance:
            raise ConfigurationError(
                f"boundary blend distance must be at least {minimum_blend_distance:.3f} m "
                "to resolve ten physics steps at target speed"
            )
        pair_speeds = {pair.name: pair.test_relative_speed_mps for pair in tube.anti_tunneling_pairs}
        required_pair_speed = 1.25 * launch.target_exit_speed_mps
        if pair_speeds.get("rocket_cradle", 0.0) < required_pair_speed:
            raise ConfigurationError(
                "rocket_cradle anti-tunneling test speed must be at least 1.25 times release speed"
            )
        exit_pose = centerline.exit_pose
        centerline_report = CenterlineReport(
            length_m=centerline.length_m,
            exit_altitude_m=exit_pose.position_m[2],
            exit_downrange_m=exit_pose.position_m[0],
            exit_inclination_deg=exit_pose.inclination_deg,
            maximum_absolute_curvature_per_m=maximum_curvature,
            peak_normal_acceleration_mps2=peak_normal_acceleration,
            peak_normal_jerk_mps3=peak_normal_jerk,
        )

    ignition = rocket.ignition
    for label, value in (
        ("rocket.ignition.delay_s", ignition.delay_s),
        ("rocket.ignition.minimum_cart_clearance_m", ignition.minimum_cart_clearance_m),
        ("rocket.ignition.minimum_relative_speed_mps", ignition.minimum_relative_speed_mps),
        ("rocket.ignition.no_recontact_dwell_s", ignition.no_recontact_dwell_s),
        ("rocket.ignition.separation_timeout_s", ignition.separation_timeout_s),
        ("rocket.ignition.maximum_contact_impulse_ns", ignition.maximum_contact_impulse_ns),
        ("rocket.ignition.maximum_angular_rate_deg_s", ignition.maximum_angular_rate_deg_s),
    ):
        _finite_positive(label, value, allow_zero=label.endswith("delay_s"))
    if ignition.minimum_cart_clearance_m >= tube.exit_brake_track_length_m:
        raise ConfigurationError("minimum ignition clearance is incompatible with the exit track geometry")
    trigger = ignition.trigger
    trigger_fields = (
        ("minimum_altitude_m", trigger.minimum_altitude_m),
        ("maximum_flight_path_angle_deg", trigger.maximum_flight_path_angle_deg),
        ("maximum_vertical_speed_mps", trigger.maximum_vertical_speed_mps),
    )
    if trigger.model == "safety_gates_only_v1":
        if any(value is not None for _, value in trigger_fields):
            raise ConfigurationError(
                "safety_gates_only_v1 ignition trigger may not declare trajectory thresholds"
            )
    elif trigger.model == "trajectory_thresholds_v1":
        if all(value is None for _, value in trigger_fields):
            raise ConfigurationError(
                "trajectory_thresholds_v1 ignition trigger requires at least one threshold"
            )
        for field, value in trigger_fields:
            if value is not None and not math.isfinite(value):
                raise ConfigurationError(f"rocket.ignition.trigger.{field} must be finite")
        if trigger.minimum_altitude_m is not None and trigger.minimum_altitude_m < 0.0:
            raise ConfigurationError("minimum ignition altitude must be nonnegative")
        if (
            trigger.maximum_flight_path_angle_deg is not None
            and not -90.0 <= trigger.maximum_flight_path_angle_deg <= 90.0
        ):
            raise ConfigurationError("maximum ignition flight-path angle must be within [-90, 90] degrees")
    else:
        raise ConfigurationError(f"unsupported ignition trigger model {trigger.model!r}")

    if rocket.aft_clearance_marker not in config.markers:
        raise ConfigurationError(f"rocket aft marker {rocket.aft_clearance_marker!r} is not configured")
    missing_markers = sorted(set(ALL_MARKERS) - set(config.markers))
    if missing_markers:
        raise ConfigurationError(f"required named markers are missing: {missing_markers}")
    for name, marker in config.markers.items():
        if marker.body not in ALL_BODIES:
            raise ConfigurationError(f"marker {name!r} targets unknown body {marker.body!r}")
        if len(marker.offset_m) != 3 or any(not math.isfinite(value) for value in marker.offset_m):
            raise ConfigurationError(f"marker {name!r} must have a finite three-vector offset")
        expected_body = MARKER_BODY_OWNERSHIP.get(name)
        if expected_body is not None and marker.body != expected_body:
            raise ConfigurationError(
                f"marker {name!r} must be attached to body {expected_body!r}, got {marker.body!r}"
            )
    if config.markers[rocket.aft_clearance_marker].body != "rocket":
        raise ConfigurationError("rocket aft-clearance marker must be attached to the rocket body")

    evidence = config.evidence
    if config.schema_version == 3:
        if evidence is None:
            raise ConfigurationError("schema_version 3 requires evidence configuration")
        if evidence.free_flight_start_event != "separation_confirmed":
            raise ConfigurationError(
                "evidence.free_flight_start_event must be 'separation_confirmed'"
            )
        for label, value in (
            ("evidence.free_flight_duration_s", evidence.free_flight_duration_s),
            ("evidence.completion_margin_s", evidence.completion_margin_s),
            (
                "evidence.maximum_exit_speed_relative_change",
                evidence.maximum_exit_speed_relative_change,
            ),
            ("evidence.maximum_peak_load_change_g", evidence.maximum_peak_load_change_g),
            (
                "evidence.maximum_drag_work_relative_change",
                evidence.maximum_drag_work_relative_change,
            ),
        ):
            _finite_positive(
                label,
                value,
                allow_zero=label.endswith("duration_s") or label.endswith("margin_s"),
            )
        if evidence.atmosphere_stage_refinement_factor < 2:
            raise ConfigurationError(
                "evidence.atmosphere_stage_refinement_factor must be at least 2"
            )
        assert tube.exterior_atmosphere is not None
        if (
            evidence.free_flight_duration_s > 0.0
            and tube.exterior_atmosphere.model == "constant_v1"
        ):
            raise ConfigurationError(
                "positive schema-v3 free-flight evidence requires an altitude-dependent exterior atmosphere"
            )

    physics_rate = 1.0 / sim.physics_dt_s
    render_rate = 1.0 / sim.render_dt_s
    _finite_positive("output.telemetry_rate_hz", config.output.telemetry_rate_hz)
    if config.output.telemetry_rate_hz > physics_rate * (1.0 + 1e-9):
        raise ConfigurationError("telemetry rate may not exceed the physics rate")
    if config.output.diagnostics_format != "jsonl":
        raise ConfigurationError("baseline diagnostics format must be 'jsonl'")
    braking = braking_preflight(config)
    minimum_run_time = None
    if config.schema_version == 3:
        assert evidence is not None
        guided_time_upper = 2.0 * tube.length_m / launch.target_exit_speed_mps
        minimum_run_time = (
            guided_time_upper
            + ignition.separation_timeout_s
            + max(braking.stop_time_s, evidence.free_flight_duration_s)
            + evidence.completion_margin_s
        )
        if sim.maximum_run_time_s < minimum_run_time:
            raise ConfigurationError(
                f"simulation.maximum_run_time_s must be at least {minimum_run_time:.3f} s "
                "for the resolved evidence horizon and concurrent cart stop"
            )
    return PreflightReport(
        braking=braking,
        physics_rate_hz=physics_rate,
        render_rate_hz=render_rate,
        launch_distance_required_m=launch_distance_required,
        launch_distance_available_m=available_launch_distance,
        centerline=centerline_report,
        swept_envelope=swept_envelope,
        minimum_run_time_s=minimum_run_time,
    )
