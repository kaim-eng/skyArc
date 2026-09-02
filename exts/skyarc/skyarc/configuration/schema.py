# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Immutable schema-version-2/3 scenario values in declared SI units."""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping, Tuple

from ..linalg import Vec3


@dataclass(frozen=True)
class ExecutionProfile:
    """One declared way of launching the application (section 12).

    Whether a run is evidence-grade is a property of the profile, not of what someone
    named it. Section 12 makes time advancement a property of the launched application
    rather than only of the scenario -- fixed time stepping is disabled in the base
    experience used by the standalone runner and enabled in the full interactive
    experience -- so the same scenario can integrate differently under two profiles.
    Both facts therefore live here, where preflight can read them.
    """

    name: str
    is_evidence: bool
    fixed_time_stepping: bool
    description: str

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("execution profile name may not be empty")
        if self.is_evidence and not self.fixed_time_stepping:
            raise ValueError(
                f"execution profile {self.name!r} is evidence-grade but does not use fixed time "
                "stepping; section 12 requires every evidence profile to fix it"
            )


EXECUTION_PROFILES: Mapping[str, ExecutionProfile] = MappingProxyType(
    {
        profile.name: profile
        for profile in (
            ExecutionProfile(
                name="interactive_rendered",
                is_evidence=False,
                fixed_time_stepping=True,
                description="Full interactive experience with the UI panel and viewport.",
            ),
            ExecutionProfile(
                name="kit_physics_only",
                is_evidence=False,
                fixed_time_stepping=False,
                description="Kit integration tests without rendering; base-experience stepping.",
            ),
            ExecutionProfile(
                name="headless_evidence_physics_only",
                is_evidence=True,
                fixed_time_stepping=True,
                description="Headless evidence run without rendering, for physics verification.",
            ),
            ExecutionProfile(
                name="headless_evidence_rendered",
                is_evidence=True,
                fixed_time_stepping=True,
                description="Headless evidence run with capture, for rendered acceptance evidence.",
            ),
        )
    }
)
"""The closed set of execution profiles a scenario may select.

A substring match on the profile name cannot carry this guarantee: a profile called
``headless_rendered`` used for an evidence run would accept an unresolved ``auto`` backend
and produce an archived configuration that does not reproduce the run.
"""


@dataclass(frozen=True)
class ExperimentConfig:
    experiment_id: str
    condition_id: str
    parent_condition_id: str | None
    replicate_id: int
    seed: int


@dataclass(frozen=True)
class SimulationConfig:
    backend: str
    device: str
    physics_dt_s: float
    render_dt_s: float
    reference_density_kg_m3: float
    maximum_run_time_s: float
    profile: str
    substeps: int = 1
    ccd_enabled: bool = False
    release_command_latency_s: float = 0.0
    release_confirmation_steps: int = 1


@dataclass(frozen=True)
class ModelsConfig:
    launch_force: str
    atmosphere: str
    guide: str
    coupling: str
    separation_actuator: str
    cart_brake: str
    rocket_motor: str
    rocket_aerodynamics: str
    observer: str


@dataclass(frozen=True)
class StageConfig:
    name: str
    length_m: float
    effective_density_ratio: float
    color_rgb: Vec3
    opacity: float = 1.0


@dataclass(frozen=True)
class AntiTunnelingPairConfig:
    name: str
    test_relative_speed_mps: float


@dataclass(frozen=True)
class CenterlineSegmentConfig:
    """One schema-v3 planar centerline segment in resolved arc-length form."""

    type: str
    length_m: float
    initial_angle_deg: float | None = None
    start_curvature_per_m: float | None = None
    end_curvature_per_m: float | None = None
    radius_m: float | None = None
    signed_turn_deg: float | None = None


@dataclass(frozen=True)
class ExitTrackConfig:
    type: str
    length_m: float
    inclination_deg: float
    curvature_per_m: float


@dataclass(frozen=True)
class ExteriorAtmosphereConfig:
    model: str
    reference_ratio: float
    reference_altitude_m: float
    scale_height_m: float | None = None

    def density_ratio(self, altitude_m: float) -> float:
        if self.model == "constant_v1":
            return self.reference_ratio
        if self.model == "exponential_v1":
            if self.scale_height_m is None:
                raise ValueError("exponential_v1 requires scale_height_m")
            return self.reference_ratio * math.exp(
                -(altitude_m - self.reference_altitude_m) / self.scale_height_m
            )
        raise ValueError(f"unsupported exterior atmosphere model {self.model!r}")


@dataclass(frozen=True)
class TubeConfig:
    angle_deg: float
    inner_diameter_m: float
    exit_brake_track_length_m: float
    exit_track_grade_deg: float
    exterior_effective_density_ratio: float
    guide_clearance_m: float
    stages: Tuple[StageConfig, ...]
    anti_tunneling_pairs: Tuple[AntiTunnelingPairConfig, ...] = ()
    geometry_mode: str = "straight"
    centerline: Tuple[CenterlineSegmentConfig, ...] = ()
    exit_track: ExitTrackConfig | None = None
    exterior_atmosphere: ExteriorAtmosphereConfig | None = None

    @property
    def length_m(self) -> float:
        if self.geometry_mode == "planar_centerline":
            return sum(segment.length_m for segment in self.centerline)
        return sum(stage.length_m for stage in self.stages)


@dataclass(frozen=True)
class CartConfig:
    mass_kg: float
    length_m: float
    width_m: float
    height_m: float
    drag_coefficient: float
    frontal_area_m2: float
    brake_force_limit_n: float
    brake_jerk_limit_mps3: float
    brake_stop_margin_m: float
    stopped_speed_threshold_mps: float
    guide_resistance_n: float
    inertia_mode: str
    maximum_resultant_load_g: float | None = None


@dataclass(frozen=True)
class GuidedAerodynamicsConfig:
    drag_coefficient: float
    reference_area_m2: float
    axial_air_velocity_mps: float
    boundary_blend_distance_m: float
    force_model: str


@dataclass(frozen=True)
class MotorConfig:
    model: str
    thrust_n: float
    burn_duration_s: float


@dataclass(frozen=True)
class IgnitionConfig:
    delay_s: float
    minimum_cart_clearance_m: float
    minimum_relative_speed_mps: float
    no_recontact_dwell_s: float
    separation_timeout_s: float
    maximum_contact_impulse_ns: float
    maximum_angular_rate_deg_s: float


@dataclass(frozen=True)
class RocketConfig:
    initial_mass_kg: float
    length_m: float
    diameter_m: float
    drag_coefficient: float
    reference_area_m2: float
    inertia_mode: str
    aft_clearance_marker: str
    motor: MotorConfig
    ignition: IgnitionConfig


@dataclass(frozen=True)
class ForcePositionPointConfig:
    position_m: float
    force_n: float


@dataclass(frozen=True)
class LaunchControlConfig:
    mode: str
    target_exit_speed_mps: float
    maximum_force_n: float
    maximum_acceleration_mps2: float
    force_ramp_up_distance_m: float
    force_ramp_down_distance_m: float
    maximum_resultant_load_g: float | None = None
    maximum_normal_jerk_mps3: float | None = None
    force_vs_position: Tuple[ForcePositionPointConfig, ...] = ()


@dataclass(frozen=True)
class EvidenceConfig:
    free_flight_start_event: str
    free_flight_duration_s: float
    completion_margin_s: float
    atmosphere_stage_refinement_factor: int
    maximum_exit_speed_relative_change: float
    maximum_peak_load_change_g: float
    maximum_drag_work_relative_change: float


@dataclass(frozen=True)
class MarkerConfig:
    body: str
    offset_m: Vec3


@dataclass(frozen=True)
class OutputConfig:
    directory: str
    telemetry_rate_hz: float
    diagnostics_format: str
    validity_policy: str
    criterion_policy: str
    capture_profile: str


@dataclass(frozen=True)
class Stage2ConstraintConfig:
    """Upper stage reduced to the parameters that determine its delta-v.

    Deliberately not a simulated body.  The project's question is whether the launcher can
    replace a first stage, and for that question a full upper-stage model collapses to one
    number.  ``model`` is the seam: a later ``full_stage_v1`` becomes a variable-mass
    component in the effect path and the same screen reads a measured insertion state.
    Validation of the values themselves lives in :mod:`..launcher.feasibility`, so the
    config layer and the screen cannot disagree about what is admissible.
    """

    model: str = "parametric_deltav_v1"
    specific_impulse_s: float = 350.0
    propellant_mass_fraction: float = 0.85
    target_orbit_altitude_m: float = 200000.0
    loss_allowance_mps: float = 500.0


@dataclass(frozen=True)
class ScenarioConfig:
    schema_version: int
    experiment: ExperimentConfig
    simulation: SimulationConfig
    models: ModelsConfig
    tube: TubeConfig
    cart: CartConfig
    guided_phase_aerodynamics: GuidedAerodynamicsConfig
    rocket: RocketConfig
    launch_control: LaunchControlConfig
    markers: Mapping[str, MarkerConfig]
    output: OutputConfig
    evidence: EvidenceConfig | None = None
    stage2_constraint: Stage2ConstraintConfig | None = None
