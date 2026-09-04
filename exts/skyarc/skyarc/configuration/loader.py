# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Strict YAML-to-schema loading with source and resolved-configuration hashes."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, fields, is_dataclass
from enum import Enum
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Sequence

import yaml

from .errors import ConfigurationError
from .schema import (
    AntiTunnelingPairConfig,
    CartConfig,
    CenterlineSegmentConfig,
    EvidenceConfig,
    ExperimentConfig,
    ExitTrackConfig,
    ExteriorAtmosphereConfig,
    ForcePositionPointConfig,
    GuidedAerodynamicsConfig,
    IgnitionConfig,
    IgnitionTriggerConfig,
    LaunchControlConfig,
    MarkerConfig,
    ModelsConfig,
    MotorConfig,
    OutputConfig,
    RocketConfig,
    ScenarioConfig,
    SimulationConfig,
    Stage2ConstraintConfig,
    StageConfig,
    TubeConfig,
)
from .validation import PreflightReport, validate_scenario


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_unique_mapping(loader: _UniqueKeyLoader, node: yaml.MappingNode, deep: bool = False) -> dict:
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ConfigurationError(f"duplicate YAML key {key!r}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


@dataclass(frozen=True)
class LoadedScenario:
    config: ScenarioConfig
    preflight: PreflightReport
    source_sha256: str
    resolved_sha256: str
    source_path: str | None = None


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise ConfigurationError(f"{path} must be a string-keyed mapping")
    return value


def _sequence(value: Any, path: str) -> Sequence[Any]:
    if not isinstance(value, (list, tuple)):
        raise ConfigurationError(f"{path} must be a sequence")
    return value


def _reject_unknown(data: Mapping[str, Any], allowed: set[str], path: str) -> None:
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise ConfigurationError(f"unknown keys in {path}: {unknown}")


def _required(data: Mapping[str, Any], key: str, path: str) -> Any:
    if key not in data:
        raise ConfigurationError(f"missing required key {path}.{key}")
    return data[key]


def _string(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value:
        raise ConfigurationError(f"{path} must be a non-empty string")
    return value


def _nullable_string(value: Any, path: str) -> str | None:
    return None if value is None else _string(value, path)


def _number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigurationError(f"{path} must be a number")
    number = float(value)
    if not math.isfinite(number):
        # Every numeric leaf reaches the schema through here, including vector components,
        # so rejecting non-finite values at this one point closes the whole class. YAML
        # spells them `.nan` and `.inf`; section 14 forbids undocumented NaN downstream.
        raise ConfigurationError(f"{path} must be finite, got {value!r}")
    return number


def _integer(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigurationError(f"{path} must be an integer")
    return value


def _boolean(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigurationError(f"{path} must be a boolean")
    return value


def _vec3(value: Any, path: str) -> tuple[float, float, float]:
    values = _sequence(value, path)
    if len(values) != 3:
        raise ConfigurationError(f"{path} must contain exactly three numbers")
    return (_number(values[0], path), _number(values[1], path), _number(values[2], path))


def _parse_experiment(value: Any) -> ExperimentConfig:
    data = _mapping(value, "experiment")
    allowed = {"experiment_id", "condition_id", "parent_condition_id", "replicate_id", "seed"}
    _reject_unknown(data, allowed, "experiment")
    return ExperimentConfig(
        experiment_id=_string(_required(data, "experiment_id", "experiment"), "experiment.experiment_id"),
        condition_id=_string(_required(data, "condition_id", "experiment"), "experiment.condition_id"),
        parent_condition_id=_nullable_string(data.get("parent_condition_id"), "experiment.parent_condition_id"),
        replicate_id=_integer(_required(data, "replicate_id", "experiment"), "experiment.replicate_id"),
        seed=_integer(_required(data, "seed", "experiment"), "experiment.seed"),
    )


def _parse_simulation(value: Any) -> SimulationConfig:
    data = _mapping(value, "simulation")
    allowed = {
        "backend", "device", "physics_dt_s", "render_dt_s", "reference_density_kg_m3",
        "maximum_run_time_s", "profile", "substeps", "ccd_enabled",
        "release_command_latency_s", "release_confirmation_steps",
    }
    _reject_unknown(data, allowed, "simulation")
    return SimulationConfig(
        backend=_string(_required(data, "backend", "simulation"), "simulation.backend"),
        device=_string(_required(data, "device", "simulation"), "simulation.device"),
        physics_dt_s=_number(_required(data, "physics_dt_s", "simulation"), "simulation.physics_dt_s"),
        render_dt_s=_number(_required(data, "render_dt_s", "simulation"), "simulation.render_dt_s"),
        reference_density_kg_m3=_number(
            _required(data, "reference_density_kg_m3", "simulation"),
            "simulation.reference_density_kg_m3",
        ),
        maximum_run_time_s=_number(
            _required(data, "maximum_run_time_s", "simulation"), "simulation.maximum_run_time_s"
        ),
        profile=_string(_required(data, "profile", "simulation"), "simulation.profile"),
        substeps=_integer(data.get("substeps", 1), "simulation.substeps"),
        ccd_enabled=_boolean(data.get("ccd_enabled", False), "simulation.ccd_enabled"),
        release_command_latency_s=_number(
            data.get("release_command_latency_s", 0.0), "simulation.release_command_latency_s"
        ),
        release_confirmation_steps=_integer(
            data.get("release_confirmation_steps", 1), "simulation.release_confirmation_steps"
        ),
    )


def _parse_models(value: Any) -> ModelsConfig:
    data = _mapping(value, "models")
    fields = {
        "launch_force", "atmosphere", "guide", "coupling", "separation_actuator",
        "cart_brake", "rocket_motor", "rocket_aerodynamics", "observer",
    }
    _reject_unknown(data, fields, "models")
    return ModelsConfig(**{
        field: _string(_required(data, field, "models"), f"models.{field}")
        for field in sorted(fields)
    })


def _parse_tube(value: Any, schema_version: int) -> TubeConfig:
    data = _mapping(value, "tube")
    if schema_version == 2:
        allowed = {
            "angle_deg", "inner_diameter_m", "exit_brake_track_length_m", "exit_track_grade_deg",
            "exterior_effective_density_ratio", "guide_clearance_m", "stages", "anti_tunneling_pairs",
        }
    elif schema_version == 3:
        allowed = {
            "geometry_mode", "inner_diameter_m", "exterior_atmosphere", "centerline",
            "exit_track", "guide_clearance_m", "stages", "anti_tunneling_pairs",
        }
    else:
        raise ConfigurationError(f"unsupported schema_version {schema_version!r}; expected 2 or 3")
    _reject_unknown(data, allowed, "tube")
    stages = []
    for index, raw_stage in enumerate(_sequence(_required(data, "stages", "tube"), "tube.stages")):
        path = f"tube.stages[{index}]"
        stage = _mapping(raw_stage, path)
        _reject_unknown(stage, {"name", "length_m", "effective_density_ratio", "color_rgb", "opacity"}, path)
        stages.append(
            StageConfig(
                name=_string(_required(stage, "name", path), path + ".name"),
                length_m=_number(_required(stage, "length_m", path), path + ".length_m"),
                effective_density_ratio=_number(
                    _required(stage, "effective_density_ratio", path), path + ".effective_density_ratio"
                ),
                color_rgb=_vec3(_required(stage, "color_rgb", path), path + ".color_rgb"),
                opacity=_number(stage.get("opacity", 1.0), path + ".opacity"),
            )
        )
    pairs = []
    for index, raw_pair in enumerate(_sequence(data.get("anti_tunneling_pairs", []), "tube.anti_tunneling_pairs")):
        path = f"tube.anti_tunneling_pairs[{index}]"
        pair = _mapping(raw_pair, path)
        _reject_unknown(pair, {"name", "test_relative_speed_mps"}, path)
        pairs.append(
            AntiTunnelingPairConfig(
                name=_string(_required(pair, "name", path), path + ".name"),
                test_relative_speed_mps=_number(
                    _required(pair, "test_relative_speed_mps", path), path + ".test_relative_speed_mps"
                ),
            )
        )
    common = dict(
        inner_diameter_m=_number(_required(data, "inner_diameter_m", "tube"), "tube.inner_diameter_m"),
        guide_clearance_m=_number(data.get("guide_clearance_m", 0.0), "tube.guide_clearance_m"),
        stages=tuple(stages),
        anti_tunneling_pairs=tuple(pairs),
    )
    if schema_version == 2:
        return TubeConfig(
            angle_deg=_number(_required(data, "angle_deg", "tube"), "tube.angle_deg"),
            exit_brake_track_length_m=_number(
                _required(data, "exit_brake_track_length_m", "tube"), "tube.exit_brake_track_length_m"
            ),
            exit_track_grade_deg=_number(data.get("exit_track_grade_deg", 0.0), "tube.exit_track_grade_deg"),
            exterior_effective_density_ratio=_number(
                _required(data, "exterior_effective_density_ratio", "tube"),
                "tube.exterior_effective_density_ratio",
            ),
            **common,
        )

    geometry_mode = _string(_required(data, "geometry_mode", "tube"), "tube.geometry_mode")
    raw_segments = _sequence(_required(data, "centerline", "tube"), "tube.centerline")
    segments = []
    for index, raw_segment in enumerate(raw_segments):
        path = f"tube.centerline[{index}]"
        segment = _mapping(raw_segment, path)
        segment_type = _string(_required(segment, "type", path), path + ".type")
        if segment_type == "straight":
            fields = {"type", "length_m", "initial_angle_deg"}
            _reject_unknown(segment, fields, path)
            segments.append(
                CenterlineSegmentConfig(
                    type=segment_type,
                    length_m=_number(_required(segment, "length_m", path), path + ".length_m"),
                    initial_angle_deg=(
                        _number(segment["initial_angle_deg"], path + ".initial_angle_deg")
                        if "initial_angle_deg" in segment
                        else None
                    ),
                )
            )
        elif segment_type == "clothoid":
            fields = {"type", "length_m", "start_curvature_per_m", "end_curvature_per_m"}
            _reject_unknown(segment, fields, path)
            segments.append(
                CenterlineSegmentConfig(
                    type=segment_type,
                    length_m=_number(_required(segment, "length_m", path), path + ".length_m"),
                    start_curvature_per_m=_number(
                        _required(segment, "start_curvature_per_m", path), path + ".start_curvature_per_m"
                    ),
                    end_curvature_per_m=_number(
                        _required(segment, "end_curvature_per_m", path), path + ".end_curvature_per_m"
                    ),
                )
            )
        elif segment_type == "circular_arc":
            fields = {"type", "radius_m", "signed_turn_deg"}
            _reject_unknown(segment, fields, path)
            radius = _number(_required(segment, "radius_m", path), path + ".radius_m")
            turn = _number(_required(segment, "signed_turn_deg", path), path + ".signed_turn_deg")
            segments.append(
                CenterlineSegmentConfig(
                    type=segment_type,
                    length_m=abs(math.radians(turn) * radius),
                    radius_m=radius,
                    signed_turn_deg=turn,
                )
            )
        else:
            raise ConfigurationError(
                f"{path}.type must be 'straight', 'clothoid', or 'circular_arc', got {segment_type!r}"
            )

    if not segments:
        raise ConfigurationError("tube.centerline must contain at least one segment")
    first_angle = segments[0].initial_angle_deg
    if first_angle is None:
        raise ConfigurationError("tube.centerline[0].initial_angle_deg is required")

    exit_data = _mapping(_required(data, "exit_track", "tube"), "tube.exit_track")
    _reject_unknown(exit_data, {"type", "length_m", "inclination_deg", "curvature_per_m"}, "tube.exit_track")
    exit_track = ExitTrackConfig(
        type=_string(_required(exit_data, "type", "tube.exit_track"), "tube.exit_track.type"),
        length_m=_number(_required(exit_data, "length_m", "tube.exit_track"), "tube.exit_track.length_m"),
        inclination_deg=_number(
            _required(exit_data, "inclination_deg", "tube.exit_track"), "tube.exit_track.inclination_deg"
        ),
        curvature_per_m=_number(
            _required(exit_data, "curvature_per_m", "tube.exit_track"), "tube.exit_track.curvature_per_m"
        ),
    )
    atmosphere_data = _mapping(
        _required(data, "exterior_atmosphere", "tube"), "tube.exterior_atmosphere"
    )
    _reject_unknown(
        atmosphere_data,
        {"model", "reference_ratio", "reference_altitude_m", "scale_height_m"},
        "tube.exterior_atmosphere",
    )
    atmosphere = ExteriorAtmosphereConfig(
        model=_string(
            _required(atmosphere_data, "model", "tube.exterior_atmosphere"),
            "tube.exterior_atmosphere.model",
        ),
        reference_ratio=_number(
            _required(atmosphere_data, "reference_ratio", "tube.exterior_atmosphere"),
            "tube.exterior_atmosphere.reference_ratio",
        ),
        reference_altitude_m=_number(
            _required(atmosphere_data, "reference_altitude_m", "tube.exterior_atmosphere"),
            "tube.exterior_atmosphere.reference_altitude_m",
        ),
        scale_height_m=(
            _number(atmosphere_data["scale_height_m"], "tube.exterior_atmosphere.scale_height_m")
            if "scale_height_m" in atmosphere_data
            else None
        ),
    )
    return TubeConfig(
        angle_deg=first_angle,
        exit_brake_track_length_m=exit_track.length_m,
        exit_track_grade_deg=exit_track.inclination_deg,
        exterior_effective_density_ratio=atmosphere.reference_ratio,
        geometry_mode=geometry_mode,
        centerline=tuple(segments),
        exit_track=exit_track,
        exterior_atmosphere=atmosphere,
        **common,
    )


def _parse_cart(value: Any) -> CartConfig:
    data = _mapping(value, "cart")
    fields = {
        "mass_kg", "length_m", "width_m", "height_m", "drag_coefficient", "frontal_area_m2",
        "brake_force_limit_n", "brake_jerk_limit_mps3", "brake_stop_margin_m",
        "stopped_speed_threshold_mps", "guide_resistance_n", "inertia_mode",
        "maximum_resultant_load_g",
    }
    _reject_unknown(data, fields, "cart")
    numeric = fields - {"inertia_mode", "guide_resistance_n", "maximum_resultant_load_g"}
    values = {field: _number(_required(data, field, "cart"), f"cart.{field}") for field in numeric}
    values["guide_resistance_n"] = _number(data.get("guide_resistance_n", 0.0), "cart.guide_resistance_n")
    values["inertia_mode"] = _string(_required(data, "inertia_mode", "cart"), "cart.inertia_mode")
    values["maximum_resultant_load_g"] = (
        _number(data["maximum_resultant_load_g"], "cart.maximum_resultant_load_g")
        if "maximum_resultant_load_g" in data
        else None
    )
    return CartConfig(**values)


def _parse_guided(value: Any) -> GuidedAerodynamicsConfig:
    data = _mapping(value, "guided_phase_aerodynamics")
    fields = {"drag_coefficient", "reference_area_m2", "axial_air_velocity_mps", "boundary_blend_distance_m", "force_model"}
    _reject_unknown(data, fields, "guided_phase_aerodynamics")
    return GuidedAerodynamicsConfig(
        drag_coefficient=_number(_required(data, "drag_coefficient", "guided_phase_aerodynamics"), "guided_phase_aerodynamics.drag_coefficient"),
        reference_area_m2=_number(_required(data, "reference_area_m2", "guided_phase_aerodynamics"), "guided_phase_aerodynamics.reference_area_m2"),
        axial_air_velocity_mps=_number(data.get("axial_air_velocity_mps", 0.0), "guided_phase_aerodynamics.axial_air_velocity_mps"),
        boundary_blend_distance_m=_number(data.get("boundary_blend_distance_m", 0.0), "guided_phase_aerodynamics.boundary_blend_distance_m"),
        force_model=_string(_required(data, "force_model", "guided_phase_aerodynamics"), "guided_phase_aerodynamics.force_model"),
    )


def _parse_rocket(value: Any, schema_version: int) -> RocketConfig:
    data = _mapping(value, "rocket")
    fields = {"initial_mass_kg", "length_m", "diameter_m", "drag_coefficient", "reference_area_m2", "inertia_mode", "aft_clearance_marker", "motor", "ignition"}
    _reject_unknown(data, fields, "rocket")
    motor_data = _mapping(_required(data, "motor", "rocket"), "rocket.motor")
    _reject_unknown(motor_data, {"model", "thrust_n", "burn_duration_s"}, "rocket.motor")
    ignition_data = _mapping(_required(data, "ignition", "rocket"), "rocket.ignition")
    ignition_fields = {"delay_s", "minimum_cart_clearance_m", "minimum_relative_speed_mps", "no_recontact_dwell_s", "separation_timeout_s", "maximum_contact_impulse_ns", "maximum_angular_rate_deg_s", "trigger"}
    _reject_unknown(ignition_data, ignition_fields, "rocket.ignition")
    raw_trigger = ignition_data.get("trigger")
    if raw_trigger is None:
        if schema_version >= 3:
            raise ConfigurationError("missing required key rocket.ignition.trigger")
        trigger_data: Mapping[str, Any] = {"model": "safety_gates_only_v1"}
    else:
        trigger_data = _mapping(raw_trigger, "rocket.ignition.trigger")
    trigger_fields = {
        "model",
        "minimum_altitude_m",
        "maximum_flight_path_angle_deg",
        "maximum_vertical_speed_mps",
    }
    _reject_unknown(trigger_data, trigger_fields, "rocket.ignition.trigger")
    return RocketConfig(
        initial_mass_kg=_number(_required(data, "initial_mass_kg", "rocket"), "rocket.initial_mass_kg"),
        length_m=_number(_required(data, "length_m", "rocket"), "rocket.length_m"),
        diameter_m=_number(_required(data, "diameter_m", "rocket"), "rocket.diameter_m"),
        drag_coefficient=_number(_required(data, "drag_coefficient", "rocket"), "rocket.drag_coefficient"),
        reference_area_m2=_number(_required(data, "reference_area_m2", "rocket"), "rocket.reference_area_m2"),
        inertia_mode=_string(_required(data, "inertia_mode", "rocket"), "rocket.inertia_mode"),
        aft_clearance_marker=_string(_required(data, "aft_clearance_marker", "rocket"), "rocket.aft_clearance_marker"),
        motor=MotorConfig(
            model=_string(_required(motor_data, "model", "rocket.motor"), "rocket.motor.model"),
            thrust_n=_number(_required(motor_data, "thrust_n", "rocket.motor"), "rocket.motor.thrust_n"),
            burn_duration_s=_number(_required(motor_data, "burn_duration_s", "rocket.motor"), "rocket.motor.burn_duration_s"),
        ),
        ignition=IgnitionConfig(
            **{
                field: _number(_required(ignition_data, field, "rocket.ignition"), f"rocket.ignition.{field}")
                for field in sorted(ignition_fields - {"trigger"})
            },
            trigger=IgnitionTriggerConfig(
                model=_string(_required(trigger_data, "model", "rocket.ignition.trigger"), "rocket.ignition.trigger.model"),
                **{
                    field: (
                        None
                        if trigger_data.get(field) is None
                        else _number(trigger_data[field], f"rocket.ignition.trigger.{field}")
                    )
                    for field in sorted(trigger_fields - {"model"})
                },
            ),
        ),
    )


def _parse_force_position_points(value: Any) -> tuple[ForcePositionPointConfig, ...]:
    points = []
    for index, raw_point in enumerate(
        _sequence(value, "launch_control.force_vs_position")
    ):
        path = f"launch_control.force_vs_position[{index}]"
        point = _mapping(raw_point, path)
        _reject_unknown(point, {"position_m", "force_n"}, path)
        points.append(
            ForcePositionPointConfig(
                position_m=_number(_required(point, "position_m", path), path + ".position_m"),
                force_n=_number(_required(point, "force_n", path), path + ".force_n"),
            )
        )
    return tuple(points)


def _parse_launch(value: Any) -> LaunchControlConfig:
    data = _mapping(value, "launch_control")
    fields = {
        "mode", "target_exit_speed_mps", "maximum_force_n", "maximum_acceleration_mps2",
        "force_ramp_up_distance_m", "force_ramp_down_distance_m",
        "maximum_resultant_load_g", "maximum_normal_jerk_mps3",
        "force_vs_position",
    }
    _reject_unknown(data, fields, "launch_control")
    return LaunchControlConfig(
        mode=_string(_required(data, "mode", "launch_control"), "launch_control.mode"),
        target_exit_speed_mps=_number(_required(data, "target_exit_speed_mps", "launch_control"), "launch_control.target_exit_speed_mps"),
        maximum_force_n=_number(_required(data, "maximum_force_n", "launch_control"), "launch_control.maximum_force_n"),
        maximum_acceleration_mps2=_number(_required(data, "maximum_acceleration_mps2", "launch_control"), "launch_control.maximum_acceleration_mps2"),
        force_ramp_up_distance_m=_number(data.get("force_ramp_up_distance_m", 0.0), "launch_control.force_ramp_up_distance_m"),
        force_ramp_down_distance_m=_number(data.get("force_ramp_down_distance_m", 0.0), "launch_control.force_ramp_down_distance_m"),
        maximum_resultant_load_g=(
            _number(data["maximum_resultant_load_g"], "launch_control.maximum_resultant_load_g")
            if "maximum_resultant_load_g" in data
            else None
        ),
        maximum_normal_jerk_mps3=(
            _number(data["maximum_normal_jerk_mps3"], "launch_control.maximum_normal_jerk_mps3")
            if "maximum_normal_jerk_mps3" in data
            else None
        ),
        force_vs_position=_parse_force_position_points(data.get("force_vs_position", ())),
    )


def _parse_stage2_constraint(value: Any) -> Stage2ConstraintConfig:
    data = _mapping(value, "stage2_constraint")
    fields = {
        "model",
        "specific_impulse_s",
        "propellant_mass_fraction",
        "target_orbit_altitude_m",
        "assumed_unmodeled_loss_mps",
    }
    _reject_unknown(data, fields, "stage2_constraint")
    return Stage2ConstraintConfig(
        model=_string(
            _required(data, "model", "stage2_constraint"), "stage2_constraint.model"
        ),
        specific_impulse_s=_number(
            _required(data, "specific_impulse_s", "stage2_constraint"),
            "stage2_constraint.specific_impulse_s",
        ),
        propellant_mass_fraction=_number(
            _required(data, "propellant_mass_fraction", "stage2_constraint"),
            "stage2_constraint.propellant_mass_fraction",
        ),
        target_orbit_altitude_m=_number(
            _required(data, "target_orbit_altitude_m", "stage2_constraint"),
            "stage2_constraint.target_orbit_altitude_m",
        ),
        assumed_unmodeled_loss_mps=_number(
            _required(data, "assumed_unmodeled_loss_mps", "stage2_constraint"),
            "stage2_constraint.assumed_unmodeled_loss_mps",
        ),
    )


def _parse_evidence(value: Any) -> EvidenceConfig:
    data = _mapping(value, "evidence")
    fields = {
        "free_flight_start_event", "free_flight_duration_s", "completion_margin_s",
        "atmosphere_stage_refinement_factor", "maximum_exit_speed_relative_change",
        "maximum_peak_load_change_g", "maximum_drag_work_relative_change",
    }
    _reject_unknown(data, fields, "evidence")
    return EvidenceConfig(
        free_flight_start_event=_string(
            _required(data, "free_flight_start_event", "evidence"), "evidence.free_flight_start_event"
        ),
        free_flight_duration_s=_number(
            _required(data, "free_flight_duration_s", "evidence"), "evidence.free_flight_duration_s"
        ),
        completion_margin_s=_number(
            _required(data, "completion_margin_s", "evidence"), "evidence.completion_margin_s"
        ),
        atmosphere_stage_refinement_factor=_integer(
            _required(data, "atmosphere_stage_refinement_factor", "evidence"),
            "evidence.atmosphere_stage_refinement_factor",
        ),
        maximum_exit_speed_relative_change=_number(
            _required(data, "maximum_exit_speed_relative_change", "evidence"),
            "evidence.maximum_exit_speed_relative_change",
        ),
        maximum_peak_load_change_g=_number(
            _required(data, "maximum_peak_load_change_g", "evidence"),
            "evidence.maximum_peak_load_change_g",
        ),
        maximum_drag_work_relative_change=_number(
            _required(data, "maximum_drag_work_relative_change", "evidence"),
            "evidence.maximum_drag_work_relative_change",
        ),
    )


def _parse_markers(value: Any) -> Mapping[str, MarkerConfig]:
    data = _mapping(value, "markers")
    markers = {}
    for name, raw_marker in data.items():
        path = f"markers.{name}"
        marker = _mapping(raw_marker, path)
        _reject_unknown(marker, {"body", "offset_m"}, path)
        markers[name] = MarkerConfig(
            body=_string(_required(marker, "body", path), path + ".body"),
            offset_m=_vec3(_required(marker, "offset_m", path), path + ".offset_m"),
        )
    return MappingProxyType(markers)


def _parse_output(value: Any) -> OutputConfig:
    data = _mapping(value, "output")
    fields = {"directory", "telemetry_rate_hz", "diagnostics_format", "validity_policy", "criterion_policy", "capture_profile"}
    _reject_unknown(data, fields, "output")
    return OutputConfig(
        directory=_string(_required(data, "directory", "output"), "output.directory"),
        telemetry_rate_hz=_number(_required(data, "telemetry_rate_hz", "output"), "output.telemetry_rate_hz"),
        diagnostics_format=_string(_required(data, "diagnostics_format", "output"), "output.diagnostics_format"),
        validity_policy=_string(data.get("validity_policy", "per_field_v1"), "output.validity_policy"),
        criterion_policy=_string(data.get("criterion_policy", "baseline_v1"), "output.criterion_policy"),
        capture_profile=_string(data.get("capture_profile", "interactive_v1"), "output.capture_profile"),
    )


def _plain_value(value: Any) -> Any:
    """Convert frozen schema values into a deterministic JSON-compatible tree."""
    if is_dataclass(value):
        return {item.name: _plain_value(getattr(value, item.name)) for item in fields(value)}
    if isinstance(value, Mapping):
        return {str(key): _plain_value(item) for key, item in sorted(value.items())}
    if isinstance(value, (list, tuple)):
        return [_plain_value(item) for item in value]
    if isinstance(value, (set, frozenset)):
        return sorted(_plain_value(item) for item in value)
    if isinstance(value, Enum):
        return value.value
    return value


def _resolved_hash(config: ScenarioConfig) -> str:
    resolved = _plain_value(config)
    if config.schema_version == 2:
        # Preserve the established schema-v2 identity. Adding optional schema-v3 storage
        # to the shared dataclasses must not change the resolved hash of an unchanged v2
        # configuration or invalidate its existing evidence lineage.
        resolved.pop("evidence", None)
        for field in ("geometry_mode", "centerline", "exit_track", "exterior_atmosphere"):
            resolved["tube"].pop(field, None)
        resolved["cart"].pop("maximum_resultant_load_g", None)
        resolved["rocket"]["ignition"].pop("trigger", None)
        resolved["launch_control"].pop("maximum_resultant_load_g", None)
        resolved["launch_control"].pop("maximum_normal_jerk_mps3", None)
        if not resolved["launch_control"].get("force_vs_position"):
            resolved["launch_control"].pop("force_vs_position", None)
    payload = json.dumps(resolved, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def load_mapping(
    value: Any,
    *,
    source_bytes: bytes | None = None,
    source_path: str | None = None,
) -> LoadedScenario:
    data = _mapping(value, "root")
    schema_version = _integer(_required(data, "schema_version", "root"), "schema_version")
    if schema_version not in (2, 3):
        raise ConfigurationError(f"unsupported schema_version {schema_version!r}; expected 2 or 3")
    root_fields = {
        "schema_version", "experiment", "simulation", "models", "tube", "cart",
        "guided_phase_aerodynamics", "rocket", "launch_control", "markers", "output",
    }
    if schema_version == 3:
        root_fields.add("evidence")
        # Optional: a scenario may be a pure launcher study with no upper stage declared.
        # Absent means "do not score feasibility", not "score it with defaults" -- a
        # silently defaulted stage would put a margin in the record that nobody authored.
        root_fields.add("stage2_constraint")
    _reject_unknown(data, root_fields, "root")
    config = ScenarioConfig(
        schema_version=schema_version,
        experiment=_parse_experiment(_required(data, "experiment", "root")),
        simulation=_parse_simulation(_required(data, "simulation", "root")),
        models=_parse_models(_required(data, "models", "root")),
        tube=_parse_tube(_required(data, "tube", "root"), schema_version),
        cart=_parse_cart(_required(data, "cart", "root")),
        guided_phase_aerodynamics=_parse_guided(_required(data, "guided_phase_aerodynamics", "root")),
        rocket=_parse_rocket(_required(data, "rocket", "root"), schema_version),
        launch_control=_parse_launch(_required(data, "launch_control", "root")),
        markers=_parse_markers(_required(data, "markers", "root")),
        output=_parse_output(_required(data, "output", "root")),
        evidence=(
            _parse_evidence(_required(data, "evidence", "root"))
            if schema_version == 3
            else None
        ),
        stage2_constraint=(
            _parse_stage2_constraint(data["stage2_constraint"])
            if schema_version == 3 and "stage2_constraint" in data
            else None
        ),
    )
    preflight = validate_scenario(config)
    if source_bytes is None:
        source_bytes = json.dumps(value, sort_keys=True, separators=(",", ":"), default=str).encode("utf-8")
    return LoadedScenario(
        config=config,
        preflight=preflight,
        source_sha256=hashlib.sha256(source_bytes).hexdigest(),
        resolved_sha256=_resolved_hash(config),
        source_path=source_path,
    )


def load_yaml(path: str | Path) -> LoadedScenario:
    file_path = Path(path)
    try:
        source = file_path.read_bytes()
    except OSError as exc:
        raise ConfigurationError(f"could not read configuration {file_path}: {exc}") from exc
    try:
        value = yaml.load(source.decode("utf-8"), Loader=_UniqueKeyLoader)
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise ConfigurationError(f"invalid YAML in {file_path}: {exc}") from exc
    return load_mapping(value, source_bytes=source, source_path=str(file_path.resolve()))
