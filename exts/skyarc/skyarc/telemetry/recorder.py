# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Streaming core CSV, event JSONL, and diagnostic JSONL recorder."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from typing import Any, Iterable, Mapping, Protocol, Tuple, runtime_checkable

from ..components.diagnostics import DiagnosticRecord, DiagnosticSchema
from ..effects.adapter import AppliedEffects
from ..effects.aggregator import AggregatedEffects
from ..events import Event
from ..launcher.geometry import TubePath, normal_jerk_mps3, path_pose
from ..linalg import add, dot, norm, sub
from ..names import (
    BODY_CART,
    BODY_ROCKET,
    PAIR_ROCKET_CRADLE,
    SLOT_ATMOSPHERE,
    SLOT_CART_BRAKE,
    SLOT_GUIDE,
    SLOT_LAUNCH_FORCE,
    SLOT_ROCKET_AERODYNAMICS,
    SLOT_ROCKET_MOTOR,
)
from ..state import Observation, SimulationState
from ..state_machine import MissionState
from .energy import EnergyAccumulator, EnergySnapshot
from .paths import RunPaths
from .schema import CORE_TELEMETRY_SCHEMA_V2, TelemetrySchema
from .summary import RunSummary, build_summary


def _plain(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_plain(item) for item in value]
    return value


def _diagnostic_sidecar(schemas: Iterable[DiagnosticSchema]) -> dict[str, Any]:
    result = {}
    for schema in schemas:
        if schema.namespace in result:
            raise ValueError(f"duplicate diagnostic namespace {schema.namespace!r}")
        result[schema.namespace] = {
            "version": schema.version,
            "fields": {
                name: {
                    "unit": field.unit,
                    "shape": list(field.shape),
                    "description": field.description,
                }
                for name, field in schema.fields.items()
            },
        }
    return result


@dataclass(frozen=True)
class StepTelemetryInput:
    pre_state: SimulationState
    observation: Observation
    command_snapshots: Mapping[str, Mapping[str, Any]]
    accepted_effects: AggregatedEffects
    applied_effects: AppliedEffects
    post_state: SimulationState
    post_observation: Observation
    mission_state: MissionState
    interlock_allowed: bool | None = None
    abort_active: bool = False


@runtime_checkable
class TelemetrySink(Protocol):
    def record_step(self, record: StepTelemetryInput) -> None: ...

    def record_events(self, events: Iterable[Event]) -> Tuple[Event, ...]: ...

    def record_diagnostics(
        self,
        records: Iterable[DiagnosticRecord],
        *,
        time_s: float,
        step_index: int,
    ) -> None: ...

    def finalize(self, *, termination_reason: str, mission_phase: str) -> Any: ...


class TelemetryRecorder:
    """Write one immutable run instance without buffering the full trajectory in memory."""

    def __init__(
        self,
        paths: RunPaths,
        initial_state: SimulationState,
        layout: TubePath,
        *,
        telemetry_rate_hz: float,
        target_exit_speed_mps: float | None = None,
        attached_load_limit_g: float | None = None,
        cart_load_limit_g: float | None = None,
        gravity_mps2: tuple[float, float, float] = (0.0, 0.0, -9.81),
        schema: TelemetrySchema = CORE_TELEMETRY_SCHEMA_V2,
        diagnostic_schemas: Iterable[DiagnosticSchema] = (),
    ) -> None:
        if not math.isfinite(telemetry_rate_hz) or telemetry_rate_hz <= 0.0:
            raise ValueError("telemetry rate must be finite and positive")
        self._paths = paths
        self._layout = layout
        self._schema = schema
        self._target_exit_speed = target_exit_speed_mps
        self._attached_limit = attached_load_limit_g
        self._cart_limit = cart_load_limit_g
        self._gravity = gravity_mps2
        self._period_s = 1.0 / telemetry_rate_hz
        self._next_sample_time_s = initial_state.time_s
        self._energy = EnergyAccumulator(initial_state, gravity_mps2=gravity_mps2)
        self._sample_count = 0
        self._event_count = 0
        self._sequence = 0
        self._first_event_time: dict[str, float] = {}
        self._peak_load_g = 0.0
        self._maximum_gap_m = 0.0
        self._actual_exit_speed: float | None = None
        self._last_state = initial_state
        self._closed = False

        resolved_diagnostic_schemas = tuple(diagnostic_schemas)
        diagnostic_metadata = _diagnostic_sidecar(resolved_diagnostic_schemas)
        self._diagnostic_versions = {
            schema.namespace: schema.version for schema in resolved_diagnostic_schemas
        }
        with paths.telemetry_schema.open("w", encoding="utf-8", newline="\n") as stream:
            json.dump(schema.sidecar(diagnostic_schemas=diagnostic_metadata), stream, indent=2, sort_keys=True)
            stream.write("\n")
        self._telemetry_stream = paths.telemetry_csv.open("w", encoding="utf-8", newline="")
        self._events_stream = paths.events_jsonl.open("w", encoding="utf-8", newline="\n")
        self._diagnostics_stream = paths.diagnostics_jsonl.open("w", encoding="utf-8", newline="\n")
        self._writer = csv.DictWriter(self._telemetry_stream, fieldnames=schema.csv_columns)
        self._writer.writeheader()

    @property
    def paths(self) -> RunPaths:
        return self._paths

    @property
    def sample_count(self) -> int:
        return self._sample_count

    @property
    def event_count(self) -> int:
        return self._event_count

    @property
    def energy(self) -> EnergySnapshot:
        return self._energy.snapshot

    def _force(self, effects: AggregatedEffects | AppliedEffects, body: str) -> tuple[float, float, float]:
        load = effects.loads.get(body)
        return (0.0, 0.0, 0.0) if load is None else load.force_n

    @staticmethod
    def _slot_force(
        effects: AggregatedEffects | AppliedEffects,
        body: str,
        slot: str,
    ) -> tuple[float, float, float]:
        load = effects.loads.get(body)
        if load is None:
            return (0.0, 0.0, 0.0)
        return load.force_by_slot.get(slot, (0.0, 0.0, 0.0))

    def _sample_values(self, record: StepTelemetryInput, energy: EnergySnapshot) -> dict[str, Any]:
        before = record.pre_state
        after = record.post_state
        observation = record.post_observation
        pose = path_pose(self._layout, observation.axial.s_cart_m)
        dt_s = after.time_s - before.time_s
        values: dict[str, Any] = {
            "time_s": after.time_s,
            "step_index": after.step_index,
            "mission.phase": record.mission_state.phase.value,
            "mission.stage_index": record.mission_state.stage_index,
            "mission.stage_name": record.mission_state.stage_name,
            "mission.cart_branch": record.mission_state.cart_branch.value,
            "mission.rocket_branch": record.mission_state.rocket_branch.value,
            "observation.stage_index": observation.axial.stage_index,
            "observation.stage_name": observation.axial.stage_name,
            "observation.effective_density_ratio": observation.axial.effective_density_ratio,
            "observation.coupled": observation.coupled,
            "observation.separation_gap_m": observation.axial.separation_gap_m,
            "observation.separation_rate_mps": observation.axial.separation_rate_mps,
            "observation.cart_s_m": observation.axial.s_cart_m,
            "observation.rocket_s_m": observation.axial.s_rocket_m,
            "geometry.segment_index": pose.segment_index,
            "geometry.nearest_path_error_m": norm(sub(after.body(BODY_CART).position, pose.position_m)),
            "geometry.signed_curvature_per_m": pose.signed_curvature_per_m,
            "geometry.radius_m": (
                None
                if abs(pose.signed_curvature_per_m) <= 1e-15
                else 1.0 / abs(pose.signed_curvature_per_m)
            ),
            # Null once the closure is incomplete, so a reader cannot mistake a
            # translation-only figure for the full mechanical energy.
            "energy.kinetic_j": energy.kinetic_j if energy.valid else None,
            "energy.potential_j": energy.potential_j if energy.valid else None,
            # Per-slot wrench work remains measurable when rotational kinetic energy is
            # unavailable. Only the total mechanical-energy identity is invalidated.
            "energy.work_launch_j": energy.work_j["launch"],
            "energy.work_thrust_j": energy.work_j["thrust"],
            "energy.work_drag_j": energy.work_j["drag"],
            "energy.work_brake_j": energy.work_j["brake"],
            "energy.work_resistance_j": energy.work_j["resistance"],
            "energy.work_separation_j": energy.work_j["separation"],
            "energy.work_guide_reaction_j": energy.work_j["guide_reaction"],
            "energy.residual_j": energy.residual_j if energy.valid else None,
            "energy.normalized_residual": (
                energy.normalized_residual if energy.valid else None
            ),
            "energy.closure_valid": energy.valid,
            "rocket.impulse_ns": energy.rocket_impulse_ns,
            "interlock.allowed": record.interlock_allowed,
            "abort.active": record.abort_active,
            "target.exit_speed_mps": self._target_exit_speed,
            "actual.rocket_axial_speed_mps": observation.axial.rocket_axial_velocity_mps,
        }
        snapshots = record.command_snapshots
        launch = snapshots.get(SLOT_LAUNCH_FORCE, {})
        brake = snapshots.get(SLOT_CART_BRAKE, {})
        motor = snapshots.get(SLOT_ROCKET_MOTOR, {})
        guide = snapshots.get(SLOT_GUIDE, {})
        values.update(
            {
                "command.launch_force_n": launch.get("last_force_n"),
                "command.launch_acceleration_mps2": launch.get("last_acceleration_command_mps2"),
                "command.brake_force_n": brake.get("last_force_n"),
                "command.brake_hold_force_n": brake.get("hold_force_n"),
                "command.rocket_thrust_n": motor.get("last_thrust_n"),
            }
        )
        for phase, state in (("pre", before), ("post", after)):
            for body_name in (BODY_CART, BODY_ROCKET):
                body = state.body(body_name)
                for index, axis in enumerate("xyz"):
                    values[f"{phase}.{body_name}.position_m.{axis}"] = body.position[index]
                    values[f"{phase}.{body_name}.velocity_mps.{axis}"] = body.linear_velocity[index]
                for index, component in enumerate("wxyz"):
                    values[f"{phase}.{body_name}.orientation.{component}"] = body.orientation[index]
        for body_name in (BODY_CART, BODY_ROCKET):
            for index, axis in enumerate("xyz"):
                acceleration = (
                    None
                    if dt_s <= 0.0
                    else (
                        after.body(body_name).linear_velocity[index]
                        - before.body(body_name).linear_velocity[index]
                    )
                    / dt_s
                )
                values[f"pre.{body_name}.acceleration_mps2.{axis}"] = None
                values[f"post.{body_name}.acceleration_mps2.{axis}"] = acceleration
        for channel, effects in (
            ("accepted", record.accepted_effects),
            ("applied", record.applied_effects),
        ):
            for body_name in (BODY_CART, BODY_ROCKET):
                force = self._force(effects, body_name)
                for index, axis in enumerate("xyz"):
                    values[f"{channel}.{body_name}.force_n.{axis}"] = force[index]
        for index, axis in enumerate("xyz"):
            values[f"geometry.tangent.{axis}"] = pose.tangent[index]
            values[f"geometry.normal.{axis}"] = pose.normal[index]

        normal_load = float(guide.get("last_normal_load_mps2", 0.0))
        pre_speed = record.observation.axial.cart_axial_velocity_mps
        post_speed = observation.axial.cart_axial_velocity_mps
        tangent_acceleration = 0.0 if dt_s <= 0.0 else (post_speed - pre_speed) / dt_s
        jerk = normal_jerk_mps3(
            pre_speed,
            tangent_acceleration,
            pose.signed_curvature_per_m,
            pose.curvature_rate_per_m2,
        )
        if record.observation.coupled:
            force = add(
                self._force(record.accepted_effects, BODY_CART),
                self._force(record.accepted_effects, BODY_ROCKET),
            )
            mass = before.body(BODY_CART).mass_kg + before.body(BODY_ROCKET).mass_kg
            limit = self._attached_limit
            owner = "attached_assembly"
        else:
            force = self._force(record.accepted_effects, BODY_CART)
            mass = before.body(BODY_CART).mass_kg
            limit = self._cart_limit
            owner = "post_release_cart"
        resultant_g = math.hypot(dot(force, pose.tangent) / mass, normal_load) / 9.81
        signed_normal = pre_speed * pre_speed * pose.signed_curvature_per_m - dot(
            self._gravity, pose.normal
        )
        cart_mass = before.body(BODY_CART).mass_kg
        rocket_mass = before.body(BODY_ROCKET).mass_kg
        accepted = record.accepted_effects
        cart_launch = dot(self._slot_force(accepted, BODY_CART, SLOT_LAUNCH_FORCE), pose.tangent)
        cart_drag = dot(self._slot_force(accepted, BODY_CART, SLOT_ATMOSPHERE), pose.tangent)
        cart_resistance = dot(self._slot_force(accepted, BODY_CART, SLOT_GUIDE), pose.tangent)
        cart_brake = dot(self._slot_force(accepted, BODY_CART, SLOT_CART_BRAKE), pose.tangent)
        rocket_drag = dot(
            self._slot_force(accepted, BODY_ROCKET, SLOT_ROCKET_AERODYNAMICS),
            pose.tangent,
        )
        rocket_thrust = dot(
            self._slot_force(accepted, BODY_ROCKET, SLOT_ROCKET_MOTOR),
            pose.tangent,
        )
        values.update(
            {
                "load.guide_normal_acceleration_mps2": signed_normal,
                "load.guide_normal_bound_mps2": normal_load,
                "load.normal_jerk_mps3": jerk,
                "load.resultant_proper_g": resultant_g,
                "load.limit_owner": owner,
                "load.remaining_margin_g": None if limit is None else limit - resultant_g,
                "load.cart.launch_force_n": cart_launch,
                "load.cart.drag_force_n": cart_drag,
                "load.cart.resistance_force_n": cart_resistance,
                "load.cart.brake_force_n": cart_brake,
                "load.cart.gravity_force_n": cart_mass * dot(self._gravity, pose.tangent),
                "load.cart.tangential_non_grav_mps2": dot(
                    self._force(accepted, BODY_CART), pose.tangent
                )
                / cart_mass,
                "load.rocket.drag_force_n": rocket_drag,
                "load.rocket.thrust_force_n": rocket_thrust,
                "load.rocket.gravity_force_n": rocket_mass * dot(self._gravity, pose.tangent),
                "load.rocket.tangential_non_grav_mps2": dot(
                    self._force(accepted, BODY_ROCKET), pose.tangent
                )
                / rocket_mass,
                "load.rocket_cradle_contact_impulse_ns": after.contact(
                    PAIR_ROCKET_CRADLE
                ).magnitude_ns,
            }
        )
        return values

    def _should_sample(self, time_s: float) -> bool:
        if time_s + 1e-12 < self._next_sample_time_s:
            return False
        while self._next_sample_time_s <= time_s + 1e-12:
            self._next_sample_time_s += self._period_s
        return True

    def record_step(self, record: StepTelemetryInput) -> None:
        if self._closed:
            raise RuntimeError("cannot record to a closed telemetry stream")
        energy = self._energy.update(
            record.pre_state,
            record.post_state,
            record.applied_effects,
        )
        self._last_state = record.post_state
        values = self._sample_values(record, energy)
        load = values["load.resultant_proper_g"]
        self._peak_load_g = max(self._peak_load_g, float(load))
        self._maximum_gap_m = max(
            self._maximum_gap_m,
            record.post_observation.axial.separation_gap_m,
        )
        pre_exit_s = record.observation.axial.marker_s_m.get("assembly_exit", -math.inf)
        post_exit_s = record.post_observation.axial.marker_s_m.get("assembly_exit", -math.inf)
        if self._actual_exit_speed is None and post_exit_s >= self._layout.length_m:
            if pre_exit_s < self._layout.length_m and post_exit_s > pre_exit_s:
                fraction = (self._layout.length_m - pre_exit_s) / (post_exit_s - pre_exit_s)
                self._actual_exit_speed = (
                    record.observation.axial.rocket_axial_velocity_mps
                    + fraction
                    * (
                        record.post_observation.axial.rocket_axial_velocity_mps
                        - record.observation.axial.rocket_axial_velocity_mps
                    )
                )
            else:
                self._actual_exit_speed = record.post_observation.axial.rocket_axial_velocity_mps
        if not self._should_sample(record.post_state.time_s):
            return
        normalized = self._schema.normalize(values)
        row = {}
        for name, value in normalized.items():
            row[name] = "" if value is None else value
            row[f"{name}__valid"] = value is not None
        self._writer.writerow(row)
        self._telemetry_stream.flush()
        self._sample_count += 1

    def record_events(self, events: Iterable[Event]) -> Tuple[Event, ...]:
        if self._closed:
            raise RuntimeError("cannot record to a closed event stream")
        sequenced = []
        for event in events:
            assigned = event.with_sequence(self._sequence)
            self._sequence += 1
            self._event_count += 1
            self._first_event_time.setdefault(assigned.name, assigned.time_s)
            json.dump(
                {
                    "name": assigned.name,
                    "time_s": assigned.time_s,
                    "step_index": assigned.step_index,
                    "source": assigned.source,
                    "sequence": assigned.sequence,
                    "data": _plain(assigned.data),
                },
                self._events_stream,
                sort_keys=True,
                allow_nan=False,
            )
            self._events_stream.write("\n")
            sequenced.append(assigned)
        self._events_stream.flush()
        return tuple(sequenced)

    def record_diagnostics(
        self,
        records: Iterable[DiagnosticRecord],
        *,
        time_s: float,
        step_index: int,
    ) -> None:
        if self._closed:
            raise RuntimeError("cannot record to a closed diagnostic stream")
        for record in records:
            if not record.values:
                raise ValueError("diagnostic records must contain at least one registered value")
            namespaces = {name.split(".", 1)[0] for name in record.values}
            if len(namespaces) != 1:
                raise ValueError("one diagnostic record may use only one registered namespace")
            namespace = next(iter(namespaces))
            if self._diagnostic_versions.get(namespace) != record.schema_version:
                raise ValueError(
                    f"diagnostic namespace/version {namespace!r}/{record.schema_version!r} "
                    "was not registered in the telemetry sidecar"
                )
            json.dump(
                {
                    "time_s": time_s,
                    "step_index": step_index,
                    "source": record.source,
                    "schema_version": record.schema_version,
                    "values": _plain(record.values),
                },
                self._diagnostics_stream,
                sort_keys=True,
                allow_nan=False,
            )
            self._diagnostics_stream.write("\n")
        self._diagnostics_stream.flush()

    def finalize(self, *, termination_reason: str, mission_phase: str) -> RunSummary:
        if self._closed:
            raise RuntimeError("telemetry recorder was already finalized")
        summary = build_summary(
            termination_reason=termination_reason,
            mission_phase=mission_phase,
            elapsed_s=self._last_state.time_s,
            physics_steps=self._last_state.step_index,
            telemetry_samples=self._sample_count,
            event_count=self._event_count,
            target_exit_speed_mps=self._target_exit_speed,
            actual_exit_speed_mps=self._actual_exit_speed,
            peak_resultant_load_g=self._peak_load_g,
            maximum_separation_gap_m=self._maximum_gap_m,
            energy=self._energy.snapshot,
            first_event_time_s=self._first_event_time,
        )
        with self._paths.summary_json.open("w", encoding="utf-8", newline="\n") as stream:
            json.dump(summary.to_dict(), stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        self.close()
        return summary

    def close(self) -> None:
        if self._closed:
            return
        self._telemetry_stream.close()
        self._events_stream.close()
        self._diagnostics_stream.close()
        self._closed = True

    def __enter__(self) -> "TelemetryRecorder":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:  # type: ignore[no-untyped-def]
        self.close()
