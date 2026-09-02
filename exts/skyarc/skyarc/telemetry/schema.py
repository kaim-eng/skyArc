# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Versioned metadata for the flat core telemetry stream."""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Iterable, Mapping


VALUE_TYPES = frozenset({"float", "int", "bool", "string"})
FRAMES = frozenset({"none", "world", "body", "path"})
SAMPLE_PHASES = frozenset(
    {
        "pre_state",
        "observation",
        "command",
        "accepted_effects",
        "applied_effects",
        "post_state",
        "derived_post",
    }
)


class TelemetrySchemaError(ValueError):
    """Raised when schema metadata or a sample violates the telemetry contract."""


@dataclass(frozen=True)
class TelemetryField:
    value_type: str
    unit: str
    frame: str
    sample_phase: str
    nullable: bool = True
    description: str = ""

    def __post_init__(self) -> None:
        if self.value_type not in VALUE_TYPES:
            raise TelemetrySchemaError(f"unsupported telemetry type {self.value_type!r}")
        if not isinstance(self.unit, str) or not self.unit.strip():
            raise TelemetrySchemaError("telemetry fields must declare a non-empty SI unit or '1'")
        if self.frame not in FRAMES:
            raise TelemetrySchemaError(f"unsupported telemetry frame {self.frame!r}")
        if self.sample_phase not in SAMPLE_PHASES:
            raise TelemetrySchemaError(f"unsupported sample phase {self.sample_phase!r}")

    def validate(self, value: Any, name: str) -> Any:
        if value is None:
            if not self.nullable:
                raise TelemetrySchemaError(f"required telemetry field {name!r} is null")
            return None
        if self.value_type == "bool":
            if not isinstance(value, bool):
                raise TelemetrySchemaError(f"telemetry field {name!r} must be boolean")
            return value
        if self.value_type == "int":
            if isinstance(value, bool) or not isinstance(value, int):
                raise TelemetrySchemaError(f"telemetry field {name!r} must be an integer")
            return value
        if self.value_type == "string":
            if not isinstance(value, str):
                raise TelemetrySchemaError(f"telemetry field {name!r} must be a string")
            return value
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TelemetrySchemaError(f"telemetry field {name!r} must be numeric")
        result = float(value)
        if not math.isfinite(result):
            raise TelemetrySchemaError(f"telemetry field {name!r} must be finite or null")
        return result


@dataclass(frozen=True)
class TelemetrySchema:
    version: str
    fields: Mapping[str, TelemetryField]
    validity_policy: str = "per_field_v1"
    quaternion_order: str = "wxyz"

    def __post_init__(self) -> None:
        if not self.version:
            raise TelemetrySchemaError("telemetry schema version may not be empty")
        if self.validity_policy != "per_field_v1":
            raise TelemetrySchemaError("only explicit per-field validity is currently supported")
        if self.quaternion_order != "wxyz":
            raise TelemetrySchemaError("the core rigid-body boundary requires wxyz quaternions")
        copied = dict(self.fields)
        if not copied:
            raise TelemetrySchemaError("telemetry schema must contain fields")
        for name, metadata in copied.items():
            if not isinstance(name, str) or not name or name.startswith(".") or name.endswith("."):
                raise TelemetrySchemaError(f"invalid telemetry field name {name!r}")
            if not isinstance(metadata, TelemetryField):
                raise TelemetrySchemaError(f"metadata for {name!r} is not a TelemetryField")
        object.__setattr__(self, "fields", MappingProxyType(copied))

    @property
    def csv_columns(self) -> tuple[str, ...]:
        columns = []
        for name in self.fields:
            columns.extend((name, f"{name}__valid"))
        return tuple(columns)

    def normalize(self, values: Mapping[str, Any]) -> Mapping[str, Any]:
        unknown = sorted(set(values) - set(self.fields))
        if unknown:
            raise TelemetrySchemaError(f"sample contains unregistered telemetry fields: {unknown}")
        normalized = {}
        for name, metadata in self.fields.items():
            normalized[name] = metadata.validate(values.get(name), name)
        return MappingProxyType(normalized)

    def sidecar(self, *, diagnostic_schemas: Mapping[str, Any] | None = None) -> dict[str, Any]:
        return {
            "schema_version": self.version,
            "validity_policy": self.validity_policy,
            "quaternion_order": self.quaternion_order,
            "fields": {
                name: {
                    "type": field.value_type,
                    "unit": field.unit,
                    "frame": field.frame,
                    "sample_phase": field.sample_phase,
                    "nullable": field.nullable,
                    "description": field.description,
                }
                for name, field in self.fields.items()
            },
            "diagnostic_schemas": dict(diagnostic_schemas or {}),
        }


def _field(
    value_type: str,
    unit: str,
    frame: str,
    phase: str,
    *,
    nullable: bool = True,
    description: str = "",
) -> TelemetryField:
    return TelemetryField(value_type, unit, frame, phase, nullable, description)


def _core_fields() -> dict[str, TelemetryField]:
    fields: dict[str, TelemetryField] = {
        "time_s": _field("float", "s", "none", "post_state", nullable=False),
        "step_index": _field("int", "1", "none", "post_state", nullable=False),
        "mission.phase": _field("string", "1", "none", "derived_post", nullable=False),
        "mission.stage_index": _field("int", "1", "none", "derived_post", nullable=False),
        "mission.stage_name": _field("string", "1", "none", "derived_post", nullable=False),
        "mission.cart_branch": _field("string", "1", "none", "derived_post", nullable=False),
        "mission.rocket_branch": _field("string", "1", "none", "derived_post", nullable=False),
        "observation.stage_index": _field("int", "1", "none", "observation", nullable=False),
        "observation.stage_name": _field("string", "1", "none", "observation", nullable=False),
        "observation.effective_density_ratio": _field("float", "1", "none", "observation", nullable=False),
        "observation.coupled": _field("bool", "1", "none", "observation", nullable=False),
        "observation.separation_gap_m": _field("float", "m", "path", "observation", nullable=False),
        "observation.separation_rate_mps": _field("float", "m/s", "path", "observation", nullable=False),
        "observation.cart_s_m": _field("float", "m", "path", "observation", nullable=False),
        "observation.rocket_s_m": _field("float", "m", "path", "observation", nullable=False),
        "command.launch_force_n": _field("float", "N", "path", "command"),
        "command.launch_acceleration_mps2": _field("float", "m/s^2", "path", "command"),
        "command.brake_force_n": _field("float", "N", "path", "command"),
        "command.brake_hold_force_n": _field("float", "N", "path", "command"),
        "command.rocket_thrust_n": _field("float", "N", "body", "command"),
        "geometry.segment_index": _field("int", "1", "none", "derived_post"),
        "geometry.nearest_path_error_m": _field("float", "m", "world", "derived_post"),
        "geometry.signed_curvature_per_m": _field("float", "1/m", "path", "derived_post"),
        "geometry.radius_m": _field("float", "m", "path", "derived_post"),
        "load.guide_normal_acceleration_mps2": _field("float", "m/s^2", "path", "derived_post"),
        "load.guide_normal_bound_mps2": _field("float", "m/s^2", "path", "derived_post"),
        "load.normal_jerk_mps3": _field("float", "m/s^3", "path", "derived_post"),
        "load.resultant_proper_g": _field("float", "G", "path", "derived_post"),
        "load.limit_owner": _field("string", "1", "none", "derived_post"),
        "load.remaining_margin_g": _field("float", "G", "none", "derived_post"),
        # The state and closure channels below are nullable and are written as null with
        # their validity column false whenever a body rotates, because rotational kinetic
        # energy cannot be closed without modeled body inertia and a partial figure would
        # read as a complete one. The seven ``work_*`` channels are deliberately *not*
        # gated that way: each is an integral of applied force and torque against measured
        # displacement, so it is complete on its own terms and does not depend on inertia.
        # What rotation invalidates is the identity they feed, not the terms themselves.
        "energy.kinetic_j": _field("float", "J", "world", "derived_post"),
        "energy.potential_j": _field("float", "J", "world", "derived_post"),
        "energy.work_launch_j": _field("float", "J", "world", "derived_post"),
        "energy.work_thrust_j": _field("float", "J", "world", "derived_post"),
        "energy.work_drag_j": _field("float", "J", "world", "derived_post"),
        "energy.work_brake_j": _field("float", "J", "world", "derived_post"),
        "energy.work_resistance_j": _field("float", "J", "world", "derived_post"),
        "energy.work_separation_j": _field("float", "J", "world", "derived_post"),
        # Full wrench work done by a backend-commanded guide reaction rather than by a
        # workless constraint. Zero for any backend that enforces the path geometrically.
        "energy.work_guide_reaction_j": _field(
            "float",
            "J",
            "world",
            "derived_post",
            description="Accumulated force and torque work by the commanded guide reaction.",
        ),
        "energy.residual_j": _field("float", "J", "world", "derived_post"),
        "energy.normalized_residual": _field("float", "1", "none", "derived_post"),
        "energy.closure_valid": _field("bool", "1", "none", "derived_post", nullable=False),
        "rocket.impulse_ns": _field("float", "N*s", "world", "derived_post", nullable=False),
        "interlock.allowed": _field("bool", "1", "none", "derived_post"),
        "abort.active": _field("bool", "1", "none", "derived_post", nullable=False),
        "target.exit_speed_mps": _field("float", "m/s", "path", "derived_post"),
        "actual.rocket_axial_speed_mps": _field("float", "m/s", "path", "derived_post", nullable=False),
        "load.cart.launch_force_n": _field("float", "N", "path", "accepted_effects", nullable=False),
        "load.cart.drag_force_n": _field("float", "N", "path", "accepted_effects", nullable=False),
        "load.cart.resistance_force_n": _field("float", "N", "path", "accepted_effects", nullable=False),
        "load.cart.brake_force_n": _field("float", "N", "path", "accepted_effects", nullable=False),
        "load.cart.gravity_force_n": _field("float", "N", "path", "derived_post", nullable=False),
        "load.cart.tangential_non_grav_mps2": _field("float", "m/s^2", "path", "derived_post", nullable=False),
        "load.rocket.drag_force_n": _field("float", "N", "path", "accepted_effects", nullable=False),
        "load.rocket.thrust_force_n": _field("float", "N", "path", "accepted_effects", nullable=False),
        "load.rocket.gravity_force_n": _field("float", "N", "path", "derived_post", nullable=False),
        "load.rocket.tangential_non_grav_mps2": _field("float", "m/s^2", "path", "derived_post", nullable=False),
        "load.rocket_cradle_contact_impulse_ns": _field("float", "N*s", "world", "post_state", nullable=False),
    }
    for phase in ("pre", "post"):
        sample_phase = "pre_state" if phase == "pre" else "post_state"
        for body in ("cart", "rocket"):
            for axis in "xyz":
                fields[f"{phase}.{body}.position_m.{axis}"] = _field(
                    "float", "m", "world", sample_phase, nullable=False
                )
                fields[f"{phase}.{body}.velocity_mps.{axis}"] = _field(
                    "float", "m/s", "world", sample_phase, nullable=False
                )
                fields[f"{phase}.{body}.acceleration_mps2.{axis}"] = _field(
                    "float", "m/s^2", "world", "derived_post"
                )
            for component in "wxyz":
                fields[f"{phase}.{body}.orientation.{component}"] = _field(
                    "float", "1", "world", sample_phase, nullable=False
                )
    for channel, phase in (("accepted", "accepted_effects"), ("applied", "applied_effects")):
        for body in ("cart", "rocket"):
            for axis in "xyz":
                fields[f"{channel}.{body}.force_n.{axis}"] = _field(
                    "float", "N", "world", phase, nullable=False
                )
    for axis in "xyz":
        fields[f"geometry.tangent.{axis}"] = _field("float", "1", "world", "derived_post")
        fields[f"geometry.normal.{axis}"] = _field("float", "1", "world", "derived_post")
    return fields


FIELDS_ADDED_AFTER_V1 = frozenset({"energy.work_guide_reaction_j"})
"""Every column introduced after ``core_telemetry_v1`` was published.

A published version must be defined by what it *contained*, not by what the current field
set happens to be minus a hole. Deriving v1 as "v2 without one field" looks equivalent and
is not: the next column added to ``_core_fields`` would silently appear in v1 as well, and
a v1 run replayed against it would validate against a column set that never existed --
exactly the confusion the versioned sidecar exists to prevent. Adding a column therefore
means adding its name here.
"""

CORE_TELEMETRY_V1_FIELD_DIGEST = (
    "f08cda445caa6f6324c9bf05048d4225b5981ee14671a3a27f81819d83064245"
)
"""SHA-256 over v1's sorted column names, newline-joined.

The exclusion set above says which columns v1 lacks; this says what v1 *is*. Together they
turn "someone edited ``_core_fields`` and forgot v1" from a silent schema mutation into an
import-time failure.
"""


def _field_name_digest(names: Iterable[str]) -> str:
    return hashlib.sha256("\n".join(sorted(names)).encode("utf-8")).hexdigest()


_CORE_FIELDS_V1 = {
    name: metadata
    for name, metadata in _core_fields().items()
    if name not in FIELDS_ADDED_AFTER_V1
}
_ACTUAL_V1_DIGEST = _field_name_digest(_CORE_FIELDS_V1)
if _ACTUAL_V1_DIGEST != CORE_TELEMETRY_V1_FIELD_DIGEST:
    raise TelemetrySchemaError(
        "core_telemetry_v1 no longer has the field set it was published with "
        f"(digest {_ACTUAL_V1_DIGEST}, expected {CORE_TELEMETRY_V1_FIELD_DIGEST}). "
        "A column was added to or removed from the core schema without deciding whether "
        "v1 ever had it; add new names to FIELDS_ADDED_AFTER_V1, or repin this digest only "
        "if v1 itself is genuinely being redefined."
    )

CORE_TELEMETRY_SCHEMA_V1 = TelemetrySchema(
    version="core_telemetry_v1", fields=_CORE_FIELDS_V1
)
"""Frozen original core field set for readers and replay of existing V1 runs."""


CORE_TELEMETRY_SCHEMA_V2 = TelemetrySchema(version="core_telemetry_v2", fields=_core_fields())
"""Adds ``energy.work_guide_reaction_j`` to ``core_telemetry_v1``.

The addition is backward compatible for a reader that selects columns by name, but the
version is bumped anyway: section 14 requires the sidecar to identify the exact field set,
and a run recorded before this column existed must not be mistaken for one whose guide
reaction did no work. ``CORE_TELEMETRY_SCHEMA_V1`` remains a distinct frozen schema rather
than an alias, so compatibility never depends on a misleading constant.
"""
