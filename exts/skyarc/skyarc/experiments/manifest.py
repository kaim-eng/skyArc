# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Complete, versioned Section 14.1 experiment manifests."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Iterable, Mapping, Tuple

from ..components.contract import ComponentDescriptor
from ..configuration.loader import LoadedScenario
from ..configuration.schema import EXECUTION_PROFILES
from ..names import ALL_SLOTS
from ..state import SimulationState
from ..telemetry.paths import RunPaths
from ..telemetry.summary import RunSummary
from .contrasts import factor_diff
from .criteria import CriterionPolicy, EvidenceWindow, resolve_evidence_window
from .hashing import CodeIdentity, canonical_value, interpreter_version, sha256_value
from .random_streams import NamedRandomStreams


class ManifestError(ValueError):
    """Raised when a run cannot produce a complete attributable manifest."""


def _nonempty(value: str, label: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ManifestError(f"{label} may not be empty")


def _sha256(value: str, label: str) -> None:
    if len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
        raise ManifestError(f"{label} must be a lower-case SHA-256 digest")


def sha256_file(path: str | Path) -> str:
    file_path = Path(path)
    if not file_path.is_file():
        raise ManifestError(f"cannot hash missing file {file_path}")
    return hashlib.sha256(file_path.read_bytes()).hexdigest()


@dataclass(frozen=True)
class ComponentProvenance:
    slot: str
    model_id: str
    model_version: str
    parameter_schema_version: str
    code: CodeIdentity
    resolved_parameter_sha256: str
    determinism: str
    required_backend_capabilities: Tuple[str, ...]

    @classmethod
    def create(
        cls,
        descriptor: ComponentDescriptor,
        *,
        resolved_parameters: Mapping[str, Any],
        code: CodeIdentity,
    ) -> "ComponentProvenance":
        if descriptor.code_hash != code.sha256:
            raise ManifestError(
                f"descriptor code hash for slot {descriptor.slot!r} does not match its dependency closure"
            )
        return cls(
            slot=descriptor.slot,
            model_id=descriptor.model_id,
            model_version=descriptor.model_version,
            parameter_schema_version=descriptor.parameter_schema_version,
            code=code,
            resolved_parameter_sha256=sha256_value(resolved_parameters),
            determinism=descriptor.determinism.value,
            required_backend_capabilities=descriptor.required_backend_capabilities,
        )

    def __post_init__(self) -> None:
        if self.slot not in ALL_SLOTS:
            raise ManifestError(f"unknown component slot {self.slot!r}")
        for label, value in (
            ("model id", self.model_id),
            ("model version", self.model_version),
            ("parameter schema version", self.parameter_schema_version),
            ("determinism", self.determinism),
        ):
            _nonempty(value, label)
        _sha256(self.code.sha256, "component code hash")
        _sha256(self.resolved_parameter_sha256, "resolved parameter hash")

    def to_dict(self) -> dict[str, object]:
        return {
            "slot": self.slot,
            "model_id": self.model_id,
            "model_version": self.model_version,
            "parameter_schema_version": self.parameter_schema_version,
            "code": self.code.to_dict(),
            "resolved_parameter_sha256": self.resolved_parameter_sha256,
            "determinism": self.determinism,
            "required_backend_capabilities": list(self.required_backend_capabilities),
        }


@dataclass(frozen=True)
class SoftwareProvenance:
    isaac_sim_build: str
    repository_revision: str
    python_version: str = ""
    """Resolved interpreter. Defaults to the running process rather than a declared value.

    The build and repository revision are supplied by the caller and nothing can verify
    them, so the one part of the stack the process can observe directly is observed instead
    of asserted.
    """

    def __post_init__(self) -> None:
        _nonempty(self.isaac_sim_build, "Isaac Sim build")
        _nonempty(self.repository_revision, "repository revision")
        if not self.python_version.strip():
            object.__setattr__(self, "python_version", interpreter_version())

    def to_dict(self) -> dict[str, str]:
        return {
            "isaac_sim_build": self.isaac_sim_build,
            "repository_revision": self.repository_revision,
            "python_version": self.python_version,
        }


@dataclass(frozen=True)
class NumericalProvenance:
    backend: str
    device: str
    solver: str
    physics_dt_s: float
    render_dt_s: float
    substeps: int
    ccd_enabled: bool
    contact_settings: Mapping[str, Any]
    execution_profile: str
    fixed_time_stepping: bool
    aggregator_pre_step_order: int

    def __post_init__(self) -> None:
        for label, value in (("backend", self.backend), ("device", self.device), ("solver", self.solver)):
            _nonempty(value, label)
        if self.backend.strip().lower() == "auto" or self.device.strip().lower() == "auto":
            raise ManifestError("manifests require resolved backend and device identities, not 'auto'")
        if not math.isfinite(self.physics_dt_s) or self.physics_dt_s <= 0.0:
            raise ManifestError("physics timestep must be finite and positive")
        if not math.isfinite(self.render_dt_s) or self.render_dt_s <= 0.0:
            raise ManifestError("render timestep must be finite and positive")
        if self.substeps <= 0 or self.aggregator_pre_step_order < 0:
            raise ManifestError("substeps must be positive and callback order nonnegative")
        profile = EXECUTION_PROFILES.get(self.execution_profile)
        if profile is None:
            raise ManifestError(f"unknown execution profile {self.execution_profile!r}")
        if profile.fixed_time_stepping != self.fixed_time_stepping:
            raise ManifestError("fixed-time-stepping claim disagrees with the execution profile")
        normalized = canonical_value(self.contact_settings)
        if not isinstance(normalized, dict):
            raise ManifestError("contact settings must be a mapping")
        object.__setattr__(self, "contact_settings", MappingProxyType(normalized))

    def to_dict(self) -> dict[str, object]:
        return {
            "backend": self.backend,
            "device": self.device,
            "solver": self.solver,
            "physics_dt_s": self.physics_dt_s,
            "render_dt_s": self.render_dt_s,
            "substeps": self.substeps,
            "ccd_enabled": self.ccd_enabled,
            "contact_settings": dict(self.contact_settings),
            "execution_profile": self.execution_profile,
            "fixed_time_stepping": self.fixed_time_stepping,
            "aggregator_pre_step_order": self.aggregator_pre_step_order,
        }


@dataclass(frozen=True)
class SchemaProvenance:
    observation: str
    telemetry: str
    outcome: str

    def __post_init__(self) -> None:
        _nonempty(self.observation, "observation schema")
        _nonempty(self.telemetry, "telemetry schema")
        _nonempty(self.outcome, "outcome schema")

    def to_dict(self, criterion_policy: CriterionPolicy) -> dict[str, str]:
        return {
            "observation": self.observation,
            "telemetry": self.telemetry,
            "outcome": self.outcome,
            "criterion_policy": criterion_policy.version,
        }


@dataclass(frozen=True)
class ExperimentManifest:
    payload: Mapping[str, Any]

    def __post_init__(self) -> None:
        normalized = canonical_value(self.payload)
        if not isinstance(normalized, dict):
            raise ManifestError("manifest payload must be a mapping")
        object.__setattr__(self, "payload", MappingProxyType(normalized))

    @property
    def sha256(self) -> str:
        return sha256_value(self.payload)

    def to_dict(self) -> dict[str, Any]:
        return dict(canonical_value(self.payload))

    def write(self, path: str | Path) -> None:
        output = Path(path)
        if output.exists():
            raise ManifestError(f"refusing to overwrite existing manifest {output}")
        output.write_text(
            json.dumps(self.to_dict(), indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )


def build_manifest(
    *,
    paths: RunPaths,
    loaded: LoadedScenario,
    components: Iterable[ComponentProvenance],
    initial_state: SimulationState,
    scene_sha256: str,
    software: SoftwareProvenance,
    numerical: NumericalProvenance,
    random_streams: NamedRandomStreams,
    schemas: SchemaProvenance,
    criterion_policy: CriterionPolicy,
    summary: RunSummary,
    controlled_factors: Mapping[str, Any],
    parent_factors: Mapping[str, Any] | None = None,
    resolved_geometry: Mapping[str, Any] | None = None,
) -> ExperimentManifest:
    """Build a complete manifest, rejecting missing slots and false pairing claims."""
    config = loaded.config
    identity = config.experiment
    if paths.run_instance_id == "":
        raise ManifestError("run instance id may not be empty")
    expected_path_identity = (
        identity.experiment_id,
        identity.condition_id,
        str(identity.replicate_id),
        paths.run_instance_id,
    )
    actual_path_identity = (
        paths.root.parents[2].name,
        paths.root.parents[1].name,
        paths.root.parent.name,
        paths.root.name,
    )
    if actual_path_identity != expected_path_identity:
        raise ManifestError(
            f"run path identity {actual_path_identity!r} disagrees with configuration {expected_path_identity!r}"
        )
    if random_streams.master_seed != identity.seed:
        raise ManifestError("random-stream master seed disagrees with the resolved configuration")
    if criterion_policy.version != config.output.criterion_policy:
        raise ManifestError("criterion policy disagrees with the resolved configuration")
    records = tuple(components)
    by_slot = {record.slot: record for record in records}
    if len(by_slot) != len(records):
        raise ManifestError("manifest component slots may not be duplicated")
    missing = sorted(set(ALL_SLOTS) - set(by_slot))
    extra = sorted(set(by_slot) - set(ALL_SLOTS))
    if missing or extra:
        raise ManifestError(f"manifest component inventory mismatch; missing={missing}, extra={extra}")
    configured_models = {
        slot: getattr(config.models, slot)
        for slot in ALL_SLOTS
        if hasattr(config.models, slot)
    }
    model_mismatches = {
        slot: (configured, by_slot[slot].model_id)
        for slot, configured in configured_models.items()
        if by_slot[slot].model_id != configured
    }
    if model_mismatches:
        raise ManifestError(f"manifest component selection disagrees with configuration: {model_mismatches}")
    stream_names = set(random_streams.seeds)
    if stream_names != set(ALL_SLOTS):
        raise ManifestError(
            f"random stream inventory mismatch; missing={sorted(set(ALL_SLOTS) - stream_names)}, "
            f"extra={sorted(stream_names - set(ALL_SLOTS))}"
        )
    simulation = config.simulation
    numerical_mismatches = []
    for label, actual, expected in (
        ("physics_dt_s", numerical.physics_dt_s, simulation.physics_dt_s),
        ("render_dt_s", numerical.render_dt_s, simulation.render_dt_s),
        ("substeps", numerical.substeps, simulation.substeps),
        ("ccd_enabled", numerical.ccd_enabled, simulation.ccd_enabled),
        ("execution_profile", numerical.execution_profile, simulation.profile),
    ):
        if actual != expected:
            numerical_mismatches.append((label, expected, actual))
    if simulation.backend.strip().lower() != "auto" and numerical.backend != simulation.backend:
        numerical_mismatches.append(("backend", simulation.backend, numerical.backend))
    if simulation.device.strip().lower() != "auto" and numerical.device != simulation.device:
        numerical_mismatches.append(("device", simulation.device, numerical.device))
    if numerical_mismatches:
        raise ManifestError(f"numerical provenance disagrees with configuration: {numerical_mismatches}")
    if schemas.outcome != summary.schema_version:
        raise ManifestError("outcome schema identity disagrees with the run summary")
    _sha256(scene_sha256, "scene hash")
    _sha256(loaded.source_sha256, "source configuration hash")
    _sha256(loaded.resolved_sha256, "resolved configuration hash")

    declared_factors = canonical_value(controlled_factors)
    if not isinstance(declared_factors, dict):
        raise ManifestError("controlled factors must be a mapping")
    if config.schema_version == 3 and resolved_geometry is None:
        raise ManifestError("schema-version-3 manifests require the resolved centerline segment list")
    geometry = canonical_value(resolved_geometry) if resolved_geometry is not None else None
    geometry_hash = sha256_value(geometry) if geometry is not None else None
    initial_state_hash = sha256_value(initial_state)
    schema_factors = schemas.to_dict(criterion_policy)
    current_factors = {
        "declared": declared_factors,
        "components": {
            slot: {
                "model_id": by_slot[slot].model_id,
                "model_version": by_slot[slot].model_version,
                "parameter_schema_version": by_slot[slot].parameter_schema_version,
                "code_sha256": by_slot[slot].code.sha256,
                "resolved_parameter_sha256": by_slot[slot].resolved_parameter_sha256,
            }
            for slot in ALL_SLOTS
        },
        "scene_sha256": scene_sha256,
        "initial_state_sha256": initial_state_hash,
        "source_configuration_sha256": loaded.source_sha256,
        "resolved_configuration_sha256": loaded.resolved_sha256,
        "resolved_geometry_sha256": geometry_hash,
        "software": software.to_dict(),
        "numerical": numerical.to_dict(),
        "random": random_streams.to_dict(),
        "schemas": schema_factors,
        "criterion_policy_sha256": criterion_policy.sha256,
    }
    if identity.parent_condition_id is None:
        if parent_factors is not None:
            raise ManifestError("a baseline condition may not declare parent factors")
        diff: Mapping[str, Mapping[str, Any]] = MappingProxyType({})
    else:
        if parent_factors is None:
            raise ManifestError("a treatment condition requires its parent's factor set")
        diff = factor_diff(parent_factors, current_factors)
        if not diff:
            raise ManifestError("a treatment condition must differ from its parent")

    expected_window = resolve_evidence_window(config)
    evidence_window = expected_window.to_dict()
    outcome = summary.to_dict()
    criterion_result = criterion_policy.evaluate(outcome)
    payload = {
        "schema_version": "experiment_manifest_v1",
        "identity": {
            "experiment_id": identity.experiment_id,
            "condition_id": identity.condition_id,
            "replicate_id": identity.replicate_id,
            "run_instance_id": paths.run_instance_id,
        },
        "lineage": {
            "parent_condition_id": identity.parent_condition_id,
            "controlled_factors": current_factors,
            "factor_diff": {name: dict(change) for name, change in diff.items()},
        },
        "components": [by_slot[slot].to_dict() for slot in ALL_SLOTS],
        "hashes": {
            "scene_sha256": scene_sha256,
            "initial_state_sha256": initial_state_hash,
            "source_configuration_sha256": loaded.source_sha256,
            "resolved_configuration_sha256": loaded.resolved_sha256,
            "resolved_geometry_sha256": geometry_hash,
        },
        "resolved_geometry": geometry,
        "software": software.to_dict(),
        "numerical": numerical.to_dict(),
        "random": random_streams.to_dict(),
        "schemas": schema_factors,
        "criterion_policy": {
            **criterion_policy.to_dict(),
            "sha256": criterion_policy.sha256,
        },
        "outcome": {
            "termination_reason": summary.termination_reason,
            "metrics": outcome,
            "evidence_window": evidence_window,
            "criterion_result": criterion_result.to_dict(),
        },
    }
    return ExperimentManifest(payload)
