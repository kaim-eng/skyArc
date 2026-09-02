# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Common backend-neutral lifecycle for replaceable component models."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from types import MappingProxyType
from typing import Any, Mapping, Tuple

from ..effects.types import EffectBatch
from ..events import Event
from ..names import ALL_SLOTS
from ..state import MarkerSpec, Observation, SimulationState
from .diagnostics import DiagnosticRecord


def _deep_freeze(value: Any) -> Any:
    """Recursively freeze common configuration containers without importing a schema."""
    if isinstance(value, Mapping):
        return MappingProxyType({key: _deep_freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_deep_freeze(item) for item in value)
    if isinstance(value, (set, frozenset)):
        return frozenset(_deep_freeze(item) for item in value)
    return value


class Determinism(str, Enum):
    """A component's reproducibility claim for the experiment manifest."""

    DETERMINISTIC = "deterministic"
    SEEDED = "seeded"
    NONDETERMINISTIC = "nondeterministic"


@dataclass(frozen=True)
class ComponentDescriptor:
    """Stable identity and backend requirements for one selected model."""

    slot: str
    model_id: str
    model_version: str
    parameter_schema_version: str
    code_hash: str
    determinism: Determinism = Determinism.DETERMINISTIC
    required_backend_capabilities: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if self.slot not in ALL_SLOTS:
            raise ValueError(f"unknown component slot {self.slot!r}")
        for label, value in (
            ("model_id", self.model_id),
            ("model_version", self.model_version),
            ("parameter_schema_version", self.parameter_schema_version),
            ("code_hash", self.code_hash),
        ):
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"component {label} may not be empty")
        if not isinstance(self.determinism, Determinism):
            raise ValueError(f"invalid determinism claim {self.determinism!r}")
        if any(not isinstance(name, str) or not name for name in self.required_backend_capabilities):
            raise ValueError("required backend capability names must be non-empty strings")
        if len(set(self.required_backend_capabilities)) != len(self.required_backend_capabilities):
            raise ValueError("required backend capabilities may not contain duplicates")


@dataclass(frozen=True)
class ScenarioContext:
    """Immutable run context supplied once during component preparation."""

    scenario_id: str
    parameters: Mapping[str, Any] = field(default_factory=dict)
    markers: Mapping[str, MarkerSpec] = field(default_factory=dict)
    backend_capabilities: Mapping[str, bool] = field(default_factory=dict)
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.scenario_id:
            raise ValueError("scenario_id may not be empty")
        object.__setattr__(self, "parameters", _deep_freeze(self.parameters))
        object.__setattr__(self, "markers", MappingProxyType(dict(self.markers)))
        object.__setattr__(self, "backend_capabilities", MappingProxyType(dict(self.backend_capabilities)))
        object.__setattr__(self, "metadata", _deep_freeze(self.metadata))


@dataclass(frozen=True)
class StepOutput:
    """Physical effects and non-physical records produced during one lifecycle call."""

    effects: EffectBatch
    events: Tuple[Event, ...] = ()
    diagnostics: Tuple[DiagnosticRecord, ...] = ()

    @classmethod
    def empty(cls, source: str) -> "StepOutput":
        return cls(effects=EffectBatch.empty(source))


class Component(ABC):
    """Six-method lifecycle implemented by every replaceable component.

    Components never receive a backend object and cannot mutate the scene. Pre-step logic
    sees only the selected observation. Post-step logic may inspect the immutable latent
    state for diagnostics and event detection, but any physical request still has to travel
    through the returned :class:`StepOutput` and the adapter.
    """

    @property
    @abstractmethod
    def descriptor(self) -> ComponentDescriptor:
        """Identity, version, determinism, and capability requirements."""

    @abstractmethod
    def prepare(self, context: ScenarioContext) -> None:
        """Validate parameters and allocate bounded model-owned state."""

    @abstractmethod
    def reset(self, initial_state: SimulationState) -> None:
        """Restore model-owned state for a new execution."""

    @abstractmethod
    def pre_step(self, observation: Observation) -> StepOutput:
        """Return effects and records computed before integration."""

    @abstractmethod
    def post_step(self, state: SimulationState) -> StepOutput:
        """Return post-integration events and diagnostics."""

    @abstractmethod
    def snapshot_state(self) -> Mapping[str, Any]:
        """Return a bounded JSON-safe snapshot of model-owned state."""

    def validate_output(self, output: StepOutput) -> None:
        """Check that a lifecycle result truthfully identifies this component's slot."""
        if output.effects.source != self.descriptor.slot:
            raise ValueError(
                f"component in slot {self.descriptor.slot!r} returned effects from "
                f"{output.effects.source!r}"
            )
        if any(record.source != self.descriptor.slot for record in output.diagnostics):
            raise ValueError("component returned a diagnostic record attributed to another slot")
