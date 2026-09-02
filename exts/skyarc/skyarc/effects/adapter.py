# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Backend adapter boundary shared by the analytic and Isaac Sim implementations."""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping, Protocol, Tuple, runtime_checkable

from ..state import SimulationState
from .aggregator import AggregatedEffects, BodyLoad
from .types import CollisionPairCommand, ConstraintCommand, MassUpdate


@dataclass(frozen=True)
class BackendCapabilities:
    """Resolved backend identity and preflight-testable capabilities."""

    backend: str
    device: str
    features: Mapping[str, bool] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.backend or not self.device:
            raise ValueError("backend and device identifiers may not be empty")
        if any(not isinstance(name, str) or not isinstance(enabled, bool) for name, enabled in self.features.items()):
            raise ValueError("backend capabilities must map string names to booleans")
        object.__setattr__(self, "features", MappingProxyType(dict(self.features)))

    def supports(self, capability: str) -> bool:
        return self.features.get(capability, False)


@dataclass(frozen=True)
class AppliedEffects:
    """What an adapter reports it actually applied during one pre-step.

    This record is intentionally distinct from :class:`AggregatedEffects`; retaining both
    lets telemetry expose adapter rejection, clamping, or frame-conversion errors.
    """

    loads: Mapping[str, BodyLoad] = field(default_factory=dict)
    mass_updates: Tuple[MassUpdate, ...] = ()
    constraint_commands: Tuple[ConstraintCommand, ...] = ()
    collision_commands: Tuple[CollisionPairCommand, ...] = ()
    warnings: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "loads", MappingProxyType(dict(self.loads)))

    @classmethod
    def exactly(cls, effects: AggregatedEffects) -> "AppliedEffects":
        return cls(
            loads=MappingProxyType(dict(effects.loads)),
            mass_updates=effects.mass_updates,
            constraint_commands=effects.constraint_commands,
            collision_commands=effects.collision_commands,
        )


@runtime_checkable
class BackendAdapter(Protocol):
    """Only object permitted to translate effects into simulator mutations."""

    @property
    def capabilities(self) -> BackendCapabilities:
        """Resolved identity and capabilities used by preflight."""
        ...

    def read_state(self) -> SimulationState:
        """Read immutable latent ground truth at the current simulation instant."""
        ...

    def apply(self, effects: AggregatedEffects) -> AppliedEffects:
        """Apply one accepted pre-step effect set and report what was applied."""
        ...

    def step(self) -> None:
        """Advance exactly one configured physics step."""
        ...

    def resync(self) -> None:
        """Make runtime constraint/filter writes visible to the next integration step."""
        ...

    def reset(self) -> None:
        """Reconstruct initial state, filters, constraints, and backend views."""
        ...
