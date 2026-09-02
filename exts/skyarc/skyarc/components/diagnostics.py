# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Bounded, schema-described component diagnostics.

Diagnostic values are deliberately much narrower than arbitrary JSON. A component may
emit finite scalar booleans, numbers, or strings, and fixed-shape arrays of those values.
Every key is registered with a unit and shape before a run starts. This preserves the
extensibility requested in section 9.2 without admitting tensors, callbacks, histories, or
ambiguous ad-hoc fields into telemetry.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping, Sequence, Tuple


DEFAULT_RESERVED_KEYS = frozenset(
    {
        "time_s",
        "step_index",
        "sequence",
        "source",
        "model_id",
        "state",
        "valid",
    }
)


class DiagnosticError(ValueError):
    """Raised when diagnostic data violates its registered schema."""


@dataclass(frozen=True)
class DiagnosticField:
    """Metadata for one fully-qualified diagnostic key.

    ``shape=()`` declares a scalar. Array dimensions are fixed positive integers. The
    total number of scalar leaves is additionally bounded by the owning schema.
    """

    unit: str
    shape: Tuple[int, ...] = ()
    description: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.unit, str) or not self.unit.strip():
            raise DiagnosticError("a diagnostic field must declare a non-empty unit")
        if any(not isinstance(size, int) or isinstance(size, bool) or size <= 0 for size in self.shape):
            raise DiagnosticError(f"diagnostic shape must contain positive integers, got {self.shape!r}")


def _shape_and_count(value: Any, *, max_string_length: int) -> Tuple[Tuple[int, ...], int]:
    if isinstance(value, bool):
        return (), 1
    if isinstance(value, int):
        return (), 1
    if isinstance(value, float):
        if not math.isfinite(value):
            raise DiagnosticError("diagnostic numbers must be finite")
        return (), 1
    if isinstance(value, str):
        if len(value) > max_string_length:
            raise DiagnosticError(
                f"diagnostic string has {len(value)} characters; limit is {max_string_length}"
            )
        return (), 1
    if isinstance(value, (list, tuple)):
        if not value:
            raise DiagnosticError("diagnostic arrays may not be empty; their shape would be ambiguous")
        child_shapes = []
        leaves = 0
        for child in value:
            child_shape, child_leaves = _shape_and_count(child, max_string_length=max_string_length)
            child_shapes.append(child_shape)
            leaves += child_leaves
        if any(shape != child_shapes[0] for shape in child_shapes[1:]):
            raise DiagnosticError("diagnostic arrays must be rectangular")
        return (len(value),) + child_shapes[0], leaves
    raise DiagnosticError(
        "diagnostic values must be booleans, finite numbers, strings, or bounded arrays of those types; "
        f"got {type(value).__name__}"
    )


def _freeze_value(value: Any) -> Any:
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_value(child) for child in value)
    return value


@dataclass(frozen=True)
class DiagnosticSchema:
    """Registered metadata and bounds for one component's diagnostics."""

    namespace: str
    version: str
    fields: Mapping[str, DiagnosticField]
    reserved_keys: frozenset[str] = DEFAULT_RESERVED_KEYS
    maximum_fields: int = 64
    maximum_scalar_values: int = 256
    maximum_string_length: int = 256

    def __post_init__(self) -> None:
        if not self.namespace or "." in self.namespace or not self.namespace.isidentifier():
            raise DiagnosticError(
                f"diagnostic namespace must be one identifier without dots, got {self.namespace!r}"
            )
        if not self.version:
            raise DiagnosticError("diagnostic schema version may not be empty")
        if self.maximum_fields <= 0 or self.maximum_scalar_values <= 0 or self.maximum_string_length <= 0:
            raise DiagnosticError("diagnostic schema bounds must be positive")
        if len(self.fields) > self.maximum_fields:
            raise DiagnosticError(
                f"schema registers {len(self.fields)} fields; limit is {self.maximum_fields}"
            )
        prefix = self.namespace + "."
        copied = dict(self.fields)
        for key, metadata in copied.items():
            if not isinstance(key, str) or not key.startswith(prefix) or key == prefix:
                raise DiagnosticError(f"diagnostic key {key!r} must begin with {prefix!r}")
            leaf_name = key[len(prefix) :]
            if leaf_name in self.reserved_keys or key in self.reserved_keys:
                raise DiagnosticError(f"diagnostic key {key!r} collides with a reserved core key")
            if not isinstance(metadata, DiagnosticField):
                raise DiagnosticError(f"metadata for {key!r} is not a DiagnosticField")
        object.__setattr__(self, "fields", MappingProxyType(copied))

    def validate(self, values: Mapping[str, Any]) -> Mapping[str, Any]:
        """Validate and immutably copy one diagnostic record."""
        if not isinstance(values, Mapping):
            raise DiagnosticError("diagnostics must be a mapping")
        if len(values) > self.maximum_fields:
            raise DiagnosticError(f"record has {len(values)} fields; limit is {self.maximum_fields}")
        unknown = sorted(set(values) - set(self.fields))
        if unknown:
            raise DiagnosticError(f"diagnostic keys are not registered in schema {self.version!r}: {unknown}")

        frozen = {}
        total_leaves = 0
        for key in sorted(values):
            if key in self.reserved_keys or key.rsplit(".", 1)[-1] in self.reserved_keys:
                raise DiagnosticError(f"diagnostic key {key!r} collides with a reserved core key")
            shape, leaves = _shape_and_count(
                values[key], max_string_length=self.maximum_string_length
            )
            expected = self.fields[key].shape
            if shape != expected:
                raise DiagnosticError(
                    f"diagnostic {key!r} has shape {shape}, but schema declares {expected}"
                )
            total_leaves += leaves
            frozen[key] = _freeze_value(values[key])
        if total_leaves > self.maximum_scalar_values:
            raise DiagnosticError(
                f"record contains {total_leaves} scalar values; limit is {self.maximum_scalar_values}"
            )
        return MappingProxyType(frozen)


@dataclass(frozen=True)
class DiagnosticRecord:
    """One validated diagnostic record from one component."""

    source: str
    schema_version: str
    values: Mapping[str, Any] = field(default_factory=dict)

    @classmethod
    def create(
        cls,
        *,
        source: str,
        schema: DiagnosticSchema,
        values: Mapping[str, Any],
    ) -> "DiagnosticRecord":
        if not source:
            raise DiagnosticError("diagnostic source may not be empty")
        return cls(source=source, schema_version=schema.version, values=schema.validate(values))
