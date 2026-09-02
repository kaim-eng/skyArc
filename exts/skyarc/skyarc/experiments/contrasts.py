# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Exact factor diffs, condition lineage, and paired incremental contrasts."""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Iterable, Mapping, Tuple

from .hashing import canonical_value


class ContrastError(ValueError):
    """Raised when condition lineage or pairing is not attributable."""


def factor_diff(before: Mapping[str, Any], after: Mapping[str, Any]) -> Mapping[str, Mapping[str, Any]]:
    """Return an exact leaf-level diff with explicit addition/removal presence flags."""
    left = canonical_value(before)
    right = canonical_value(after)
    if not isinstance(left, dict) or not isinstance(right, dict):
        raise ContrastError("factor sets must be string-keyed mappings")
    changes: dict[str, Mapping[str, Any]] = {}

    def visit(path: str, old: Any, new: Any, old_present: bool, new_present: bool) -> None:
        if old_present and new_present and isinstance(old, dict) and isinstance(new, dict):
            for key in sorted(set(old) | set(new)):
                visit(
                    f"{path}.{key}" if path else key,
                    old.get(key),
                    new.get(key),
                    key in old,
                    key in new,
                )
            return
        if old_present == new_present and old == new:
            return
        changes[path] = MappingProxyType(
            {
                "before_present": old_present,
                "before": old if old_present else None,
                "after_present": new_present,
                "after": new if new_present else None,
            }
        )

    visit("", left, right, True, True)
    return MappingProxyType(changes)


@dataclass(frozen=True)
class ConditionResult:
    experiment_id: str
    condition_id: str
    parent_condition_id: str | None
    replicate_id: str | int
    factors: Mapping[str, Any]
    initial_state_sha256: str
    stream_seeds: Mapping[str, int]
    metrics: Mapping[str, float]

    def __post_init__(self) -> None:
        if not self.experiment_id or not self.condition_id:
            raise ContrastError("experiment and condition identifiers may not be empty")
        normalized_factors = canonical_value(self.factors)
        if not isinstance(normalized_factors, dict):
            raise ContrastError("condition factors must be a mapping")
        if not self.initial_state_sha256:
            raise ContrastError("initial-state hash may not be empty")
        if any(isinstance(seed, bool) or not isinstance(seed, int) for seed in self.stream_seeds.values()):
            raise ContrastError("stream seeds must be integers")
        if any(not isinstance(name, str) or not name for name in self.stream_seeds):
            raise ContrastError("stream names must be non-empty strings")
        if any(not isinstance(name, str) or not name for name in self.metrics):
            raise ContrastError("contrast metric names must be non-empty strings")
        if any(
            isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
            for value in self.metrics.values()
        ):
            raise ContrastError("contrast metrics must be finite numbers")
        object.__setattr__(self, "factors", MappingProxyType(normalized_factors))
        object.__setattr__(self, "stream_seeds", MappingProxyType(dict(self.stream_seeds)))
        object.__setattr__(self, "metrics", MappingProxyType({k: float(v) for k, v in self.metrics.items()}))


@dataclass(frozen=True)
class PairedContrast:
    schema_version: str
    kind: str
    experiment_id: str
    replicate_id: str
    reference_condition_id: str
    treatment_condition_id: str
    factor_diff: Mapping[str, Mapping[str, Any]]
    metric_delta: Mapping[str, float]

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "kind": self.kind,
            "experiment_id": self.experiment_id,
            "replicate_id": self.replicate_id,
            "reference_condition_id": self.reference_condition_id,
            "treatment_condition_id": self.treatment_condition_id,
            "factor_diff": {name: dict(change) for name, change in self.factor_diff.items()},
            "metric_delta": dict(self.metric_delta),
        }


def _pair_key(value: ConditionResult) -> tuple[str, str, str]:
    return value.experiment_id, value.condition_id, str(value.replicate_id)


def _assert_paired(reference: ConditionResult, treatment: ConditionResult) -> None:
    if reference.initial_state_sha256 != treatment.initial_state_sha256:
        raise ContrastError(
            f"conditions {reference.condition_id!r} and {treatment.condition_id!r} have different initial states"
        )
    if dict(reference.stream_seeds) != dict(treatment.stream_seeds):
        raise ContrastError(
            f"conditions {reference.condition_id!r} and {treatment.condition_id!r} do not use paired streams"
        )
    if set(reference.metrics) != set(treatment.metrics):
        raise ContrastError("paired conditions must report the same contrast metrics")


def paired_contrasts(results: Iterable[ConditionResult]) -> Tuple[PairedContrast, ...]:
    """Compute each adjacent increment and every non-baseline condition versus baseline."""
    values = tuple(results)
    by_key = {_pair_key(item): item for item in values}
    if len(by_key) != len(values):
        raise ContrastError("condition results contain duplicate experiment/condition/replicate identities")
    groups: dict[tuple[str, str], list[ConditionResult]] = {}
    for item in values:
        groups.setdefault((item.experiment_id, str(item.replicate_id)), []).append(item)
    for group, members in groups.items():
        roots = [item.condition_id for item in members if item.parent_condition_id is None]
        if len(roots) != 1:
            raise ContrastError(
                f"experiment/replicate {group!r} must have exactly one baseline root, found {roots}"
            )
    output = []
    for treatment in values:
        if treatment.parent_condition_id is None:
            continue
        parent_key = (
            treatment.experiment_id,
            treatment.parent_condition_id,
            str(treatment.replicate_id),
        )
        if parent_key not in by_key:
            raise ContrastError(
                f"condition {treatment.condition_id!r} is missing parent {treatment.parent_condition_id!r} "
                f"for replicate {treatment.replicate_id!r}"
            )
        parent = by_key[parent_key]
        chain = {treatment.condition_id}
        baseline = parent
        while baseline.parent_condition_id is not None:
            if baseline.condition_id in chain:
                raise ContrastError("condition lineage contains a cycle")
            chain.add(baseline.condition_id)
            ancestor_key = (
                baseline.experiment_id,
                baseline.parent_condition_id,
                str(baseline.replicate_id),
            )
            if ancestor_key not in by_key:
                raise ContrastError(
                    f"condition {baseline.condition_id!r} is missing parent {baseline.parent_condition_id!r}"
                )
            baseline = by_key[ancestor_key]

        references = (("incremental", parent), ("versus_baseline", baseline))
        for kind, reference in references:
            _assert_paired(reference, treatment)
            output.append(
                PairedContrast(
                    schema_version="paired_contrast_v1",
                    kind=kind,
                    experiment_id=treatment.experiment_id,
                    replicate_id=str(treatment.replicate_id),
                    reference_condition_id=reference.condition_id,
                    treatment_condition_id=treatment.condition_id,
                    factor_diff=factor_diff(reference.factors, treatment.factors),
                    metric_delta={
                        name: treatment.metrics[name] - reference.metrics[name]
                        for name in sorted(treatment.metrics)
                    },
                )
            )
    return tuple(output)
