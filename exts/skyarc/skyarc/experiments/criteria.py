# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Versioned outcome policies with thresholds outside component implementations."""

from __future__ import annotations

import math
import operator
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping, Tuple

from ..configuration.schema import ScenarioConfig
from .hashing import sha256_value


class CriterionError(ValueError):
    """Raised when a criterion policy or evaluation input is invalid."""


@dataclass(frozen=True)
class EvidenceWindow:
    start_event: str
    duration_s: float
    completion_margin_s: float

    def __post_init__(self) -> None:
        if not self.start_event:
            raise CriterionError("evidence start event may not be empty")
        if not math.isfinite(self.duration_s) or self.duration_s < 0.0:
            raise CriterionError("evidence duration must be finite and nonnegative")
        if not math.isfinite(self.completion_margin_s) or self.completion_margin_s < 0.0:
            raise CriterionError("completion margin must be finite and nonnegative")

    def to_dict(self) -> dict[str, object]:
        return {
            "start_event": self.start_event,
            "duration_s": self.duration_s,
            "completion_margin_s": self.completion_margin_s,
        }


_OPERATORS = {
    "eq": operator.eq,
    "lt": operator.lt,
    "lte": operator.le,
    "gt": operator.gt,
    "gte": operator.ge,
}


@dataclass(frozen=True)
class CriterionRule:
    metric: str
    operation: str
    threshold: str | bool | int | float

    def __post_init__(self) -> None:
        if not self.metric:
            raise CriterionError("criterion metric may not be empty")
        if self.operation not in _OPERATORS:
            raise CriterionError(f"unsupported criterion operation {self.operation!r}")
        if isinstance(self.threshold, float) and not math.isfinite(self.threshold):
            raise CriterionError("criterion thresholds must be finite")

    def to_dict(self) -> dict[str, object]:
        return {
            "metric": self.metric,
            "operation": self.operation,
            "threshold": self.threshold,
        }


@dataclass(frozen=True)
class CriterionResult:
    policy_version: str
    policy_sha256: str
    passed: bool
    rules: Tuple[Mapping[str, object], ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "policy_version": self.policy_version,
            "policy_sha256": self.policy_sha256,
            "passed": self.passed,
            "rules": [dict(item) for item in self.rules],
        }


@dataclass(frozen=True)
class CriterionPolicy:
    version: str
    evidence_window: EvidenceWindow
    rules: Tuple[CriterionRule, ...]

    def __post_init__(self) -> None:
        if not self.version:
            raise CriterionError("criterion policy version may not be empty")
        metrics = [rule.metric for rule in self.rules]
        if not self.rules or len(set(metrics)) != len(metrics):
            raise CriterionError("criterion policies need at least one rule and no duplicate metrics")

    def to_dict(self) -> dict[str, object]:
        return {
            "version": self.version,
            "evidence_window": self.evidence_window.to_dict(),
            "rules": [rule.to_dict() for rule in self.rules],
        }

    @property
    def sha256(self) -> str:
        return sha256_value(self.to_dict())

    def evaluate(self, metrics: Mapping[str, Any]) -> CriterionResult:
        results = []
        for rule in self.rules:
            actual = metrics.get(rule.metric)
            valid = actual is not None
            if isinstance(actual, float) and not math.isfinite(actual):
                valid = False
            try:
                passed = bool(valid and _OPERATORS[rule.operation](actual, rule.threshold))
            except TypeError:
                valid = False
                passed = False
            results.append(
                MappingProxyType(
                    {
                        "metric": rule.metric,
                        "operation": rule.operation,
                        "threshold": rule.threshold,
                        "actual": actual if valid else None,
                        "valid": valid,
                        "passed": passed,
                    }
                )
            )
        return CriterionResult(self.version, self.sha256, all(item["passed"] for item in results), tuple(results))


BASELINE_V1 = CriterionPolicy(
    version="baseline_v1",
    evidence_window=EvidenceWindow(
        start_event="separation_confirmed",
        duration_s=0.5,
        completion_margin_s=0.0,
    ),
    rules=(
        CriterionRule("termination_reason", "eq", "complete"),
        CriterionRule("mission_phase", "eq", "complete"),
        CriterionRule("exit_speed_relative_error", "lte", 0.05),
    ),
)

CURVED_REFERENCE_V1 = CriterionPolicy(
    version="curved_reference_v1",
    evidence_window=EvidenceWindow(
        start_event="separation_confirmed",
        duration_s=66.0,
        completion_margin_s=2.0,
    ),
    rules=(
        CriterionRule("termination_reason", "eq", "complete"),
        CriterionRule("mission_phase", "eq", "complete"),
        CriterionRule("exit_speed_relative_error", "lte", 0.001),
        CriterionRule("peak_resultant_load_g", "lte", 10.0),
        CriterionRule("stage2_margin_mps", "gte", 0.0),
    ),
)

CRITERION_POLICIES: Mapping[str, CriterionPolicy] = MappingProxyType(
    {policy.version: policy for policy in (BASELINE_V1, CURVED_REFERENCE_V1)}
)


def get_criterion_policy(version: str) -> CriterionPolicy:
    try:
        return CRITERION_POLICIES[version]
    except KeyError:
        raise CriterionError(
            f"unknown criterion policy {version!r}; known policies: {sorted(CRITERION_POLICIES)}"
        ) from None


def resolve_evidence_window(config: ScenarioConfig) -> EvidenceWindow:
    if config.evidence is not None:
        return EvidenceWindow(
            start_event=config.evidence.free_flight_start_event,
            duration_s=config.evidence.free_flight_duration_s,
            completion_margin_s=config.evidence.completion_margin_s,
        )
    return get_criterion_policy(config.output.criterion_policy).evidence_window
