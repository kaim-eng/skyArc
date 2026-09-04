# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Experiment provenance, policies, pairing, and contrasts."""

from .contrasts import ConditionResult, ContrastError, PairedContrast, factor_diff, paired_contrasts
from .criteria import (
    BASELINE_V1,
    CURVED_REFERENCE_V1,
    CRITERION_POLICIES,
    CriterionError,
    CriterionPolicy,
    CriterionResult,
    CriterionRule,
    EvidenceWindow,
    get_criterion_policy,
    resolve_evidence_window,
)
from .hashing import (
    EXCLUDED_CLOSURE_DIRECTORIES,
    CodeIdentity,
    HashingError,
    builtin_package_code_identity,
    canonical_json,
    canonical_value,
    hash_dependency_closure,
    interpreter_version,
    sha256_value,
)
from .manifest import (
    ComponentProvenance,
    ExperimentManifest,
    ManifestError,
    NumericalProvenance,
    SchemaProvenance,
    SoftwareProvenance,
    build_manifest,
    sha256_file,
)
from .random_streams import NamedRandomStreams, RandomStreamError

__all__ = [
    "BASELINE_V1",
    "CURVED_REFERENCE_V1",
    "CRITERION_POLICIES",
    "EXCLUDED_CLOSURE_DIRECTORIES",
    "CodeIdentity",
    "ComponentProvenance",
    "ConditionResult",
    "ContrastError",
    "CriterionError",
    "CriterionPolicy",
    "CriterionResult",
    "CriterionRule",
    "EvidenceWindow",
    "ExperimentManifest",
    "HashingError",
    "ManifestError",
    "interpreter_version",
    "NamedRandomStreams",
    "NumericalProvenance",
    "PairedContrast",
    "RandomStreamError",
    "SchemaProvenance",
    "SoftwareProvenance",
    "build_manifest",
    "builtin_package_code_identity",
    "canonical_json",
    "canonical_value",
    "factor_diff",
    "get_criterion_policy",
    "hash_dependency_closure",
    "paired_contrasts",
    "resolve_evidence_window",
    "sha256_file",
    "sha256_value",
]
