# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Versioned scenario schema, strict loading, hashing, and preflight validation."""

from .errors import ConfigurationError
from .loader import LoadedScenario, load_mapping, load_yaml
from .schema import EXECUTION_PROFILES, ExecutionProfile, ScenarioConfig
from .validation import (
    BrakingReport,
    CenterlineReport,
    PreflightReport,
    braking_preflight,
    resolve_centerline,
    resolve_tube_layout,
    validate_scenario,
)

__all__ = [
    "BrakingReport",
    "CenterlineReport",
    "ConfigurationError",
    "EXECUTION_PROFILES",
    "ExecutionProfile",
    "LoadedScenario",
    "PreflightReport",
    "ScenarioConfig",
    "braking_preflight",
    "load_mapping",
    "load_yaml",
    "resolve_centerline",
    "resolve_tube_layout",
    "validate_scenario",
]
