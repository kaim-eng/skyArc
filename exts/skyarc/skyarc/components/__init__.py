# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Backend-neutral component lifecycle and diagnostic contracts."""

from .contract import Component, ComponentDescriptor, Determinism, ScenarioContext, StepOutput
from .diagnostics import (
    DEFAULT_RESERVED_KEYS,
    DiagnosticError,
    DiagnosticField,
    DiagnosticRecord,
    DiagnosticSchema,
)
from .observers import GroundTruthObserver
from .registry import ComponentRegistry, ModelRegistration, RegistryError

__all__ = [
    "Component",
    "ComponentRegistry",
    "ComponentDescriptor",
    "DEFAULT_RESERVED_KEYS",
    "Determinism",
    "DiagnosticError",
    "DiagnosticField",
    "DiagnosticRecord",
    "DiagnosticSchema",
    "GroundTruthObserver",
    "ModelRegistration",
    "RegistryError",
    "ScenarioContext",
    "StepOutput",
]
