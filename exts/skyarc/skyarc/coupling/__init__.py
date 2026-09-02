# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Coupling, release, separation, and ignition-interlock models."""

from .coupling import FixedJointCoupling
from .interlock import IgnitionInterlock, InterlockDecision
from .release import ReleaseError, ReleasePhase, ReleaseTransaction
from .separation import (
    NoneSeparationActuator,
    SeparationMeasurement,
    SeparationMonitor,
    SeparationStatus,
    measure_separation,
)

__all__ = [
    "FixedJointCoupling",
    "IgnitionInterlock",
    "InterlockDecision",
    "NoneSeparationActuator",
    "ReleaseError",
    "ReleasePhase",
    "ReleaseTransaction",
    "SeparationMeasurement",
    "SeparationMonitor",
    "SeparationStatus",
    "measure_separation",
]
