# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Backend-neutral rocket propulsion and free-flight aerodynamics."""

from .aerodynamics import QuadraticPointDrag
from .ignition import IgnitionTriggerDecision, TrajectoryIgnitionTrigger
from .motor import ConstantMassThrustMotor

__all__ = [
    "ConstantMassThrustMotor",
    "IgnitionTriggerDecision",
    "QuadraticPointDrag",
    "TrajectoryIgnitionTrigger",
]
