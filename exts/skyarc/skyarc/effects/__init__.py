# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Typed backend-neutral effects, their validation, aggregation, and the backend adapter.

Models return effects; only the adapter applies them. Nothing in this package below
``backends.isaac`` imports Isaac Sim.
"""

from .adapter import AppliedEffects, BackendAdapter, BackendCapabilities
from .aggregator import AggregatedEffects, BodyLoad, aggregate, axial_force, axial_slot_force
from .types import (
    CollisionAction,
    CollisionPairCommand,
    ConstraintAction,
    ConstraintCommand,
    EffectBatch,
    Frame,
    MassUpdate,
    MomentumPolicy,
    Wrench,
)
from .validation import EffectValidationError, validate_batch

__all__ = [
    "AppliedEffects",
    "AggregatedEffects",
    "BackendAdapter",
    "BackendCapabilities",
    "BodyLoad",
    "CollisionAction",
    "CollisionPairCommand",
    "ConstraintAction",
    "ConstraintCommand",
    "EffectBatch",
    "EffectValidationError",
    "Frame",
    "MassUpdate",
    "MomentumPolicy",
    "Wrench",
    "aggregate",
    "axial_force",
    "axial_slot_force",
    "validate_batch",
]
