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

"""Namespaced, bounded, JSON-serializable simulation events.

Section 5.1 requires events to be namespaced, bounded and schema-described; section 14
requires them to carry monotonic sequence numbers. The sequence number is assigned by the
recorder rather than by the producer, because only the recorder sees the global order.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping

# Event names are a closed set. Section 11 rejects runtime-generated telemetry schemas, and
# the same reasoning applies to event names: dashboards and queries have to stay stable
# across configurations with different stage counts.
EVENT_STAGE_TRANSITION = "stage_transition"
EVENT_STATE_TRANSITION = "state_transition"
EVENT_EXIT_PLANE_CROSSED = "exit_plane_crossed"
EVENT_FORCE_RAMP_DOWN = "force_ramp_down"
EVENT_RELEASE_STEP = "release_step"
EVENT_RELEASE_CONFIRMED = "release_confirmed"
EVENT_BRAKE_ELIGIBLE = "brake_eligible"
EVENT_CART_STOPPED = "cart_stopped"
EVENT_SEPARATION_CONFIRMED = "separation_confirmed"
EVENT_IGNITION = "ignition"
EVENT_BURNOUT = "burnout"
EVENT_FLIGHT_WINDOW_COMPLETE = "flight_window_complete"
EVENT_ABORT = "abort"
EVENT_RECONTACT = "recontact"
EVENT_INTERLOCK_BLOCKED = "interlock_blocked"

EVENT_NAMES = frozenset(
    {
        EVENT_STAGE_TRANSITION,
        EVENT_STATE_TRANSITION,
        EVENT_EXIT_PLANE_CROSSED,
        EVENT_FORCE_RAMP_DOWN,
        EVENT_RELEASE_STEP,
        EVENT_RELEASE_CONFIRMED,
        EVENT_BRAKE_ELIGIBLE,
        EVENT_CART_STOPPED,
        EVENT_SEPARATION_CONFIRMED,
        EVENT_IGNITION,
        EVENT_BURNOUT,
        EVENT_FLIGHT_WINDOW_COMPLETE,
        EVENT_ABORT,
        EVENT_RECONTACT,
        EVENT_INTERLOCK_BLOCKED,
    }
)


class EventError(ValueError):
    """Raised when an event violates the naming or payload contract."""


MAX_EVENT_FIELDS = 64
MAX_EVENT_NODES = 256
MAX_EVENT_STRING_LENGTH = 256
MAX_EVENT_NESTING_DEPTH = 8


def _freeze_json_value(value: Any, *, depth: int = 0) -> tuple[Any, int]:
    """Validate, bound, and recursively freeze one JSON-safe event value."""
    if depth > MAX_EVENT_NESTING_DEPTH:
        raise EventError(f"event payload nesting exceeds {MAX_EVENT_NESTING_DEPTH} levels")
    if value is None or isinstance(value, bool) or isinstance(value, int):
        return value, 1
    if isinstance(value, float):
        if not math.isfinite(value):
            raise EventError("event payload numbers must be finite")
        return value, 1
    if isinstance(value, str):
        if len(value) > MAX_EVENT_STRING_LENGTH:
            raise EventError(
                f"event payload string has {len(value)} characters; limit is {MAX_EVENT_STRING_LENGTH}"
            )
        return value, 1
    if isinstance(value, (list, tuple)):
        frozen = []
        nodes = 1
        for item in value:
            frozen_item, child_nodes = _freeze_json_value(item, depth=depth + 1)
            frozen.append(frozen_item)
            nodes += child_nodes
        return tuple(frozen), nodes
    if isinstance(value, Mapping):
        frozen_mapping = {}
        nodes = 1
        for key, item in value.items():
            if not isinstance(key, str) or not key:
                raise EventError("event payload mapping keys must be non-empty strings")
            if len(key) > MAX_EVENT_STRING_LENGTH:
                raise EventError(
                    f"event payload key has {len(key)} characters; limit is {MAX_EVENT_STRING_LENGTH}"
                )
            frozen_item, child_nodes = _freeze_json_value(item, depth=depth + 1)
            frozen_mapping[key] = frozen_item
            nodes += 1 + child_nodes
        return MappingProxyType(frozen_mapping), nodes
    raise EventError(
        "event payload values must be null, booleans, finite numbers, strings, arrays, "
        f"or string-keyed mappings; got {type(value).__name__}"
    )


@dataclass(frozen=True)
class Event:
    """One recorded occurrence.

    Attributes:
        name: One of :data:`EVENT_NAMES`.
        time_s: Simulation time of the occurrence. For a boundary crossing this is the
            interpolated crossing time (section 7), not the end of the step that detected it.
        step_index: Physics step during which the event was detected.
        source: Slot or subsystem that produced the event.
        data: JSON-safe payload validated against the diagnostic value rules.
        sequence: Monotonic sequence number, assigned by the recorder.
    """

    name: str
    time_s: float
    step_index: int
    source: str
    data: Mapping[str, Any] = field(default_factory=dict)
    sequence: int = -1

    def __post_init__(self) -> None:
        if self.name not in EVENT_NAMES:
            raise EventError(f"unknown event name '{self.name}'; events are a closed set")
        if isinstance(self.time_s, bool) or not isinstance(self.time_s, (int, float)) or not math.isfinite(self.time_s):
            raise EventError("event time_s must be a finite number")
        if isinstance(self.step_index, bool) or not isinstance(self.step_index, int) or self.step_index < 0:
            raise EventError("event step_index must be a nonnegative integer")
        if not isinstance(self.source, str) or not self.source.strip():
            raise EventError("event source must be a non-empty string")
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int) or self.sequence < -1:
            raise EventError("event sequence must be -1 (unassigned) or a nonnegative integer")
        if not isinstance(self.data, Mapping):
            raise EventError("event data must be a mapping")
        if len(self.data) > MAX_EVENT_FIELDS:
            raise EventError(f"event has {len(self.data)} fields; limit is {MAX_EVENT_FIELDS}")
        frozen_data, nodes = _freeze_json_value(self.data)
        if nodes > MAX_EVENT_NODES:
            raise EventError(f"event payload contains {nodes} nodes; limit is {MAX_EVENT_NODES}")
        object.__setattr__(self, "data", frozen_data)

    def with_sequence(self, sequence: int) -> "Event":
        """Return a copy carrying the recorder-assigned monotonic sequence number."""
        return Event(
            name=self.name,
            time_s=self.time_s,
            step_index=self.step_index,
            source=self.source,
            data=self.data,
            sequence=sequence,
        )
