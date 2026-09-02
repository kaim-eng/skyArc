# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
import unittest

import _bootstrap  # noqa: F401

from skyarc.events import EVENT_STAGE_TRANSITION, Event, EventError


class EventContractTests(unittest.TestCase):
    def test_event_payload_is_a_bounded_immutable_copy(self) -> None:
        source = {"stage": 2, "samples": [1.0, 2.0], "metadata": {"valid": True}}
        event = Event(
            name=EVENT_STAGE_TRANSITION,
            time_s=1.5,
            step_index=360,
            source="state_machine",
            data=source,
        )
        source["samples"][0] = 99.0
        self.assertEqual(event.data["samples"], (1.0, 2.0))
        self.assertEqual(event.with_sequence(7).sequence, 7)
        with self.assertRaises(TypeError):
            event.data["stage"] = 3  # type: ignore[index]
        with self.assertRaises(TypeError):
            event.data["metadata"]["valid"] = False  # type: ignore[index]

    def test_invalid_event_metadata_and_payload_are_rejected(self) -> None:
        common = {
            "name": EVENT_STAGE_TRANSITION,
            "time_s": 0.0,
            "step_index": 0,
            "source": "state_machine",
        }
        for changes in (
            {"time_s": math.nan},
            {"step_index": -1},
            {"source": " "},
            {"sequence": -2},
            {"data": {"value": math.inf}},
            {"data": {"value": object()}},
            {"data": {1: "non-string key"}},
        ):
            with self.subTest(changes=changes):
                with self.assertRaises(EventError):
                    Event(**(common | changes))


if __name__ == "__main__":
    unittest.main()
