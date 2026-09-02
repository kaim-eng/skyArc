# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
import unittest

import _bootstrap  # noqa: F401

from skyarc.components.diagnostics import (
    DiagnosticError,
    DiagnosticField,
    DiagnosticRecord,
    DiagnosticSchema,
)


class DiagnosticContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.schema = DiagnosticSchema(
            namespace="electromagnetic",
            version="1",
            fields={
                "electromagnetic.saturated": DiagnosticField(unit="1"),
                "electromagnetic.force_components": DiagnosticField(unit="N", shape=(3,)),
                "electromagnetic.mode": DiagnosticField(unit="1"),
            },
            maximum_scalar_values=8,
            maximum_string_length=16,
        )

    def test_valid_record_is_immutable_and_arrays_are_frozen(self) -> None:
        source = [1.0, 2.0, 3.0]
        record = DiagnosticRecord.create(
            source="launch_force",
            schema=self.schema,
            values={
                "electromagnetic.saturated": False,
                "electromagnetic.force_components": source,
                "electromagnetic.mode": "target_speed",
            },
        )
        source[0] = 99.0
        self.assertEqual(record.values["electromagnetic.force_components"], (1.0, 2.0, 3.0))
        with self.assertRaises(TypeError):
            record.values["new"] = 1  # type: ignore[index]

    def test_unknown_unregistered_and_wrong_shape_values_are_rejected(self) -> None:
        with self.assertRaises(DiagnosticError):
            self.schema.validate({"electromagnetic.unregistered": 1.0})
        with self.assertRaises(DiagnosticError):
            self.schema.validate({"electromagnetic.force_components": [1.0, 2.0]})
        with self.assertRaises(DiagnosticError):
            self.schema.validate({"electromagnetic.force_components": [[1.0], [2.0], [3.0]]})

    def test_non_serializable_non_finite_and_oversized_values_are_rejected(self) -> None:
        with self.assertRaises(DiagnosticError):
            self.schema.validate({"electromagnetic.mode": object()})
        with self.assertRaises(DiagnosticError):
            self.schema.validate({"electromagnetic.saturated": math.nan})
        with self.assertRaises(DiagnosticError):
            self.schema.validate({"electromagnetic.mode": "far-too-long-for-schema"})

    def test_reserved_collision_and_bad_namespace_are_rejected(self) -> None:
        with self.assertRaises(DiagnosticError):
            DiagnosticSchema(
                namespace="electromagnetic",
                version="1",
                fields={"electromagnetic.time_s": DiagnosticField(unit="s")},
            )
        with self.assertRaises(DiagnosticError):
            DiagnosticSchema(namespace="bad.namespace", version="1", fields={})


if __name__ == "__main__":
    unittest.main()
