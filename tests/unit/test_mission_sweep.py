# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

import _bootstrap  # noqa: F401

from skyarc.configuration import load_yaml
from skyarc.experiments import NamedRandomStreams
from skyarc.names import ALL_SLOTS


PROJECT = Path(__file__).resolve().parents[2]
RUNNER_PATH = PROJECT / "standalone" / "sweep_mission.py"
SPEC = importlib.util.spec_from_file_location("sweep_mission", RUNNER_PATH)
if SPEC is None or SPEC.loader is None:  # pragma: no cover
    raise RuntimeError("could not load production mission sweep runner")
sweep_mission = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = sweep_mission
SPEC.loader.exec_module(sweep_mission)


class MissionSweepTests(unittest.TestCase):
    def test_plan_materializes_strict_paired_configs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            generated = sweep_mission.materialize_conditions(
                PROJECT / "configs" / "mission_sweep.yaml",
                Path(temporary),
            )
            self.assertEqual(
                [item.condition_id for item in generated],
                ["ignition_40km", "ignition_35km", "ignition_43km"],
            )
            self.assertEqual(generated[1].parent_condition_id, "ignition_40km")
            loaded = [load_yaml(item.configuration).config for item in generated]
            self.assertEqual(
                [item.rocket.ignition.trigger.minimum_altitude_m for item in loaded],
                [40_000.0, 35_000.0, 43_000.0],
            )
            self.assertEqual(len({item.experiment.seed for item in loaded}), 1)
            self.assertEqual(len({item.experiment.replicate_id for item in loaded}), 1)

    def test_manifest_aggregation_uses_paired_contrast_contract(self) -> None:
        streams = dict(NamedRandomStreams(12345, ALL_SLOTS).seeds)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifests = []
            for condition, parent, altitude, margin in (
                ("reference", None, 40_000.0, 100.0),
                ("lower", "reference", 35_000.0, 50.0),
            ):
                path = root / f"{condition}.json"
                path.write_text(
                    json.dumps(
                        {
                            "identity": {
                                "experiment_id": "sweep",
                                "condition_id": condition,
                                "replicate_id": 0,
                            },
                            "lineage": {
                                "parent_condition_id": parent,
                                "controlled_factors": {
                                    "declared": {
                                        "rocket.ignition.trigger.minimum_altitude_m": altitude
                                    }
                                },
                            },
                            "hashes": {"initial_state_sha256": "a" * 64},
                            "random": {"streams": streams},
                            "outcome": {
                                "criterion_result": {"passed": margin >= 0.0},
                                "metrics": {
                                    "stage2_margin_mps": margin,
                                    "handoff_altitude_m": altitude,
                                },
                            },
                        }
                    ),
                    encoding="utf-8",
                )
                manifests.append(path)
            aggregate = sweep_mission.aggregate_manifests(
                manifests,
                metrics=("stage2_margin_mps", "handoff_altitude_m"),
            )
        self.assertEqual(aggregate["paired_stream_seeds"], streams)
        self.assertEqual(aggregate["contrast_status"], "complete")
        self.assertEqual(
            aggregate["contrast_metrics"],
            ["stage2_margin_mps", "handoff_altitude_m"],
        )
        self.assertEqual(len(aggregate["contrasts"]), 2)
        self.assertEqual(
            aggregate["contrasts"][0]["metric_delta"]["stage2_margin_mps"],
            -50.0,
        )

    def test_manifest_aggregation_preserves_failed_conditions_with_null_metrics(self) -> None:
        streams = dict(NamedRandomStreams(12345, ALL_SLOTS).seeds)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifests = []
            for condition, parent, altitude in (
                ("reference", None, 40_000.0),
                ("lower", "reference", 35_000.0),
            ):
                path = root / f"{condition}.json"
                path.write_text(
                    json.dumps(
                        {
                            "identity": {
                                "experiment_id": "sweep",
                                "condition_id": condition,
                                "replicate_id": 0,
                            },
                            "lineage": {
                                "parent_condition_id": parent,
                                "controlled_factors": {
                                    "declared": {
                                        "rocket.ignition.trigger.minimum_altitude_m": altitude
                                    }
                                },
                            },
                            "hashes": {"initial_state_sha256": "a" * 64},
                            "random": {"streams": streams},
                            "outcome": {
                                "criterion_result": {
                                    "passed": False,
                                    "reason": "pre_handoff_abort",
                                },
                                "metrics": {
                                    "stage2_margin_mps": None,
                                    "apogee_altitude_m": 31_117.0,
                                },
                            },
                        }
                    ),
                    encoding="utf-8",
                )
                manifests.append(path)
            aggregate = sweep_mission.aggregate_manifests(
                manifests,
                metrics=("stage2_margin_mps", "apogee_altitude_m"),
            )
        self.assertEqual(aggregate["contrast_status"], "partial_missing_metrics")
        self.assertEqual(aggregate["contrast_metrics"], ["apogee_altitude_m"])
        self.assertIsNone(aggregate["conditions"][0]["metrics"]["stage2_margin_mps"])
        self.assertEqual(
            aggregate["conditions"][0]["unavailable_metrics"],
            ["stage2_margin_mps"],
        )
        self.assertEqual(len(aggregate["contrasts"]), 2)
        self.assertEqual(
            aggregate["contrasts"][0]["metric_delta"]["apogee_altitude_m"],
            0.0,
        )


if __name__ == "__main__":
    unittest.main()
