# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import importlib.util
import hashlib
import json
import math
import pathlib
import unittest

import _bootstrap  # noqa: F401

from skyarc.configuration import load_yaml
from skyarc.launcher.production import (
    load_production_fixture,
    validate_fixture_against_scenario,
)


PROJECT = pathlib.Path(__file__).resolve().parents[2]
RUNNER_PATH = PROJECT / "standalone" / "sweep_brake_design.py"
SPEC = importlib.util.spec_from_file_location("sweep_brake_design", RUNNER_PATH)
if SPEC is None or SPEC.loader is None:  # pragma: no cover
    raise RuntimeError("could not load brake design sweep runner")
sweep_brake_design = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(sweep_brake_design)


class BrakeDesignSweepTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.sweep = sweep_brake_design.build_sweep(
            PROJECT / "configs" / "curved_2kms.yaml"
        )

    def test_full_default_factorial_is_present(self) -> None:
        expected = (
            len(sweep_brake_design.ARCHITECTURES)
            * len(sweep_brake_design.DEFAULT_G_VALUES)
            * len(sweep_brake_design.DEFAULT_JERK_VALUES_MPS3)
        )
        self.assertEqual(self.sweep["case_count"], expected)
        self.assertEqual(len(self.sweep["cases"]), expected)

    def test_reference_case_preserves_the_committed_design(self) -> None:
        reference = self.sweep["reference_configuration_case"]
        self.assertEqual(reference["cart_mass_kg"], 250.0)
        self.assertEqual(reference["jerk_limit_mps3"], 50.0)
        self.assertEqual(reference["configured_track_m"], 25000.0)
        self.assertGreater(reference["total_track_m"], 24000.0)
        self.assertLess(reference["total_track_m"], 25000.0)

    def test_selection_is_the_first_conventional_case_below_ten_km(self) -> None:
        selected = self.sweep["recommendation"]
        self.assertEqual(selected["architecture"], "induction_plate")
        self.assertEqual(selected["brake_limit_g"], 30.0)
        self.assertEqual(selected["jerk_limit_mps3"], 300.0)
        self.assertLessEqual(selected["total_track_m"], 10000.0)
        self.assertLess(selected["cart_mass_kg"], 50.0)

    def test_cart_mass_is_independent_of_jerk_within_one_duty(self) -> None:
        matches = [
            case
            for case in self.sweep["cases"]
            if case["status"] == "feasible"
            and case["architecture"] == "induction_plate"
            and case["brake_limit_g"] == 30.0
        ]
        self.assertEqual(len(matches), len(sweep_brake_design.DEFAULT_JERK_VALUES_MPS3))
        self.assertEqual(len({case["cart_mass_kg"] for case in matches}), 1)

    def test_recovery_and_margin_are_explicit(self) -> None:
        selected = self.sweep["recommendation"]
        self.assertAlmostEqual(
            selected["total_track_m"],
            selected["active_track_m"] + selected["stop_margin_m"],
            places=9,
        )
        self.assertAlmostEqual(
            selected["assumed_grid_return_energy_j"],
            0.9 * selected["recoverable_brake_energy_j"],
            places=6,
        )
        self.assertTrue(selected["regeneration_required"])

    def test_markdown_names_the_evidence_boundary(self) -> None:
        report = sweep_brake_design.render_markdown(self.sweep)
        self.assertIn("not qualification evidence", report)
        self.assertIn("30 G", report)
        self.assertIn("300 m/s³", report)

    def test_nonfinite_factor_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            sweep_brake_design.build_sweep(
                PROJECT / "configs" / "curved_2kms.yaml",
                g_values=(math.nan,),
            )

    def test_candidate_configuration_is_bound_to_selected_case(self) -> None:
        selected = self.sweep["recommendation"]
        candidate_path = PROJECT / "configs" / "curved_2kms_brake_30g_candidate.yaml"
        fixture_path = PROJECT / "configs" / "brake_30g_induction_candidate_fixture.json"
        loaded = load_yaml(candidate_path)
        config = loaded.config
        fixture = load_production_fixture(fixture_path)
        validate_fixture_against_scenario(fixture, config)

        self.assertEqual(selected["case_id"], "induction_plate__30g__300j")
        self.assertAlmostEqual(config.cart.mass_kg, selected["cart_mass_kg"], places=12)
        self.assertAlmostEqual(
            config.cart.brake_force_limit_n,
            selected["peak_brake_force_n"],
            places=9,
        )
        self.assertEqual(config.cart.brake_jerk_limit_mps3, selected["jerk_limit_mps3"])
        self.assertEqual(config.tube.exit_brake_track_length_m, 10000.0)
        self.assertEqual(config.launch_control.maximum_resultant_load_g, 10.0)
        self.assertEqual(config.launch_control.maximum_force_n, 11800.0)
        self.assertGreaterEqual(config.cart.maximum_resultant_load_g, 30.0)
        self.assertLess(
            loaded.preflight.braking.required_distance_m,
            selected["total_track_m"],
        )
        self.assertGreater(loaded.preflight.braking.remaining_margin_m, 0.0)

        raw_fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
        self.assertAlmostEqual(
            raw_fixture["cradle"]["mass_kg"], selected["cart_mass_kg"], places=12
        )

    def test_candidate_qualification_evidence_is_bound_and_passes(self) -> None:
        evidence = PROJECT / "artifacts" / "design" / "brake_30g_qualification"
        config_path = PROJECT / "configs" / "curved_2kms_brake_30g_candidate.yaml"
        fixture_path = PROJECT / "configs" / "brake_30g_induction_candidate_fixture.json"
        config_hash = hashlib.sha256(config_path.read_bytes()).hexdigest()
        fixture_hash = hashlib.sha256(fixture_path.read_bytes()).hexdigest()
        curved_runner_hash = hashlib.sha256(
            (PROJECT / "standalone" / "qualify_curved_guide.py").read_bytes()
        ).hexdigest()
        anti_runner_hash = hashlib.sha256(
            (PROJECT / "standalone" / "qualify_anti_tunneling.py").read_bytes()
        ).hexdigest()

        curved = []
        for name, expected_dt in (
            ("curved_physx_cpu_1ms.json", 0.001),
            ("curved_physx_cpu_0p5ms.json", 0.0005),
            ("curved_physx_cpu_0p25ms.json", 0.00025),
        ):
            artifact = json.loads((evidence / name).read_text(encoding="utf-8"))
            self.assertTrue(artifact["passed"])
            self.assertEqual(artifact["requested"]["physics_dt_s"], expected_dt)
            self.assertEqual(artifact["provenance"]["configuration_sha256"], config_hash)
            self.assertEqual(artifact["provenance"]["fixture_sha256"], fixture_hash)
            self.assertEqual(artifact["provenance"]["runner_sha256"], curved_runner_hash)
            curved.append(artifact)
        for coarse, fine in zip(curved[:-1], curved[1:], strict=True):
            coarse_probe = coarse["probes"]["curved_force_guide"]
            fine_probe = fine["probes"]["curved_force_guide"]
            self.assertLessEqual(
                abs(fine_probe["final_speed_mps"] - coarse_probe["final_speed_mps"])
                / abs(coarse_probe["final_speed_mps"]),
                0.001,
            )
            self.assertLessEqual(
                abs(
                    fine_probe["peak_resultant_proper_load_g"]
                    - coarse_probe["peak_resultant_proper_load_g"]
                ),
                0.02,
            )

        for name, expected_dt, expected_ccd in (
            ("anti_tunneling_1ms_ccd_disabled.json", 0.001, False),
            ("anti_tunneling_0p5ms_ccd_disabled.json", 0.0005, False),
            ("anti_tunneling_0p25ms_ccd_disabled.json", 0.00025, False),
            ("anti_tunneling_1ms_ccd_enabled.json", 0.001, True),
        ):
            artifact = json.loads((evidence / name).read_text(encoding="utf-8"))
            self.assertTrue(artifact["passed"])
            self.assertEqual(artifact["requested"]["physics_dt_s"], expected_dt)
            self.assertEqual(artifact["requested"]["ccd_enabled"], expected_ccd)
            self.assertEqual(artifact["provenance"]["fixture_sha256"], fixture_hash)
            self.assertEqual(artifact["provenance"]["runner_sha256"], anti_runner_hash)

        mission = json.loads(
            (evidence / "mission_brake_stop_65s.json").read_text(encoding="utf-8")
        )
        self.assertTrue(mission["passed"])
        self.assertFalse(mission["completed_mission"])
        self.assertEqual(mission["requested_max_steps"], 65000)
        self.assertIn("cart_stopped", mission["event_names"])
        self.assertLess(mission["final_cart_speed_mps"], 0.001)
        self.assertLess(
            mission["telemetry_summary"]["peak_resultant_load_g"],
            load_yaml(config_path).config.cart.maximum_resultant_load_g,
        )

        events = [
            json.loads(line)
            for line in (evidence / "events.jsonl").read_text(encoding="utf-8").splitlines()
        ]
        stopped = [event for event in events if event["name"] == "cart_stopped"]
        self.assertEqual(len(stopped), 1)
        self.assertAlmostEqual(stopped[0]["data"]["track_progress_m"], 8000.0, places=3)
        self.assertLess(abs(stopped[0]["data"]["speed_mps"]), 0.001)


if __name__ == "__main__":
    unittest.main()
