# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import hashlib
import math
import unittest
from dataclasses import replace
from pathlib import Path

import _bootstrap  # noqa: F401
import yaml

from skyarc.configuration import (
    EXECUTION_PROFILES,
    ConfigurationError,
    ExecutionProfile,
    braking_preflight,
    load_mapping,
    load_yaml,
    resolve_tube_layout,
    validate_scenario,
)
from skyarc.configuration.schema import CenterlineSegmentConfig


BASELINE = Path(__file__).resolve().parents[2] / "configs" / "baseline.yaml"
CURVED = Path(__file__).resolve().parents[2] / "configs" / "curved_2kms.yaml"


class ConfigurationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.loaded = load_yaml(BASELINE)

    def raw_baseline(self):  # type: ignore[no-untyped-def]
        return yaml.safe_load(BASELINE.read_text(encoding="utf-8"))

    def raw_curved(self):  # type: ignore[no-untyped-def]
        return yaml.safe_load(CURVED.read_text(encoding="utf-8"))

    def test_baseline_loads_hashes_and_matches_braking_review_arithmetic(self) -> None:
        loaded_again = load_yaml(BASELINE)
        self.assertEqual(self.loaded.config.schema_version, 2)
        self.assertEqual(self.loaded.resolved_sha256, loaded_again.resolved_sha256)
        # Repinned when the optional schema-v3 `stage2_constraint` field was added to
        # ScenarioConfig. The resolved hash covers the resolved *schema*, not only the
        # authored values, so adding a field is exactly the kind of change it exists to
        # make visible -- the baseline's own values are unchanged, and every preflight
        # figure below still reproduces.
        self.assertEqual(
            self.loaded.resolved_sha256,
            "d3bb80a4d3ea86eba8ff6ebd34ea83454b04542ce24dbdc06df86a238feff7be",
        )
        self.assertEqual(
            self.loaded.source_sha256,
            hashlib.sha256(BASELINE.read_bytes()).hexdigest(),
        )
        self.assertAlmostEqual(self.loaded.preflight.braking.required_distance_m, 34.1348, places=4)
        self.assertAlmostEqual(self.loaded.preflight.braking.remaining_margin_m, 0.8652, places=4)
        self.assertAlmostEqual(self.loaded.preflight.launch_distance_required_m, 62.5, places=6)
        self.assertEqual(self.loaded.preflight.launch_distance_available_m, 83.0)

    def test_unknown_root_and_nested_keys_are_rejected(self) -> None:
        raw = self.raw_baseline()
        raw["mystery"] = 1
        with self.assertRaisesRegex(ConfigurationError, "unknown keys in root"):
            load_mapping(raw)
        raw = self.raw_baseline()
        raw["cart"]["silent_typo"] = 1
        with self.assertRaisesRegex(ConfigurationError, "unknown keys in cart"):
            load_mapping(raw)

    def test_schema_v3_requires_a_valid_trajectory_ignition_trigger(self) -> None:
        raw = self.raw_curved()
        del raw["rocket"]["ignition"]["trigger"]
        with self.assertRaisesRegex(ConfigurationError, "rocket.ignition.trigger"):
            load_mapping(raw)

        raw = self.raw_curved()
        raw["rocket"]["ignition"]["trigger"] = {"model": "trajectory_thresholds_v1"}
        with self.assertRaisesRegex(ConfigurationError, "requires at least one threshold"):
            load_mapping(raw)

        raw = self.raw_curved()
        raw["rocket"]["ignition"]["trigger"] = {
            "model": "trajectory_thresholds_v1",
            "maximum_flight_path_angle_deg": 91.0,
        }
        with self.assertRaisesRegex(ConfigurationError, r"within \[-90, 90\]"):
            load_mapping(raw)

        # Schema v2 retains its established behavior and resolved identity when the
        # new trigger block is absent.
        self.assertEqual(
            self.loaded.config.rocket.ignition.trigger.model,
            "safety_gates_only_v1",
        )

    def test_force_vs_position_requires_a_strictly_ordered_table(self) -> None:
        raw = self.raw_baseline()
        raw["launch_control"]["mode"] = "force_vs_position"
        with self.assertRaisesRegex(ConfigurationError, "requires at least two table points"):
            load_mapping(raw)

        raw["launch_control"]["force_vs_position"] = [
            {"position_m": 0.0, "force_n": 12000.0},
            {"position_m": 90.0, "force_n": 12000.0},
        ]
        loaded = load_mapping(raw)
        self.assertEqual(len(loaded.config.launch_control.force_vs_position), 2)
        self.assertNotEqual(loaded.resolved_sha256, self.loaded.resolved_sha256)

        raw["launch_control"]["force_vs_position"][1]["position_m"] = 0.0
        with self.assertRaisesRegex(ConfigurationError, "strictly increasing"):
            load_mapping(raw)

        raw = self.raw_baseline()
        raw["launch_control"]["mode"] = "force_vs_position"
        raw["launch_control"]["force_vs_position"] = [
            {"position_m": 0.0, "force_n": 0.0},
            {"position_m": 90.0, "force_n": 0.0},
        ]
        with self.assertRaisesRegex(ConfigurationError, "selected launch-force mode cannot overcome"):
            load_mapping(raw)

    def test_missing_required_key_is_rejected(self) -> None:
        raw = self.raw_baseline()
        del raw["tube"]["stages"]
        with self.assertRaisesRegex(ConfigurationError, "missing required key tube.stages"):
            load_mapping(raw)

    def test_braking_shortfall_rejected_and_upward_grade_assists(self) -> None:
        config = self.loaded.config
        short_track = replace(config, tube=replace(config.tube, exit_brake_track_length_m=30.0))
        with self.assertRaisesRegex(ConfigurationError, "braking requires"):
            braking_preflight(short_track)
        upward = replace(config, tube=replace(config.tube, exit_track_grade_deg=45.0))
        self.assertLess(
            braking_preflight(upward).required_distance_m,
            braking_preflight(config).required_distance_m,
        )

    def _with_simulation(self, **changes):  # type: ignore[no-untyped-def]
        config = self.loaded.config
        return replace(config, simulation=replace(config.simulation, **changes))

    def test_ccd_backend_device_constraints_are_preflight_errors(self) -> None:
        newton = self._with_simulation(backend="newton", device="cpu", ccd_enabled=True)
        with self.assertRaisesRegex(ConfigurationError, "explicitly selected PhysX backend"):
            validate_scenario(newton)
        cuda = self._with_simulation(backend="physx", device="cuda:0", ccd_enabled=True)
        with self.assertRaisesRegex(ConfigurationError, "non-CUDA device"):
            validate_scenario(cuda)

    def test_ccd_requires_a_pinned_device_because_auto_may_resolve_to_cuda(self) -> None:
        # An unpinned device is not a lesser case of the CUDA rule: `auto` can resolve to
        # CUDA, where the target build ignores CCD with a warning instead of refusing it,
        # leaving the archived configuration claiming a setting that was never in force.
        unpinned = self._with_simulation(backend="physx", device="auto", ccd_enabled=True)
        with self.assertRaisesRegex(ConfigurationError, "explicitly pinned device"):
            validate_scenario(unpinned)
        pinned = self._with_simulation(backend="physx", device="cpu", ccd_enabled=True)
        self.assertIsNotNone(validate_scenario(pinned))

    def test_evidence_status_comes_from_the_profile_table_not_the_profile_name(self) -> None:
        for name, profile in EXECUTION_PROFILES.items():
            with self.subTest(profile=name):
                scenario = self._with_simulation(profile=name)
                if profile.is_evidence:
                    with self.assertRaisesRegex(ConfigurationError, "must pin backend and device"):
                        validate_scenario(scenario)
                    pinned = self._with_simulation(profile=name, backend="physx", device="cpu")
                    self.assertIsNotNone(validate_scenario(pinned))
                else:
                    self.assertIsNotNone(validate_scenario(scenario))

    def test_evidence_auto_rejection_is_case_and_whitespace_insensitive(self) -> None:
        for backend, device in (("AUTO", "cpu"), ("physx", " Auto ")):
            with self.subTest(backend=backend, device=device):
                scenario = self._with_simulation(
                    profile="headless_evidence_physics_only",
                    backend=backend,
                    device=device,
                )
                with self.assertRaisesRegex(ConfigurationError, "must pin backend and device"):
                    validate_scenario(scenario)

    def test_curved_evidence_must_match_the_phase0_selected_target(self) -> None:
        curved = load_yaml(CURVED).config
        evidence = replace(
            curved,
            simulation=replace(
                curved.simulation,
                profile="headless_evidence_physics_only",
                backend="physx",
                device="cpu",
            ),
        )
        self.assertIsNotNone(validate_scenario(evidence))
        for backend, device in (("newton", "cpu"), ("physx", "cuda:0")):
            with self.subTest(backend=backend, device=device):
                rejected = replace(
                    evidence,
                    simulation=replace(evidence.simulation, backend=backend, device=device),
                )
                with self.assertRaisesRegex(ConfigurationError, "Phase 0 selected target"):
                    validate_scenario(rejected)

    def test_unknown_profile_is_rejected_rather_than_treated_as_exploratory(self) -> None:
        # The previous substring rule accepted an unresolved backend for any profile whose
        # name merely lacked the word "evidence".
        unknown = self._with_simulation(profile="headless_rendered")
        with self.assertRaisesRegex(ConfigurationError, "unknown simulation.profile"):
            validate_scenario(unknown)

    def test_evidence_profiles_must_use_fixed_time_stepping(self) -> None:
        self.assertTrue(all(p.fixed_time_stepping for p in EXECUTION_PROFILES.values() if p.is_evidence))
        with self.assertRaises(ValueError):
            ExecutionProfile(
                name="bad_evidence",
                is_evidence=True,
                fixed_time_stepping=False,
                description="evidence without fixed time stepping",
            )

    def test_non_finite_numbers_are_rejected_at_parse_time(self) -> None:
        for section, field in (
            ("tube", "angle_deg"),
            ("tube", "exit_track_grade_deg"),
            ("guided_phase_aerodynamics", "axial_air_velocity_mps"),
        ):
            for value in (float("nan"), float("inf"), float("-inf")):
                with self.subTest(field=f"{section}.{field}", value=value):
                    raw = self.raw_baseline()
                    raw[section][field] = value
                    with self.assertRaisesRegex(ConfigurationError, "must be finite"):
                        load_mapping(raw)

    def test_non_finite_signed_values_reaching_preflight_directly_are_rejected(self) -> None:
        # validate_scenario and braking_preflight are exported and callable on a config
        # built in code, which does not pass through the loader's numeric parsing.
        config = self.loaded.config
        for value in (float("nan"), float("inf")):
            with self.subTest(value=value):
                nan_grade = replace(config, tube=replace(config.tube, exit_track_grade_deg=value))
                with self.assertRaisesRegex(ConfigurationError, "exit_track_grade_deg must be finite"):
                    braking_preflight(nan_grade)
                with self.assertRaisesRegex(ConfigurationError, "must be finite"):
                    validate_scenario(nan_grade)
                nan_angle = replace(config, tube=replace(config.tube, angle_deg=value))
                with self.assertRaisesRegex(ConfigurationError, "angle_deg must be finite"):
                    validate_scenario(nan_angle)
                nan_air = replace(
                    config,
                    guided_phase_aerodynamics=replace(
                        config.guided_phase_aerodynamics, axial_air_velocity_mps=value
                    ),
                )
                with self.assertRaisesRegex(ConfigurationError, "axial_air_velocity_mps must be finite"):
                    validate_scenario(nan_air)

    def test_geometry_clearance_and_marker_identity_are_preflight_errors(self) -> None:
        config = self.loaded.config
        too_wide = replace(config, cart=replace(config.cart, width_m=3.0))
        with self.assertRaisesRegex(ConfigurationError, "does not fit"):
            validate_scenario(too_wide)
        missing_marker = dict(config.markers)
        del missing_marker[config.rocket.aft_clearance_marker]
        with self.assertRaisesRegex(ConfigurationError, "aft marker"):
            validate_scenario(replace(config, markers=missing_marker))
        wrong_body = dict(config.markers)
        wrong_body["rocket_stagnation"] = replace(
            wrong_body["rocket_stagnation"], body="cart"
        )
        with self.assertRaisesRegex(ConfigurationError, "rocket_stagnation.*rocket"):
            validate_scenario(replace(config, markers=wrong_body))

        curved = load_yaml(CURVED).config
        longitudinally_oversized = replace(
            curved,
            cart=replace(curved.cart, length_m=1000.0),
        )
        with self.assertRaisesRegex(ConfigurationError, "swept-envelope.*cart.*requires radius"):
            validate_scenario(longitudinally_oversized)

    def test_tailwind_feasibility_uses_the_least_dense_stage(self) -> None:
        # A vacuum stage receives no aerodynamic tailwind assistance. Crediting it with the
        # assistance from the densest stage can make a force that cannot overcome grade pass.
        config = self.loaded.config
        tailwind = replace(
            config,
            guided_phase_aerodynamics=replace(
                config.guided_phase_aerodynamics,
                axial_air_velocity_mps=500.0,
            ),
            launch_control=replace(config.launch_control, maximum_force_n=2500.0),
        )
        with self.assertRaisesRegex(ConfigurationError, "cannot overcome grade"):
            validate_scenario(tailwind)

    def test_resolved_hash_changes_with_resolved_defaults_or_values(self) -> None:
        raw = self.raw_baseline()
        first = load_mapping(raw)
        raw["experiment"]["seed"] += 1
        second = load_mapping(raw)
        self.assertNotEqual(first.resolved_sha256, second.resolved_sha256)

    def test_curved_schema_loads_and_matches_reference_preflight(self) -> None:
        loaded = load_yaml(CURVED)
        self.assertEqual(loaded.config.schema_version, 3)
        self.assertEqual(loaded.config.tube.geometry_mode, "planar_centerline")
        self.assertEqual(len(loaded.config.tube.centerline), 4)
        layout = resolve_tube_layout(loaded.config)
        self.assertAlmostEqual(layout.length_m, 54115.9266, places=3)
        self.assertEqual(layout.stage_index(54000.0), 3)
        self.assertIsNotNone(loaded.preflight.centerline)
        report = loaded.preflight.centerline
        assert report is not None
        self.assertAlmostEqual(report.length_m, 54115.9266, places=3)
        self.assertAlmostEqual(report.exit_altitude_m, 30976.566, places=2)
        self.assertAlmostEqual(report.exit_downrange_m, 43300.225, places=2)
        self.assertAlmostEqual(report.exit_inclination_deg, 15.0, places=5)
        self.assertLess(report.peak_normal_jerk_mps3, 50.0)
        self.assertAlmostEqual(loaded.preflight.braking.required_distance_m, 23852.59, places=1)
        self.assertAlmostEqual(loaded.preflight.braking.stop_time_s, 20.92, places=2)
        self.assertAlmostEqual(loaded.preflight.minimum_run_time_s, 124.1159, places=3)
        clearance = loaded.preflight.swept_envelope
        assert clearance is not None
        self.assertEqual(clearance.limiting_body, "cart")
        self.assertAlmostEqual(clearance.minimum_vehicle_wall_clearance_m, 0.0115470, places=6)
        self.assertLess(clearance.polyline_chord_error_bound_m, 0.0051)
        atmosphere = loaded.config.tube.exterior_atmosphere
        assert atmosphere is not None and atmosphere.scale_height_m is not None
        self.assertAlmostEqual(
            atmosphere.density_ratio(atmosphere.reference_altitude_m + atmosphere.scale_height_m),
            atmosphere.reference_ratio / math.e,
        )

    def test_curved_track_and_curvature_continuity_are_enforced(self) -> None:
        raw = self.raw_curved()
        raw["tube"]["exit_track"]["inclination_deg"] = 0.0
        with self.assertRaisesRegex(ConfigurationError, "inclination must equal"):
            load_mapping(raw)

        raw = self.raw_curved()
        raw["tube"]["centerline"][1]["start_curvature_per_m"] = -1e-6
        with self.assertRaisesRegex(ConfigurationError, "curvature is discontinuous"):
            load_mapping(raw)

    def test_localized_downrange_reversal_cannot_hide_between_preflight_samples(self) -> None:
        curved = load_yaml(CURVED).config
        curvature = math.pi / 9.0
        centerline = (
            CenterlineSegmentConfig("straight", 10020.0, initial_angle_deg=0.0),
            CenterlineSegmentConfig(
                "clothoid", 1.0, start_curvature_per_m=0.0, end_curvature_per_m=curvature
            ),
            CenterlineSegmentConfig(
                "circular_arc", 9.0, radius_m=9.0 / math.pi, signed_turn_deg=180.0
            ),
            CenterlineSegmentConfig(
                "clothoid", 1.0, start_curvature_per_m=curvature, end_curvature_per_m=0.0
            ),
            CenterlineSegmentConfig(
                "clothoid", 1.0, start_curvature_per_m=0.0, end_curvature_per_m=-curvature
            ),
            CenterlineSegmentConfig(
                "circular_arc", 9.0, radius_m=9.0 / math.pi, signed_turn_deg=-180.0
            ),
            CenterlineSegmentConfig(
                "clothoid", 1.0, start_curvature_per_m=-curvature, end_curvature_per_m=0.0
            ),
            CenterlineSegmentConfig("straight", 9980.0),
        )
        total_length = 20022.0
        hostile_tube = replace(
            curved.tube,
            angle_deg=0.0,
            centerline=centerline,
            stages=(
                replace(
                    curved.tube.stages[-1],
                    length_m=total_length,
                ),
            ),
            exit_track_grade_deg=0.0,
            exit_track=replace(curved.tube.exit_track, inclination_deg=0.0),
            exterior_atmosphere=replace(
                curved.tube.exterior_atmosphere,
                reference_altitude_m=0.0,
            ),
        )
        hostile = replace(curved, tube=hostile_tube)
        with self.assertRaisesRegex(ConfigurationError, "reverses downrange direction"):
            validate_scenario(hostile)

    def test_curved_vector_load_jerk_and_numerical_gates_are_enforced(self) -> None:
        mutations = (
            (("cart", "maximum_resultant_load_g"), 9.9, "cart brake and guide-normal"),
            (("launch_control", "maximum_force_n"), 30000.0, "peak curvature"),
            (("launch_control", "maximum_normal_jerk_mps3"), 49.0, "resolved normal jerk"),
            (("guided_phase_aerodynamics", "boundary_blend_distance_m"), 10.0, "ten physics steps"),
            (("simulation", "maximum_run_time_s"), 124.0, "must be at least"),
        )
        for path, value, message in mutations:
            with self.subTest(path=path):
                raw = self.raw_curved()
                raw[path[0]][path[1]] = value
                with self.assertRaisesRegex(ConfigurationError, message):
                    load_mapping(raw)

    def test_curved_atmosphere_stage_and_release_pair_rules_are_enforced(self) -> None:
        raw = self.raw_curved()
        raw["tube"]["exterior_atmosphere"] = {
            "model": "constant_v1",
            "reference_ratio": 0.015,
            "reference_altitude_m": 30976.6,
        }
        with self.assertRaisesRegex(ConfigurationError, "altitude-dependent"):
            load_mapping(raw)

        raw = self.raw_curved()
        raw["tube"]["stages"][-1]["length_m"] += 1.0
        with self.assertRaisesRegex(ConfigurationError, "stage arc lengths"):
            load_mapping(raw)

        raw = self.raw_curved()
        raw["tube"]["anti_tunneling_pairs"][0]["test_relative_speed_mps"] = 40.0
        with self.assertRaisesRegex(ConfigurationError, "braking-relative clearance speed"):
            load_mapping(raw)

        raw = self.raw_curved()
        raw["evidence"]["atmosphere_stage_refinement_factor"] = 1
        with self.assertRaisesRegex(ConfigurationError, "must be at least 2"):
            load_mapping(raw)

    def test_schema_v2_does_not_silently_accept_schema_v3_fields(self) -> None:
        raw = self.raw_baseline()
        raw["evidence"] = {"free_flight_duration_s": 0.0}
        with self.assertRaisesRegex(ConfigurationError, "unknown keys in root"):
            load_mapping(raw)


if __name__ == "__main__":
    unittest.main()
