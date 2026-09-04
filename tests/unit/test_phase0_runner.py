# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import importlib.util
import hashlib
import json
import math
import sys
import unittest
from datetime import datetime, timezone
from pathlib import Path
from tempfile import TemporaryDirectory

import _bootstrap  # noqa: F401  (puts the pure extension package on sys.path)


PROJECT = Path(__file__).resolve().parents[2]
RUNNER = Path(__file__).resolve().parents[2] / "standalone" / "qualify_phase0.py"
SPEC = importlib.util.spec_from_file_location("qualify_phase0", RUNNER)
assert SPEC is not None and SPEC.loader is not None
qualify_phase0 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(qualify_phase0)

ANTI_TUNNELING_RUNNER = (
    Path(__file__).resolve().parents[2] / "standalone" / "qualify_anti_tunneling.py"
)
ANTI_TUNNELING_FIXTURE = (
    Path(__file__).resolve().parents[2]
    / "configs"
    / "phase0_anti_tunneling_slab_cradle.json"
)
ANTI_TUNNELING_ARTIFACTS = (
    (
        PROJECT / "artifacts" / "phase0" / "anti_tunneling"
        / "physx_slab_cradle_discrete_1ms.json",
        0.001,
        False,
    ),
    (
        PROJECT / "artifacts" / "phase0" / "anti_tunneling"
        / "physx_slab_cradle_discrete_0p5ms.json",
        0.0005,
        False,
    ),
    (
        PROJECT / "artifacts" / "phase0" / "anti_tunneling"
        / "physx_slab_cradle_discrete_0p25ms.json",
        0.00025,
        False,
    ),
    (
        PROJECT / "artifacts" / "phase0" / "anti_tunneling"
        / "physx_slab_cradle_ccd_1ms.json",
        0.001,
        True,
    ),
)
ANTI_TUNNELING_SPEC = importlib.util.spec_from_file_location(
    "qualify_anti_tunneling", ANTI_TUNNELING_RUNNER
)
assert ANTI_TUNNELING_SPEC is not None and ANTI_TUNNELING_SPEC.loader is not None
qualify_anti_tunneling = importlib.util.module_from_spec(ANTI_TUNNELING_SPEC)
ANTI_TUNNELING_SPEC.loader.exec_module(qualify_anti_tunneling)

STANDALONE = PROJECT / "standalone"
if str(STANDALONE) not in sys.path:
    # qualify_curved_guide imports helpers from qualify_anti_tunneling by module name.
    sys.path.insert(0, str(STANDALONE))

CURVED_GUIDE_RUNNER = STANDALONE / "qualify_curved_guide.py"
CURVED_GUIDE_SPEC = importlib.util.spec_from_file_location(
    "qualify_curved_guide", CURVED_GUIDE_RUNNER
)
assert CURVED_GUIDE_SPEC is not None and CURVED_GUIDE_SPEC.loader is not None
qualify_curved_guide = importlib.util.module_from_spec(CURVED_GUIDE_SPEC)
CURVED_GUIDE_SPEC.loader.exec_module(qualify_curved_guide)

CURVED_GUIDE_CONFIGURATION = PROJECT / "configs" / "curved_2kms.yaml"
CURVED_GUIDE_FIXTURE = ANTI_TUNNELING_FIXTURE
CURVED_GUIDE_SOURCE_ROOT = PROJECT / "exts" / "skyarc" / "skyarc"
CURVED_GUIDE_ARTIFACT = (
    PROJECT / "artifacts" / "phase0" / "curved_guide" / "physx_cpu_1ms_full_profile_v2.json"
)
CURVED_GUIDE_GLOBAL_CONTROL = (
    PROJECT / "artifacts" / "phase0" / "curved_guide" / "physx_cpu_1ms_global_control.json"
)
CURVED_GUIDE_REFINEMENT_ARTIFACTS = (
    (CURVED_GUIDE_ARTIFACT, 0.001),
    (
        PROJECT / "artifacts" / "phase0" / "curved_guide"
        / "physx_cpu_0p5ms_full_profile.json",
        0.0005,
    ),
    (
        PROJECT / "artifacts" / "phase0" / "curved_guide"
        / "physx_cpu_0p25ms_full_profile.json",
        0.00025,
    ),
)

# The gains are part of the result identity. The runner defaults do not reproduce the
# accepted artifact, so they are pinned here as literals rather than read back from it.
ACCEPTED_GAINS = {
    "normal_kp_per_s2": 400.0,
    "normal_kd_per_s": 40.0,
    "attitude_kp_per_s2": 2500.0,
    "attitude_kd_per_s": 100.0,
}

# Independently specified acceptance limits. Comparing an artifact's peak against the
# same artifact's acceptance block proves only that the file is self-consistent.
ACCEPTED_LIMITS = {
    "maximum_tracking_error_m": 0.05,
    "maximum_attitude_error_deg": 1.0,
    "maximum_resultant_load_g": 10.0,
    "maximum_attachment_load_g": 10.0,
    "maximum_attachment_geometry_error_m": 0.001,
    "maximum_backend_force_relative_error": 0.05,
    "maximum_reaction_correction_relative_error": 0.05,
    "maximum_attached_pair_force_n": 1e-6,
}


class Phase0RunnerContractTests(unittest.TestCase):
    def test_nonfinite_measurements_serialize_as_explicit_invalid_values(self) -> None:
        converted = qualify_phase0._json_value(
            {"nan": math.nan, "positive_infinity": math.inf, "negative_infinity": -math.inf}
        )
        rendered = json.dumps(converted, allow_nan=False)
        self.assertNotIn("NaN", rendered)
        self.assertNotIn("Infinity", rendered)
        for value in converted.values():
            self.assertEqual(value["valid"], False)
            self.assertIsNone(value["value"])
            self.assertEqual(value["reason"], "nonfinite measurement")

    def test_runtime_pass_requires_every_critical_resolved_setting(self) -> None:
        probe = {
            "switch_returned": True,
            "active_engine": "physx",
            "physics_dt_s": 0.001,
            "device": "cpu",
            "settings": {
                "/app/player/useFixedTimeStepping": True,
                "/app/runLoops/main/rateLimitEnabled": False,
            },
        }
        self.assertTrue(
            qualify_phase0._runtime_selection_passed(
                probe, requested_backend="physx", requested_dt_s=0.001
            )
        )
        for path, invalid in (
            ("/app/player/useFixedTimeStepping", False),
            ("/app/runLoops/main/rateLimitEnabled", True),
        ):
            with self.subTest(path=path):
                hostile = {**probe, "settings": {**probe["settings"], path: invalid}}
                self.assertFalse(
                    qualify_phase0._runtime_selection_passed(
                        hostile, requested_backend="physx", requested_dt_s=0.001
                    )
                )

    def test_default_artifact_path_is_unique_and_backend_scoped(self) -> None:
        started_at = datetime(2026, 8, 31, 12, 34, 56, 789, tzinfo=timezone.utc)
        first = qualify_phase0._default_output_path(RUNNER, "physx", started_at, "run-a")
        second = qualify_phase0._default_output_path(RUNNER, "physx", started_at, "run-b")
        self.assertNotEqual(first, second)
        self.assertEqual(first.parent.name, "physx")
        self.assertEqual(first.parent.parent.name, "phase0")
        self.assertTrue(first.name.endswith("_run-a.json"))

    def test_quaternion_error_accepts_antipodal_representation(self) -> None:
        self.assertEqual(
            qualify_phase0._quaternion_error(
                [-1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]
            ),
            0.0,
        )

    def test_anti_tunneling_requires_physical_impact_without_prior_traversal(self) -> None:
        self.assertTrue(
            qualify_anti_tunneling._anti_tunneling_passed(
                impact_observed=True,
                full_barrier_traversal_observed=False,
                samples_finite=True,
                speed_gate_met=True,
            )
        )
        for hostile in (
            {
                "impact_observed": False,
                "full_barrier_traversal_observed": False,
                "samples_finite": True,
            },
            {
                "impact_observed": True,
                "full_barrier_traversal_observed": True,
                "samples_finite": True,
            },
            {
                "impact_observed": True,
                "full_barrier_traversal_observed": False,
                "samples_finite": False,
            },
        ):
            with self.subTest(hostile=hostile):
                self.assertFalse(
                    qualify_anti_tunneling._anti_tunneling_passed(
                        **hostile, speed_gate_met=True
                    )
                )
        self.assertFalse(
            qualify_anti_tunneling._anti_tunneling_passed(
                impact_observed=True,
                full_barrier_traversal_observed=False,
                samples_finite=True,
                speed_gate_met=False,
            )
        )

    def test_anti_tunneling_fixture_pins_production_geometry(self) -> None:
        fixture = qualify_anti_tunneling._load_fixture(ANTI_TUNNELING_FIXTURE)
        self.assertEqual(fixture["pair_name"], "rocket_cradle")
        self.assertEqual(fixture["impact_case"], "vertical_saddle_system")
        self.assertEqual(fixture["minimum_test_relative_speed_mps"], 100.0)
        self.assertEqual(fixture["rocket"]["shape"], "cylinder")
        self.assertEqual(fixture["rocket"]["axis"], "X")
        self.assertEqual(fixture["rocket"]["length_m"], 4.0)
        self.assertEqual(fixture["rocket"]["diameter_m"], 1.0)
        self.assertEqual(fixture["cradle"]["topology"], "slab_three_saddles_v1")
        self.assertEqual(fixture["cradle"]["saddle_stations_m"], [-1.5, 0.0, 1.5])

    def test_anti_tunneling_fixture_rejects_wrong_or_impossible_geometry(self) -> None:
        healthy = json.loads(ANTI_TUNNELING_FIXTURE.read_text(encoding="utf-8"))
        hostile_cases = (
            ("rocket shape", ("rocket", "shape"), "box", "X-axis cylinder"),
            (
                "cradle topology",
                ("cradle", "topology"),
                "closed_box",
                "slab_three_saddles_v1",
            ),
            (
                "contact outside rocket",
                ("cradle", "saddle_contact_offset_m"),
                0.5,
                "outside the rocket radius",
            ),
            (
                "pad outside slab",
                ("cradle", "outer_width_m"),
                0.9,
                "exceed the slab width",
            ),
            (
                "pad outside height envelope",
                ("cradle", "saddle_pad_width_m"),
                0.4,
                "exceed the height envelope",
            ),
        )
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "fixture.json"
            for label, keys, value, message in hostile_cases:
                fixture = json.loads(json.dumps(healthy))
                fixture[keys[0]][keys[1]] = value
                path.write_text(json.dumps(fixture), encoding="utf-8")
                with self.subTest(label=label), self.assertRaisesRegex(ValueError, message):
                    qualify_anti_tunneling._load_fixture(path)

    def test_anti_tunneling_artifact_path_pins_treatment_and_timestep(self) -> None:
        started_at = datetime(2026, 8, 31, 12, 34, 56, 789, tzinfo=timezone.utc)
        discrete = qualify_anti_tunneling._default_output_path(
            ANTI_TUNNELING_RUNNER,
            started_at,
            "run-a",
            ccd_enabled=False,
            dt_s=0.001,
        )
        ccd = qualify_anti_tunneling._default_output_path(
            ANTI_TUNNELING_RUNNER,
            started_at,
            "run-b",
            ccd_enabled=True,
            dt_s=0.00025,
        )
        self.assertEqual(discrete.parent.name, "anti_tunneling")
        self.assertIn("_discrete_1000us_", discrete.name)
        self.assertIn("_ccd_250us_", ccd.name)

    def test_slab_cradle_matrix_is_current_and_passes_without_traversal(self) -> None:
        runner_hash = hashlib.sha256(ANTI_TUNNELING_RUNNER.read_bytes()).hexdigest()
        fixture_hash = hashlib.sha256(ANTI_TUNNELING_FIXTURE.read_bytes()).hexdigest()
        artifacts = []
        for path, expected_dt_s, expected_ccd in ANTI_TUNNELING_ARTIFACTS:
            with self.subTest(path=path.name):
                artifact = json.loads(
                    path.read_text(encoding="utf-8"),
                    parse_constant=lambda value: self.fail(
                        f"non-finite JSON constant in {path.name}: {value}"
                    ),
                )
                artifacts.append(artifact)
                self.assertTrue(artifact["passed"])
                self.assertEqual(artifact["provenance"]["runner_sha256"], runner_hash)
                self.assertEqual(artifact["provenance"]["fixture_sha256"], fixture_hash)
                self.assertEqual(artifact["requested"]["physics_dt_s"], expected_dt_s)
                self.assertIs(artifact["requested"]["ccd_enabled"], expected_ccd)
                self.assertEqual(artifact["requested"]["test_relative_speed_mps"], 100.0)
                self.assertEqual(artifact["probes"]["runtime_selection"]["solver_type"], "TGS")
                outcome = artifact["probes"]["rocket_cradle_pair"]
                self.assertEqual(outcome["rocket_shape"], "cylinder")
                self.assertEqual(outcome["rocket_axis"], "X")
                self.assertEqual(outcome["cradle_topology"], "slab_three_saddles_v1")
                self.assertIs(outcome["continuous_vertical_walls"], False)
                self.assertEqual(outcome["saddle_count"], 3)
                self.assertEqual(
                    outcome["acceptance"]["minimum_qualified_relative_speed_mps"],
                    100.0,
                )
                self.assertIs(
                    outcome["acceptance"]["test_speed_meets_declared_minimum"],
                    True,
                )
                self.assertTrue(outcome["impact_observed"])
                self.assertFalse(outcome["full_barrier_traversal_observed"])

        discrete_1ms, _, _, ccd_1ms = artifacts
        discrete_outcome = discrete_1ms["probes"]["rocket_cradle_pair"]
        ccd_outcome = ccd_1ms["probes"]["rocket_cradle_pair"]
        self.assertEqual(
            discrete_outcome["maximum_reported_contact_impulse_ns"],
            ccd_outcome["maximum_reported_contact_impulse_ns"],
        )
        self.assertEqual(discrete_outcome["samples"], ccd_outcome["samples"])

    def test_curved_guide_artifact_is_bound_to_the_current_runner_helper_and_config(
        self,
    ) -> None:
        artifact = json.loads(
            CURVED_GUIDE_ARTIFACT.read_text(encoding="utf-8"),
            parse_constant=lambda value: self.fail(f"non-finite JSON constant: {value}"),
        )
        provenance = artifact["provenance"]
        # Every hash is recomputed from the file it claims to identify. Reading any of
        # them back out of the artifact would make editing that file break nothing.
        for key, path in (
            ("runner_sha256", CURVED_GUIDE_RUNNER),
            ("helper_sha256", ANTI_TUNNELING_RUNNER),
            ("configuration_sha256", CURVED_GUIDE_CONFIGURATION),
            ("fixture_sha256", CURVED_GUIDE_FIXTURE),
        ):
            with self.subTest(artifact_key=key):
                self.assertEqual(
                    provenance[key], hashlib.sha256(path.read_bytes()).hexdigest()
                )
        expected_closure = qualify_curved_guide._source_closure(CURVED_GUIDE_SOURCE_ROOT)
        recorded_closure = provenance["project_source_closure"]
        self.assertEqual(recorded_closure["sha256"], expected_closure["sha256"])
        self.assertEqual(recorded_closure["files"], expected_closure["files"])
        for production_source in (
            "effects/backends/isaac.py",
            "extension.py",
            "launcher/production.py",
            "launcher/scene.py",
        ):
            with self.subTest(production_source=production_source):
                self.assertIn(production_source, recorded_closure["files"])
        self.assertEqual(artifact["requested"]["coordinate_frame"], "co_moving")
        self.assertEqual(artifact["requested"]["physics_dt_s"], 0.001)
        production_geometry = artifact["requested"]["production_geometry"]
        self.assertEqual(
            production_geometry["cradle_topology"], "slab_three_saddles_v1"
        )
        self.assertEqual(production_geometry["rocket_shape"], "cylinder")
        self.assertEqual(production_geometry["rocket_axis"], "X")
        # Evidence is produced by the Windows Isaac build while this contract suite is
        # also run from WSL. Bind the stable project-relative identity, not host syntax.
        fixture_path = production_geometry["fixture_path"].replace("\\", "/")
        self.assertTrue(
            fixture_path.endswith("/configs/phase0_anti_tunneling_slab_cradle.json"),
            fixture_path,
        )

    def test_curved_guide_artifact_records_the_gains_that_produced_it(self) -> None:
        artifact = json.loads(CURVED_GUIDE_ARTIFACT.read_text(encoding="utf-8"))
        guide = artifact["probes"]["curved_force_guide"]
        for name, expected in ACCEPTED_GAINS.items():
            with self.subTest(gain=name):
                self.assertEqual(artifact["requested"][name], expected)
                self.assertEqual(guide["controller_gains"][name], expected)

    def test_curved_guide_acceptance_limits_match_independently_pinned_values(self) -> None:
        artifact = json.loads(CURVED_GUIDE_ARTIFACT.read_text(encoding="utf-8"))
        acceptance = artifact["probes"]["curved_force_guide"]["acceptance"]
        self.assertEqual(dict(acceptance), ACCEPTED_LIMITS)

    def test_curved_guide_reference_artifact_passes_every_mechanism_gate(self) -> None:
        artifact = json.loads(CURVED_GUIDE_ARTIFACT.read_text(encoding="utf-8"))
        self.assertTrue(artifact["passed"])
        for name in artifact["gates_evaluated"]:
            with self.subTest(probe=name):
                self.assertTrue(artifact["probes"][name]["passed"])

    def test_curved_guide_full_profile_converges_under_timestep_refinement(self) -> None:
        artifacts = []
        expected_closure = qualify_curved_guide._source_closure(CURVED_GUIDE_SOURCE_ROOT)
        for path, expected_dt_s in CURVED_GUIDE_REFINEMENT_ARTIFACTS:
            with self.subTest(path=path.name):
                artifact = json.loads(
                    path.read_text(encoding="utf-8"),
                    parse_constant=lambda value: self.fail(
                        f"non-finite JSON constant in {path.name}: {value}"
                    ),
                )
                artifacts.append(artifact)
                self.assertTrue(artifact["passed"])
                self.assertEqual(artifact["requested"]["physics_dt_s"], expected_dt_s)
                self.assertEqual(
                    artifact["provenance"]["runner_sha256"],
                    hashlib.sha256(CURVED_GUIDE_RUNNER.read_bytes()).hexdigest(),
                )
                self.assertEqual(
                    artifact["provenance"]["helper_sha256"],
                    hashlib.sha256(ANTI_TUNNELING_RUNNER.read_bytes()).hexdigest(),
                )
                self.assertEqual(
                    artifact["provenance"]["fixture_sha256"],
                    hashlib.sha256(CURVED_GUIDE_FIXTURE.read_bytes()).hexdigest(),
                )
                self.assertEqual(
                    artifact["provenance"]["project_source_closure"]["sha256"],
                    expected_closure["sha256"],
                )
                guide = artifact["probes"]["curved_force_guide"]
                self.assertLessEqual(
                    guide["peak_centerline_tracking_error_m"],
                    ACCEPTED_LIMITS["maximum_tracking_error_m"],
                )
                self.assertLessEqual(
                    guide["peak_attachment_geometry_error_m"],
                    ACCEPTED_LIMITS["maximum_attachment_geometry_error_m"],
                )

        for coarse, fine in zip(artifacts[:-1], artifacts[1:], strict=True):
            coarse_guide = coarse["probes"]["curved_force_guide"]
            fine_guide = fine["probes"]["curved_force_guide"]
            exit_speed_relative_change = abs(
                fine_guide["final_speed_mps"] - coarse_guide["final_speed_mps"]
            ) / abs(coarse_guide["final_speed_mps"])
            peak_load_change_g = abs(
                fine_guide["peak_resultant_proper_load_g"]
                - coarse_guide["peak_resultant_proper_load_g"]
            )
            self.assertLessEqual(exit_speed_relative_change, 0.001)
            self.assertLessEqual(peak_load_change_g, 0.02)
            self.assertLessEqual(
                abs(fine_guide["physics_steps"] - 2 * coarse_guide["physics_steps"]),
                1,
            )
        guide = artifact["probes"]["curved_force_guide"]
        # Compared against the pinned literals, not against the artifact's own block.
        self.assertLessEqual(
            guide["peak_centerline_tracking_error_m"],
            ACCEPTED_LIMITS["maximum_tracking_error_m"],
        )
        self.assertLessEqual(
            guide["peak_attachment_geometry_error_m"],
            ACCEPTED_LIMITS["maximum_attachment_geometry_error_m"],
        )
        self.assertLessEqual(
            guide["peak_resultant_proper_load_g"],
            ACCEPTED_LIMITS["maximum_resultant_load_g"],
        )
        self.assertAlmostEqual(guide["final_speed_mps"], 2000.0, delta=0.01)
        self.assertTrue(guide["reached_end"])
        self.assertEqual(guide["transform_writes_during_run"], 0)

    def test_curved_guide_artifact_discloses_unmasked_peaks_and_sample_counts(self) -> None:
        artifact = json.loads(CURVED_GUIDE_ARTIFACT.read_text(encoding="utf-8"))
        guide = artifact["probes"]["curved_force_guide"]
        counts = guide["sample_counts"]
        self.assertGreater(counts["windowed"], 0)
        self.assertGreater(counts["reaction_gate_included"], 0)
        # An unmasked peak can never be smaller than the masked peak it discloses.
        unmasked = guide["unmasked"]
        for key in (
            "peak_guide_reaction_command_relative_error",
            "peak_inferred_attachment_proper_load_g",
            "peak_backend_applied_force_relative_error",
        ):
            with self.subTest(metric=key):
                self.assertGreaterEqual(unmasked[key], guide[key])

    def test_curved_guide_gate_rejects_a_run_that_evaluated_no_samples(self) -> None:
        """A short slice must not pass gates whose peaks are zero by initialization."""
        healthy = {
            "reached_end": True,
            "windowed_sample_count": 5000,
            "reaction_sample_count": 4800,
            "transform_writes_during_run": 0,
            "peak_tracking_error_m": 0.0005,
            "peak_attitude_error_deg": 0.001,
            "peak_resultant_load_g": 6.9,
            "peak_attachment_load_g": 6.9,
            "peak_attachment_geometry_error_m": 2.4e-06,
            "peak_backend_force_relative_error": 0.0007,
            "peak_reaction_relative_error": 0.01,
            "maximum_attached_pair_force_n": 0.0,
            "acceptance": ACCEPTED_LIMITS,
        }
        self.assertTrue(qualify_curved_guide._curved_guide_passed(**healthy))
        # The exact shape of the defect this guards: a run that stopped before the
        # settling window leaves all three windowed peaks at 0.0, which compare as
        # satisfying every limit.
        vacuous = {
            **healthy,
            "windowed_sample_count": 0,
            "reaction_sample_count": 0,
            "peak_resultant_load_g": 0.0,
            "peak_attachment_load_g": 0.0,
            "peak_backend_force_relative_error": 0.0,
            "peak_reaction_relative_error": 0.0,
        }
        self.assertFalse(qualify_curved_guide._curved_guide_passed(**vacuous))
        for field, hostile_value in (
            ("reached_end", False),
            ("windowed_sample_count", 0),
            ("reaction_sample_count", 0),
            ("transform_writes_during_run", 1),
            ("peak_tracking_error_m", 0.051),
            ("peak_attitude_error_deg", 1.1),
            ("peak_resultant_load_g", 10.1),
            ("peak_attachment_load_g", 10.1),
            ("peak_attachment_geometry_error_m", 0.0011),
            ("peak_backend_force_relative_error", 0.051),
            ("peak_reaction_relative_error", 0.051),
            ("maximum_attached_pair_force_n", 1.0),
        ):
            with self.subTest(field=field):
                self.assertFalse(
                    qualify_curved_guide._curved_guide_passed(
                        **{**healthy, field: hostile_value}
                    )
                )

    def test_global_coordinate_control_fails_the_same_gates(self) -> None:
        """The co-moving frame must be load-bearing, not decorative.

        This control runs the same runner and the same gate set in global coordinates.
        If it ever passes, the translated frame is not what makes the run succeed and the
        documented rejection of global coordinates no longer has evidence behind it.
        """
        artifact = json.loads(CURVED_GUIDE_GLOBAL_CONTROL.read_text(encoding="utf-8"))
        self.assertEqual(artifact["requested"]["coordinate_frame"], "global")
        self.assertEqual(
            artifact["provenance"]["runner_sha256"],
            hashlib.sha256(CURVED_GUIDE_RUNNER.read_bytes()).hexdigest(),
        )
        self.assertFalse(artifact["passed"])
        self.assertFalse(artifact["probes"]["curved_force_guide"]["passed"])

    def test_candidate_audit_is_informational_and_never_contributes_a_pass(self) -> None:
        artifact = json.loads(CURVED_GUIDE_ARTIFACT.read_text(encoding="utf-8"))
        audit = artifact["probes"]["candidate_capability_audit"]
        self.assertTrue(audit["informational"])
        self.assertNotIn("passed", audit)
        self.assertIn("candidate_capability_audit", artifact["gates_not_evaluated"])
        # The other authored candidates have explicit terminal dispositions; none is
        # left in an ambiguous "unmeasured" state after the production decision.
        self.assertEqual(
            sorted(audit["formally_dispositioned_candidates"]),
            ["curvature_resolved_joint_chain", "measured_kinematic", "native_path_constraint"],
        )
        self.assertIn(
            "selected for system-level production",
            audit["force_resolved_path_controller"]["disposition"],
        )


if __name__ == "__main__":
    unittest.main()
