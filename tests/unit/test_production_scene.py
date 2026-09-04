# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import json
import hashlib
import math
import unittest
from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory

import _bootstrap  # noqa: F401

from skyarc.configuration import load_yaml, resolve_tube_layout
from skyarc.launcher.production import (
    build_production_scene_plan,
    load_production_fixture,
    validate_fixture_against_scenario,
)


PROJECT = Path(__file__).resolve().parents[2]
CONFIGURATION = PROJECT / "configs" / "curved_2kms.yaml"
FIXTURE = PROJECT / "configs" / "phase0_anti_tunneling_open_cradle.json"
EXTENSION_TOML = (
    PROJECT / "exts" / "skyarc" / "config" / "extension.toml"
)
PRODUCTION_RUNNER = PROJECT / "standalone" / "run_launcher.py"
PRODUCTION_SMOKE = PROJECT / "artifacts" / "production" / "scene_smoke.json"
ROCKET_VISUAL_ASSET = (
    PROJECT / "assets" / "vehicles" / "jupiter_c" / "Explorer_JupiterC_NoStage1.usdc"
)
ROCKET_VISUAL_MANIFEST = (
    PROJECT
    / "assets"
    / "vehicles"
    / "jupiter_c"
    / "Explorer_JupiterC_NoStage1.manifest.json"
)


class ProductionSceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.loaded = load_yaml(CONFIGURATION)
        cls.layout = resolve_tube_layout(cls.loaded.config)
        cls.fixture = load_production_fixture(FIXTURE)

    def test_fixture_and_scenario_are_one_geometry_condition(self) -> None:
        validate_fixture_against_scenario(self.fixture, self.loaded.config)
        self.assertEqual(self.fixture.rocket.length_m, 4.0)
        self.assertEqual(self.fixture.rocket.diameter_m, 1.0)
        self.assertEqual(self.fixture.cradle.outer_length_m, 4.2)
        self.assertEqual(self.fixture.cradle.outer_width_m, 1.25)
        self.assertEqual(self.fixture.cradle.outer_height_m, 1.4)

    def test_scene_plan_pins_the_approved_non_constraint_treatment(self) -> None:
        plan = build_production_scene_plan(
            self.loaded.config, self.layout, self.fixture, maximum_curve_spacing_m=250.0
        )
        self.assertEqual(plan.backend, "physx")
        self.assertEqual(plan.device, "cpu")
        self.assertEqual(plan.candidate, "force_resolved_path_controller_v1")
        self.assertEqual(plan.coordinate_frame, "translated_accelerating_v1")
        self.assertIn("not_solver_constraint_reaction", plan.reaction_evidence)

    def test_stage_bands_cover_the_resolved_centerline_without_gaps(self) -> None:
        plan = build_production_scene_plan(
            self.loaded.config, self.layout, self.fixture, maximum_curve_spacing_m=250.0
        )
        self.assertEqual(plan.tube_bands[0].start_s_m, 0.0)
        self.assertTrue(
            math.isclose(plan.tube_bands[-1].end_s_m, self.layout.length_m, abs_tol=1e-6)
        )
        for previous, current in zip(plan.tube_bands[:-1], plan.tube_bands[1:], strict=True):
            self.assertEqual(previous.end_s_m, current.start_s_m)
            self.assertEqual(previous.points_m[-1], current.points_m[0])
        self.assertTrue(all(len(band.points_m) >= 2 for band in plan.tube_bands))

    def test_exit_track_starts_at_the_same_resolved_exit_marker(self) -> None:
        plan = build_production_scene_plan(self.loaded.config, self.layout, self.fixture)
        self.assertEqual(plan.exit_track_points_m[0], plan.exit_marker_position_m)
        distance = math.dist(plan.exit_track_points_m[0], plan.exit_track_points_m[1])
        self.assertTrue(
            math.isclose(
                distance,
                self.loaded.config.tube.exit_brake_track_length_m,
                rel_tol=0.0,
                abs_tol=1e-6,
            )
        )

    def test_fixture_mismatch_and_duplicate_json_keys_fail_closed(self) -> None:
        hostile_rocket = replace(self.fixture.rocket, length_m=3.9)
        with self.assertRaisesRegex(ValueError, "rocket length"):
            validate_fixture_against_scenario(
                replace(self.fixture, rocket=hostile_rocket), self.loaded.config
            )
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "duplicate.json"
            source = FIXTURE.read_text(encoding="utf-8")
            path.write_text(source.replace('"pair_name":', '"pair_name": "rocket_cradle",\n  "pair_name":'), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate JSON key"):
                load_production_fixture(path)

    def test_fixture_rejects_a_rocket_that_intersects_the_cradle_floor(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "floor_intersection.json"
            source = json.loads(FIXTURE.read_text(encoding="utf-8"))
            source["cradle"]["outer_height_m"] = 1.2
            path.write_text(json.dumps(source), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "intersects the cradle floor"):
                load_production_fixture(path)

    def test_extension_entrypoint_does_not_replace_the_isaac_free_package_root(self) -> None:
        manifest = EXTENSION_TOML.read_text(encoding="utf-8")
        self.assertIn(
            'name = "skyarc.extension"', manifest
        )
        self.assertNotIn('name = "skyarc"\n', manifest)

    def test_standalone_runner_constructs_simulation_app_before_scene_import(self) -> None:
        source = PRODUCTION_RUNNER.read_text(encoding="utf-8")
        # Matched on the module, not on a one-line import form: the point is the ordering,
        # and pinning the exact spelling breaks the moment the import gains a second name.
        self.assertLess(
            source.index("from isaacsim import SimulationApp"),
            source.index("from skyarc.launcher.scene import"),
        )
        self.assertIn("finally:", source)
        self.assertIn("app.close()", source)

    def test_target_build_scene_smoke_is_bound_and_passes(self) -> None:
        artifact = json.loads(PRODUCTION_SMOKE.read_text(encoding="utf-8"))
        self.assertTrue(artifact["passed"])
        self.assertEqual(artifact["backend"], "physx")
        self.assertIn("cpu", artifact["device"].lower())
        self.assertEqual(artifact["solver_type"], "TGS")
        self.assertEqual(artifact["cradle_topology"], "open_front_u")
        self.assertEqual(artifact["rocket_shape"], "cylinder_x")
        self.assertEqual(artifact["rocket_length_m"], 4.0)
        self.assertEqual(artifact["rocket_diameter_m"], 1.0)
        self.assertEqual(artifact["rocket_visual_redistribution_status"], "cleared")
        self.assertEqual(
            artifact["rocket_visual_reference_portability"],
            "development_absolute_source_reference",
        )
        self.assertEqual(
            artifact["rocket_visual_asset_sha256"],
            hashlib.sha256(ROCKET_VISUAL_ASSET.read_bytes()).hexdigest(),
        )
        self.assertEqual(
            artifact["rocket_visual_manifest_sha256"],
            hashlib.sha256(ROCKET_VISUAL_MANIFEST.read_bytes()).hexdigest(),
        )
        self.assertEqual(
            artifact["configuration_source_sha256"],
            hashlib.sha256(CONFIGURATION.read_bytes()).hexdigest(),
        )
        self.assertEqual(
            artifact["fixture_sha256"], hashlib.sha256(FIXTURE.read_bytes()).hexdigest()
        )


if __name__ == "__main__":
    unittest.main()
