# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Contracts for the production mission wiring that do not require a Kit application."""

from __future__ import annotations

import ast
import hashlib
import json
import math
import unittest
from pathlib import Path

import _bootstrap  # noqa: F401

from skyarc.configuration import load_yaml, resolve_tube_layout
from skyarc.effects.adapter import AppliedEffects
from skyarc.effects.aggregator import BodyLoad
from skyarc.launcher.geometry import path_pose
from skyarc.launcher.path_controller import LaunchProfileReferenceFrame
from skyarc.launcher.production import (
    build_production_scene_plan,
    combined_pitch_inertia_kg_m2,
    load_production_fixture,
    resolve_initial_solver_states,
)
from skyarc.linalg import dot, norm, sub
from skyarc.names import (
    BODY_CART,
    BODY_ROCKET,
    SLOT_BACKEND_ADAPTER,
    SLOT_BODY_OWNERSHIP,
    SLOT_GUIDE,
)
from skyarc.state import BodyState, SimulationState
from skyarc.telemetry import (
    CORE_TELEMETRY_SCHEMA_V1,
    CORE_TELEMETRY_SCHEMA_V2,
    EnergyAccumulator,
)
from skyarc.telemetry.energy import SLOT_WORK_TERMS, WORK_TERMS


PROJECT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = PROJECT / "exts" / "skyarc" / "skyarc"
CONFIGURATION = PROJECT / "configs" / "curved_2kms.yaml"
FIXTURE = PROJECT / "configs" / "phase0_anti_tunneling_open_cradle.json"
MISSION_RUNNER = PROJECT / "standalone" / "run_mission.py"
MISSION_SMOKE = PROJECT / "artifacts" / "production" / "mission_smoke.json"
MISSION_TELEMETRY = PROJECT / "artifacts" / "production" / "mission_telemetry.json"
ISAAC_ROOT = PROJECT.parent / "IsaacSim"
ISAAC_RELEASE = ISAAC_ROOT / "_build" / "windows-x86_64" / "release"


def source_closure(root: Path) -> dict[str, object]:
    files = sorted(
        path
        for path in root.rglob("*.py")
        if "__pycache__" not in path.parts and "tests" not in path.parts
    )
    file_hashes: dict[str, str] = {}
    digest = hashlib.sha256()
    for path in files:
        relative = path.relative_to(root).as_posix()
        file_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        file_hashes[relative] = file_hash
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(bytes.fromhex(file_hash))
        digest.update(b"\0")
    return {"sha256": digest.hexdigest(), "files": file_hashes}

ACCEPTED_GAINS = {
    "normal_kp_per_s2": 400.0,
    "normal_kd_per_s": 40.0,
    "attitude_kp_per_s2": 2500.0,
    "attitude_kd_per_s": 100.0,
}
"""The Phase 0 accepted curved-guide gains, restated as literals.

Reading them back out of the artifact that used them would prove only that the file is
self-consistent. They are pinned identically in ``test_phase0_runner.py``; production must
reproduce the qualified condition rather than merely record whatever it ran with.
"""

KIT_BOUNDARY_MODULES = frozenset(
    {
        "extension.py",
        "effects/backends/isaac.py",
        "launcher/production_runtime.py",
        "launcher/scene.py",
    }
)
"""The only modules permitted to import Isaac Sim, Omni, USD, Warp or NumPy.

Section 12 records that ``numpy`` cannot even be imported by the bundled interpreter
outside a Kit application, so the rule is not stylistic: a stray import anywhere else makes
the whole pure unit suite unrunnable. This set is the executable form of that rule.
"""

FORBIDDEN_ROOTS = ("isaacsim", "omni", "carb", "pxr", "warp", "numpy", "usdrt")


def _module_roots(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    roots: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            roots.add(node.module.split(".")[0])
    return roots


class ImportBoundaryTests(unittest.TestCase):
    def test_only_the_named_boundary_modules_import_isaac_omni_or_usd(self) -> None:
        offenders: dict[str, list[str]] = {}
        boundary_seen: set[str] = set()
        for path in sorted(PACKAGE_ROOT.rglob("*.py")):
            package_relative = path.relative_to(PACKAGE_ROOT)
            relative = package_relative.as_posix()
            if "tests" in package_relative.parts[:-1]:
                continue
            forbidden = sorted(_module_roots(path) & set(FORBIDDEN_ROOTS))
            if relative in KIT_BOUNDARY_MODULES:
                if forbidden:
                    boundary_seen.add(relative)
                continue
            if forbidden:
                offenders[relative] = forbidden
        self.assertEqual(offenders, {})
        # Every declared boundary module must actually be one; a stale entry would quietly
        # widen the permission after the module it named stopped needing it.
        self.assertEqual(boundary_seen, set(KIT_BOUNDARY_MODULES))

    def test_the_mission_runner_starts_the_app_before_importing_the_runtime(self) -> None:
        source = MISSION_RUNNER.read_text(encoding="utf-8")
        self.assertLess(
            source.index("from isaacsim import SimulationApp"),
            source.index(
                "from skyarc.launcher.production_runtime import "
                "ProductionMissionRuntime"
            ),
        )
        self.assertIn("finally:", source)
        self.assertIn("app.close()", source)
        # A bounded run must not be reportable as a completed mission.
        self.assertIn('"completed_mission": completed', source)
        # Reset replay is a post-run probe; it must not erase the outcome being reported.
        self.assertLess(
            source.index("run_phase = mission.mission_state.phase"),
            source.index("mission.reset()"),
        )
        self.assertIn("and run_phase is not MissionPhase.ABORT", source)


class GuideReactionAccountingTests(unittest.TestCase):
    def test_the_backend_adapter_slot_has_its_own_work_term(self) -> None:
        self.assertIn("guide_reaction", WORK_TERMS)
        self.assertEqual(SLOT_WORK_TERMS[SLOT_BACKEND_ADAPTER], "guide_reaction")
        # The backend slot is not a component slot: it may not own a body wrench through
        # the accepted-effect path, only report one it applied itself.
        self.assertNotIn(SLOT_BACKEND_ADAPTER, SLOT_BODY_OWNERSHIP)
        self.assertEqual(CORE_TELEMETRY_SCHEMA_V1.version, "core_telemetry_v1")
        self.assertNotIn("energy.work_guide_reaction_j", CORE_TELEMETRY_SCHEMA_V1.fields)
        self.assertIn("energy.work_guide_reaction_j", CORE_TELEMETRY_SCHEMA_V2.fields)
        self.assertEqual(CORE_TELEMETRY_SCHEMA_V2.version, "core_telemetry_v2")

    def test_published_v1_cannot_silently_acquire_a_later_column(self) -> None:
        """v1 must be what it contained, not "the current field set minus a hole".

        Asserting only that the one v2 column is absent would keep passing while any
        *other* new column leaked into v1, and a v1 run replayed against it would then
        validate against a field set that never existed.
        """
        from skyarc.telemetry import schema as schema_module

        self.assertEqual(
            schema_module._field_name_digest(CORE_TELEMETRY_SCHEMA_V1.fields),
            schema_module.CORE_TELEMETRY_V1_FIELD_DIGEST,
        )
        # The exclusion set and the digest have to agree about the difference; neither
        # alone would catch a column added to the core set and forgotten here.
        self.assertEqual(
            set(CORE_TELEMETRY_SCHEMA_V2.fields) - set(CORE_TELEMETRY_SCHEMA_V1.fields),
            set(schema_module.FIELDS_ADDED_AFTER_V1),
        )
        # A column appearing in the core set without being declared post-v1 must be an
        # import-time failure, not a silent v1 mutation.
        self.assertNotEqual(
            schema_module._field_name_digest(
                set(CORE_TELEMETRY_SCHEMA_V1.fields) | {"energy.work_future_j"}
            ),
            schema_module.CORE_TELEMETRY_V1_FIELD_DIGEST,
        )

    def test_reaction_work_is_attributed_to_its_own_channel_not_to_resistance(self) -> None:
        def state(x_m: float) -> SimulationState:
            return SimulationState(
                time_s=0.0 if x_m == 0.0 else 1.0,
                step_index=0 if x_m == 0.0 else 1,
                dt_s=1.0,
                bodies={
                    BODY_CART: BodyState(
                        name=BODY_CART, position=(x_m, 0.0, 0.0), mass_kg=250.0
                    ),
                    BODY_ROCKET: BodyState(name=BODY_ROCKET, mass_kg=150.0),
                },
            ).frozen()

        before = state(0.0)
        after = state(2.0)
        accumulator = EnergyAccumulator(before, gravity_mps2=(0.0, 0.0, 0.0))
        applied = AppliedEffects(
            loads={
                BODY_CART: BodyLoad(
                    body=BODY_CART,
                    force_n=(15.0, 0.0, 0.0),
                    force_by_slot={
                        SLOT_GUIDE: (-5.0, 0.0, 0.0),
                        SLOT_BACKEND_ADAPTER: (20.0, 0.0, 0.0),
                    },
                )
            }
        )
        snapshot = accumulator.update(before, after, applied)
        self.assertAlmostEqual(snapshot.work_j["guide_reaction"], 40.0, places=9)
        self.assertAlmostEqual(snapshot.work_j["resistance"], -10.0, places=9)

    def test_reaction_work_includes_backend_torque(self) -> None:
        before = SimulationState(
            time_s=0.0,
            step_index=0,
            dt_s=1.0,
            bodies={BODY_CART: BodyState(name=BODY_CART, mass_kg=250.0)},
        ).frozen()
        after = SimulationState(
            time_s=1.0,
            step_index=1,
            dt_s=1.0,
            bodies={
                BODY_CART: BodyState(
                    name=BODY_CART,
                    mass_kg=250.0,
                    angular_velocity=(0.0, 2.0, 0.0),
                )
            },
        ).frozen()
        applied = AppliedEffects(
            loads={
                BODY_CART: BodyLoad(
                    body=BODY_CART,
                    torque_nm=(0.0, 10.0, 0.0),
                    torque_by_slot={SLOT_BACKEND_ADAPTER: (0.0, 10.0, 0.0)},
                )
            }
        )
        snapshot = EnergyAccumulator(
            before, gravity_mps2=(0.0, 0.0, 0.0)
        ).update(before, after, applied)
        # Trapezoidal angular displacement is 1 rad over this step.
        self.assertAlmostEqual(snapshot.work_j["guide_reaction"], 10.0, places=9)
        self.assertFalse(snapshot.valid)


class InitialPlacementTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = load_yaml(CONFIGURATION).config
        cls.layout = resolve_tube_layout(cls.config)
        cls.fixture = load_production_fixture(FIXTURE)
        cls.plan = build_production_scene_plan(cls.config, cls.layout, cls.fixture)

    def test_the_qualified_assembly_com_starts_on_the_tube_entrance(self) -> None:
        frame = LaunchProfileReferenceFrame(
            self.layout, target_exit_speed_mps=self.config.launch_control.target_exit_speed_mps
        )
        reference = frame.sample(0.0)
        states = resolve_initial_solver_states(self.layout, self.plan, self.fixture, reference)
        cart = states[BODY_CART]
        rocket = states[BODY_ROCKET]
        assembly_mass = self.fixture.cradle.mass_kg + self.fixture.rocket.mass_kg
        global_cart = tuple(
            cart.position_m[index] + reference.position_m[index] for index in range(3)
        )
        global_rocket = tuple(
            rocket.position_m[index] + reference.position_m[index] for index in range(3)
        )
        entrance = path_pose(self.layout, 0.0)
        # Phase 0 qualifies the assembly COM at s=0. The cart extends into the straight
        # entrance lead-in while launch control follows COM progress.
        self.assertLess(self.layout.axial_position(global_cart), 0.0)
        self.assertAlmostEqual(
            norm(sub(global_rocket, global_cart)),
            self.plan.cart_to_rocket_offset_m,
            places=9,
        )
        # The rocket is ahead of the cart along the tube, never behind it.
        self.assertGreater(dot(sub(global_rocket, global_cart), entrance.tangent), 0.0)

        # The entrance is straight, so the mass-weighted COM lies on the centerline.
        com = tuple(
            (
                self.fixture.cradle.mass_kg * global_cart[index]
                + self.fixture.rocket.mass_kg * global_rocket[index]
            )
            / assembly_mass
            for index in range(3)
        )
        com_s_m = self.layout.axial_position(com)
        self.assertAlmostEqual(com_s_m, 0.0, delta=1e-6)
        # The residual is the nearest-point projection's own search tolerance, not a
        # placement error: the centre of mass is on the straight entrance by construction.
        self.assertAlmostEqual(
            norm(sub(com, entrance.position_m)), 0.0, delta=1e-9
        )
        self.assertEqual(cart.linear_velocity_mps, (0.0, 0.0, 0.0))
        self.assertEqual(rocket.angular_velocity_radps, (0.0, 0.0, 0.0))

    def test_combined_pitch_inertia_adds_the_parallel_axis_term(self) -> None:
        cart = [0.0] * 9
        rocket = [0.0] * 9
        cart[4] = 100.0
        rocket[4] = 25.0
        combined = combined_pitch_inertia_kg_m2(
            cart, rocket, cart_mass_kg=250.0, rocket_mass_kg=150.0, offset_m=3.26
        )
        self.assertAlmostEqual(
            combined, 125.0 + 250.0 * 150.0 / 400.0 * 3.26**2, places=9
        )
        with self.assertRaisesRegex(RuntimeError, "3x3 inertia tensor"):
            combined_pitch_inertia_kg_m2(
                cart[:8], rocket, cart_mass_kg=250.0, rocket_mass_kg=150.0, offset_m=3.26
            )


class MissionArtifactTests(unittest.TestCase):
    """Bind the target-build mission artifacts to the sources that produced them."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.smoke = json.loads(
            MISSION_SMOKE.read_text(encoding="utf-8"),
            parse_constant=lambda value: (_ for _ in ()).throw(
                AssertionError(f"non-finite JSON constant: {value}")
            ),
        )
        cls.telemetry = json.loads(MISSION_TELEMETRY.read_text(encoding="utf-8"))

    def test_both_artifacts_are_bound_to_the_current_runner_and_inputs(self) -> None:
        for label, artifact in (("smoke", self.smoke), ("telemetry", self.telemetry)):
            with self.subTest(artifact=label):
                self.assertEqual(artifact["schema"], "vacuum_tube_production_mission_v1")
                # Recomputed from the files the artifact claims to identify; reading the
                # hashes back out of it would make editing any of them break nothing.
                self.assertEqual(
                    artifact["runner_sha256"],
                    hashlib.sha256(MISSION_RUNNER.read_bytes()).hexdigest(),
                )
                self.assertEqual(
                    artifact["configuration_source_sha256"],
                    hashlib.sha256(CONFIGURATION.read_bytes()).hexdigest(),
                )
                self.assertEqual(
                    artifact["fixture_sha256"],
                    hashlib.sha256(FIXTURE.read_bytes()).hexdigest(),
                )
                provenance = artifact["provenance"]
                expected_closure = source_closure(PACKAGE_ROOT)
                self.assertEqual(
                    provenance["project_source_closure"]["sha256"],
                    expected_closure["sha256"],
                )
                self.assertEqual(
                    provenance["project_source_closure"]["files"],
                    expected_closure["files"],
                )
                for key, path in (
                    ("runner_sha256", MISSION_RUNNER),
                    ("configuration_sha256", CONFIGURATION),
                    ("fixture_sha256", FIXTURE),
                    ("experience_sha256", ISAAC_RELEASE / "apps" / "isaacsim.exp.full.kit"),
                    ("version_file_sha256", ISAAC_ROOT / "VERSION"),
                    (
                        "python_executable_sha256",
                        ISAAC_RELEASE / "kit" / "python" / "kit.exe",
                    ),
                ):
                    with self.subTest(artifact=label, provenance_key=key):
                        self.assertEqual(
                            provenance[key], hashlib.sha256(path.read_bytes()).hexdigest()
                        )
                self.assertTrue(artifact["passed"])
                self.assertEqual(artifact["backend"], "physx")
                self.assertIn("cpu", artifact["device"].lower())
                self.assertEqual(artifact["solver_type"], "TGS")
                self.assertIs(artifact["fixed_time_stepping"], True)
                self.assertIs(artifact["rate_limit_enabled"], False)
                self.assertEqual(artifact["controller_gains"], ACCEPTED_GAINS)
                self.assertEqual(
                    artifact["candidate"], "force_resolved_path_controller_v1"
                )
                self.assertIn("not_solver_constraint_reaction", artifact["reaction_evidence"])
                # A bounded run is never reportable as a finished mission.
                self.assertIsNotNone(artifact["requested_max_steps"])
                self.assertFalse(artifact["completed_mission"])
                self.assertIsNone(artifact["abort_reason"])

    def test_the_guided_run_tracks_the_centerline_and_the_commanded_profile(self) -> None:
        smoke = self.smoke
        self.assertEqual(smoke["physics_steps"], smoke["requested_max_steps"])
        self.assertLess(
            smoke["peak_centerline_tracking_error_m"], 0.01 * smoke["guide_clearance_m"]
        )
        self.assertLess(smoke["peak_attitude_error_deg"], 1.0)
        # The translated frame is what keeps solver coordinates small. Anything on the
        # order of the 54 km tube here would mean the frame had stopped following.
        self.assertLess(smoke["peak_solver_offset_m"], 10.0)
        # The 45-degree entrance makes this the sharpest available check that the launcher
        # engaged at all: with no launcher force the assembly rolls backwards instead.
        expected_speed_mps = (
            smoke["reference_profile_acceleration_mps2"] * smoke["final_time_s"]
        )
        self.assertAlmostEqual(
            smoke["final_cart_speed_mps"],
            expected_speed_mps,
            delta=0.01 * expected_speed_mps,
        )

    def test_the_stop_rebuild_reset_returns_the_authored_state_and_clears_history(self) -> None:
        reset = self.smoke["reset_replay"]
        self.assertTrue(reset["passed"])
        self.assertEqual(reset["time_s"], 0.0)
        self.assertEqual(reset["step_index"], 0)
        self.assertEqual(reset["event_count"], 0)
        self.assertEqual(reset["mission_phase"], "idle")
        for key in (
            "cart_position_error_m",
            "rocket_position_error_m",
            "cart_velocity_error_mps",
            "rocket_velocity_error_mps",
        ):
            with self.subTest(measurement=key):
                self.assertLessEqual(reset[key], reset["maximum_state_error"])

    def test_the_telemetry_run_records_the_versioned_core_schema(self) -> None:
        run = self.telemetry["telemetry_run"]
        self.assertEqual(run["core_schema_version"], CORE_TELEMETRY_SCHEMA_V2.version)
        self.assertTrue(run["guide_reaction_work_column"])
        summary = self.telemetry["telemetry_summary"]
        self.assertEqual(summary["schema_version"], "run_summary_v1")
        self.assertEqual(summary["termination_reason"], "step_budget_exhausted")
        self.assertGreater(summary["telemetry_samples"], 1)
        self.assertGreater(summary["event_count"], 0)
        self.assertLessEqual(summary["peak_resultant_load_g"], 10.0)
        # Energy closure is deliberately reported invalid while a body rotates: the
        # translational identity cannot absorb rotational kinetic energy without modeled
        # inertia, and section 14 requires that be a flagged null rather than a number.
        self.assertFalse(summary["energy_closure_valid"])
        self.assertIsNone(summary["energy_residual_j"])
        self.assertIn("rotational kinetic energy", summary["energy_closure_defect"])


if __name__ == "__main__":
    unittest.main()
