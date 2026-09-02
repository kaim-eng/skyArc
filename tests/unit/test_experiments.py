# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import hashlib
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import _bootstrap  # noqa: F401

from skyarc.components import ComponentDescriptor, Determinism
from skyarc.configuration import load_yaml
from skyarc.experiments import (
    BASELINE_V1,
    ComponentProvenance,
    ConditionResult,
    ContrastError,
    HashingError,
    ManifestError,
    NamedRandomStreams,
    NumericalProvenance,
    SchemaProvenance,
    SoftwareProvenance,
    build_manifest,
    factor_diff,
    hash_dependency_closure,
    paired_contrasts,
    resolve_evidence_window,
)
from skyarc.names import ALL_SLOTS, BODY_CART, BODY_ROCKET
from skyarc.state import BodyState, SimulationState
from skyarc.telemetry import (
    CORE_TELEMETRY_SCHEMA_V2,
    RunPaths,
    RunSummary,
)


BASELINE = Path(__file__).resolve().parents[2] / "configs" / "baseline.yaml"
CURVED = Path(__file__).resolve().parents[2] / "configs" / "curved_2kms.yaml"


def complete_summary() -> RunSummary:
    return RunSummary(
        schema_version="run_summary_v1",
        termination_reason="complete",
        mission_phase="complete",
        elapsed_s=2.0,
        physics_steps=480,
        telemetry_samples=240,
        event_count=10,
        target_exit_speed_mps=50.0,
        actual_exit_speed_mps=49.5,
        exit_speed_relative_error=0.01,
        peak_resultant_load_g=2.0,
        maximum_separation_gap_m=2.0,
        rocket_impulse_ns=10.0,
        energy_residual_j=0.1,
        normalized_energy_residual=0.001,
        energy_closure_valid=True,
        energy_closure_defect=None,
        first_event_time_s={"separation_confirmed": 1.0},
    )


def initial_state() -> SimulationState:
    return SimulationState(
        time_s=0.0,
        step_index=0,
        dt_s=1.0 / 240.0,
        bodies={
            BODY_CART: BodyState(name=BODY_CART, mass_kg=250.0),
            BODY_ROCKET: BodyState(name=BODY_ROCKET, mass_kg=150.0),
        },
    ).frozen()


class DependencyClosureTests(unittest.TestCase):
    def test_builtin_closure_records_the_interpreter_and_excludes_test_sources(self) -> None:
        from skyarc.experiments import (
            EXCLUDED_CLOSURE_DIRECTORIES,
            builtin_package_code_identity,
            interpreter_version,
        )

        identity = builtin_package_code_identity()

        # The standard library's numeric formatting, hashing and serialization behaviour is
        # version-dependent, so the interpreter is an input to what the code does. Inside Kit
        # it is pinned transitively by the Isaac Sim build, but that field is caller-supplied
        # and unverifiable, and a pure-core evidence run can execute under any interpreter.
        self.assertEqual(identity.external_versions.get("python"), interpreter_version())
        self.assertIn("PyYAML", identity.external_versions)

        # Section 15 places the Kit integration tests inside this package. Hashing them would
        # make a test-only edit change every component's code hash, so a test change would
        # read as a behavioural change and the section 14.1 contrasts spanning it would be
        # refused or misattributed.
        self.assertIn("tests", EXCLUDED_CLOSURE_DIRECTORIES)
        offending = [
            name
            for name in identity.file_sha256
            if set(Path(name).parts[:-1]) & EXCLUDED_CLOSURE_DIRECTORIES
        ]
        self.assertEqual(offending, [])

        # Keys stay package-relative so the identity is the same whether the package is
        # imported as `skyarc` or `skyarc`.
        self.assertTrue(identity.file_sha256)
        for name in identity.file_sha256:
            self.assertFalse(Path(name).is_absolute(), name)
            self.assertNotIn("isaacsim", name)

    def test_shared_source_and_external_version_are_part_of_code_identity(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            component = root / "component.py"
            helper = root / "helper.py"
            component.write_text("from .helper import value\n", encoding="utf-8")
            helper.write_text("value = 1\n", encoding="utf-8")
            first = hash_dependency_closure(
                root,
                component_sources=("component.py",),
                dependency_sources=("helper.py",),
                external_versions={"solver": "1.0"},
            )
            repeated = hash_dependency_closure(
                root,
                component_sources=("component.py",),
                dependency_sources=("helper.py",),
                external_versions={"solver": "1.0"},
            )
            self.assertEqual(first, repeated)
            helper.write_text("value = 2\n", encoding="utf-8")
            changed_helper = hash_dependency_closure(
                root,
                component_sources=("component.py",),
                dependency_sources=("helper.py",),
                external_versions={"solver": "1.0"},
            )
            changed_external = hash_dependency_closure(
                root,
                component_sources=("component.py",),
                dependency_sources=("helper.py",),
                external_versions={"solver": "2.0"},
            )
            self.assertNotEqual(first.sha256, changed_helper.sha256)
            self.assertNotEqual(changed_helper.sha256, changed_external.sha256)

    def test_dependency_closure_rejects_escapes_duplicates_and_nonfinite_values(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "source.py"
            source.write_text("x = 1\n", encoding="utf-8")
            with self.assertRaises(HashingError):
                hash_dependency_closure(root, component_sources=(source,), dependency_sources=(source,))
            with self.assertRaises(HashingError):
                hash_dependency_closure(root, component_sources=(Path(temporary).parent / "outside.py",))


class RandomAndCriterionTests(unittest.TestCase):
    def test_named_streams_are_access_order_independent_and_pair_across_conditions(self) -> None:
        first = NamedRandomStreams(42, ("atmosphere", "observer"))
        observer_first = first.stream("observer").random()
        atmosphere_first = first.stream("atmosphere").random()
        second = NamedRandomStreams(42, ("observer", "atmosphere"))
        atmosphere_second = second.stream("atmosphere").random()
        observer_second = second.stream("observer").random()
        self.assertEqual(observer_first, observer_second)
        self.assertEqual(atmosphere_first, atmosphere_second)
        self.assertEqual(dict(first.seeds), dict(second.seeds))
        with self.assertRaises(ValueError):
            first.stream("undeclared")

    def test_versioned_criterion_policy_and_evidence_resolution(self) -> None:
        passing = BASELINE_V1.evaluate(complete_summary().to_dict())
        failing = BASELINE_V1.evaluate(
            replace(complete_summary(), exit_speed_relative_error=0.051).to_dict()
        )
        missing = BASELINE_V1.evaluate({"termination_reason": "complete"})
        self.assertTrue(passing.passed)
        self.assertFalse(failing.passed)
        self.assertFalse(missing.passed)
        self.assertEqual(len(BASELINE_V1.sha256), 64)
        self.assertEqual(resolve_evidence_window(load_yaml(BASELINE).config).duration_s, 0.5)
        curved = load_yaml(CURVED).config
        self.assertEqual(
            resolve_evidence_window(curved).duration_s,
            curved.evidence.free_flight_duration_s,  # type: ignore[union-attr]
        )


class LineageAndContrastTests(unittest.TestCase):
    def test_factor_diff_preserves_additions_removals_and_nested_changes(self) -> None:
        diff = factor_diff(
            {"model": {"drag": "a", "guide": "g"}, "removed": 1},
            {"model": {"drag": "b", "guide": "g"}, "added": 2},
        )
        self.assertEqual(set(diff), {"added", "model.drag", "removed"})
        self.assertFalse(diff["added"]["before_present"])
        self.assertFalse(diff["removed"]["after_present"])

    def test_ablation_chain_produces_paired_adjacent_and_baseline_contrasts(self) -> None:
        streams = dict(NamedRandomStreams(7, ("observer",)).seeds)
        common = {
            "experiment_id": "study",
            "replicate_id": 0,
            "initial_state_sha256": "initial",
            "stream_seeds": streams,
        }
        values = (
            ConditionResult(
                **common,
                condition_id="A",
                parent_condition_id=None,
                factors={"x": False, "y": False},
                metrics={"speed": 10.0},
            ),
            ConditionResult(
                **common,
                condition_id="A_X",
                parent_condition_id="A",
                factors={"x": True, "y": False},
                metrics={"speed": 12.0},
            ),
            ConditionResult(
                **common,
                condition_id="A_X_Y",
                parent_condition_id="A_X",
                factors={"x": True, "y": True},
                metrics={"speed": 15.0},
            ),
        )
        contrasts = paired_contrasts(values)
        self.assertEqual(len(contrasts), 4)
        lookup = {
            (item.kind, item.reference_condition_id, item.treatment_condition_id): item
            for item in contrasts
        }
        self.assertEqual(lookup[("incremental", "A_X", "A_X_Y")].metric_delta["speed"], 3.0)
        self.assertEqual(lookup[("versus_baseline", "A", "A_X_Y")].metric_delta["speed"], 5.0)
        unpaired = replace(values[-1], stream_seeds={"observer": 1})
        with self.assertRaises(ContrastError):
            paired_contrasts((*values[:-1], unpaired))


class ManifestTests(unittest.TestCase):
    def test_complete_manifest_records_inventory_identity_policy_and_outcome(self) -> None:
        loaded = load_yaml(BASELINE)
        with tempfile.TemporaryDirectory() as temporary:
            temp = Path(temporary)
            source = temp / "model.py"
            source.write_text("MODEL_VERSION = 1\n", encoding="utf-8")
            code = hash_dependency_closure(temp, component_sources=(source,))
            components = tuple(
                ComponentProvenance.create(
                    ComponentDescriptor(
                        slot=slot,
                        model_id=(
                            getattr(loaded.config.models, slot)
                            if hasattr(loaded.config.models, slot)
                            else f"{slot}_v1"
                        ),
                        model_version="1.0.0",
                        parameter_schema_version="1",
                        code_hash=code.sha256,
                        determinism=Determinism.DETERMINISTIC,
                    ),
                    resolved_parameters={"slot": slot},
                    code=code,
                )
                for slot in ALL_SLOTS
            )
            paths = RunPaths.create(
                temp / "outputs",
                experiment_id=loaded.config.experiment.experiment_id,
                condition_id=loaded.config.experiment.condition_id,
                replicate_id=loaded.config.experiment.replicate_id,
            )
            manifest = build_manifest(
                paths=paths,
                loaded=loaded,
                components=components,
                initial_state=initial_state(),
                scene_sha256=hashlib.sha256(b"scene").hexdigest(),
                software=SoftwareProvenance("6.0.1-rc.7", "revision"),
                numerical=NumericalProvenance(
                    backend="analytic",
                    device="cpu",
                    solver="semi_implicit_euler_v1",
                    physics_dt_s=loaded.config.simulation.physics_dt_s,
                    render_dt_s=loaded.config.simulation.render_dt_s,
                    substeps=loaded.config.simulation.substeps,
                    ccd_enabled=False,
                    contact_settings={"reporting": "impulse"},
                    execution_profile=loaded.config.simulation.profile,
                    fixed_time_stepping=True,
                    aggregator_pre_step_order=0,
                ),
                random_streams=NamedRandomStreams(loaded.config.experiment.seed, ALL_SLOTS),
                schemas=SchemaProvenance(
                    observation="ground_truth_observation_v1",
                    telemetry=CORE_TELEMETRY_SCHEMA_V2.version,
                    outcome="run_summary_v1",
                ),
                criterion_policy=BASELINE_V1,
                summary=complete_summary(),
                controlled_factors={"models": {"guide": loaded.config.models.guide}},
            )
            payload = manifest.to_dict()
            self.assertEqual(len(payload["components"]), len(ALL_SLOTS))
            self.assertEqual(payload["identity"]["run_instance_id"], paths.run_instance_id)
            self.assertTrue(payload["outcome"]["criterion_result"]["passed"])
            self.assertEqual(payload["criterion_policy"]["sha256"], BASELINE_V1.sha256)
            manifest.write(paths.experiment_manifest)
            self.assertTrue(paths.experiment_manifest.is_file())
            with self.assertRaises(ManifestError):
                manifest.write(paths.experiment_manifest)

    def test_manifest_rejects_incomplete_component_inventory(self) -> None:
        loaded = load_yaml(BASELINE)
        with tempfile.TemporaryDirectory() as temporary:
            paths = RunPaths.create(
                temporary,
                experiment_id=loaded.config.experiment.experiment_id,
                condition_id=loaded.config.experiment.condition_id,
                replicate_id=loaded.config.experiment.replicate_id,
            )
            with self.assertRaisesRegex(ManifestError, "inventory"):
                build_manifest(
                    paths=paths,
                    loaded=loaded,
                    components=(),
                    initial_state=initial_state(),
                    scene_sha256=hashlib.sha256(b"scene").hexdigest(),
                    software=SoftwareProvenance("build", "revision"),
                    numerical=NumericalProvenance(
                        backend="analytic",
                        device="cpu",
                        solver="solver",
                        physics_dt_s=0.01,
                        render_dt_s=0.01,
                        substeps=1,
                        ccd_enabled=False,
                        contact_settings={},
                        execution_profile="interactive_rendered",
                        fixed_time_stepping=True,
                        aggregator_pre_step_order=0,
                    ),
                    random_streams=NamedRandomStreams(42, ()),
                    schemas=SchemaProvenance("observation", "telemetry", "outcome"),
                    criterion_policy=BASELINE_V1,
                    summary=complete_summary(),
                    controlled_factors={},
                )


if __name__ == "__main__":
    unittest.main()
