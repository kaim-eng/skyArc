# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import csv
import json
import math
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import _bootstrap  # noqa: F401

from skyarc.components import DiagnosticField, DiagnosticRecord, DiagnosticSchema
from skyarc.configuration import load_yaml, resolve_tube_layout
from skyarc.effects import EffectBatch, Frame, Wrench, aggregate
from skyarc.effects.adapter import AppliedEffects
from skyarc.effects.aggregator import BodyLoad
from skyarc.effects.backends import AnalyticBackend
from skyarc.events import EVENT_ABORT, EVENT_IGNITION, EVENT_STAGE_TRANSITION, Event
from skyarc.launcher import TubeLayout, TubeStage, path_pose
from skyarc.launcher.feasibility import Stage2Constraint
from skyarc.names import (
    BODY_CART,
    BODY_ROCKET,
    JOINT_COUPLING,
    JOINT_GUIDE,
    PAIR_ROCKET_CRADLE,
    SLOT_BACKEND_ADAPTER,
    SLOT_GUIDE,
    SLOT_LAUNCH_FORCE,
)
from skyarc.orchestrator import build_mission
from skyarc.state import AxialQuantities, BodyState, Observation, SimulationState
from skyarc.state_machine import MissionState
from skyarc.telemetry import (
    CORE_TELEMETRY_SCHEMA_V2,
    EnergyAccumulator,
    RunPaths,
    StepTelemetryInput,
    TelemetryRecorder,
    TelemetrySchemaError,
)


BASELINE = Path(__file__).resolve().parents[2] / "configs" / "baseline.yaml"


def flat_layout() -> TubeLayout:
    return TubeLayout(
        origin_m=(0.0, 0.0, 0.0),
        angle_deg=0.0,
        stages=(TubeStage("vacuum", 10.0, 0.0),),
        exterior_effective_density_ratio=0.0,
    )


def state(time_s: float, cart_x: float, cart_speed: float) -> SimulationState:
    return SimulationState(
        time_s=time_s,
        step_index=round(time_s * 10),
        dt_s=0.1,
        bodies={
            BODY_CART: BodyState(
                name=BODY_CART,
                position=(cart_x, 0.0, 0.0),
                linear_velocity=(cart_speed, 0.0, 0.0),
                mass_kg=1.0,
            ),
            BODY_ROCKET: BodyState(name=BODY_ROCKET, mass_kg=1.0),
        },
        joint_active={JOINT_COUPLING: False, JOINT_GUIDE: True},
        collision_pair_active={PAIR_ROCKET_CRADLE: True},
    ).frozen()


def observation(value: SimulationState) -> Observation:
    return Observation(
        source_model="test",
        time_s=value.time_s,
        step_index=value.step_index,
        dt_s=value.dt_s,
        state=value,
        axial=AxialQuantities(
            s_cart_m=value.body(BODY_CART).position[0],
            s_rocket_m=value.body(BODY_ROCKET).position[0],
            marker_s_m={"assembly_exit": value.body(BODY_ROCKET).position[0]},
            cart_axial_velocity_mps=value.body(BODY_CART).linear_velocity[0],
            rocket_axial_velocity_mps=value.body(BODY_ROCKET).linear_velocity[0],
            assembly_mass_kg=1.0,
            stage_index=0,
            stage_name="vacuum",
            effective_density_ratio=0.0,
            separation_gap_m=0.0,
            separation_rate_mps=0.0,
        ),
        coupled=False,
    )


class TelemetrySchemaAndPathTests(unittest.TestCase):
    def test_schema_rejects_unknown_nonfinite_and_missing_required_values(self) -> None:
        required = {
            name: (
                False
                if field.value_type == "bool"
                else 0
                if field.value_type == "int"
                else "x"
                if field.value_type == "string"
                else 0.0
            )
            for name, field in CORE_TELEMETRY_SCHEMA_V2.fields.items()
            if not field.nullable
        }
        normalized = CORE_TELEMETRY_SCHEMA_V2.normalize(required)
        self.assertIsNone(normalized["geometry.radius_m"])
        with self.assertRaisesRegex(TelemetrySchemaError, "unregistered"):
            CORE_TELEMETRY_SCHEMA_V2.normalize({**required, "unknown": 1.0})
        with self.assertRaisesRegex(TelemetrySchemaError, "finite or null"):
            CORE_TELEMETRY_SCHEMA_V2.normalize({**required, "time_s": math.nan})
        missing = dict(required)
        del missing["time_s"]
        with self.assertRaisesRegex(TelemetrySchemaError, "required"):
            CORE_TELEMETRY_SCHEMA_V2.normalize(missing)

    def test_run_instance_paths_are_unique_not_content_derived_and_do_not_escape(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            first = RunPaths.create(
                temporary,
                experiment_id="experiment",
                condition_id="condition",
                replicate_id=0,
            )
            second = RunPaths.create(
                temporary,
                experiment_id="experiment",
                condition_id="condition",
                replicate_id=0,
            )
            self.assertNotEqual(first.run_instance_id, second.run_instance_id)
            self.assertNotEqual(first.root, second.root)
            with self.assertRaises(FileExistsError):
                RunPaths.create(
                    temporary,
                    experiment_id="experiment",
                    condition_id="condition",
                    replicate_id=0,
                    run_instance_id=first.run_instance_id,
                )
            with self.assertRaises(ValueError):
                RunPaths.create(
                    temporary,
                    experiment_id="../escape",
                    condition_id="condition",
                    replicate_id=0,
                )


class EnergyAndRecorderTests(unittest.TestCase):
    def test_summary_captures_apogee_handoff_and_stage2_margin(self) -> None:
        before = state(0.0, 0.0, 0.0)
        rocket = replace(
            before.body(BODY_ROCKET),
            position=(1200.0, 0.0, 40_000.0),
            linear_velocity=(1900.0, 0.0, 100.0),
        )
        after = replace(
            before,
            time_s=1.0,
            step_index=1,
            bodies={**before.bodies, BODY_ROCKET: rocket},
        ).frozen()
        with tempfile.TemporaryDirectory() as temporary:
            paths = RunPaths.create(
                temporary,
                experiment_id="experiment",
                condition_id="condition",
                replicate_id=0,
            )
            recorder = TelemetryRecorder(
                paths,
                before,
                flat_layout(),
                telemetry_rate_hz=10.0,
                stage2_constraint=Stage2Constraint(target_orbit_altitude_m=200_000.0),
            )
            recorder.record_step(
                StepTelemetryInput(
                    pre_state=before,
                    observation=observation(before),
                    command_snapshots={},
                    accepted_effects=aggregate((), before),
                    applied_effects=AppliedEffects.exactly(aggregate((), before)),
                    post_state=after,
                    post_observation=observation(after),
                    mission_state=MissionState(),
                )
            )
            recorder.record_events((Event(EVENT_IGNITION, 1.0, 1, "test"),))
            summary = recorder.finalize(termination_reason="test", mission_phase="idle")

        self.assertEqual(summary.schema_version, "run_summary_v2")
        self.assertEqual(summary.apogee_time_s, 1.0)
        self.assertEqual(summary.apogee_altitude_m, 40_000.0)
        self.assertEqual(summary.handoff_time_s, 1.0)
        self.assertEqual(summary.handoff_altitude_m, 40_000.0)
        self.assertEqual(summary.handoff_downrange_m, 1200.0)
        self.assertAlmostEqual(summary.handoff_speed_mps, math.hypot(1900.0, 100.0))
        self.assertGreaterEqual(summary.pre_handoff_rocket_drag_loss_mps, 0.0)
        self.assertIsNotNone(summary.stage2_measured_alignment_loss_mps)
        self.assertEqual(summary.stage2_assumed_unmodeled_loss_mps, 500.0)
        self.assertIsNotNone(summary.stage2_margin_mps)

    def test_energy_identity_includes_resistance_work(self) -> None:
        before = state(0.0, 0.0, 0.0)
        after = state(1.0, 4.0, 8.0)
        batches = (
            EffectBatch(
                source=SLOT_LAUNCH_FORCE,
                wrenches=(
                    Wrench(
                        source=SLOT_LAUNCH_FORCE,
                        body=BODY_CART,
                        force_n=(10.0, 0.0, 0.0),
                        application_point_m=(0.0, 0.0, 0.0),
                        frame=Frame.WORLD,
                    ),
                ),
            ),
            EffectBatch(
                source=SLOT_GUIDE,
                wrenches=(
                    Wrench(
                        source=SLOT_GUIDE,
                        body=BODY_CART,
                        force_n=(-2.0, 0.0, 0.0),
                        application_point_m=(0.0, 0.0, 0.0),
                        frame=Frame.WORLD,
                    ),
                ),
            ),
        )
        accepted = aggregate(batches, before)
        energy = EnergyAccumulator(before, gravity_mps2=(0.0, 0.0, 0.0))
        result = energy.update(before, after, accepted)
        self.assertAlmostEqual(result.work_j["launch"], 40.0)
        self.assertAlmostEqual(result.work_j["resistance"], -8.0)
        self.assertAlmostEqual(result.mechanical_change_j, 32.0)
        self.assertAlmostEqual(result.residual_j, 0.0)
        rotating = SimulationState(
            time_s=0.0,
            step_index=0,
            dt_s=0.1,
            bodies={
                BODY_CART: BodyState(name=BODY_CART, angular_velocity=(0.0, 0.0, 1.0)),
            },
        ).frozen()
        # Rotation makes the translational identity incomplete, so the energy channels are
        # marked invalid rather than raising. A rocket rotating within its own section 10.4
        # ignition gate (5 deg/s, eight orders of magnitude above the noise tolerance) is a
        # valid physical state, and section 14 requires an unavailable value to be recorded
        # as null with a validity flag rather than to abort the record carrying it.
        spinning = EnergyAccumulator(rotating)
        self.assertFalse(spinning.snapshot.valid)
        self.assertIn("without modeled body inertia", str(spinning.snapshot.invalid_reason))

        # Invalidity latches: the accumulated mechanical change has already missed a
        # rotational contribution, so later residuals stay suspect even once rotation stops.
        settled = SimulationState(
            time_s=0.1,
            step_index=1,
            dt_s=0.1,
            bodies={BODY_CART: BodyState(name=BODY_CART)},
        ).frozen()
        self.assertFalse(spinning.update(rotating, settled, aggregate((), rotating)).valid)

        # A run that never rotates stays valid and reports a closed residual.
        self.assertTrue(result.valid)
        self.assertIsNone(result.invalid_reason)

    def test_every_force_emitting_slot_maps_to_a_work_term(self) -> None:
        # A slot that can apply a wrench but has no work term would have its work vanish
        # from the residual with nothing to attribute the discrepancy to. Section 10.3
        # anticipates replacing the baseline passive actuator with a pusher, so the
        # separation slot in particular must already carry a term.
        from skyarc.names import SLOT_BODY_OWNERSHIP, SLOT_SEPARATION_ACTUATOR
        from skyarc.telemetry.energy import SLOT_WORK_TERMS

        for slot, bodies in SLOT_BODY_OWNERSHIP.items():
            if bodies:
                self.assertIn(slot, SLOT_WORK_TERMS, f"slot {slot!r} may apply force")

        before = state(0.0, 0.0, 0.0)
        after = state(0.1, 1.0, 0.0)  # cart advances one metre along +x
        energy = EnergyAccumulator(before, gravity_mps2=(0.0, 0.0, 0.0))
        pusher = AppliedEffects(
            loads={
                BODY_CART: BodyLoad(
                    body=BODY_CART,
                    force_n=(1000.0, 0.0, 0.0),
                    force_by_slot={SLOT_SEPARATION_ACTUATOR: (1000.0, 0.0, 0.0)},
                )
            }
        )
        snapshot = energy.update(before, after, pusher)
        self.assertAlmostEqual(snapshot.work_j["separation"], 1000.0)
        self.assertAlmostEqual(sum(snapshot.work_j.values()), 1000.0)

        # An unmapped slot is a wiring error and must be loud, not silently absorbed.
        rogue = AppliedEffects(
            loads={
                BODY_CART: BodyLoad(
                    body=BODY_CART,
                    force_n=(1.0, 0.0, 0.0),
                    force_by_slot={"telemetry_sink": (1.0, 0.0, 0.0)},
                )
            }
        )
        with self.assertRaisesRegex(ValueError, "maps to no work term"):
            energy.update(before, after, rogue)

    def test_rotating_run_keeps_full_wrench_work_reportable(self) -> None:
        before = state(0.0, 0.0, 0.0)
        after = SimulationState(
            time_s=0.1,
            step_index=1,
            dt_s=0.1,
            bodies={
                BODY_CART: BodyState(
                    name=BODY_CART,
                    angular_velocity=(0.0, 2.0, 0.0),
                    mass_kg=1.0,
                ),
                BODY_ROCKET: BodyState(name=BODY_ROCKET, mass_kg=1.0),
            },
            joint_active={JOINT_COUPLING: False, JOINT_GUIDE: True},
            collision_pair_active={PAIR_ROCKET_CRADLE: True},
        ).frozen()
        accepted = aggregate((), before)
        applied = AppliedEffects(
            loads={
                BODY_CART: BodyLoad(
                    body=BODY_CART,
                    torque_nm=(0.0, 10.0, 0.0),
                    torque_by_slot={SLOT_BACKEND_ADAPTER: (0.0, 10.0, 0.0)},
                )
            }
        )
        with tempfile.TemporaryDirectory() as temporary:
            paths = RunPaths.create(
                temporary,
                experiment_id="experiment",
                condition_id="condition",
                replicate_id=0,
            )
            recorder = TelemetryRecorder(
                paths,
                before,
                flat_layout(),
                telemetry_rate_hz=10.0,
                target_exit_speed_mps=10.0,
            )
            recorder.record_step(
                StepTelemetryInput(
                    pre_state=before,
                    observation=observation(before),
                    command_snapshots={},
                    accepted_effects=accepted,
                    applied_effects=applied,
                    post_state=after,
                    post_observation=observation(after),
                    mission_state=MissionState(),
                )
            )
            summary = recorder.finalize(termination_reason="test", mission_phase="idle")
            with paths.telemetry_csv.open(newline="", encoding="utf-8") as stream:
                row = next(csv.DictReader(stream))
            self.assertFalse(summary.energy_closure_valid)
            self.assertEqual(row["energy.residual_j"], "")
            self.assertEqual(row["energy.residual_j__valid"], "False")
            self.assertAlmostEqual(float(row["energy.work_guide_reaction_j"]), 1.0)
            self.assertEqual(row["energy.work_guide_reaction_j__valid"], "True")

    def test_recorder_writes_explicit_validity_sequences_and_registered_diagnostics(self) -> None:
        before = state(0.0, 0.0, 0.0)
        after = state(0.1, 0.0, 0.0)
        accepted = aggregate((), before)
        diagnostic_schema = DiagnosticSchema(
            namespace="testmodel",
            version="1",
            fields={"testmodel.temperature": DiagnosticField("K")},
        )
        diagnostic = DiagnosticRecord.create(
            source="test_slot",
            schema=diagnostic_schema,
            values={"testmodel.temperature": 300.0},
        )
        with tempfile.TemporaryDirectory() as temporary:
            paths = RunPaths.create(
                temporary,
                experiment_id="experiment",
                condition_id="condition",
                replicate_id=0,
            )
            recorder = TelemetryRecorder(
                paths,
                before,
                flat_layout(),
                telemetry_rate_hz=10.0,
                target_exit_speed_mps=10.0,
                diagnostic_schemas=(diagnostic_schema,),
            )
            recorder.record_step(
                StepTelemetryInput(
                    pre_state=before,
                    observation=observation(before),
                    command_snapshots={},
                    accepted_effects=accepted,
                    applied_effects=AppliedEffects.exactly(accepted),
                    post_state=after,
                    post_observation=observation(after),
                    mission_state=MissionState(),
                )
            )
            sequenced = recorder.record_events(
                (
                    Event(EVENT_STAGE_TRANSITION, 0.05, 1, "test", {"stage": 1}),
                    Event(EVENT_ABORT, 0.1, 1, "test", {"reason": "test"}),
                )
            )
            recorder.record_diagnostics((diagnostic,), time_s=0.1, step_index=1)
            summary = recorder.finalize(termination_reason="test", mission_phase="abort")
            self.assertEqual([event.sequence for event in sequenced], [0, 1])
            self.assertEqual(summary.telemetry_samples, 1)

            with paths.telemetry_csv.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["geometry.radius_m"], "")
            self.assertEqual(rows[0]["geometry.radius_m__valid"], "False")
            self.assertEqual(rows[0]["time_s__valid"], "True")
            self.assertNotIn("nan", paths.telemetry_csv.read_text(encoding="utf-8").lower())
            events = [json.loads(line) for line in paths.events_jsonl.read_text(encoding="utf-8").splitlines()]
            self.assertEqual([event["sequence"] for event in events], [0, 1])
            diagnostics = [
                json.loads(line) for line in paths.diagnostics_jsonl.read_text(encoding="utf-8").splitlines()
            ]
            self.assertEqual(diagnostics[0]["values"]["testmodel.temperature"], 300.0)
            sidecar = json.loads(paths.telemetry_schema.read_text(encoding="utf-8"))
            self.assertEqual(sidecar["validity_policy"], "per_field_v1")
            self.assertEqual(sidecar["quaternion_order"], "wxyz")
            self.assertIn("testmodel", sidecar["diagnostic_schemas"])

            other_schema = DiagnosticSchema(
                namespace="other",
                version="1",
                fields={"other.value": DiagnosticField("1")},
            )
            other_record = DiagnosticRecord.create(
                source="test_slot",
                schema=other_schema,
                values={"other.value": 1.0},
            )
            closed_paths = RunPaths.create(
                temporary,
                experiment_id="experiment",
                condition_id="unregistered",
                replicate_id=0,
            )
            second_recorder = TelemetryRecorder(
                closed_paths,
                before,
                flat_layout(),
                telemetry_rate_hz=10.0,
            )
            with self.assertRaisesRegex(ValueError, "not registered"):
                second_recorder.record_diagnostics((other_record,), time_s=0.0, step_index=0)
            second_recorder.close()


class TelemetryMissionIntegrationTests(unittest.TestCase):
    def test_complete_mission_streams_and_finalizes_telemetry(self) -> None:
        loaded = load_yaml(BASELINE)
        config = loaded.config
        layout = resolve_tube_layout(config)
        initial_pose = path_pose(layout, 0.0)
        half_angle = -0.5 * math.radians(initial_pose.inclination_deg)
        orientation = (math.cos(half_angle), 0.0, math.sin(half_angle), 0.0)
        initial = SimulationState(
            time_s=0.0,
            step_index=0,
            dt_s=config.simulation.physics_dt_s,
            bodies={
                BODY_CART: BodyState(
                    name=BODY_CART,
                    position=initial_pose.position_m,
                    orientation=orientation,
                    mass_kg=config.cart.mass_kg,
                ),
                BODY_ROCKET: BodyState(
                    name=BODY_ROCKET,
                    position=initial_pose.position_m,
                    orientation=orientation,
                    mass_kg=config.rocket.initial_mass_kg,
                ),
            },
            joint_active={JOINT_COUPLING: True, JOINT_GUIDE: True},
            collision_pair_active={PAIR_ROCKET_CRADLE: True},
        ).frozen()
        backend = AnalyticBackend(initial, layout)
        with tempfile.TemporaryDirectory() as temporary:
            paths = RunPaths.create(
                temporary,
                experiment_id=config.experiment.experiment_id,
                condition_id=config.experiment.condition_id,
                replicate_id=config.experiment.replicate_id,
            )
            recorder = TelemetryRecorder(
                paths,
                initial,
                layout,
                telemetry_rate_hz=20.0,
                target_exit_speed_mps=config.launch_control.target_exit_speed_mps,
                attached_load_limit_g=config.launch_control.maximum_resultant_load_g,
                cart_load_limit_g=config.cart.maximum_resultant_load_g,
            )
            mission = build_mission(
                config,
                layout,
                backend,
                free_flight_duration_s=0.5,
                telemetry_sink=recorder,
            )
            result = mission.run()
            self.assertEqual(result.mission_state.phase.value, "complete")
            self.assertIsNotNone(result.telemetry_summary)
            summary = json.loads(paths.summary_json.read_text(encoding="utf-8"))
            self.assertEqual(summary["termination_reason"], "complete")
            self.assertGreater(summary["telemetry_samples"], 1)
            self.assertGreater(summary["event_count"], 1)
            event_rows = [
                json.loads(line) for line in paths.events_jsonl.read_text(encoding="utf-8").splitlines()
            ]
            self.assertEqual(
                [row["sequence"] for row in event_rows],
                list(range(len(event_rows))),
            )
            self.assertTrue(paths.telemetry_csv.is_file())
            self.assertTrue(paths.telemetry_schema.is_file())


if __name__ == "__main__":
    unittest.main()
