# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Execute the common mission orchestrator on the production PhysX/CPU launcher scene.

``SimulationApp`` is constructed before any extension-dependent import.  This runner owns no
mission logic of its own: it resolves one configuration, builds the qualified scene, hands
the resulting :class:`IsaacPhysxBackend` to the same ``build_mission`` used by the analytic
suite, and steps that orchestrator.  Everything the orchestrator does -- observation,
component pre-step, aggregation, backend apply, release resync, integration, read-back,
boundary detection, post-step, telemetry -- stays where it already is.

``--max-steps`` bounds the run for smoke and integration use.  A bounded run is reported
with ``completed_mission: false``; it is not a shorter version of an accepted result.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
import traceback
from pathlib import Path
from typing import Any


def _arguments() -> argparse.Namespace:
    project = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--configuration", type=Path, default=project / "configs" / "curved_2kms.yaml")
    parser.add_argument(
        "--fixture",
        type=Path,
        default=project / "configs" / "phase0_anti_tunneling_open_cradle.json",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--max-steps",
        type=int,
        help="Stop after this many physics steps and report an incomplete mission.",
    )
    parser.add_argument(
        "--telemetry-directory",
        type=Path,
        help="Write a full telemetry run instance beneath this directory.",
    )
    parser.add_argument(
        "--reset-replay",
        action="store_true",
        help="After the run, exercise the stop/rebuild reset and report the restored state.",
    )
    parser.add_argument("--visuals", action="store_true", help="Author the tube visualization bands.")
    parser.add_argument("--summary", type=Path)
    return parser.parse_args()


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _source_closure(root: Path) -> dict[str, Any]:
    """Hash every production Python source, excluding caches and tests."""
    files = sorted(
        path
        for path in root.rglob("*.py")
        if "__pycache__" not in path.parts and "tests" not in path.parts
    )
    if not files:
        raise RuntimeError(f"project source closure is empty: {root}")
    file_hashes: dict[str, str] = {}
    digest = hashlib.sha256()
    for path in files:
        relative = path.relative_to(root).as_posix()
        file_hash = _sha256(path)
        file_hashes[relative] = file_hash
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(bytes.fromhex(file_hash))
        digest.update(b"\0")
    return {
        "root": str(root.resolve()),
        "sha256": digest.hexdigest(),
        "files": file_hashes,
    }


def _git_head(repository: Path) -> str | None:
    """Resolve HEAD without invoking a host-dependent Git executable."""
    git_directory = repository / ".git"
    head_path = git_directory / "HEAD"
    if not head_path.is_file():
        return None
    head = head_path.read_text(encoding="utf-8").strip()
    if not head.startswith("ref: "):
        return head or None
    reference = head.removeprefix("ref: ")
    loose_reference = git_directory / reference
    if loose_reference.is_file():
        return loose_reference.read_text(encoding="utf-8").strip() or None
    packed_references = git_directory / "packed-refs"
    if packed_references.is_file():
        for line in packed_references.read_text(encoding="utf-8").splitlines():
            if line.startswith(("#", "^")):
                continue
            fields = line.split(" ", 1)
            if len(fields) == 2 and fields[1] == reference:
                return fields[0]
    return None


def main() -> int:
    args = _arguments()
    if args.max_steps is not None and args.max_steps <= 0:
        raise SystemExit("--max-steps must be positive when supplied")
    if args.reset_replay and args.telemetry_directory is not None:
        # ``SimulationOrchestrator.reset`` refuses to replay into a live run instance
        # because a second pass would append to a record that claims to be one run.
        raise SystemExit("--reset-replay requires a run without --telemetry-directory")
    isaac_path = os.environ.get("ISAAC_PATH")
    if not isaac_path:
        raise SystemExit("ISAAC_PATH is not set; launch through the target build's python.bat")
    project = Path(__file__).resolve().parents[1]
    release_root = Path(isaac_path).resolve()
    isaac_root = release_root.parents[2]
    experience = release_root / "apps" / "isaacsim.exp.full.kit"
    version_file = isaac_root / "VERSION"
    executable = Path(sys.executable).resolve()
    package_root = project / "exts" / "skyarc" / "skyarc"
    app = None
    recorder = None
    try:
        from isaacsim import SimulationApp

        app = SimulationApp(
            {
                "headless": bool(args.headless),
                "extra_args": [
                    "--/app/player/useFixedTimeStepping=true",
                    "--/app/runLoops/main/rateLimitEnabled=false",
                    "--/app/settings/persistent=0",
                ],
            },
            experience=str(experience),
        )

        import carb.settings
        import omni.kit.app
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import UsdPhysics

        extension_root = project / "exts" / "skyarc"
        extension_manager = omni.kit.app.get_app().get_extension_manager()
        extension_manager.add_path(str(extension_root.parent.resolve()))
        if not extension_manager.set_extension_enabled_immediate(
            "skyarc", True
        ):
            raise RuntimeError("failed to enable local vacuum-tube launcher extension")

        package_parent = extension_root
        if str(package_parent) not in sys.path:
            sys.path.insert(0, str(package_parent))
        from skyarc.configuration import load_yaml, resolve_tube_layout
        from skyarc.launcher.geometry import CurvedTubeLayout
        from skyarc.launcher.production import (
            build_production_scene_plan,
            load_production_fixture,
        )
        from skyarc.launcher.production_runtime import ProductionMissionRuntime
        from skyarc.names import BODY_CART, BODY_ROCKET
        from skyarc.state_machine import MissionPhase
        from skyarc.telemetry import (
            CORE_TELEMETRY_SCHEMA_V2,
            RunPaths,
            TelemetryRecorder,
        )

        loaded = load_yaml(args.configuration)
        config = loaded.config
        layout = resolve_tube_layout(config)
        if not isinstance(layout, CurvedTubeLayout):
            raise RuntimeError("production mission requires a schema-v3 curved layout")
        fixture = load_production_fixture(args.fixture)
        plan = build_production_scene_plan(config, layout, fixture)

        stage_utils.create_new_stage()
        stage = stage_utils.get_current_stage()
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr().Set((0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        SimulationManager.set_default_physics_scene("/World/PhysicsScene")
        switched = SimulationManager.switch_physics_engine(plan.backend, verbose=False)
        SimulationManager.setup_simulation(dt=config.simulation.physics_dt_s, device=plan.device)
        SimulationManager.enable_ccd(False, physics_scene="/World/PhysicsScene")

        # Read gravity back from the authored scene rather than restating it, so a scene
        # change cannot silently desynchronize the controller and the solver.
        gravity_direction = scene.GetGravityDirectionAttr().Get()
        gravity_magnitude = float(scene.GetGravityMagnitudeAttr().Get())
        gravity_mps2 = tuple(float(value) * gravity_magnitude for value in gravity_direction)

        settings = carb.settings.get_settings()
        # Sampled here, before the runtime starts the timeline, which is the same point in
        # the lifecycle the Phase 0 runtime-selection probe reads them at. Starting playback
        # turns the main run loop's rate limiter back on; that does not throttle this run,
        # which drives ``SimulationManager.step`` directly rather than through app updates,
        # but it does mean a post-run read is not comparable to the qualified evidence. Both
        # readings are reported so the difference is visible instead of inferred.
        startup_settings = {
            "fixed_time_stepping": settings.get("/app/player/useFixedTimeStepping"),
            "rate_limit_enabled": settings.get("/app/runLoops/main/rateLimitEnabled"),
        }

        sink_factory = None
        telemetry_run: dict[str, object] | None = None
        if args.telemetry_directory is not None:
            args.telemetry_directory.mkdir(parents=True, exist_ok=True)

            def sink_factory(initial_state):  # type: ignore[misc]
                nonlocal recorder, telemetry_run
                paths = RunPaths.create(
                    args.telemetry_directory,
                    experiment_id=config.experiment.experiment_id,
                    condition_id=config.experiment.condition_id,
                    replicate_id=config.experiment.replicate_id,
                )
                recorder = TelemetryRecorder(
                    paths,
                    initial_state,
                    layout,
                    telemetry_rate_hz=config.output.telemetry_rate_hz,
                    target_exit_speed_mps=config.launch_control.target_exit_speed_mps,
                    attached_load_limit_g=config.launch_control.maximum_resultant_load_g,
                    cart_load_limit_g=config.cart.maximum_resultant_load_g,
                    gravity_mps2=gravity_mps2,
                )
                telemetry_run = {
                    "core_schema_version": CORE_TELEMETRY_SCHEMA_V2.version,
                    "run_instance_id": paths.run_instance_id,
                    "guide_reaction_work_column": (
                        "energy.work_guide_reaction_j" in CORE_TELEMETRY_SCHEMA_V2.fields
                    ),
                }
                return recorder

        runtime = ProductionMissionRuntime(
            stage,
            config,
            layout,
            fixture,
            plan,
            gravity_mps2=gravity_mps2,
            telemetry_sink_factory=sink_factory,
            author_visuals=bool(args.visuals),
            update=app.update,
        )

        mission = runtime.mission
        backend = runtime.backend
        peak_tracking_error_m = 0.0
        peak_attitude_error_deg = 0.0
        peak_commanded_normal_force_n = 0.0
        started = time.perf_counter()
        steps = 0
        while mission.mission_state.phase not in (MissionPhase.COMPLETE, MissionPhase.ABORT):
            if args.max_steps is not None and steps >= args.max_steps:
                break
            mission.step()
            steps += 1
            reaction = backend.last_guide_reaction
            if reaction is not None:
                peak_tracking_error_m = max(peak_tracking_error_m, reaction.tracking_error_m)
                peak_attitude_error_deg = max(
                    peak_attitude_error_deg, abs(math.degrees(reaction.attitude_error_rad))
                )
                peak_commanded_normal_force_n = max(
                    peak_commanded_normal_force_n, abs(reaction.commanded_normal_force_n)
                )
        wall_time_s = time.perf_counter() - started
        final_state = backend.read_state()
        run_phase = mission.mission_state.phase
        run_abort_reason = mission.mission_state.abort_reason
        run_events = tuple(mission.events)
        completed = run_phase is MissionPhase.COMPLETE
        # Captured before any reset replay. ``IsaacPhysxBackend.reset`` zeroes this counter,
        # so reading it while assembling the summary would report the restored initial
        # state's offset instead of the run's peak -- exactly the measurement the disclosed
        # post-release precision limitation is supposed to be bounded by.
        peak_solver_offset_m = backend.peak_solver_offset_m

        telemetry_summary = None
        if recorder is not None:
            telemetry_summary = recorder.finalize(
                termination_reason=(
                    run_abort_reason
                    if run_phase is MissionPhase.ABORT
                    else ("complete" if completed else "step_budget_exhausted")
                )
                or "abort",
                mission_phase=run_phase.value,
            ).to_dict()
            recorder.close()
            recorder = None

        reset_probe: dict[str, object] | None = None
        if args.reset_replay:
            # The whole orchestrator reset, not just the adapter's: the point of the probe
            # is that components, the state machine and the backend all return together.
            mission.reset()
            restored = backend.read_state()
            authored = runtime.initial_solver_states
            reference = runtime.reference_frame.sample(0.0)
            measured: dict[str, float] = {}
            for body in (BODY_CART, BODY_ROCKET):
                expected_position = tuple(
                    authored[body].position_m[index] + reference.position_m[index]
                    for index in range(3)
                )
                expected_velocity = tuple(
                    authored[body].linear_velocity_mps[index] + reference.velocity_mps[index]
                    for index in range(3)
                )
                actual = restored.body(body)
                measured[f"{body}_position_error_m"] = max(
                    abs(actual.position[index] - expected_position[index]) for index in range(3)
                )
                measured[f"{body}_velocity_error_mps"] = max(
                    abs(actual.linear_velocity[index] - expected_velocity[index])
                    for index in range(3)
                )
            # The tolerance is the same float32 round-trip allowance the Phase 0 reset
            # probe used; it is deliberately much tighter than the guide clearance so a
            # centimetre of reset misplacement cannot pass as a rounding residue.
            tolerance_m = 1e-5
            reset_probe = {
                **measured,
                "time_s": restored.time_s,
                "step_index": restored.step_index,
                "mission_phase": mission.mission_state.phase.value,
                "event_count": len(mission.events),
                "maximum_state_error": tolerance_m,
                "passed": bool(
                    restored.time_s == 0.0
                    and restored.step_index == 0
                    and not mission.events
                    and all(value <= tolerance_m for value in measured.values())
                ),
            }

        summary: dict[str, object] = {
            "schema": "vacuum_tube_production_mission_v1",
            "configuration": str(args.configuration.resolve()),
            "configuration_source_sha256": _sha256(args.configuration.resolve()),
            "configuration_resolved_sha256": loaded.resolved_sha256,
            "fixture": str(args.fixture.resolve()),
            "fixture_sha256": _sha256(args.fixture.resolve()),
            "runner_sha256": _sha256(Path(__file__).resolve()),
            "provenance": {
                "runner_sha256": _sha256(Path(__file__).resolve()),
                "project_source_closure": _source_closure(package_root),
                "configuration_sha256": _sha256(args.configuration.resolve()),
                "fixture_sha256": _sha256(args.fixture.resolve()),
                "experience_sha256": _sha256(experience),
                "version_file_sha256": _sha256(version_file),
                "python_executable_sha256": _sha256(executable),
                "isaac_sim_source_git_revision": _git_head(isaac_root),
            },
            "candidate": plan.candidate,
            "coordinate_frame": plan.coordinate_frame,
            "reaction_evidence": plan.reaction_evidence,
            "backend": SimulationManager.get_active_physics_engine(),
            "device": str(SimulationManager.get_device()),
            "physics_dt_s": SimulationManager.get_physics_dt(),
            "solver_type": str(SimulationManager.get_solver_type()),
            "engine_switch_returned": bool(switched),
            "fixed_time_stepping": startup_settings["fixed_time_stepping"],
            "rate_limit_enabled": startup_settings["rate_limit_enabled"],
            "fixed_time_stepping_during_playback": settings.get(
                "/app/player/useFixedTimeStepping"
            ),
            "rate_limit_enabled_during_playback": settings.get(
                "/app/runLoops/main/rateLimitEnabled"
            ),
            "controller_gains": {
                "normal_kp_per_s2": runtime.guide_reaction.gains.normal_kp_per_s2,
                "normal_kd_per_s": runtime.guide_reaction.gains.normal_kd_per_s,
                "attitude_kp_per_s2": runtime.guide_reaction.gains.attitude_kp_per_s2,
                "attitude_kd_per_s": runtime.guide_reaction.gains.attitude_kd_per_s,
            },
            "reference_profile_acceleration_mps2": runtime.reference_frame.acceleration_mps2,
            "reference_profile_exit_time_s": runtime.reference_frame.exit_time_s,
            "requested_max_steps": args.max_steps,
            "physics_steps": steps,
            "wall_time_s": wall_time_s,
            "physics_steps_per_wall_second": steps / wall_time_s if wall_time_s > 0.0 else None,
            "mission_phase": run_phase.value,
            "abort_reason": run_abort_reason,
            "completed_mission": completed,
            "event_names": sorted({event.name for event in run_events}),
            "event_count": len(run_events),
            "final_time_s": final_state.time_s,
            "final_cart_speed_mps": math.sqrt(
                sum(value * value for value in final_state.body(BODY_CART).linear_velocity)
            ),
            "peak_centerline_tracking_error_m": peak_tracking_error_m,
            "peak_attitude_error_deg": peak_attitude_error_deg,
            "peak_commanded_normal_force_n": peak_commanded_normal_force_n,
            "peak_solver_offset_m": peak_solver_offset_m,
            "guide_clearance_m": config.tube.guide_clearance_m,
            "telemetry_summary": telemetry_summary,
            "telemetry_run": telemetry_run,
            "reset_replay": reset_probe,
        }
        summary["passed"] = bool(
            summary["backend"] == "physx"
            and "cpu" in str(summary["device"]).lower()
            and summary["fixed_time_stepping"] is True
            and summary["rate_limit_enabled"] is False
            and steps > 0
            and run_phase is not MissionPhase.ABORT
            and peak_tracking_error_m <= config.tube.guide_clearance_m
            and (reset_probe is None or bool(reset_probe["passed"]))
        )

        rendered = json.dumps(summary, indent=2, sort_keys=True, allow_nan=False)
        if args.summary is not None:
            args.summary.parent.mkdir(parents=True, exist_ok=True)
            args.summary.write_text(rendered + "\n", encoding="utf-8")
        print(rendered)
        sys.stdout.flush()
        return 0 if summary["passed"] else 2
    except BaseException:
        # ``SimulationApp.close`` terminates the process, so an exception that escaped into
        # the ``finally`` below would be reported as a silent exit code zero. Print it here,
        # while the interpreter is still alive to do so.
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        return 3
    finally:
        if recorder is not None:
            recorder.close()
        if app is not None:
            app.close()


if __name__ == "__main__":
    raise SystemExit(main())
