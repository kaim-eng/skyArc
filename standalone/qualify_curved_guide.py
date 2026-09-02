# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Qualify the force-resolved curved-path guide on the selected CPU PhysX build.

The candidate is deliberately explicit: it applies a measured world-frame guide force and
attitude torque, never writes transforms while running, and compares backend acceleration with
the commanded reaction.  It is not represented as a native path joint because the target build
does not expose one.  Release and stop/rebuild reset use the already-qualified external fixed
joint and startup-authored collision pair.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import sys
import time
import traceback
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

from qualify_anti_tunneling import (
    _flatten_numbers,
    _git_head,
    _json_value,
    _sha256_file,
    _vector,
)


def _source_closure(root: Path) -> dict[str, Any]:
    """Hash the conservative project source closure used by this evidence runner."""
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
        file_hash = _sha256_file(path)
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


def _arguments() -> argparse.Namespace:
    project = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--configuration", type=Path, default=project / "configs" / "curved_2kms.yaml")
    parser.add_argument(
        "--fixture",
        type=Path,
        default=project / "configs" / "phase0_anti_tunneling_open_cradle.json",
    )
    parser.add_argument("--physics-dt-s", type=float, default=0.001)
    parser.add_argument("--target-exit-speed-mps", type=float, default=2000.0)
    parser.add_argument("--start-s-m", type=float, default=0.0)
    parser.add_argument("--end-s-m", type=float)
    parser.add_argument("--telemetry-stride", type=int, default=100)
    parser.add_argument("--normal-kp-per-s2", type=float, default=400.0)
    parser.add_argument("--normal-kd-per-s", type=float, default=0.0)
    parser.add_argument("--attitude-kp-per-s2", type=float, default=2500.0)
    parser.add_argument("--attitude-kd-per-s", type=float, default=100.0)
    parser.add_argument(
        "--coordinate-frame",
        choices=("co_moving", "global"),
        default="co_moving",
        help="Use a translated accelerating frame to avoid float32 drift, or retain global coordinates as a control.",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def _dot(left: list[float] | tuple[float, ...], right: list[float] | tuple[float, ...]) -> float:
    return sum(a * b for a, b in zip(left, right, strict=True))


def _sub(left: list[float], right: list[float] | tuple[float, ...]) -> list[float]:
    return [a - b for a, b in zip(left, right, strict=True)]


def _norm(vector: list[float]) -> float:
    return math.sqrt(_dot(vector, vector))


def _quaternion(value: Any) -> list[float]:
    converted = _json_value(value)
    while isinstance(converted, list) and len(converted) == 1:
        converted = converted[0]
    if not isinstance(converted, list) or len(converted) != 4:
        raise RuntimeError(f"expected a four-component quaternion, got {converted!r}")
    return [float(component) for component in converted]


def _maximum_error(left: list[float], right: list[float]) -> float:
    return max(abs(a - b) for a, b in zip(left, right, strict=True))


def _wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _path_orientation(angle_rad: float) -> list[float]:
    return [math.cos(0.5 * angle_rad), 0.0, -math.sin(0.5 * angle_rad), 0.0]


def _forward_angle(quaternion_wxyz: list[float]) -> float:
    w, x, y, z = quaternion_wxyz
    forward_x = 1.0 - 2.0 * (y * y + z * z)
    forward_z = 2.0 * (x * z - w * y)
    return math.atan2(forward_z, forward_x)


def _curved_guide_passed(
    *,
    reached_end: bool,
    windowed_sample_count: int,
    reaction_sample_count: int,
    transform_writes_during_run: int,
    peak_tracking_error_m: float,
    peak_attitude_error_deg: float,
    peak_resultant_load_g: float,
    peak_attachment_load_g: float,
    peak_attachment_geometry_error_m: float,
    peak_backend_force_relative_error: float,
    peak_reaction_relative_error: float,
    maximum_attached_pair_force_n: float,
    acceptance: Mapping[str, float],
) -> bool:
    """Decide the curved-guide mechanism verdict from measured peaks.

    The sample counts are preconditions rather than statistics.  A windowed or gated peak
    that never saw a sample is zero by initialization, not by measurement, so without
    them any run shorter than the settling window satisfies the load, backend-force and
    reaction gates without ever having evaluated one of them.
    """
    if not reached_end:
        return False
    if windowed_sample_count <= 0 or reaction_sample_count <= 0:
        return False
    if transform_writes_during_run != 0:
        return False
    return (
        peak_tracking_error_m <= acceptance["maximum_tracking_error_m"]
        and peak_attitude_error_deg <= acceptance["maximum_attitude_error_deg"]
        and peak_resultant_load_g <= acceptance["maximum_resultant_load_g"]
        and peak_attachment_load_g <= acceptance["maximum_attachment_load_g"]
        and peak_attachment_geometry_error_m
        <= acceptance["maximum_attachment_geometry_error_m"]
        and peak_backend_force_relative_error
        <= acceptance["maximum_backend_force_relative_error"]
        and peak_reaction_relative_error
        <= acceptance["maximum_reaction_correction_relative_error"]
        and maximum_attached_pair_force_n <= acceptance["maximum_attached_pair_force_n"]
    )


def _default_output(script: Path, started: datetime, run_id: str) -> Path:
    stamp = started.strftime("%Y%m%dT%H%M%S.%fZ")
    return script.parents[1] / "artifacts" / "phase0" / "curved_guide" / f"{stamp}_{run_id}.json"


def _progress(label: str) -> None:
    print(f"CURVED_GUIDE_PROGRESS={label}", flush=True)


def main() -> int:
    args = _arguments()
    for name, value in (
        ("physics timestep", args.physics_dt_s),
        ("target exit speed", args.target_exit_speed_mps),
        ("start path coordinate", args.start_s_m),
    ):
        if not math.isfinite(value) or (name != "start path coordinate" and value <= 0.0):
            raise SystemExit(f"{name} must be finite" + (" and positive" if name != "start path coordinate" else ""))
    if args.telemetry_stride <= 0:
        raise SystemExit("--telemetry-stride must be positive")
    isaac_path = os.environ.get("ISAAC_PATH")
    if not isaac_path:
        raise SystemExit("ISAAC_PATH is not set; launch through the target build's python.bat")

    release_root = Path(isaac_path).resolve()
    isaac_root = release_root.parents[2]
    experience = release_root / "apps" / "isaacsim.exp.full.kit"
    script = Path(__file__).resolve()
    helper = script.with_name("qualify_anti_tunneling.py")
    project_source_root = script.parents[1] / "exts" / "skyarc" / "skyarc"
    project_source_closure = _source_closure(project_source_root)
    started = datetime.now(timezone.utc)
    run_id = uuid.uuid4().hex
    output = args.output or _default_output(script, started, run_id)
    if output.exists() and not args.overwrite:
        raise SystemExit(f"artifact exists: {output}; pass --overwrite or choose another path")
    version_file = isaac_root / "VERSION"
    executable = Path(sys.executable).resolve()
    result: dict[str, Any] = {
        "schema": "vacuum_tube_curved_guide_qualification_v1",
        "run_id": run_id,
        "started_utc": started.isoformat(),
        "requested": {
            "backend": "physx",
            "device": "cpu",
            "physics_dt_s": args.physics_dt_s,
            "target_exit_speed_mps": args.target_exit_speed_mps,
            "start_s_m": args.start_s_m,
            "end_s_m": args.end_s_m,
            "configuration": str(args.configuration.resolve()),
            "fixture": str(args.fixture.resolve()),
            "candidate": "force_resolved_path_controller_v1",
            "coordinate_frame": args.coordinate_frame,
            "production_geometry": {
                "cradle_topology": "open_front_u",
                "rocket_shape": "cylinder",
                "rocket_axis": "X",
                "fixture_path": str(args.fixture.resolve()),
            },
            "ccd_enabled": False,
            # The gains are part of the result identity: the runner defaults do not
            # reproduce the accepted artifact, so binding only the runner and
            # configuration hashes under-specifies the run.
            "normal_kp_per_s2": args.normal_kp_per_s2,
            "normal_kd_per_s": args.normal_kd_per_s,
            "attitude_kp_per_s2": args.attitude_kp_per_s2,
            "attitude_kd_per_s": args.attitude_kd_per_s,
            "telemetry_stride": args.telemetry_stride,
        },
        "host": {
            "python": platform.python_version(),
            "platform": platform.platform(),
            "isaac_sim_version": version_file.read_text(encoding="utf-8").strip(),
        },
        "provenance": {
            "runner_path": str(script),
            "runner_sha256": _sha256_file(script),
            "helper_path": str(helper),
            "helper_sha256": _sha256_file(helper),
            "project_source_closure": project_source_closure,
            "configuration_sha256": _sha256_file(args.configuration.resolve()),
            "fixture_sha256": _sha256_file(args.fixture.resolve()),
            "experience_sha256": _sha256_file(experience),
            "version_file_sha256": _sha256_file(version_file),
            "python_executable_sha256": _sha256_file(executable),
            "isaac_sim_source_git_revision": _git_head(isaac_root),
        },
        "probes": {},
        "passed": False,
    }
    app = None
    exit_code = 2
    try:
        _progress("constructing_simulation_app")
        from isaacsim import SimulationApp

        app = SimulationApp(
            {
                "headless": True,
                "extra_args": [
                    "--/app/player/useFixedTimeStepping=true",
                    "--/app/runLoops/main/rateLimitEnabled=false",
                    "--/app/settings/persistent=0",
                ],
            },
            experience=str(experience),
        )
        _progress("simulation_app_ready")

        package_parent = script.parents[1] / "exts" / "skyarc"
        sys.path.insert(0, str(package_parent))
        import carb.settings
        import isaacsim.core.experimental.utils.app as app_utils
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.prims import RigidPrim
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import Gf, UsdPhysics
        from skyarc.configuration.loader import load_yaml
        from skyarc.configuration.validation import resolve_tube_layout
        from skyarc.launcher.geometry import CurvedTubeLayout, path_pose
        from skyarc.launcher.production import (
            build_production_scene_plan,
            load_production_fixture,
        )
        from skyarc.launcher.scene import build_launcher_scene

        # Counts every transform write this runner performs, so that
        # "no transform writes during integration" is a measured zero rather than
        # an asserted literal.
        transform_write_counter = [0]

        def author_stopped_rigid_state(
            prim_path: str,
            position: list[float],
            orientation_wxyz: list[float],
            linear_velocity: list[float],
            angular_velocity_rad_s: list[float],
        ) -> dict[str, str]:
            """Author the USD state used by the next physics rebuild."""
            transform_write_counter[0] += 1
            prim = stage.GetPrimAtPath(prim_path)
            translate_attr = prim.GetAttribute("xformOp:translate")
            orient_attr = prim.GetAttribute("xformOp:orient")
            translate_type = str(translate_attr.GetTypeName())
            orient_type = str(orient_attr.GetTypeName())
            translate_value = (
                Gf.Vec3f(*position) if translate_type == "float3" else Gf.Vec3d(*position)
            )
            w, x, y, z = orientation_wxyz
            orient_value = (
                Gf.Quatf(w, Gf.Vec3f(x, y, z))
                if orient_type == "quatf"
                else Gf.Quatd(w, Gf.Vec3d(x, y, z))
            )
            if not translate_attr.Set(translate_value) or not orient_attr.Set(orient_value):
                raise RuntimeError(f"failed to author stopped rigid transform for {prim_path}")
            rigid_api = UsdPhysics.RigidBodyAPI(prim)
            rigid_api.CreateVelocityAttr().Set(Gf.Vec3f(*linear_velocity))
            rigid_api.CreateAngularVelocityAttr().Set(
                Gf.Vec3f(*(math.degrees(value) for value in angular_velocity_rad_s))
            )
            return {"translate_type": translate_type, "orient_type": orient_type}

        loaded = load_yaml(args.configuration)
        config = loaded.config
        layout = resolve_tube_layout(config)
        if not isinstance(layout, CurvedTubeLayout):
            raise RuntimeError("curved-guide qualification requires a curved layout")
        fixture = load_production_fixture(args.fixture)
        scene_plan = build_production_scene_plan(config, layout, fixture)
        end_s = layout.length_m if args.end_s_m is None else args.end_s_m
        if not (0.0 <= args.start_s_m < end_s <= layout.length_m):
            raise RuntimeError("qualification interval must satisfy 0 <= start < end <= path length")
        result["requested"]["end_s_m"] = end_s

        # Runtime/source capability audit. D6 joints exist, but no path/spline joint type is
        # exposed; a D6 joint has only fixed local axes and cannot encode this centerline.
        native_types = {
            name: hasattr(UsdPhysics, name)
            for name in ("PathJoint", "SplineJoint", "D6Joint", "PrismaticJoint")
        }
        candidate_audit = {
            "usd_physics_types": native_types,
            "native_path_constraint": {
                "supported": native_types["PathJoint"] or native_types["SplineJoint"],
                "disposition": "reject: target USD/PhysX schema exposes no path or spline joint",
            },
            "curvature_resolved_joint_chain": {
                "supported": native_types["PrismaticJoint"],
                "disposition": "reject: fixed-axis joints require unqualified live topology switching through clothoids",
            },
            "measured_kinematic": {
                "supported": True,
                "disposition": "fallback only: transform prescription cannot validate physical reaction",
            },
            "force_resolved_path_controller": {
                "supported": True,
                "disposition": (
                    "selected for system-level production: no transform writes; reaction is commanded "
                    "and acceleration-verified, not a solver constraint reaction; "
                    "the default translated frame reconstructs global SI state and adds the exact "
                    "uniform fictitious body force"
                ),
            },
            # This audit records dispositions; it measures nothing. It must not
            # contribute an unconditional pass to the aggregate result.
            "informational": True,
            "measured_candidates": ["force_resolved_path_controller"],
            "formally_dispositioned_candidates": [
                "native_path_constraint",
                "curvature_resolved_joint_chain",
                "measured_kinematic",
            ],
        }
        result["probes"]["candidate_capability_audit"] = candidate_audit

        settings = carb.settings.get_settings()
        stage_utils.create_new_stage()
        stage = stage_utils.get_current_stage()
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr().Set((0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        SimulationManager.set_default_physics_scene("/World/PhysicsScene")
        switched = SimulationManager.switch_physics_engine("physx", verbose=False)
        SimulationManager.setup_simulation(dt=args.physics_dt_s, device="cpu")
        SimulationManager.enable_ccd(False, physics_scene="/World/PhysicsScene")
        runtime = {
            "switch_returned": switched,
            "active_engine": SimulationManager.get_active_physics_engine(),
            "device": str(SimulationManager.get_device()),
            "physics_dt_s": SimulationManager.get_physics_dt(),
            "solver_type": SimulationManager.get_solver_type(),
            "scene_ccd_enabled": SimulationManager.is_ccd_enabled("/World/PhysicsScene"),
            "settings": {
                key: settings.get(key)
                for key in (
                    "/app/player/useFixedTimeStepping",
                    "/app/runLoops/main/rateLimitEnabled",
                    "/exts/isaacsim.core.simulation_manager/default_engine",
                )
            },
        }
        runtime["passed"] = bool(
            switched
            and runtime["active_engine"] == "physx"
            and "cpu" in runtime["device"].lower()
            and math.isclose(runtime["physics_dt_s"], args.physics_dt_s, abs_tol=1e-12)
            and runtime["settings"]["/app/player/useFixedTimeStepping"] is True
            and runtime["settings"]["/app/runLoops/main/rateLimitEnabled"] is False
            and runtime["scene_ccd_enabled"] is False
            # TGS is part of the recorded qualified combination, so gate it rather
            # than merely recording it.
            and str(runtime["solver_type"]).upper().endswith("TGS")
        )
        result["probes"]["runtime_selection"] = runtime

        start_pose = path_pose(layout, args.start_s_m)
        start_angle = math.radians(start_pose.inclination_deg)
        start_orientation = _path_orientation(start_angle)
        cart_mass = fixture.cradle.mass_kg
        rocket_mass = fixture.rocket.mass_kg
        assembly_mass = cart_mass + rocket_mass
        offset_m = scene_plan.cart_to_rocket_offset_m
        com_from_cart_m = rocket_mass / assembly_mass * offset_m
        profile_acceleration = args.target_exit_speed_mps**2 / (2.0 * layout.length_m)
        start_speed = math.sqrt(2.0 * profile_acceleration * args.start_s_m)
        start_omega_y = -start_speed * start_pose.signed_curvature_per_m
        global_cart_position_start = [
            start_pose.position_m[i] - com_from_cart_m * start_pose.tangent[i] for i in range(3)
        ]
        global_rocket_position_start = [
            global_cart_position_start[i] + offset_m * start_pose.tangent[i] for i in range(3)
        ]
        global_cart_velocity_start = [
            start_speed * start_pose.tangent[i]
            + start_omega_y * com_from_cart_m * start_pose.normal[i]
            for i in range(3)
        ]
        rotational_offset_velocity = [
            start_omega_y * offset_m * start_pose.tangent[2],
            0.0,
            -start_omega_y * offset_m * start_pose.tangent[0],
        ]
        global_rocket_velocity_start = [
            global_cart_velocity_start[i] + rotational_offset_velocity[i] for i in range(3)
        ]
        if args.coordinate_frame == "co_moving":
            reference_position_start = list(start_pose.position_m)
            reference_velocity_start = [start_speed * component for component in start_pose.tangent]
        else:
            reference_position_start = [0.0, 0.0, 0.0]
            reference_velocity_start = [0.0, 0.0, 0.0]
        cart_position_start = [
            global_cart_position_start[i] - reference_position_start[i] for i in range(3)
        ]
        rocket_position = [
            global_rocket_position_start[i] - reference_position_start[i] for i in range(3)
        ]
        cart_velocity = [
            global_cart_velocity_start[i] - reference_velocity_start[i] for i in range(3)
        ]
        rocket_velocity = [
            global_rocket_velocity_start[i] - reference_velocity_start[i] for i in range(3)
        ]

        built_scene = build_launcher_scene(
            stage,
            config,
            scene_plan,
            cart_position_m=cart_position_start,
            rocket_position_m=rocket_position,
            orientation_wxyz=start_orientation,
            cart_velocity_mps=cart_velocity,
            rocket_velocity_mps=rocket_velocity,
            angular_velocity_radps=(0.0, start_omega_y, 0.0),
            author_visuals=False,
        )
        cart_path = built_scene.cart_path
        rocket_path = built_scene.rocket_path
        joint_path = built_scene.coupling_path
        cart = built_scene.cart
        rocket = built_scene.rocket
        fixed_joint = built_scene.coupling_joint

        app_utils.play()
        app.update()
        contact_view = SimulationManager.get_physics_simulation_view().create_rigid_contact_view(
            [rocket_path], filter_patterns=[[cart_path]], max_contact_data_count=16
        )
        # play() followed by update() advances physics before any guide force exists, so the
        # assembly free-falls for the duration of that first update.  Previously only the
        # velocities were re-authored, which discarded the accumulated fall speed but kept the
        # accumulated position error -- that residual was then reported as the run's peak
        # centerline tracking error.  Snap pose and velocity back to the authored initial
        # state so step zero starts on the centerline.
        cart.set_world_poses(positions=[cart_position_start], orientations=[start_orientation])
        rocket.set_world_poses(positions=[rocket_position], orientations=[start_orientation])
        transform_write_counter[0] += 2
        cart.set_velocities(cart_velocity, [0.0, start_omega_y, 0.0])
        rocket.set_velocities(rocket_velocity, [0.0, start_omega_y, 0.0])
        initial_cart_position, initial_cart_orientation = cart.get_world_poses()
        initial_rocket_position, initial_rocket_orientation = rocket.get_world_poses()
        initial_cart_linear, initial_cart_angular = cart.get_velocities()
        initial_rocket_linear, initial_rocket_angular = rocket.get_velocities()
        initial_cart_position = _vector(initial_cart_position)
        initial_rocket_position = _vector(initial_rocket_position)
        initial_cart_orientation = _quaternion(initial_cart_orientation)
        initial_rocket_orientation = _quaternion(initial_rocket_orientation)
        initial_cart_linear = _vector(initial_cart_linear)
        initial_rocket_linear = _vector(initial_rocket_linear)
        initial_cart_angular = _vector(initial_cart_angular)
        initial_rocket_angular = _vector(initial_rocket_angular)

        normal_kp = args.normal_kp_per_s2
        normal_kd = args.normal_kd_per_s
        angle_kp = args.attitude_kp_per_s2
        angle_kd = args.attitude_kd_per_s
        if any(
            not math.isfinite(value) or value < 0.0
            for value in (normal_kp, normal_kd, angle_kp, angle_kd)
        ):
            raise RuntimeError("guide gains must be finite and nonnegative")
        cart_inertia = _flatten_numbers(cart.get_inertias())
        rocket_inertia = _flatten_numbers(rocket.get_inertias())
        if len(cart_inertia) != 9 or len(rocket_inertia) != 9:
            raise RuntimeError("expected one 3x3 inertia tensor for each production body")
        combined_pitch_inertia = (
            float(cart_inertia[4])
            + float(rocket_inertia[4])
            + cart_mass * rocket_mass / assembly_mass * offset_m**2
        )
        # Read gravity back from the scene rather than restating it, so that a scene
        # change cannot silently desynchronise every analytic term below.
        gravity_direction = scene.GetGravityDirectionAttr().Get()
        gravity_magnitude = float(scene.GetGravityMagnitudeAttr().Get())
        gravity = [float(component) * gravity_magnitude for component in gravity_direction]
        maximum_steps = math.ceil((2.0 * layout.length_m / args.target_exit_speed_mps + 5.0) / args.physics_dt_s)
        peak_tracking = peak_normal_tracking = peak_binormal_tracking = 0.0
        peak_attitude = peak_load_g = peak_attachment_g = 0.0
        peak_commanded_load_g = 0.0
        peak_attachment_geometry_error = 0.0
        peak_reaction_command_error = peak_backend_force_error = 0.0
        maximum_attached_contact_force = 0.0
        # Every windowed or masked peak is mirrored by an unmasked counterpart and a
        # sample count.  A gate that never evaluated a sample must not report 0.0 and
        # pass, and a mask must not be able to hide a larger value from the reader.
        peak_reaction_command_error_ungated = 0.0
        peak_reaction_absolute_error_n = 0.0
        peak_attachment_g_unwindowed = 0.0
        peak_backend_force_error_unwindowed = 0.0
        windowed_sample_count = 0
        reaction_sample_count = 0
        reaction_excluded_sample_count = 0
        telemetry: list[dict[str, Any]] = []
        reached_end = False
        completed_steps = 0
        transform_writes_before_run = transform_write_counter[0]
        # Joint/view initialization produces a short solver transient even when the
        # authored rigid state is exact.  Tracking and total load remain measured from
        # step zero; reaction/attachment/backend comparisons begin after the same named
        # 0.1 s settling window for full and sliced runs.
        measurement_start_step = max(5, math.ceil(0.1 / args.physics_dt_s))
        run_started = time.perf_counter()
        last_s = args.start_s_m
        for step_index in range(maximum_steps):
            elapsed_s = step_index * args.physics_dt_s
            reference_s = (
                args.start_s_m + start_speed * elapsed_s + 0.5 * profile_acceleration * elapsed_s**2
            )
            reference_pose = path_pose(layout, reference_s)
            reference_speed = start_speed + profile_acceleration * elapsed_s
            if args.coordinate_frame == "co_moving":
                reference_position = list(reference_pose.position_m)
                reference_velocity = [reference_speed * value for value in reference_pose.tangent]
                reference_acceleration = [
                    profile_acceleration * reference_pose.tangent[i]
                    + reference_speed**2
                    * reference_pose.signed_curvature_per_m
                    * reference_pose.normal[i]
                    for i in range(3)
                ]
            else:
                reference_position = [0.0, 0.0, 0.0]
                reference_velocity = [0.0, 0.0, 0.0]
                reference_acceleration = [0.0, 0.0, 0.0]
            cart_position_values, cart_orientation_values = cart.get_world_poses()
            cart_linear_values, cart_angular_values = cart.get_velocities()
            rocket_linear_values, _ = rocket.get_velocities()
            solver_cart_position = _vector(cart_position_values)
            cart_orientation = _quaternion(cart_orientation_values)
            solver_cart_linear = _vector(cart_linear_values)
            cart_angular = _vector(cart_angular_values)
            solver_rocket_position = _vector(rocket.get_world_poses()[0])
            solver_rocket_linear = _vector(rocket_linear_values)
            cart_position = [solver_cart_position[i] + reference_position[i] for i in range(3)]
            rocket_position_now = [
                solver_rocket_position[i] + reference_position[i] for i in range(3)
            ]
            cart_linear = [solver_cart_linear[i] + reference_velocity[i] for i in range(3)]
            rocket_linear = [solver_rocket_linear[i] + reference_velocity[i] for i in range(3)]
            assembly_com_position = [
                (cart_mass * cart_position[i] + rocket_mass * rocket_position_now[i]) / assembly_mass
                for i in range(3)
            ]
            pre_com_velocity = [
                (cart_mass * cart_linear[i] + rocket_mass * rocket_linear[i]) / assembly_mass
                for i in range(3)
            ]
            s_m = layout.axial_position(tuple(assembly_com_position))
            last_s = s_m
            if s_m >= end_s:
                reached_end = True
                break
            pose = path_pose(layout, s_m)
            displacement = _sub(assembly_com_position, pose.position_m)
            tangential_projection_residual = _dot(displacement, pose.tangent)
            normal_error = _dot(displacement, pose.normal)
            binormal_error = assembly_com_position[1] - pose.position_m[1]
            tangential_speed = _dot(pre_com_velocity, pose.tangent)
            normal_speed = _dot(pre_com_velocity, pose.normal)
            target_angle = math.radians(pose.inclination_deg)
            attitude_error = _wrap_angle(target_angle - _forward_angle(cart_orientation))
            target_omega_y = -tangential_speed * pose.signed_curvature_per_m
            target_alpha_y = -(
                profile_acceleration * pose.signed_curvature_per_m
                + tangential_speed**2 * pose.curvature_rate_per_m2
            )
            gravity_t = _dot(gravity, pose.tangent)
            gravity_n = _dot(gravity, pose.normal)
            ideal_normal_force = assembly_mass * (
                tangential_speed**2 * pose.signed_curvature_per_m - gravity_n
            )
            guide_normal_force = assembly_mass * (
                tangential_speed**2 * pose.signed_curvature_per_m
                - gravity_n
                - normal_kp * normal_error
                - normal_kd * normal_speed
            )
            guide_binormal_force = assembly_mass * (
                -normal_kp * binormal_error - normal_kd * pre_com_velocity[1]
            )
            launch_force = assembly_mass * (profile_acceleration - gravity_t)
            total_force = [
                launch_force * pose.tangent[i]
                + guide_normal_force * pose.normal[i]
                + (guide_binormal_force if i == 1 else 0.0)
                for i in range(3)
            ]
            torque_y = combined_pitch_inertia * (
                target_alpha_y - angle_kp * attitude_error + angle_kd * (target_omega_y - cart_angular[1])
            )
            # Apply the resultant through the attached assembly COM. Applying it at the
            # cart COM would add an unintended r x F pitch moment which a path guide does
            # not own and which the separate attitude command would then have to fight.
            solver_assembly_com_position = [
                assembly_com_position[i] - reference_position[i] for i in range(3)
            ]
            solver_cart_force = [
                total_force[i] - cart_mass * reference_acceleration[i] for i in range(3)
            ]
            solver_rocket_force = [
                -rocket_mass * reference_acceleration[i] for i in range(3)
            ]
            com_arm = _sub(solver_assembly_com_position, solver_cart_position)
            cart_fictitious_moment_y = cart_mass * (
                com_arm[2] * reference_acceleration[0]
                - com_arm[0] * reference_acceleration[2]
            )
            cart.apply_forces_and_torques_at_pos(
                solver_cart_force,
                [0.0, torque_y + cart_fictitious_moment_y, 0.0],
                positions=solver_assembly_com_position,
                local_frame=False,
            )
            rocket.apply_forces_and_torques_at_pos(
                solver_rocket_force,
                [0.0, 0.0, 0.0],
                positions=solver_rocket_position,
                local_frame=False,
            )
            SimulationManager.step()
            next_elapsed_s = (step_index + 1) * args.physics_dt_s
            next_reference_s = (
                args.start_s_m
                + start_speed * next_elapsed_s
                + 0.5 * profile_acceleration * next_elapsed_s**2
            )
            next_reference_pose = path_pose(layout, next_reference_s)
            next_reference_speed = start_speed + profile_acceleration * next_elapsed_s
            if args.coordinate_frame == "co_moving":
                next_reference_velocity = [
                    next_reference_speed * value for value in next_reference_pose.tangent
                ]
            else:
                next_reference_velocity = [0.0, 0.0, 0.0]
            post_cart_linear = [
                value + next_reference_velocity[i]
                for i, value in enumerate(_vector(cart.get_velocities()[0]))
            ]
            post_rocket_linear = [
                value + next_reference_velocity[i]
                for i, value in enumerate(_vector(rocket.get_velocities()[0]))
            ]
            post_com_velocity = [
                (cart_mass * post_cart_linear[i] + rocket_mass * post_rocket_linear[i]) / assembly_mass
                for i in range(3)
            ]
            inferred_force = [
                assembly_mass * ((post_com_velocity[i] - pre_com_velocity[i]) / args.physics_dt_s - gravity[i])
                for i in range(3)
            ]
            backend_force_error = _norm(_sub(inferred_force, total_force)) / max(_norm(total_force), 1.0)
            rocket_proper_acceleration = [
                (post_rocket_linear[i] - rocket_linear[i]) / args.physics_dt_s - gravity[i]
                for i in range(3)
            ]
            tracking = math.hypot(normal_error, binormal_error)
            attachment_geometry_error = abs(
                _norm(_sub(rocket_position_now, cart_position)) - offset_m
            )
            # The assembly load is taken from the finite-differenced backend velocity, not
            # from the force the controller just commanded.  The commanded value is kept
            # alongside it so the two can be compared, but only the measured one is gated.
            measured_load_g = _norm(inferred_force) / assembly_mass / gravity_magnitude
            commanded_load_g = _norm(total_force) / assembly_mass / gravity_magnitude
            attachment_g = _norm(rocket_proper_acceleration) / gravity_magnitude
            reaction_absolute_error = abs(guide_normal_force - ideal_normal_force)
            reaction_error = reaction_absolute_error / max(abs(ideal_normal_force), 1.0)
            peak_tracking = max(peak_tracking, tracking)
            peak_normal_tracking = max(peak_normal_tracking, abs(normal_error))
            peak_binormal_tracking = max(peak_binormal_tracking, abs(binormal_error))
            peak_attitude = max(peak_attitude, abs(math.degrees(attitude_error)))
            peak_attachment_geometry_error = max(
                peak_attachment_geometry_error, attachment_geometry_error
            )
            peak_commanded_load_g = max(peak_commanded_load_g, commanded_load_g)
            peak_attachment_g_unwindowed = max(peak_attachment_g_unwindowed, attachment_g)
            peak_backend_force_error_unwindowed = max(
                peak_backend_force_error_unwindowed, backend_force_error
            )
            peak_reaction_command_error_ungated = max(
                peak_reaction_command_error_ungated, reaction_error
            )
            peak_reaction_absolute_error_n = max(
                peak_reaction_absolute_error_n, reaction_absolute_error
            )
            if step_index >= measurement_start_step:
                windowed_sample_count += 1
                peak_load_g = max(peak_load_g, measured_load_g)
                peak_attachment_g = max(peak_attachment_g, attachment_g)
                peak_backend_force_error = max(peak_backend_force_error, backend_force_error)
                if abs(ideal_normal_force) >= 0.5 * assembly_mass * gravity_magnitude:
                    reaction_sample_count += 1
                    peak_reaction_command_error = max(peak_reaction_command_error, reaction_error)
                else:
                    reaction_excluded_sample_count += 1
            # Contact is a gate, not a log line: sample it every step so a transient
            # shorter than the telemetry stride cannot pass unseen.
            contact_forces, _, _, _, pair_counts, _ = contact_view.get_contact_data(args.physics_dt_s)
            contact_force = max((abs(v) for v in _flatten_numbers(contact_forces)), default=0.0)
            maximum_attached_contact_force = max(maximum_attached_contact_force, contact_force)
            completed_steps = step_index + 1
            if step_index % args.telemetry_stride == 0:
                telemetry.append({
                    "step": step_index,
                    "s_m": s_m,
                    "speed_mps": tangential_speed,
                    "tracking_error_m": tracking,
                    "normal_tracking_error_m": normal_error,
                    "binormal_tracking_error_m": binormal_error,
                    "normal_speed_mps": normal_speed,
                    "assembly_com_velocity_mps": pre_com_velocity,
                    "tangential_projection_residual_m": tangential_projection_residual,
                    "assembly_com_position_m": assembly_com_position,
                    "path_position_m": list(pose.position_m),
                    "path_tangent": list(pose.tangent),
                    "path_normal": list(pose.normal),
                    "attitude_error_deg": math.degrees(attitude_error),
                    "signed_curvature_per_m": pose.signed_curvature_per_m,
                    "guide_normal_force_n": guide_normal_force,
                    "ideal_normal_force_n": ideal_normal_force,
                    "backend_force_relative_error": backend_force_error,
                    "resultant_proper_load_g": measured_load_g,
                    "commanded_resultant_proper_load_g": commanded_load_g,
                    "guide_reaction_absolute_error_n": reaction_absolute_error,
                    "guide_reaction_relative_error": reaction_error,
                    "within_measurement_window": step_index >= measurement_start_step,
                    "within_reaction_gate": abs(ideal_normal_force)
                    >= 0.5 * assembly_mass * gravity_magnitude,
                    "attachment_inferred_proper_load_g": attachment_g,
                    "attachment_geometry_error_m": attachment_geometry_error,
                    "attached_pair_contact_count": int(sum(_flatten_numbers(pair_counts))),
                    "attached_pair_maximum_force_n": contact_force,
                })
        wall_time_s = time.perf_counter() - run_started
        transform_writes_during_run = transform_write_counter[0] - transform_writes_before_run
        final_cart_velocity = [
            value + reference_velocity[i]
            for i, value in enumerate(_vector(cart.get_velocities()[0]))
        ]
        final_pose = path_pose(layout, min(max(last_s, 0.0), layout.length_m))
        final_speed = _dot(final_cart_velocity, final_pose.tangent)
        guide_probe = {
            "candidate": "force_resolved_path_controller_v1",
            "coordinate_frame": args.coordinate_frame,
            "coordinate_mapping": (
                "x_global=x_solver+x_reference(t); v_global=v_solver+v_reference(t); "
                "uniform fictitious force=-mass*a_reference(t)"
                if args.coordinate_frame == "co_moving"
                else "x_global=x_solver; v_global=v_solver"
            ),
            "guide_reference_point": "attached assembly center of mass",
            "cart_to_rocket_offset_m": offset_m,
            "transform_writes_during_run": transform_writes_during_run,
            "initialization_transform_writes": transform_writes_before_run,
            "gravity_mps2": gravity,
            "start_s_m": args.start_s_m,
            "end_s_m": end_s,
            "reached_end": reached_end,
            "final_s_m": last_s,
            "final_speed_mps": final_speed,
            "expected_profile_speed_mps": math.sqrt(2.0 * profile_acceleration * end_s),
            "profile_tangential_acceleration_mps2": profile_acceleration,
            "peak_centerline_tracking_error_m": peak_tracking,
            "peak_normal_tracking_error_m": peak_normal_tracking,
            "peak_binormal_tracking_error_m": peak_binormal_tracking,
            "peak_attitude_error_deg": peak_attitude,
            # Measured from finite-differenced backend velocity.
            "peak_resultant_proper_load_g": peak_load_g,
            # The value the controller commanded, for comparison only. It is not gated,
            # because comparing a command against a ceiling measures nothing.
            "peak_commanded_resultant_proper_load_g": peak_commanded_load_g,
            "peak_inferred_attachment_proper_load_g": peak_attachment_g,
            "peak_attachment_geometry_error_m": peak_attachment_geometry_error,
            "peak_guide_reaction_command_relative_error": peak_reaction_command_error,
            "peak_backend_applied_force_relative_error": peak_backend_force_error,
            "maximum_attached_pair_force_n": maximum_attached_contact_force,
            # Unmasked counterparts of every windowed or gated peak above, so no mask can
            # hide a larger value from a reader of this artifact.
            "unmasked": {
                "peak_guide_reaction_command_relative_error": peak_reaction_command_error_ungated,
                "peak_guide_reaction_absolute_error_n": peak_reaction_absolute_error_n,
                "peak_inferred_attachment_proper_load_g": peak_attachment_g_unwindowed,
                "peak_backend_applied_force_relative_error": peak_backend_force_error_unwindowed,
            },
            "sample_counts": {
                "windowed": windowed_sample_count,
                "reaction_gate_included": reaction_sample_count,
                "reaction_gate_excluded": reaction_excluded_sample_count,
                "reaction_gate_threshold_n": 0.5 * assembly_mass * gravity_magnitude,
            },
            "physics_steps": completed_steps,
            "measurement_start_step": measurement_start_step,
            "wall_time_s": wall_time_s,
            "physics_steps_per_wall_second": completed_steps / max(wall_time_s, 1e-9),
            "controller_gains": {
                "normal_kp_per_s2": normal_kp,
                "normal_kd_per_s": normal_kd,
                "attitude_kp_per_s2": angle_kp,
                "attitude_kd_per_s": angle_kd,
            },
            "combined_pitch_inertia_kg_m2": combined_pitch_inertia,
            "telemetry": telemetry,
            "acceptance": {
                "maximum_tracking_error_m": config.tube.guide_clearance_m,
                "maximum_attitude_error_deg": 1.0,
                "maximum_resultant_load_g": config.launch_control.maximum_resultant_load_g,
                "maximum_attachment_load_g": config.launch_control.maximum_resultant_load_g,
                "maximum_attachment_geometry_error_m": 0.001,
                "maximum_backend_force_relative_error": 0.05,
                "maximum_reaction_correction_relative_error": 0.05,
                "maximum_attached_pair_force_n": 1e-6,
            },
        }
        guide_probe["passed"] = _curved_guide_passed(
            reached_end=reached_end,
            windowed_sample_count=windowed_sample_count,
            reaction_sample_count=reaction_sample_count,
            transform_writes_during_run=transform_writes_during_run,
            peak_tracking_error_m=peak_tracking,
            peak_attitude_error_deg=peak_attitude,
            peak_resultant_load_g=peak_load_g,
            peak_attachment_load_g=peak_attachment_g,
            peak_attachment_geometry_error_m=peak_attachment_geometry_error,
            peak_backend_force_relative_error=peak_backend_force_error,
            peak_reaction_relative_error=peak_reaction_command_error,
            maximum_attached_pair_force_n=maximum_attached_contact_force,
            acceptance=guide_probe["acceptance"],
        )
        result["probes"]["curved_force_guide"] = guide_probe
        _progress("curved_force_guide_complete")

        # Release is meaningful only at the authored zero-curvature exit. A diagnostic
        # sub-interval must not turn a mid-bend joint mutation into release evidence.
        if math.isclose(end_s, layout.length_m, rel_tol=0.0, abs_tol=1e-9):
            app_utils.pause()
            before_cart_position = _vector(cart.get_world_poses()[0])
            before_rocket_position = _vector(rocket.get_world_poses()[0])
            before_cart_velocity = _vector(cart.get_velocities()[0])
            before_rocket_velocity = _vector(rocket.get_velocities()[0])
            fixed_joint.GetJointEnabledAttr().Set(False)
            app.update()
            after_cart_position = _vector(cart.get_world_poses()[0])
            after_rocket_position = _vector(rocket.get_world_poses()[0])
            after_cart_velocity = _vector(cart.get_velocities()[0])
            after_rocket_velocity = _vector(rocket.get_velocities()[0])
            mutation_error = max(
                _maximum_error(before_cart_position, after_cart_position),
                _maximum_error(before_rocket_position, after_rocket_position),
                _maximum_error(before_cart_velocity, after_cart_velocity),
                _maximum_error(before_rocket_velocity, after_rocket_velocity),
            )
            app_utils.play()
            release_effort_n = -1000.0
            pre_relative_speed = _dot(
                _sub(before_rocket_velocity, before_cart_velocity), final_pose.tangent
            )
            cart.apply_forces(
                [release_effort_n * value for value in final_pose.tangent], local_frame=False
            )
            SimulationManager.step()
            released_cart_velocity = _vector(cart.get_velocities()[0])
            released_rocket_velocity = _vector(rocket.get_velocities()[0])
            separating_speed_increase = _dot(
                _sub(released_rocket_velocity, released_cart_velocity), final_pose.tangent
            ) - pre_relative_speed
            release_probe = {
                "applicable": True,
                "joint_enabled_after_resync": bool(fixed_joint.GetJointEnabledAttr().Get()),
                "collision_enabled_after_resync": bool(fixed_joint.GetCollisionEnabledAttr().Get()),
                "maximum_mutation_discontinuity": mutation_error,
                "separating_effort_n": release_effort_n,
                "post_step_separating_speed_increase_mps": separating_speed_increase,
                "passed": bool(
                    not fixed_joint.GetJointEnabledAttr().Get()
                    and fixed_joint.GetCollisionEnabledAttr().Get()
                    and mutation_error <= 1e-8
                    and separating_speed_increase
                    >= 0.25 * abs(release_effort_n) / cart_mass * args.physics_dt_s
                ),
            }
        else:
            app_utils.pause()
            # A probe that did not run is not a probe that passed.  Reporting no verdict
            # keeps it out of the aggregate instead of contributing a free success.
            release_probe = {
                "applicable": False,
                "reason": "diagnostic interval ends before the zero-curvature tube exit",
            }
        result["probes"]["full_speed_release"] = release_probe

        # Stop with the joint disabled, restore the authored initial state while physics is
        # absent, then enable the joint before rebuilding physics.  A paused tensor write is
        # not sufficient here: stopping can sync the last simulated transforms back over it.
        app_utils.pause()
        fixed_joint.GetJointEnabledAttr().Set(False)
        app.update()
        app_utils.stop()
        app.update()
        cart_authored_types = author_stopped_rigid_state(
            cart_path,
            cart_position_start,
            start_orientation,
            cart_velocity,
            [0.0, start_omega_y, 0.0],
        )
        rocket_authored_types = author_stopped_rigid_state(
            rocket_path,
            rocket_position,
            start_orientation,
            rocket_velocity,
            [0.0, start_omega_y, 0.0],
        )
        fixed_joint.GetJointEnabledAttr().Set(True)
        app.update()
        app_utils.play()
        app.update()
        reset_cart = RigidPrim(cart_path)
        reset_rocket = RigidPrim(rocket_path)
        reset_contact = SimulationManager.get_physics_simulation_view().create_rigid_contact_view(
            [rocket_path], filter_patterns=[[cart_path]], max_contact_data_count=16
        )
        # Reproduce the original startup sequence exactly: after the solver instantiates
        # the fixed joint, snap pose and velocity back before the first integration step.
        # The pose half matters -- play() plus update() free-falls the assembly here just
        # as it does at startup, so omitting it would leave reset reproducing the fall
        # while initialization reproduced the authored state, and the two would disagree
        # by that fall distance.
        reset_cart.set_world_poses(
            positions=[cart_position_start], orientations=[start_orientation]
        )
        reset_rocket.set_world_poses(
            positions=[rocket_position], orientations=[start_orientation]
        )
        reset_cart.set_velocities(cart_velocity, [0.0, start_omega_y, 0.0])
        reset_rocket.set_velocities(rocket_velocity, [0.0, start_omega_y, 0.0])
        reset_cart_position_values, reset_cart_orientation_values = reset_cart.get_world_poses()
        reset_rocket_position_values, reset_rocket_orientation_values = reset_rocket.get_world_poses()
        reset_cart_velocity_values, reset_cart_angular_values = reset_cart.get_velocities()
        reset_rocket_velocity_values, reset_rocket_angular_values = reset_rocket.get_velocities()
        reset_cart_position = _vector(reset_cart_position_values)
        reset_rocket_position = _vector(reset_rocket_position_values)
        reset_cart_orientation = _quaternion(reset_cart_orientation_values)
        reset_rocket_orientation = _quaternion(reset_rocket_orientation_values)
        reset_cart_velocity = _vector(reset_cart_velocity_values)
        reset_rocket_velocity = _vector(reset_rocket_velocity_values)
        reset_cart_angular = _vector(reset_cart_angular_values)
        reset_rocket_angular = _vector(reset_rocket_angular_values)
        reset_forces, _, _, _, reset_counts, _ = reset_contact.get_contact_data(args.physics_dt_s)
        reset_force = max((abs(v) for v in _flatten_numbers(reset_forces)), default=0.0)
        reset_probe = {
            # Derived from the reads that actually happened rather than asserted.
            "views_recreated_and_read": bool(
                len(reset_cart_position) == 3
                and len(reset_rocket_position) == 3
                and reset_counts is not None
            ),
            "stopped_usd_state_authored": bool(cart_authored_types and rocket_authored_types),
            "cart_xform_attribute_types": cart_authored_types,
            "rocket_xform_attribute_types": rocket_authored_types,
            "joint_enabled": bool(fixed_joint.GetJointEnabledAttr().Get()),
            "collision_enabled": bool(fixed_joint.GetCollisionEnabledAttr().Get()),
            "cart_position_error_m": _maximum_error(
                reset_cart_position, initial_cart_position
            ),
            "rocket_position_error_m": _maximum_error(
                reset_rocket_position, initial_rocket_position
            ),
            "cart_orientation_error": _maximum_error(
                reset_cart_orientation, initial_cart_orientation
            ),
            "rocket_orientation_error": _maximum_error(
                reset_rocket_orientation, initial_rocket_orientation
            ),
            "cart_velocity_error_mps": _maximum_error(
                reset_cart_velocity, initial_cart_linear
            ),
            "rocket_velocity_error_mps": _maximum_error(
                reset_rocket_velocity, initial_rocket_linear
            ),
            "cart_angular_velocity_error_rad_s": _maximum_error(
                reset_cart_angular, initial_cart_angular
            ),
            "rocket_angular_velocity_error_rad_s": _maximum_error(
                reset_rocket_angular, initial_rocket_angular
            ),
            "pair_contact_count": int(sum(_flatten_numbers(reset_counts))),
            "pair_maximum_force_n": reset_force,
            # Reset writes and reads the same authored pose, so the residual is a
            # float32 round-trip, not a physical drift.  The former 0.01 m allowance was
            # ten times looser than the 1 mm attachment gate this same run certifies and
            # would have admitted a centimetre of reset misplacement.
            "maximum_position_error_m": 1e-5,
            "maximum_orientation_error": 1e-5,
            "maximum_velocity_error_mps": 1e-5,
            "maximum_angular_velocity_error_rad_s": 1e-5,
        }
        reset_probe["passed"] = bool(
            reset_probe["views_recreated_and_read"]
            and reset_probe["stopped_usd_state_authored"]
            and reset_probe["joint_enabled"]
            and reset_probe["collision_enabled"]
            and reset_probe["cart_position_error_m"] <= reset_probe["maximum_position_error_m"]
            and reset_probe["rocket_position_error_m"] <= reset_probe["maximum_position_error_m"]
            and reset_probe["cart_orientation_error"]
            <= reset_probe["maximum_orientation_error"]
            and reset_probe["rocket_orientation_error"]
            <= reset_probe["maximum_orientation_error"]
            and reset_probe["cart_velocity_error_mps"]
            <= reset_probe["maximum_velocity_error_mps"]
            and reset_probe["rocket_velocity_error_mps"]
            <= reset_probe["maximum_velocity_error_mps"]
            and reset_probe["cart_angular_velocity_error_rad_s"]
            <= reset_probe["maximum_angular_velocity_error_rad_s"]
            and reset_probe["rocket_angular_velocity_error_rad_s"]
            <= reset_probe["maximum_angular_velocity_error_rad_s"]
            and reset_force <= 1e-6
        )
        result["probes"]["stop_rebuild_reset"] = reset_probe
        _progress("release_reset_complete")
        # Aggregate only over probes that carry a verdict and actually ran.  Probes that
        # are informational or inapplicable are named explicitly so a run that evidenced
        # less than a full profile cannot present itself as one that evidenced more.
        evaluated = {
            name: probe
            for name, probe in result["probes"].items()
            if "passed" in probe and not probe.get("informational") and probe.get("applicable", True)
        }
        result["gates_evaluated"] = sorted(evaluated)
        result["gates_not_evaluated"] = sorted(set(result["probes"]) - set(evaluated))
        result["passed"] = bool(evaluated) and all(
            probe["passed"] for probe in evaluated.values()
        )
        exit_code = 0 if result["passed"] else 2
    except BaseException as exc:
        result["error"] = {"type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc()}
    finally:
        result["finished_utc"] = datetime.now(timezone.utc).isoformat()
        output.parent.mkdir(parents=True, exist_ok=True)
        rendered = json.dumps(_json_value(result), indent=2, sort_keys=True, allow_nan=False)
        output.write_text(rendered + "\n", encoding="utf-8")
        print(f"CURVED_GUIDE_RESULT={output}")
        print(f"CURVED_GUIDE_PASSED={result.get('passed', False)}")
        if app is not None:
            app.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
