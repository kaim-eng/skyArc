# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Run backend/mechanism qualification probes on the exact Isaac Sim build.

This is deliberately separate from the production backend adapter.  Phase 0 exists to
measure which runtime assumptions are true before that adapter and the launcher scene are
implemented.  The script always writes a JSON result, including when startup or a probe
fails, and imports Kit-provided modules only after constructing ``SimulationApp``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import sys
import traceback
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("physx", "newton"), required=True)
    parser.add_argument(
        "--collision-treatment",
        choices=("live_activation", "always_present"),
        default="live_activation",
        help="release-pair treatment to qualify",
    )
    parser.add_argument("--physics-dt-s", type=float, default=0.001)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="allow an explicitly selected --output path to replace an existing artifact",
    )
    parser.add_argument("--force-steps", type=int, default=100)
    return parser.parse_args()


def _json_value(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        if math.isfinite(value):
            return value
        return {
            "valid": False,
            "value": None,
            "reason": "nonfinite measurement",
            "nonfinite_kind": repr(value),
        }
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if hasattr(value, "numpy"):
        return _json_value(value.numpy().tolist())
    if hasattr(value, "tolist"):
        return _json_value(value.tolist())
    return str(value)


def _vector(value: Any) -> list[float]:
    converted = _json_value(value)
    while isinstance(converted, list) and len(converted) == 1:
        converted = converted[0]
    if not isinstance(converted, list) or len(converted) != 3:
        raise RuntimeError(f"expected a three-vector, got {converted!r}")
    return [float(component) for component in converted]


def _quaternion(value: Any) -> list[float]:
    converted = _json_value(value)
    while isinstance(converted, list) and len(converted) == 1:
        converted = converted[0]
    if not isinstance(converted, list) or len(converted) != 4:
        raise RuntimeError(f"expected a four-component quaternion, got {converted!r}")
    return [float(component) for component in converted]


def _maximum_absolute_error(actual: list[float], expected: list[float]) -> float:
    return max(abs(left - right) for left, right in zip(actual, expected, strict=True))


def _quaternion_error(actual: list[float], expected: list[float]) -> float:
    """Return component error while treating q and -q as the same orientation."""
    direct = _maximum_absolute_error(actual, expected)
    negated = _maximum_absolute_error(actual, [-value for value in expected])
    return min(direct, negated)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _git_head(repository: Path) -> str | None:
    """Resolve a checkout HEAD without depending on Git being installed on Kit's PATH."""
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


def _default_output_path(script_path: Path, backend: str, started_at: datetime, run_id: str) -> Path:
    timestamp = started_at.astimezone(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    return (
        script_path.resolve().parents[1]
        / "artifacts"
        / "phase0"
        / backend
        / f"{timestamp}_{run_id}.json"
    )


def _runtime_selection_passed(
    probe: dict[str, Any], *, requested_backend: str, requested_dt_s: float
) -> bool:
    settings = probe["settings"]
    return bool(
        probe["switch_returned"]
        and probe["active_engine"] == requested_backend
        and math.isclose(
            float(probe["physics_dt_s"]), requested_dt_s, rel_tol=0.0, abs_tol=1e-12
        )
        and "cpu" in str(probe["device"]).lower()
        and settings["/app/player/useFixedTimeStepping"] is True
        and settings["/app/runLoops/main/rateLimitEnabled"] is False
    )


def _flatten_numbers(value: Any) -> list[float]:
    converted = _json_value(value)
    if isinstance(converted, list):
        flattened: list[float] = []
        for item in converted:
            flattened.extend(_flatten_numbers(item))
        return flattened
    return [float(converted)]


def _progress(label: str) -> None:
    print(f"PHASE0_PROGRESS={label}", flush=True)


def main() -> int:
    args = _arguments()
    if not math.isfinite(args.physics_dt_s) or args.physics_dt_s <= 0.0:
        raise SystemExit("--physics-dt-s must be finite and positive")
    if args.force_steps <= 0:
        raise SystemExit("--force-steps must be positive")

    # ``python.bat`` deliberately names a Kit executable whose resolved path may point
    # into Packman storage.  ISAAC_PATH is the authoritative generated-launcher value.
    isaac_path = os.environ.get("ISAAC_PATH")
    if not isaac_path:
        raise SystemExit("ISAAC_PATH is not set; launch this script through the build's python.bat")
    release_root = Path(isaac_path).resolve()
    isaac_root = release_root.parents[2]
    experience_name = (
        "isaacsim.exp.full.newton.kit" if args.backend == "newton" else "isaacsim.exp.full.kit"
    )
    experience = release_root / "apps" / experience_name
    started_at = datetime.now(timezone.utc)
    run_id = uuid.uuid4().hex
    output = args.output or _default_output_path(Path(__file__), args.backend, started_at, run_id)
    if output.exists() and not args.overwrite:
        raise SystemExit(
            f"qualification artifact already exists: {output}; choose another --output or pass --overwrite"
        )
    version_file = isaac_root / "VERSION"
    executable = Path(sys.executable).resolve()
    script_path = Path(__file__).resolve()
    result: dict[str, Any] = {
        "schema": "vacuum_tube_phase0_qualification_v2",
        "run_id": run_id,
        "started_utc": started_at.isoformat(),
        "requested": {
            "backend": args.backend,
            "device": "cpu",
            "physics_dt_s": args.physics_dt_s,
            "force_steps": args.force_steps,
            "collision_treatment": args.collision_treatment,
            "experience": str(experience),
            "stepping_mode": "explicit SimulationManager.step with fixed timestep",
        },
        "host": {
            "python": platform.python_version(),
            "python_executable": sys.executable,
            "platform": platform.platform(),
            "isaac_sim_version": (
                version_file.read_text(encoding="utf-8").strip()
                if version_file.is_file()
                else None
            ),
        },
        "provenance": {
            "runner_path": str(script_path),
            "runner_sha256": _sha256_file(script_path),
            "experience_path": str(experience),
            "experience_sha256": _sha256_file(experience) if experience.is_file() else None,
            "version_file_path": str(version_file),
            "version_file_sha256": _sha256_file(version_file) if version_file.is_file() else None,
            "python_executable_path": str(executable),
            "python_executable_sha256": _sha256_file(executable) if executable.is_file() else None,
            "isaac_sim_source_git_revision": _git_head(isaac_root),
            "build_revision_environment": {
                name: os.environ.get(name)
                for name in ("BUILD_ID", "BUILD_NUMBER", "GIT_COMMIT", "CI_COMMIT_SHA")
                if os.environ.get(name)
            },
        },
        "probes": {},
        "passed": False,
    }
    simulation_app = None
    exit_code = 2
    try:
        _progress("constructing_simulation_app")
        if not experience.is_file():
            raise RuntimeError(f"Isaac Sim experience does not exist: {experience}")

        # Kit modules are intentionally unavailable until the application is constructed.
        from isaacsim import SimulationApp

        simulation_app = SimulationApp(
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

        import carb.settings
        import isaacsim.core.experimental.utils.app as app_utils
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.objects import Cube
        from isaacsim.core.experimental.prims import Articulation, GeomPrim, RigidPrim
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

        settings = carb.settings.get_settings()
        stage_utils.create_new_stage()
        stage = stage_utils.get_current_stage()
        physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        physics_scene.CreateGravityDirectionAttr().Set((0.0, 0.0, -1.0))
        physics_scene.CreateGravityMagnitudeAttr().Set(0.0)
        SimulationManager.set_default_physics_scene("/World/PhysicsScene")

        guide_assembly_path = "/World/GuideAssembly"
        guide_base_path = f"{guide_assembly_path}/GuideBase"
        guide_cart_path = f"{guide_assembly_path}/GuideCart"
        guide_rocket_path = f"{guide_assembly_path}/GuideRocket"
        root_joint_path = f"{guide_assembly_path}/RootFixedJoint"
        prismatic_joint_path = f"{guide_assembly_path}/GuidePrismaticJoint"
        fixed_joint_path = f"{guide_assembly_path}/CartRocketFixedJoint"
        UsdGeom.Xform.Define(stage, guide_assembly_path)
        guide_base = UsdGeom.Xform.Define(stage, guide_base_path)
        UsdPhysics.RigidBodyAPI.Apply(guide_base.GetPrim())
        UsdPhysics.MassAPI.Apply(guide_base.GetPrim()).CreateMassAttr(1.0)

        available = SimulationManager.get_available_physics_engines(verbose=False)
        switched = SimulationManager.switch_physics_engine(args.backend, verbose=False)
        SimulationManager.setup_simulation(dt=args.physics_dt_s, device="cpu")
        active = SimulationManager.get_active_physics_engine()
        runtime_probe = {
            "available_engines": available,
            "switch_returned": switched,
            "active_engine": active,
            "default_engine": SimulationManager.get_default_engine(),
            "device": str(SimulationManager.get_device()),
            "tensor_backend": SimulationManager.get_backend(),
            "physics_dt_s": SimulationManager.get_physics_dt(),
            "solver_type": SimulationManager.get_solver_type(),
            "stepping_mode": "explicit SimulationManager.step with fixed timestep",
            "settings": {
                path: settings.get(path)
                for path in (
                    "/app/player/useFixedTimeStepping",
                    "/app/runLoops/main/manualModeEnabled",
                    "/app/runLoops/main/rateLimitEnabled",
                    "/exts/isaacsim.core.simulation_manager/default_engine",
                    "/exts/isaacsim.physics.newton/auto_switch_on_startup",
                )
            },
        }
        runtime_probe["acceptance"] = {
            "active_engine_equals_requested": args.backend,
            "device_contains": "cpu",
            "physics_dt_absolute_tolerance_s": 1e-12,
            "fixed_time_stepping": True,
            "rate_limit_enabled": False,
        }
        runtime_probe["passed"] = _runtime_selection_passed(
            runtime_probe,
            requested_backend=args.backend,
            requested_dt_s=args.physics_dt_s,
        )
        result["probes"]["runtime_selection"] = runtime_probe
        _progress("runtime_selection_complete")

        Cube(
            paths="/World/ForceProbe",
            positions=[0.0, 0.0, 0.0],
            sizes=1.0,
            scales=[0.2, 0.2, 0.2],
        )
        force_body = RigidPrim(paths="/World/ForceProbe", masses=[2.0])
        GeomPrim(paths="/World/ForceProbe", apply_collision_apis=True)

        guide_angle_rad = math.radians(45.0)
        guide_tangent = [math.cos(guide_angle_rad), 0.0, math.sin(guide_angle_rad)]
        guide_orientation = [
            math.cos(0.5 * guide_angle_rad),
            0.0,
            -math.sin(0.5 * guide_angle_rad),
            0.0,
        ]
        guide_quaternion = Gf.Quatf(
            guide_orientation[0],
            Gf.Vec3f(*guide_orientation[1:]),
        )
        identity_quaternion = Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))
        cart_initial = [0.0, 10.0, 0.0]
        # Start with a small physical gap.  An overlapping fixture is valid only when
        # collision is suppressed and makes the always-present treatment fight the
        # fixed joint with enormous internal forces.  After release, a bounded
        # compressive-approach probe below deliberately closes this gap, so contact
        # activation/reporting remains discriminating without corrupting attached
        # guide dynamics.
        rocket_offset_m = 1.01
        rocket_initial = [
            cart_initial[index] + rocket_offset_m * guide_tangent[index] for index in range(3)
        ]
        Cube(
            paths=guide_cart_path,
            positions=cart_initial,
            orientations=guide_orientation,
            sizes=1.0,
            scales=[1.0, 0.5, 0.5],
        )
        guide_cart = RigidPrim(paths=guide_cart_path, masses=[2.0])
        GeomPrim(paths=guide_cart_path, apply_collision_apis=True)
        Cube(
            paths=guide_rocket_path,
            positions=rocket_initial,
            orientations=guide_orientation,
            sizes=1.0,
            scales=[1.0, 0.3, 0.3],
        )
        guide_rocket = RigidPrim(
            paths=guide_rocket_path,
            masses=[1.0],
            contact_filter_paths=[guide_cart_path],
            max_contact_count=16,
        )
        GeomPrim(paths=guide_rocket_path, apply_collision_apis=True)
        guide_rocket.set_enabled_contact_tracking([True])

        root_joint = UsdPhysics.FixedJoint.Define(stage, root_joint_path)
        root_joint.CreateBody1Rel().SetTargets([guide_base_path])
        UsdPhysics.ArticulationRootAPI.Apply(root_joint.GetPrim())
        PhysxSchema.PhysxArticulationAPI.Apply(
            root_joint.GetPrim()
        ).CreateEnabledSelfCollisionsAttr(True)
        root_joint.GetPrim().ApplyAPI("NewtonArticulationRootAPI")
        root_joint.GetPrim().GetAttribute("newton:selfCollisionEnabled").Set(True)

        prismatic_joint = UsdPhysics.PrismaticJoint.Define(stage, prismatic_joint_path)
        prismatic_joint.CreateBody0Rel().SetTargets([guide_base_path])
        prismatic_joint.CreateBody1Rel().SetTargets([guide_cart_path])
        prismatic_joint.CreateAxisAttr("X")
        prismatic_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*cart_initial))
        prismatic_joint.CreateLocalRot0Attr().Set(guide_quaternion)
        prismatic_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        prismatic_joint.CreateLocalRot1Attr().Set(identity_quaternion)
        prismatic_joint.CreateLowerLimitAttr(-1000.0)
        prismatic_joint.CreateUpperLimitAttr(1000.0)
        fixed_joint = UsdPhysics.FixedJoint.Define(stage, fixed_joint_path)
        fixed_joint.CreateBody0Rel().SetTargets([guide_cart_path])
        fixed_joint.CreateBody1Rel().SetTargets([guide_rocket_path])
        fixed_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(rocket_offset_m, 0.0, 0.0))
        fixed_joint.CreateLocalRot0Attr().Set(identity_quaternion)
        fixed_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        fixed_joint.CreateLocalRot1Attr().Set(identity_quaternion)
        # The coupling must not be an articulation joint.  PhysX explicitly ignores
        # per-joint collisionEnabled for articulation links, and both candidate
        # solvers may cache an articulation's topology across a live jointEnabled
        # write.  Excluding only this fixed joint leaves the cart's prismatic guide
        # as the articulation DOF while making release and pair treatment ordinary
        # solver-constraint properties that this qualification can discriminate.
        fixed_joint.CreateExcludeFromArticulationAttr().Set(True)
        collision_enabled_initial = args.collision_treatment == "always_present"
        fixed_joint.CreateCollisionEnabledAttr().Set(collision_enabled_initial)
        fixed_joint.CreateJointEnabledAttr().Set(True)
        guide_articulation = Articulation(guide_assembly_path)

        app_utils.play()
        simulation_app.update()
        guide_articulation.set_dof_gains(stiffnesses=[0.0], dampings=[0.0])
        guide_articulation.set_dof_max_efforts([1.0e6])
        guide_articulation.set_dof_velocities([0.0])
        authored_dof_armatures = _json_value(guide_articulation.get_dof_armatures())
        release_contact_view = SimulationManager.get_physics_simulation_view().create_rigid_contact_view(
            [guide_rocket_path],
            filter_patterns=[[guide_cart_path]],
            max_contact_data_count=16,
        )
        force_body.set_world_poses(positions=[0.0, 0.0, 0.0], orientations=[1.0, 0.0, 0.0, 0.0])
        force_body.set_velocities(linear_velocities=[0.0, 0.0, 0.0], angular_velocities=[0.0, 0.0, 0.0])

        force_n = 20.0
        mass_kg = 2.0
        for _ in range(args.force_steps):
            force_body.apply_forces([force_n, 0.0, 0.0], local_frame=False)
            SimulationManager.step()

        positions, _ = force_body.get_world_poses()
        linear_velocities, angular_velocities = force_body.get_velocities()
        actual_position = _vector(positions)
        actual_velocity = _vector(linear_velocities)
        actual_angular_velocity = _vector(angular_velocities)
        acceleration = force_n / mass_kg
        expected_velocity = [acceleration * args.force_steps * args.physics_dt_s, 0.0, 0.0]
        expected_position = [
            acceleration
            * args.physics_dt_s**2
            * args.force_steps
            * (args.force_steps + 1)
            / 2.0,
            0.0,
            0.0,
        ]
        velocity_error = _maximum_absolute_error(actual_velocity, expected_velocity)
        position_error = _maximum_absolute_error(actual_position, expected_position)
        force_probe = {
            "mass_kg": mass_kg,
            "world_force_n": [force_n, 0.0, 0.0],
            "steps": args.force_steps,
            "actual_position_m": actual_position,
            "expected_semi_implicit_position_m": expected_position,
            "maximum_position_error_m": position_error,
            "actual_velocity_mps": actual_velocity,
            "expected_velocity_mps": expected_velocity,
            "maximum_velocity_error_mps": velocity_error,
            "actual_angular_velocity_radps": actual_angular_velocity,
        }
        force_probe["passed"] = bool(
            velocity_error <= 1e-4
            and position_error <= 1e-4
            and max(abs(value) for value in actual_angular_velocity) <= 1e-6
        )
        result["probes"]["tensor_world_force"] = force_probe
        _progress("tensor_world_force_complete")

        assembly_force_n = 30.0
        assembly_mass_kg = 3.0
        for _ in range(args.force_steps):
            # The launcher force is the generalized force of the prismatic DOF.
            # Applying it through the articulation API is supported by both engines;
            # a rigid-body force view cannot address a non-root articulation link on
            # PhysX and would make the nominally shared test backend-specific.
            guide_articulation.set_dof_efforts([assembly_force_n])
            SimulationManager.step()

        cart_positions, cart_orientations = guide_cart.get_world_poses()
        rocket_positions, rocket_orientations = guide_rocket.get_world_poses()
        cart_linear, cart_angular = guide_cart.get_velocities()
        rocket_linear, rocket_angular = guide_rocket.get_velocities()
        cart_position = _vector(cart_positions)
        rocket_position = _vector(rocket_positions)
        cart_velocity = _vector(cart_linear)
        rocket_velocity = _vector(rocket_linear)
        cart_delta = [cart_position[index] - cart_initial[index] for index in range(3)]
        axial_displacement = sum(
            cart_delta[index] * guide_tangent[index] for index in range(3)
        )
        perpendicular = [
            cart_delta[index] - axial_displacement * guide_tangent[index] for index in range(3)
        ]
        tracking_error = math.sqrt(sum(component * component for component in perpendicular))
        expected_assembly_velocity = [
            assembly_force_n
            / assembly_mass_kg
            * args.force_steps
            * args.physics_dt_s
            * component
            for component in guide_tangent
        ]
        expected_axial_displacement = (
            assembly_force_n
            / assembly_mass_kg
            * args.physics_dt_s**2
            * args.force_steps
            * (args.force_steps + 1)
            / 2.0
        )
        axial_velocity = sum(
            cart_velocity[index] * guide_tangent[index] for index in range(3)
        )
        expected_axial_velocity = (
            assembly_force_n / assembly_mass_kg * args.force_steps * args.physics_dt_s
        )
        relative_axial_velocity_error = abs(
            axial_velocity - expected_axial_velocity
        ) / expected_axial_velocity
        inferred_effective_mass_kg = None
        inferred_joint_armature_kg = None
        if axial_velocity != 0.0 and math.isfinite(axial_velocity):
            inferred_effective_mass_kg = (
                assembly_force_n * args.force_steps * args.physics_dt_s / axial_velocity
            )
            inferred_joint_armature_kg = inferred_effective_mass_kg - assembly_mass_kg
        actual_offset = [rocket_position[index] - cart_position[index] for index in range(3)]
        expected_offset = [rocket_offset_m * component for component in guide_tangent]
        guide_probe = {
            "articulation_root_prim": root_joint_path,
            "dof_names": list(guide_articulation.dof_names),
            "dof_gains": {
                "stiffness": _json_value(guide_articulation.get_dof_gains()[0]),
                "damping": _json_value(guide_articulation.get_dof_gains()[1]),
            },
            "dof_max_effort_n": _json_value(guide_articulation.get_dof_max_efforts()),
            "dof_armature_kg": authored_dof_armatures,
            "inclination_deg": 45.0,
            "assembly_mass_kg": assembly_mass_kg,
            "assembly_world_force_n": [assembly_force_n * component for component in guide_tangent],
            "cart_position_m": cart_position,
            "rocket_position_m": rocket_position,
            "cart_velocity_mps": cart_velocity,
            "rocket_velocity_mps": rocket_velocity,
            "cart_angular_velocity_radps": _vector(cart_angular),
            "rocket_angular_velocity_radps": _vector(rocket_angular),
            "cart_orientation_wxyz": _quaternion(cart_orientations),
            "rocket_orientation_wxyz": _quaternion(rocket_orientations),
            "axial_displacement_m": axial_displacement,
            "expected_semi_implicit_axial_displacement_m": expected_axial_displacement,
            "axial_velocity_mps": axial_velocity,
            "expected_rigid_body_only_axial_velocity_mps": expected_axial_velocity,
            "relative_axial_velocity_error": relative_axial_velocity_error,
            "inferred_effective_mass_kg": inferred_effective_mass_kg,
            "inferred_effective_mass_valid": inferred_effective_mass_kg is not None,
            "inferred_joint_armature_kg": inferred_joint_armature_kg,
            "centerline_tracking_error_m": tracking_error,
            "fixed_offset_error_m": _maximum_absolute_error(actual_offset, expected_offset),
            "cart_velocity_error_mps": _maximum_absolute_error(
                cart_velocity, expected_assembly_velocity
            ),
            "rocket_velocity_error_mps": _maximum_absolute_error(
                rocket_velocity, expected_assembly_velocity
            ),
            "acceptance": {
                "maximum_tracking_error_m": 1e-4,
                "maximum_relative_displacement_error": 0.05,
                "maximum_fixed_offset_error_m": 1e-4,
                "maximum_relative_axial_velocity_error": 0.05,
                "maximum_cart_rocket_velocity_difference_mps": 1e-6,
            },
        }
        guide_probe["passed"] = bool(
            tracking_error <= 1e-4
            and abs(axial_displacement - expected_axial_displacement)
            / expected_axial_displacement
            <= 0.05
            and guide_probe["fixed_offset_error_m"] <= 1e-4
            and relative_axial_velocity_error <= 0.05
            and _maximum_absolute_error(cart_velocity, rocket_velocity) <= 1e-6
        )
        result["probes"]["inclined_prismatic_attached_force"] = guide_probe
        _progress("inclined_prismatic_force_complete")

        try:
            reaction_forces, reaction_torques = guide_articulation.get_link_incoming_joint_force()
            reaction_values = _flatten_numbers(reaction_forces) + _flatten_numbers(reaction_torques)
            reaction_probe = {
                "supported": True,
                "link_names": list(guide_articulation.link_names),
                "forces_n": _json_value(reaction_forces),
                "torques_nm": _json_value(reaction_torques),
                "passed": bool(reaction_values and all(math.isfinite(value) for value in reaction_values)),
            }
        except BaseException as exc:
            reaction_probe = {
                "supported": False,
                "error_type": type(exc).__name__,
                "error_message": str(exc),
                "passed": False,
            }
        result["probes"]["reported_joint_reaction_capability"] = reaction_probe
        _progress("joint_reaction_complete")

        try:
            (
                attached_contact_forces,
                attached_contact_points,
                attached_contact_normals,
                attached_contact_distances,
                attached_pair_counts,
                _,
            ) = release_contact_view.get_contact_data(args.physics_dt_s)
            attached_count_values = _flatten_numbers(attached_pair_counts)
            attached_force_values = _flatten_numbers(attached_contact_forces)
            attached_contact_probe = {
                "collision_treatment": args.collision_treatment,
                "coupling_topology": "fixed joint excluded from guide articulation",
                "excluded_from_articulation": bool(
                    fixed_joint.GetExcludeFromArticulationAttr().Get()
                ),
                "joint_enabled": bool(fixed_joint.GetJointEnabledAttr().Get()),
                "collision_enabled": bool(fixed_joint.GetCollisionEnabledAttr().Get()),
                "pair_counts": _json_value(attached_pair_counts),
                "total_contact_points": int(sum(attached_count_values)),
                "maximum_absolute_reported_force_n": max(
                    (abs(value) for value in attached_force_values), default=0.0
                ),
                "forces_n": _json_value(attached_contact_forces),
                "points_m": _json_value(attached_contact_points),
                "normals": _json_value(attached_contact_normals),
                "distances_m": _json_value(attached_contact_distances),
                "interpretation_gate": "ignore reports while fixed joint is enabled",
            }
            attached_contact_probe["passed"] = bool(
                attached_contact_probe["joint_enabled"]
                and attached_contact_probe["excluded_from_articulation"]
                and attached_contact_probe["collision_enabled"] is collision_enabled_initial
                and (
                    attached_contact_probe["maximum_absolute_reported_force_n"] <= 1e-6
                    if collision_enabled_initial
                    else attached_contact_probe["total_contact_points"] == 0
                )
            )
        except BaseException as exc:
            attached_contact_probe = {
                "collision_treatment": args.collision_treatment,
                "error_type": type(exc).__name__,
                "error_message": str(exc),
                "passed": False,
            }
        result["probes"]["attached_pair_contact_state"] = attached_contact_probe
        _progress("attached_contact_state_complete")

        app_utils.pause()
        before_cart_position = _vector(guide_cart.get_world_poses()[0])
        before_rocket_position = _vector(guide_rocket.get_world_poses()[0])
        before_cart_orientation = _quaternion(guide_cart.get_world_poses()[1])
        before_rocket_orientation = _quaternion(guide_rocket.get_world_poses()[1])
        before_cart_velocity = _vector(guide_cart.get_velocities()[0])
        before_rocket_velocity = _vector(guide_rocket.get_velocities()[0])
        before_relative_axial_offset = sum(
            (before_rocket_position[index] - before_cart_position[index]) * guide_tangent[index]
            for index in range(3)
        )
        fixed_joint.GetJointEnabledAttr().Set(False)
        fixed_joint.GetCollisionEnabledAttr().Set(True)
        simulation_app.update()
        after_cart_position = _vector(guide_cart.get_world_poses()[0])
        after_rocket_position = _vector(guide_rocket.get_world_poses()[0])
        after_cart_orientation = _quaternion(guide_cart.get_world_poses()[1])
        after_rocket_orientation = _quaternion(guide_rocket.get_world_poses()[1])
        after_cart_velocity = _vector(guide_cart.get_velocities()[0])
        after_rocket_velocity = _vector(guide_rocket.get_velocities()[0])
        release_separating_effort_n = -30.0
        conservative_release_acceleration_mps2 = abs(release_separating_effort_n) / assembly_mass_kg
        minimum_release_separating_speed_mps = (
            0.25 * conservative_release_acceleration_mps2 * args.physics_dt_s
        )
        minimum_release_offset_increase_m = (
            0.25 * conservative_release_acceleration_mps2 * args.physics_dt_s**2
        )
        release_probe = {
            "resync_mechanism": "paused Kit update after USD jointEnabled mutation",
            "coupling_topology": "fixed joint excluded from guide articulation",
            "excluded_from_articulation": bool(
                fixed_joint.GetExcludeFromArticulationAttr().Get()
            ),
            "joint_enabled_after_resync": bool(fixed_joint.GetJointEnabledAttr().Get()),
            "collision_enabled_after_resync": bool(fixed_joint.GetCollisionEnabledAttr().Get()),
            "cart_position_discontinuity_m": _maximum_absolute_error(
                after_cart_position, before_cart_position
            ),
            "rocket_position_discontinuity_m": _maximum_absolute_error(
                after_rocket_position, before_rocket_position
            ),
            "cart_orientation_component_discontinuity": _maximum_absolute_error(
                after_cart_orientation, before_cart_orientation
            ),
            "rocket_orientation_component_discontinuity": _maximum_absolute_error(
                after_rocket_orientation, before_rocket_orientation
            ),
            "cart_velocity_discontinuity_mps": _maximum_absolute_error(
                after_cart_velocity, before_cart_velocity
            ),
            "rocket_velocity_discontinuity_mps": _maximum_absolute_error(
                after_rocket_velocity, before_rocket_velocity
            ),
            "acceptance": {
                "maximum_mutation_discontinuity": 1e-8,
                "minimum_post_step_relative_offset_increase_m": minimum_release_offset_increase_m,
                "minimum_post_step_separating_speed_mps": minimum_release_separating_speed_mps,
            },
        }
        mutation_continuity_passed = bool(
            not release_probe["joint_enabled_after_resync"]
            and release_probe["excluded_from_articulation"]
            and release_probe["collision_enabled_after_resync"]
            and max(
                value
                for key, value in release_probe.items()
                if key.endswith(("_m", "_mps", "_discontinuity"))
            )
            <= 1e-8
        )

        # Reading the USD attributes above cannot prove that the solver consumed the
        # mutation.  On the first integrated step, a negative prismatic effort slows
        # the cart while the released rocket coasts.  Relative separation is therefore
        # a backend-observable discriminator: a still-active fixed joint cannot pass it.
        app_utils.play()
        guide_articulation.set_dof_efforts([release_separating_effort_n])
        SimulationManager.step()
        post_step_cart_position = _vector(guide_cart.get_world_poses()[0])
        post_step_rocket_position = _vector(guide_rocket.get_world_poses()[0])
        post_step_cart_velocity = _vector(guide_cart.get_velocities()[0])
        post_step_rocket_velocity = _vector(guide_rocket.get_velocities()[0])
        post_step_relative_axial_offset = sum(
            (post_step_rocket_position[index] - post_step_cart_position[index])
            * guide_tangent[index]
            for index in range(3)
        )
        post_step_separating_speed = sum(
            (post_step_rocket_velocity[index] - post_step_cart_velocity[index])
            * guide_tangent[index]
            for index in range(3)
        )
        relative_offset_increase = post_step_relative_axial_offset - before_relative_axial_offset
        solver_release_confirmed = bool(
            relative_offset_increase >= minimum_release_offset_increase_m
            and post_step_separating_speed >= minimum_release_separating_speed_mps
        )
        release_probe.update(
            {
                "mutation_continuity_passed": mutation_continuity_passed,
                "solver_discriminator": "negative cart prismatic effort while released rocket coasts",
                "solver_discriminator_effort_n": release_separating_effort_n,
                "relative_axial_offset_before_m": before_relative_axial_offset,
                "relative_axial_offset_after_first_step_m": post_step_relative_axial_offset,
                "relative_axial_offset_increase_m": relative_offset_increase,
                "post_step_separating_speed_mps": post_step_separating_speed,
                "solver_release_confirmed": solver_release_confirmed,
                "post_step_cart_position_m": post_step_cart_position,
                "post_step_rocket_position_m": post_step_rocket_position,
                "post_step_cart_velocity_mps": post_step_cart_velocity,
                "post_step_rocket_velocity_mps": post_step_rocket_velocity,
            }
        )
        release_probe["passed"] = mutation_continuity_passed and solver_release_confirmed
        result["probes"]["fixed_joint_release_continuity"] = release_probe
        _progress("solver_release_complete")

        try:
            # Contact is tested after solver release, independently of the separating
            # release discriminator.  A positive prismatic effort now drives the cart
            # into the coasting rocket.  With a functioning pair this must produce a
            # report before the bounded approach expires; a stale/ignored live
            # collision mutation lets the shapes pass through one another instead.
            contact_approach_effort_n = 30.0
            maximum_contact_approach_steps = 100
            contact_step = None
            contact_forces = contact_points = contact_normals = contact_distances = None
            pair_counts = None
            for approach_step in range(1, maximum_contact_approach_steps + 1):
                guide_articulation.set_dof_efforts([contact_approach_effort_n])
                SimulationManager.step()
                (
                    contact_forces,
                    contact_points,
                    contact_normals,
                    contact_distances,
                    pair_counts,
                    _,
                ) = release_contact_view.get_contact_data(args.physics_dt_s)
                candidate_force_values = _flatten_numbers(contact_forces)
                if sum(_flatten_numbers(pair_counts)) > 0 and max(
                    (abs(value) for value in candidate_force_values), default=0.0
                ) > 1e-6:
                    contact_step = approach_step
                    break
            count_values = _flatten_numbers(pair_counts)
            force_values = _flatten_numbers(contact_forces)
            contact_cart_position = _vector(guide_cart.get_world_poses()[0])
            contact_rocket_position = _vector(guide_rocket.get_world_poses()[0])
            contact_relative_axial_offset = sum(
                (contact_rocket_position[index] - contact_cart_position[index])
                * guide_tangent[index]
                for index in range(3)
            )
            contact_probe = {
                "collision_treatment": args.collision_treatment,
                "coupling_topology": "fixed joint excluded from guide articulation",
                "excluded_from_articulation": bool(
                    fixed_joint.GetExcludeFromArticulationAttr().Get()
                ),
                "collision_enabled_at_startup": collision_enabled_initial,
                "joint_enabled": bool(fixed_joint.GetJointEnabledAttr().Get()),
                "collision_enabled": bool(fixed_joint.GetCollisionEnabledAttr().Get()),
                "pair_counts": _json_value(pair_counts),
                "total_contact_points": int(sum(count_values)),
                "maximum_absolute_reported_force_n": max(
                    (abs(value) for value in force_values), default=0.0
                ),
                "forces_n": _json_value(contact_forces),
                "points_m": _json_value(contact_points),
                "normals": _json_value(contact_normals),
                "distances_m": _json_value(contact_distances),
                "initial_surface_gap_m": before_relative_axial_offset - 1.0,
                "post_release_relative_axial_offset_m": post_step_relative_axial_offset,
                "contact_sample_relative_axial_offset_m": contact_relative_axial_offset,
                "contact_approach_effort_n": contact_approach_effort_n,
                "contact_approach_step": contact_step,
                "maximum_contact_approach_steps": maximum_contact_approach_steps,
                "solver_release_confirmed": solver_release_confirmed,
                "sample_phase": "first reported contact during compressive approach after solver-confirmed release",
                "interpretation_gate": "consume reports only after solver release is confirmed",
                "acceptance": {
                    "minimum_contact_points": 1,
                    "contact_within_approach_steps": maximum_contact_approach_steps,
                    "requires_finite_reported_force": True,
                    "minimum_absolute_reported_force_n": 1e-6,
                    "requires_solver_release_confirmation": True,
                },
            }
            contact_probe["passed"] = bool(
                not contact_probe["joint_enabled"]
                and contact_probe["excluded_from_articulation"]
                and contact_probe["collision_enabled"]
                and contact_probe["solver_release_confirmed"]
                and attached_contact_probe["passed"]
                and contact_probe["contact_approach_step"] is not None
                and contact_probe["total_contact_points"] > 0
                and contact_probe["maximum_absolute_reported_force_n"] > 1e-6
                and force_values
                and all(math.isfinite(value) for value in force_values)
            )
        except BaseException as exc:
            contact_probe = {
                "error_type": type(exc).__name__,
                "error_message": str(exc),
                "passed": False,
            }
        result["probes"]["collision_pair_treatment_and_contact_reporting"] = contact_probe
        _progress("contact_reporting_complete")
        app_utils.pause()

        fixed_joint.GetJointEnabledAttr().Set(True)
        fixed_joint.GetCollisionEnabledAttr().Set(collision_enabled_initial)
        guide_articulation.set_dof_efforts([0.0])
        guide_cart.set_world_poses(positions=cart_initial, orientations=guide_orientation)
        guide_rocket.set_world_poses(positions=rocket_initial, orientations=guide_orientation)
        guide_cart.set_velocities(
            linear_velocities=[0.0, 0.0, 0.0], angular_velocities=[0.0, 0.0, 0.0]
        )
        guide_rocket.set_velocities(
            linear_velocities=[0.0, 0.0, 0.0], angular_velocities=[0.0, 0.0, 0.0]
        )
        simulation_app.update()
        reset_cart_position = _vector(guide_cart.get_world_poses()[0])
        reset_rocket_position = _vector(guide_rocket.get_world_poses()[0])
        reset_cart_orientation = _quaternion(guide_cart.get_world_poses()[1])
        reset_rocket_orientation = _quaternion(guide_rocket.get_world_poses()[1])
        live_reset_probe = {
            "joint_enabled": bool(fixed_joint.GetJointEnabledAttr().Get()),
            "excluded_from_articulation": bool(
                fixed_joint.GetExcludeFromArticulationAttr().Get()
            ),
            "collision_enabled": bool(fixed_joint.GetCollisionEnabledAttr().Get()),
            "cart_position_error_m": _maximum_absolute_error(
                reset_cart_position, cart_initial
            ),
            "rocket_position_error_m": _maximum_absolute_error(
                reset_rocket_position, rocket_initial
            ),
            "cart_orientation_error": _quaternion_error(
                reset_cart_orientation, guide_orientation
            ),
            "rocket_orientation_error": _quaternion_error(
                reset_rocket_orientation, guide_orientation
            ),
            "cart_velocity_mps": _vector(guide_cart.get_velocities()[0]),
            "rocket_velocity_mps": _vector(guide_rocket.get_velocities()[0]),
        }
        live_reset_probe["passed"] = bool(
            live_reset_probe["joint_enabled"]
            and live_reset_probe["excluded_from_articulation"]
            and live_reset_probe["collision_enabled"] is collision_enabled_initial
            and live_reset_probe["cart_position_error_m"] <= 1e-6
            and live_reset_probe["rocket_position_error_m"] <= 1e-6
            and live_reset_probe["cart_orientation_error"] <= 1e-6
            and live_reset_probe["rocket_orientation_error"] <= 1e-6
            and max(abs(value) for value in live_reset_probe["cart_velocity_mps"]) <= 1e-6
            and max(abs(value) for value in live_reset_probe["rocket_velocity_mps"]) <= 1e-6
        )

        # Live writes remain diagnostic only.  The accepted lifecycle always stops,
        # rebuilds physics from authored USD, and recreates every view used by release,
        # guidance, force application, and contact reporting.
        reset_probe = {"live_tensor_write_diagnostic": live_reset_probe}
        app_utils.stop()
        simulation_app.update()
        _progress("timeline_stopped_for_reset")
        app_utils.play()
        simulation_app.update()
        _progress("physics_rebuilt_for_reset")
        reset_articulation = Articulation(guide_assembly_path)
        reset_cart = RigidPrim(paths=guide_cart_path)
        reset_rocket = RigidPrim(paths=guide_rocket_path)
        reset_force_body = RigidPrim(paths="/World/ForceProbe")
        reset_contact_view = SimulationManager.get_physics_simulation_view().create_rigid_contact_view(
            [guide_rocket_path],
            filter_patterns=[[guide_cart_path]],
            max_contact_data_count=16,
        )
        reset_articulation.set_dof_gains(stiffnesses=[0.0], dampings=[0.0])
        reset_articulation.set_dof_max_efforts([1.0e6])
        reset_articulation.set_dof_efforts([0.0])

        # Exercise the recreated views through an integration step rather than merely
        # asserting that wrapper construction returned.
        SimulationManager.step()
        cold_cart_position, cold_cart_orientation = reset_cart.get_world_poses()
        cold_rocket_position, cold_rocket_orientation = reset_rocket.get_world_poses()
        cold_force_position, cold_force_orientation = reset_force_body.get_world_poses()
        cold_cart_velocity, cold_cart_angular = reset_cart.get_velocities()
        cold_rocket_velocity, cold_rocket_angular = reset_rocket.get_velocities()
        cold_force_velocity, cold_force_angular = reset_force_body.get_velocities()
        reset_contact_forces, _, _, _, reset_pair_counts, _ = reset_contact_view.get_contact_data(
            args.physics_dt_s
        )
        reset_pair_count_values = _flatten_numbers(reset_pair_counts)
        reset_contact_force_values = _flatten_numbers(reset_contact_forces)
        reset_dof_positions = _flatten_numbers(reset_articulation.get_dof_positions())
        reset_dof_velocities = _flatten_numbers(reset_articulation.get_dof_velocities())
        cold_reset_probe = {
            "recreated_views": [
                "guide_articulation",
                "guide_cart_rigid_body",
                "guide_rocket_rigid_body",
                "force_probe_rigid_body",
                "release_contact_view",
            ],
            "views_exercised_after_rebuild": True,
            "joint_enabled": bool(fixed_joint.GetJointEnabledAttr().Get()),
            "excluded_from_articulation": bool(
                fixed_joint.GetExcludeFromArticulationAttr().Get()
            ),
            "collision_enabled": bool(fixed_joint.GetCollisionEnabledAttr().Get()),
            "cart_position_error_m": _maximum_absolute_error(
                _vector(cold_cart_position), cart_initial
            ),
            "rocket_position_error_m": _maximum_absolute_error(
                _vector(cold_rocket_position), rocket_initial
            ),
            "force_position_error_m": _maximum_absolute_error(
                _vector(cold_force_position), [0.0, 0.0, 0.0]
            ),
            "cart_orientation_error": _quaternion_error(
                _quaternion(cold_cart_orientation), guide_orientation
            ),
            "rocket_orientation_error": _quaternion_error(
                _quaternion(cold_rocket_orientation), guide_orientation
            ),
            "force_orientation_error": _quaternion_error(
                _quaternion(cold_force_orientation), [1.0, 0.0, 0.0, 0.0]
            ),
            "cart_velocity_mps": _vector(cold_cart_velocity),
            "cart_angular_velocity_radps": _vector(cold_cart_angular),
            "rocket_velocity_mps": _vector(cold_rocket_velocity),
            "rocket_angular_velocity_radps": _vector(cold_rocket_angular),
            "force_velocity_mps": _vector(cold_force_velocity),
            "force_angular_velocity_radps": _vector(cold_force_angular),
            "dof_positions": reset_dof_positions,
            "dof_velocities": reset_dof_velocities,
            "dof_gains": {
                "stiffness": _json_value(reset_articulation.get_dof_gains()[0]),
                "damping": _json_value(reset_articulation.get_dof_gains()[1]),
            },
            "dof_armature_kg": _json_value(reset_articulation.get_dof_armatures()),
            "reset_pair_contact_count": int(sum(reset_pair_count_values)),
            "reset_pair_maximum_absolute_force_n": max(
                (abs(value) for value in reset_contact_force_values), default=0.0
            ),
            "acceptance": {
                "maximum_position_error_m": 1e-5,
                "maximum_orientation_component_error": 1e-6,
                "maximum_velocity_component": 1e-6,
                "maximum_dof_state_magnitude": 1e-6,
                "expected_pair_contact_state": "no contact across initial 0.01 m clearance",
            },
        }
        cold_reset_probe["passed"] = bool(
            cold_reset_probe["joint_enabled"]
            and cold_reset_probe["excluded_from_articulation"]
            and cold_reset_probe["collision_enabled"] is collision_enabled_initial
            and cold_reset_probe["cart_position_error_m"] <= 1e-5
            and cold_reset_probe["rocket_position_error_m"] <= 1e-5
            and cold_reset_probe["force_position_error_m"] <= 1e-5
            and cold_reset_probe["cart_orientation_error"] <= 1e-6
            and cold_reset_probe["rocket_orientation_error"] <= 1e-6
            and cold_reset_probe["force_orientation_error"] <= 1e-6
            and max(
                abs(value)
                for key in (
                    "cart_velocity_mps",
                    "cart_angular_velocity_radps",
                    "rocket_velocity_mps",
                    "rocket_angular_velocity_radps",
                    "force_velocity_mps",
                    "force_angular_velocity_radps",
                )
                for value in cold_reset_probe[key]
            )
            <= 1e-6
            and max((abs(value) for value in reset_dof_positions), default=0.0) <= 1e-6
            and max((abs(value) for value in reset_dof_velocities), default=0.0) <= 1e-6
            and (
                cold_reset_probe["reset_pair_maximum_absolute_force_n"] <= 1e-6
                if collision_enabled_initial
                else cold_reset_probe["reset_pair_contact_count"] == 0
            )
        )
        reset_probe["stop_rebuild_recreate_views"] = cold_reset_probe
        reset_probe["accepted_method"] = (
            "stop_rebuild_recreate_views" if cold_reset_probe["passed"] else None
        )
        reset_probe["passed"] = cold_reset_probe["passed"]
        result["probes"]["live_reset_state"] = reset_probe
        _progress("reset_complete")
        result["passed"] = all(probe["passed"] for probe in result["probes"].values())
        exit_code = 0 if result["passed"] else 2
    except BaseException as exc:
        result["error"] = {
            "type": type(exc).__name__,
            "message": str(exc),
            "traceback": traceback.format_exc(),
        }
    finally:
        result["finished_utc"] = datetime.now(timezone.utc).isoformat()
        output.parent.mkdir(parents=True, exist_ok=True)
        serialized_result = _json_value(result)
        rendered_result = json.dumps(
            serialized_result,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        output.write_text(rendered_result + "\n", encoding="utf-8")
        print(f"PHASE0_RESULT={output}")
        print(rendered_result)
        if simulation_app is not None:
            simulation_app.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
