# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Measure the named production rocket/cradle pair on the selected Isaac build.

The probe loads a separately hashed fixture describing the schema-v3 cylindrical rocket
and open-front U-shaped cradle.  The conservative impact case launches the rocket axially
into the cradle rear wall at the configured 1.25-times speed gate.  It records every
physics sample until an unconstrained body would have traversed that wall.  Passing
requires a solver response before full traversal; a proximity manifold with zero force
is evidence, but is not counted as an impact.
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


DEFAULT_FIXTURE = (
    Path(__file__).resolve().parents[1]
    / "configs"
    / "phase0_anti_tunneling_open_cradle.json"
)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--physics-dt-s", type=float, default=0.001)
    parser.add_argument("--test-relative-speed-mps", type=float, default=2500.0)
    parser.add_argument("--ccd", choices=("enabled", "disabled"), default="disabled")
    parser.add_argument("--fixture", type=Path, default=DEFAULT_FIXTURE)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="allow an explicitly selected --output path to replace an existing artifact",
    )
    return parser.parse_args()


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate fixture key {key!r}")
        result[key] = value
    return result


def _positive_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{path} must be a number")
    converted = float(value)
    if not math.isfinite(converted) or converted <= 0.0:
        raise ValueError(f"{path} must be finite and positive")
    return converted


def _load_fixture(path: Path) -> dict[str, Any]:
    fixture = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_unique_object)
    if not isinstance(fixture, dict):
        raise ValueError("fixture root must be an object")
    expected_root = {
        "schema", "pair_name", "impact_case", "initial_clearance_m", "rocket", "cradle"
    }
    if set(fixture) != expected_root:
        raise ValueError(f"fixture root keys must be exactly {sorted(expected_root)}")
    if fixture["schema"] != "vacuum_tube_anti_tunneling_fixture_v1":
        raise ValueError("unsupported anti-tunneling fixture schema")
    if fixture["pair_name"] != "rocket_cradle":
        raise ValueError("fixture pair_name must be 'rocket_cradle'")
    if fixture["impact_case"] != "axial_rear_wall":
        raise ValueError("fixture impact_case must be 'axial_rear_wall'")
    rocket = fixture["rocket"]
    cradle = fixture["cradle"]
    if not isinstance(rocket, dict) or set(rocket) != {
        "shape", "axis", "mass_kg", "length_m", "diameter_m"
    }:
        raise ValueError("fixture rocket fields do not match the v1 schema")
    if rocket["shape"] != "cylinder" or rocket["axis"] != "X":
        raise ValueError("fixture rocket must be an X-axis cylinder")
    if not isinstance(cradle, dict) or set(cradle) != {
        "topology", "mass_kg", "outer_length_m", "outer_width_m", "outer_height_m",
        "wall_thickness_m", "floor_thickness_m"
    }:
        raise ValueError("fixture cradle fields do not match the v1 schema")
    if cradle["topology"] != "open_front_u":
        raise ValueError("fixture cradle must use open_front_u topology")
    normalized = {
        **fixture,
        "initial_clearance_m": _positive_number(
            fixture["initial_clearance_m"], "initial_clearance_m"
        ),
        "rocket": {
            **rocket,
            **{
                key: _positive_number(rocket[key], f"rocket.{key}")
                for key in ("mass_kg", "length_m", "diameter_m")
            },
        },
        "cradle": {
            **cradle,
            **{
                key: _positive_number(cradle[key], f"cradle.{key}")
                for key in (
                    "mass_kg", "outer_length_m", "outer_width_m", "outer_height_m",
                    "wall_thickness_m", "floor_thickness_m"
                )
            },
        },
    }
    rocket = normalized["rocket"]
    cradle = normalized["cradle"]
    if 2.0 * cradle["wall_thickness_m"] >= cradle["outer_width_m"]:
        raise ValueError("cradle side walls leave no open interior width")
    if cradle["floor_thickness_m"] >= cradle["outer_height_m"]:
        raise ValueError("cradle floor consumes the complete open height")
    if rocket["diameter_m"] >= (
        cradle["outer_width_m"] - 2.0 * cradle["wall_thickness_m"]
    ):
        raise ValueError("rocket cylinder does not fit between cradle side walls")
    return normalized


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


def _flatten_numbers(value: Any) -> list[float]:
    converted = _json_value(value)
    if isinstance(converted, list):
        flattened: list[float] = []
        for item in converted:
            flattened.extend(_flatten_numbers(item))
        return flattened
    return [float(converted)]


def _vector(value: Any) -> list[float]:
    converted = _json_value(value)
    while isinstance(converted, list) and len(converted) == 1:
        converted = converted[0]
    if not isinstance(converted, list) or len(converted) != 3:
        raise RuntimeError(f"expected a three-vector, got {converted!r}")
    return [float(component) for component in converted]


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _git_head(repository: Path) -> str | None:
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


def _default_output_path(
    script_path: Path, started_at: datetime, run_id: str, *, ccd_enabled: bool, dt_s: float
) -> Path:
    timestamp = started_at.astimezone(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    treatment = "ccd" if ccd_enabled else "discrete"
    dt_us = round(dt_s * 1_000_000.0)
    return (
        script_path.resolve().parents[1]
        / "artifacts"
        / "phase0"
        / "anti_tunneling"
        / f"{timestamp}_{treatment}_{dt_us}us_{run_id}.json"
    )


def _runtime_selection_passed(probe: dict[str, Any], requested_dt_s: float) -> bool:
    settings = probe["settings"]
    return bool(
        probe["switch_returned"]
        and probe["active_engine"] == "physx"
        and "cpu" in str(probe["device"]).lower()
        and math.isclose(
            float(probe["physics_dt_s"]), requested_dt_s, rel_tol=0.0, abs_tol=1e-12
        )
        and settings["/app/player/useFixedTimeStepping"] is True
        and settings["/app/runLoops/main/rateLimitEnabled"] is False
    )


def _anti_tunneling_passed(
    *, impact_observed: bool, full_barrier_traversal_observed: bool, samples_finite: bool
) -> bool:
    return bool(impact_observed and not full_barrier_traversal_observed and samples_finite)


def _progress(label: str) -> None:
    print(f"ANTI_TUNNELING_PROGRESS={label}", flush=True)


def main() -> int:
    args = _arguments()
    if not math.isfinite(args.physics_dt_s) or args.physics_dt_s <= 0.0:
        raise SystemExit("--physics-dt-s must be finite and positive")
    if not math.isfinite(args.test_relative_speed_mps) or args.test_relative_speed_mps <= 0.0:
        raise SystemExit("--test-relative-speed-mps must be finite and positive")
    fixture_path = args.fixture.resolve()
    if not fixture_path.is_file():
        raise SystemExit(f"anti-tunneling fixture does not exist: {fixture_path}")
    fixture = _load_fixture(fixture_path)

    isaac_path = os.environ.get("ISAAC_PATH")
    if not isaac_path:
        raise SystemExit("ISAAC_PATH is not set; launch this script through the build's python.bat")
    release_root = Path(isaac_path).resolve()
    isaac_root = release_root.parents[2]
    experience = release_root / "apps" / "isaacsim.exp.full.kit"
    started_at = datetime.now(timezone.utc)
    run_id = uuid.uuid4().hex
    script_path = Path(__file__).resolve()
    ccd_enabled = args.ccd == "enabled"
    output = args.output or _default_output_path(
        script_path, started_at, run_id, ccd_enabled=ccd_enabled, dt_s=args.physics_dt_s
    )
    if output.exists() and not args.overwrite:
        raise SystemExit(
            f"qualification artifact already exists: {output}; choose another --output or pass --overwrite"
        )

    version_file = isaac_root / "VERSION"
    executable = Path(sys.executable).resolve()
    result: dict[str, Any] = {
        "schema": "vacuum_tube_anti_tunneling_qualification_v1",
        "run_id": run_id,
        "started_utc": started_at.isoformat(),
        "requested": {
            "backend": "physx",
            "device": "cpu",
            "physics_dt_s": args.physics_dt_s,
            "test_relative_speed_mps": args.test_relative_speed_mps,
            "ccd_enabled": ccd_enabled,
            "fixture": str(fixture_path),
            "fixture_schema": fixture["schema"],
            "impact_case": fixture["impact_case"],
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
            "fixture_path": str(fixture_path),
            "fixture_sha256": _sha256_file(fixture_path),
            "experience_path": str(experience),
            "experience_sha256": _sha256_file(experience) if experience.is_file() else None,
            "version_file_path": str(version_file),
            "version_file_sha256": _sha256_file(version_file) if version_file.is_file() else None,
            "python_executable_path": str(executable),
            "python_executable_sha256": _sha256_file(executable) if executable.is_file() else None,
            "isaac_sim_source_git_revision": _git_head(isaac_root),
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
        from isaacsim.core.experimental.prims import RigidPrim
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

        settings = carb.settings.get_settings()
        stage_utils.create_new_stage()
        stage = stage_utils.get_current_stage()
        physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        physics_scene.CreateGravityDirectionAttr().Set((0.0, 0.0, -1.0))
        physics_scene.CreateGravityMagnitudeAttr().Set(0.0)
        SimulationManager.set_default_physics_scene("/World/PhysicsScene")
        available = SimulationManager.get_available_physics_engines(verbose=False)
        switched = SimulationManager.switch_physics_engine("physx", verbose=False)
        SimulationManager.setup_simulation(dt=args.physics_dt_s, device="cpu")
        SimulationManager.enable_ccd(ccd_enabled, physics_scene="/World/PhysicsScene")
        runtime_probe = {
            "available_engines": available,
            "switch_returned": switched,
            "active_engine": SimulationManager.get_active_physics_engine(),
            "device": str(SimulationManager.get_device()),
            "tensor_backend": SimulationManager.get_backend(),
            "physics_dt_s": SimulationManager.get_physics_dt(),
            "solver_type": SimulationManager.get_solver_type(),
            "scene_ccd_enabled": SimulationManager.is_ccd_enabled("/World/PhysicsScene"),
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
        runtime_probe["passed"] = bool(
            _runtime_selection_passed(runtime_probe, args.physics_dt_s)
            and runtime_probe["scene_ccd_enabled"] is ccd_enabled
        )
        result["probes"]["runtime_selection"] = runtime_probe
        _progress("runtime_selection_complete")

        rocket_spec = fixture["rocket"]
        cradle_spec = fixture["cradle"]
        rocket_length_m = rocket_spec["length_m"]
        rocket_radius_m = 0.5 * rocket_spec["diameter_m"]
        cradle_length_m = cradle_spec["outer_length_m"]
        cradle_width_m = cradle_spec["outer_width_m"]
        cradle_height_m = cradle_spec["outer_height_m"]
        wall_thickness_m = cradle_spec["wall_thickness_m"]
        floor_thickness_m = cradle_spec["floor_thickness_m"]
        initial_clearance_m = fixture["initial_clearance_m"]
        rear_wall_center_x_m = -0.5 * cradle_length_m + 0.5 * wall_thickness_m
        rear_wall_forward_face_x_m = rear_wall_center_x_m + 0.5 * wall_thickness_m
        rocket_initial_x_m = (
            rear_wall_forward_face_x_m + 0.5 * rocket_length_m + initial_clearance_m
        )
        rocket_initial_z_m = (
            -0.5 * cradle_height_m
            + floor_thickness_m
            + rocket_radius_m
            + initial_clearance_m
        )
        rocket_path = "/World/RocketProbe"
        cradle_path = "/World/CradleProbe"
        rocket_geometry = UsdGeom.Cylinder.Define(stage, rocket_path)
        rocket_geometry.CreateAxisAttr(UsdGeom.Tokens.x)
        rocket_geometry.CreateHeightAttr(rocket_length_m)
        rocket_geometry.CreateRadiusAttr(rocket_radius_m)
        UsdPhysics.CollisionAPI.Apply(rocket_geometry.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(rocket_geometry.GetPrim())
        UsdPhysics.MassAPI.Apply(rocket_geometry.GetPrim()).CreateMassAttr(
            rocket_spec["mass_kg"]
        )
        UsdGeom.Xformable(rocket_geometry.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(rocket_initial_x_m, 0.0, rocket_initial_z_m)
        )
        rocket = RigidPrim(
            paths=rocket_path,
            masses=[rocket_spec["mass_kg"]],
            contact_filter_paths=[cradle_path],
            max_contact_count=32,
        )
        rocket.set_enabled_contact_tracking([True])

        cradle_root = UsdGeom.Xform.Define(stage, cradle_path)
        UsdPhysics.RigidBodyAPI.Apply(cradle_root.GetPrim())
        UsdPhysics.MassAPI.Apply(cradle_root.GetPrim()).CreateMassAttr(cradle_spec["mass_kg"])
        component_specs = {
            "Floor": (
                (0.0, 0.0, -0.5 * cradle_height_m + 0.5 * floor_thickness_m),
                (cradle_length_m, cradle_width_m, floor_thickness_m),
            ),
            "LeftRail": (
                (0.0, 0.5 * cradle_width_m - 0.5 * wall_thickness_m, 0.0),
                (cradle_length_m, wall_thickness_m, cradle_height_m),
            ),
            "RightRail": (
                (0.0, -0.5 * cradle_width_m + 0.5 * wall_thickness_m, 0.0),
                (cradle_length_m, wall_thickness_m, cradle_height_m),
            ),
            "RearWall": (
                (rear_wall_center_x_m, 0.0, 0.0),
                (wall_thickness_m, cradle_width_m, cradle_height_m),
            ),
        }
        for name, (translation, scale) in component_specs.items():
            component = UsdGeom.Cube.Define(stage, f"{cradle_path}/{name}")
            component.CreateSizeAttr(1.0)
            transform = UsdGeom.Xformable(component.GetPrim())
            transform.AddTranslateOp().Set(Gf.Vec3d(*translation))
            transform.AddScaleOp().Set(Gf.Vec3f(*scale))
            UsdPhysics.CollisionAPI.Apply(component.GetPrim())

        cradle = RigidPrim(paths=cradle_path, masses=[cradle_spec["mass_kg"]])
        UsdPhysics.RigidBodyAPI(cradle.prims[0]).CreateKinematicEnabledAttr(True)
        rocket_physx_api = PhysxSchema.PhysxRigidBodyAPI.Apply(rocket.prims[0])
        rocket_physx_api.CreateEnableCCDAttr(ccd_enabled)

        app_utils.play()
        simulation_app.update()
        contact_view = SimulationManager.get_physics_simulation_view().create_rigid_contact_view(
            [rocket_path], filter_patterns=[[cradle_path]], max_contact_data_count=32
        )
        rocket.set_world_poses(
            positions=[rocket_initial_x_m, 0.0, rocket_initial_z_m],
            orientations=[1.0, 0.0, 0.0, 0.0],
        )
        rocket.set_velocities(
            linear_velocities=[-args.test_relative_speed_mps, 0.0, 0.0],
            angular_velocities=[0.0, 0.0, 0.0],
        )
        cradle.set_world_poses(
            positions=[0.0, 0.0, 0.0], orientations=[1.0, 0.0, 0.0, 0.0]
        )
        cradle.set_velocities(
            linear_velocities=[0.0, 0.0, 0.0], angular_velocities=[0.0, 0.0, 0.0]
        )

        rear_wall_back_face_x_m = rear_wall_center_x_m - 0.5 * wall_thickness_m
        far_traversal_center_x_m = rear_wall_back_face_x_m - 0.5 * rocket_length_m
        free_traversal_distance_m = rocket_initial_x_m - far_traversal_center_x_m
        steps = max(
            3,
            math.ceil(
                free_traversal_distance_m
                / (args.test_relative_speed_mps * args.physics_dt_s)
            )
            + 2,
        )
        samples: list[dict[str, Any]] = []
        maximum_absolute_force_n = 0.0
        maximum_pair_count = 0
        impact_observed = False
        full_barrier_traversal_observed = False
        samples_finite = True
        for step_index in range(1, steps + 1):
            SimulationManager.step()
            position_values, _ = rocket.get_world_poses()
            linear_values, angular_values = rocket.get_velocities()
            position = _vector(position_values)
            linear_velocity = _vector(linear_values)
            angular_velocity = _vector(angular_values)
            contact_forces, points, normals, distances, pair_counts, _ = (
                contact_view.get_contact_data(args.physics_dt_s)
            )
            force_values = _flatten_numbers(contact_forces)
            pair_count_values = _flatten_numbers(pair_counts)
            step_maximum_force_n = max((abs(value) for value in force_values), default=0.0)
            step_pair_count = int(sum(pair_count_values))
            maximum_absolute_force_n = max(maximum_absolute_force_n, step_maximum_force_n)
            maximum_pair_count = max(maximum_pair_count, step_pair_count)
            step_impact = step_pair_count > 0 and step_maximum_force_n > 1e-6
            impact_observed = impact_observed or step_impact
            step_full_barrier_traversal = bool(
                position[0] < far_traversal_center_x_m and linear_velocity[0] < 0.0
            )
            # The view is sampled only after the solver step, so a sample that reports
            # both a manifold and a body beyond the far face has no defensible ordering.
            # Treat it as tunneling instead of assuming the contact happened first.
            full_barrier_traversal_observed = (
                full_barrier_traversal_observed or step_full_barrier_traversal
            )
            numeric_values = position + linear_velocity + angular_velocity + force_values
            samples_finite = samples_finite and all(math.isfinite(value) for value in numeric_values)
            samples.append(
                {
                    "step": step_index,
                    "time_s": step_index * args.physics_dt_s,
                    "position_m": position,
                    "linear_velocity_mps": linear_velocity,
                    "angular_velocity_radps": angular_velocity,
                    "pair_contact_count": step_pair_count,
                    "maximum_absolute_reported_force_n": step_maximum_force_n,
                    "contact_points_m": _json_value(points),
                    "contact_normals": _json_value(normals),
                    "contact_distances_m": _json_value(distances),
                    "impact_observed": step_impact,
                    "full_barrier_traversal": step_full_barrier_traversal,
                }
            )

        body_ccd_value = rocket_physx_api.GetEnableCCDAttr().Get()
        outcome_probe = {
            "pair_name": "rocket_cradle",
            "geometry_source": str(fixture_path),
            "geometry_sha256": _sha256_file(fixture_path),
            "impact_case": fixture["impact_case"],
            "rocket_shape": rocket_spec["shape"],
            "rocket_axis": rocket_spec["axis"],
            "rocket_mass_kg": rocket_spec["mass_kg"],
            "cradle_topology": cradle_spec["topology"],
            "cradle_components": sorted(component_specs),
            "cradle_front_open": True,
            "cradle_mass_kg": cradle_spec["mass_kg"],
            "cradle_kinematic": True,
            "rocket_dimensions_m": [
                rocket_spec["length_m"], rocket_spec["diameter_m"]
            ],
            "cradle_outer_dimensions_m": [
                cradle_length_m, cradle_width_m, cradle_height_m
            ],
            "cradle_wall_thickness_m": wall_thickness_m,
            "cradle_floor_thickness_m": floor_thickness_m,
            "initial_clearance_m": initial_clearance_m,
            "initial_rocket_center_m": [
                rocket_initial_x_m, 0.0, rocket_initial_z_m
            ],
            "rear_wall_forward_face_x_m": rear_wall_forward_face_x_m,
            "far_traversal_center_x_m": far_traversal_center_x_m,
            "initial_relative_velocity_mps": [-args.test_relative_speed_mps, 0.0, 0.0],
            "per_step_free_travel_m": args.test_relative_speed_mps * args.physics_dt_s,
            "scene_ccd_enabled": SimulationManager.is_ccd_enabled("/World/PhysicsScene"),
            "rocket_ccd_enabled": bool(body_ccd_value),
            "steps": steps,
            "maximum_pair_contact_count": maximum_pair_count,
            "maximum_absolute_reported_force_n": maximum_absolute_force_n,
            "maximum_reported_contact_impulse_ns": (
                maximum_absolute_force_n * args.physics_dt_s
            ),
            "impact_observed": impact_observed,
            "full_barrier_traversal_observed": full_barrier_traversal_observed,
            "samples_finite": samples_finite,
            "samples": samples,
            "acceptance": {
                "requires_nonzero_finite_contact_force": True,
                "allows_full_barrier_traversal_before_impact": False,
                "test_speed_is_1_25_times_design_speed": True,
            },
        }
        outcome_probe["passed"] = _anti_tunneling_passed(
            impact_observed=impact_observed,
            full_barrier_traversal_observed=full_barrier_traversal_observed,
            samples_finite=samples_finite,
        )
        result["probes"]["rocket_cradle_pair"] = outcome_probe
        _progress("rocket_cradle_pair_complete")
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
        rendered_result = json.dumps(
            _json_value(result), indent=2, sort_keys=True, allow_nan=False
        )
        output.write_text(
            rendered_result + "\n", encoding="utf-8", newline="\n"
        )
        print(f"ANTI_TUNNELING_RESULT={output}")
        print(rendered_result)
        if simulation_app is not None:
            simulation_app.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
