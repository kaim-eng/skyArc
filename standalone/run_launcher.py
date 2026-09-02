# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Build the approved schema-v3 production launcher scene in Isaac Sim.

``SimulationApp`` is constructed before any extension-dependent import.  This first
production slice validates configuration, resolves one scene plan, selects CPU PhysX,
authors the complete launcher visualization and qualified rocket/cradle geometry, creates
physics views, and optionally saves the resulting USD.  Mission execution remains owned by
the common orchestrator and is not silently approximated here.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import traceback
from pathlib import Path


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
    parser.add_argument("--save-usd", type=Path)
    parser.add_argument("--summary", type=Path)
    parser.add_argument(
        "--capture-dir",
        type=Path,
        help="Render the authored views to PNG and record their luminance statistics.",
    )
    parser.add_argument(
        "--capture-views",
        default="vehicle,full_system",
        help="Comma-separated subset of the authored camera views to capture.",
    )
    parser.add_argument(
        "--key-intensity",
        type=float,
        help="Override the authored key-light intensity, for tuning exposure.",
    )
    parser.add_argument(
        "--dome-intensity",
        type=float,
        help="Override the authored dome-fill intensity, for tuning exposure.",
    )
    parser.add_argument(
        "--capture-orbit",
        type=Path,
        help=(
            "Render an orbiting sweep of one view into an animated GIF. The scene is static "
            "at t=0; only the camera moves, so this shows geometry and never implies motion "
            "the simulation did not produce."
        ),
    )
    parser.add_argument("--orbit-view", default="full_system")
    parser.add_argument("--orbit-frames", type=int, default=36)
    parser.add_argument(
        "--orbit-degrees",
        type=float,
        default=32.0,
        help=(
            "Half-sweep in azimuth. The centerline is planar, so a full revolution would "
            "pass through edge-on where it collapses to a line."
        ),
    )
    parser.add_argument(
        "--orbit-mode",
        choices=("sweep", "revolve"),
        default="sweep",
        help="'sweep' rocks through +/- --orbit-degrees; 'revolve' goes fully around.",
    )
    parser.add_argument(
        "--orbit-margin",
        type=float,
        default=1.12,
        help=(
            "Framing slack for the full-system orbit, as a multiple of the distance that "
            "exactly contains the centerline. Ignored for the vehicle view."
        ),
    )
    parser.add_argument("--orbit-width", type=int, default=800)
    parser.add_argument("--orbit-height", type=int, default=450)
    parser.add_argument("--orbit-frame-ms", type=int, default=70)
    parser.add_argument("--capture-width", type=int, default=1280)
    parser.add_argument("--capture-height", type=int, default=720)
    parser.add_argument(
        "--capture-settle-frames",
        type=int,
        default=60,
        help=(
            "Frames rendered before the captured one. Section 16.3 requires a recorded "
            "settle interval: capturing early reproduces the black-frame symptom without "
            "any lighting fault actually being present."
        ),
    )
    return parser.parse_args()


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _camera_views(entrance, exit_position):
    """Authored viewpoints, in the global frame the visuals are authored in.

    Both are placed off the launcher's plane along ``+/-Y``: the centerline is planar in
    X-Z, so a viewpoint on the plane would look along the tube rather than across it and
    the geometry would collapse to a line.
    """
    tangent = entrance.tangent
    origin = entrance.position_m
    # The assembly runs from behind the entrance (the cart, seated so the *centre of mass*
    # sits at s=0) to the rocket nose ahead of it; centre the view on its middle rather
    # than on the entrance, or half the vehicle sits outside the frame.
    assembly = tuple(origin[index] + 0.8 * tangent[index] for index in range(3))
    midpoint = tuple(0.5 * (origin[index] + exit_position[index]) for index in range(3))
    span_m = sum(
        (exit_position[index] - origin[index]) ** 2 for index in range(3)
    ) ** 0.5
    # Distance to contain the whole climb. The default camera's horizontal field of view is
    # about 47 degrees, so the visible width is roughly 0.87 x distance; the path's 31 km
    # rise against a 16:9 frame is what binds, not its 43 km run. 0.75 x span cropped both
    # corners, which is how the first system-scale frame lost its ends.
    return {
        # Vehicle scale: the cart and rocket are a few metres against a 54 km path, so
        # they are unreadable in any view framed on the whole system.
        "vehicle": {
            "position": (assembly[0], -14.0, assembly[2] + 1.5),
            "look_at": assembly,
            "schematic": False,
        },
        "full_system": {
            "position": (midpoint[0], -1.4 * span_m, midpoint[2]),
            "look_at": midpoint,
            "schematic": True,
        },
    }


def _orbit_azimuths(half_sweep_deg: float, frames: int, mode: str):
    """Azimuth per frame for a seamlessly looping camera move.

    Both modes close the loop exactly, so the GIF has no visible cut at the wrap.

    ``sweep`` rocks through ``+/-A`` as ``A*sin(2*pi*i/N)``. This is the mode for the whole
    launcher, whose centerline is planar in X-Z: a full revolution would twice bring the
    camera edge-on, where 54 km of tube collapses to a point. The sinusoid also eases at
    the two turnarounds, where a linear ping-pong would visibly snap around.

    ``revolve`` goes all the way round at constant rate. That only makes sense for a
    subject with real extent in every direction, which at vehicle scale the cart and
    rocket have.
    """
    import math

    if mode == "revolve":
        return [360.0 * index / frames for index in range(frames)]
    return [
        half_sweep_deg * math.sin(2.0 * math.pi * index / frames) for index in range(frames)
    ]


def _path_framing(layout, path_pose, width: int, height: int, margin: float):
    """Frame the whole centerline on its own bounding box.

    ``_camera_views`` centres the system view on the midpoint of the entrance-to-exit
    chord, which is right for the bound still evidence and is deliberately left alone. But
    the path bulges well above its own chord, so that framing leaves a wide empty margin
    on one diagonal and crowds the other. Sampling the actual curve centres it properly and
    lets the camera come in closer, which is what makes the tube read at GIF resolution.
    """
    import math

    samples = [path_pose(layout, layout.length_m * i / 96.0).position_m for i in range(97)]
    xs = [p[0] for p in samples]
    zs = [p[2] for p in samples]
    centre = (0.5 * (min(xs) + max(xs)), 0.0, 0.5 * (min(zs) + max(zs)))
    half_h_fov = math.radians(47.0) / 2.0
    half_v_fov = math.atan(math.tan(half_h_fov) * height / width)
    # Worst case is azimuth zero: rotating about the vertical axis only foreshortens the
    # X extent, and the curve has no meaningful Y extent to swing into view.
    distance = margin * max(
        0.5 * (max(xs) - min(xs)) / math.tan(half_h_fov),
        0.5 * (max(zs) - min(zs)) / math.tan(half_v_fov),
    )
    return {
        "position": (centre[0], -distance, centre[2]),
        "look_at": centre,
        "schematic": True,
    }


def _orbit_pose(camera_position, look_at, azimuth_deg: float):
    """Rotate a viewpoint about the vertical axis through its target."""
    import math

    from pxr import Gf

    angle = math.radians(azimuth_deg)
    dx = camera_position[0] - look_at[0]
    dy = camera_position[1] - look_at[1]
    eye = Gf.Vec3d(
        look_at[0] + dx * math.cos(angle) - dy * math.sin(angle),
        look_at[1] + dx * math.sin(angle) + dy * math.cos(angle),
        camera_position[2],
    )
    # SetLookAt builds a world-to-camera view matrix; a camera prim carries the inverse.
    # The stage is Z-up, and neither authored viewpoint looks along +/-Z, so the up vector
    # never goes parallel to the view direction.
    view = Gf.Matrix4d().SetLookAt(eye, Gf.Vec3d(*look_at), Gf.Vec3d(0.0, 0.0, 1.0))
    return eye, view.GetInverse()


def _as_image(frame, width: int, height: int):
    """Coerce annotator output into an ``(H, W, C)`` array, or say why it cannot be.

    The annotator hands back a flat buffer until a render has actually driven the render
    product, so an unexpected shape here means "no frame was produced", not "the frame is
    oddly laid out". Reshaping blindly would turn that into a plausible-looking black
    image and defeat the whole point of the section 16.3 content check.
    """
    import numpy as np

    array = np.asarray(frame)
    if array.ndim == 3:
        return array
    expected = height * width * 4
    if array.ndim == 1 and array.size == expected:
        return array.reshape(height, width, 4)
    raise RuntimeError(
        f"the capture annotator produced no usable frame: shape {array.shape}, "
        f"size {array.size}, expected {expected} for {width}x{height} RGBA"
    )


def _luminance_statistics(rgb) -> dict[str, float]:
    """Mean and variance of Rec. 709 luma over a captured frame.

    Section 16.3 rejects a frame on content, not on encoded size: a correct dark interior
    and an incorrect noisy frame can land on either side of any size threshold. Variance
    is the part that distinguishes a rendered scene from a uniform fill, so a black frame
    and a blown-out white one both fail on the mean while a flat grey fails on variance.
    """
    import numpy as np

    pixels = np.asarray(rgb, dtype=np.float64)[:, :, :3] / 255.0
    luma = 0.2126 * pixels[:, :, 0] + 0.7152 * pixels[:, :, 1] + 0.0722 * pixels[:, :, 2]
    return {
        "mean_luminance": float(luma.mean()),
        "luminance_variance": float(luma.var()),
        "nonzero_pixel_fraction": float((luma > 1e-4).mean()),
    }


def main() -> int:
    args = _arguments()
    isaac_path = os.environ.get("ISAAC_PATH")
    if not isaac_path:
        raise SystemExit("ISAAC_PATH is not set; launch through the target build's python.bat")
    project = Path(__file__).resolve().parents[1]
    release = Path(isaac_path).resolve()
    experience = release / "apps" / "isaacsim.exp.full.kit"
    app = None
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
        import isaacsim.core.experimental.utils.app as app_utils
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
        from skyarc.launcher.geometry import CurvedTubeLayout, path_pose
        from skyarc.launcher.path_controller import (
            LaunchProfileReferenceFrame,
        )
        from skyarc.launcher.production import (
            build_production_scene_plan,
            load_production_fixture,
            resolve_initial_solver_states,
        )
        from skyarc.launcher.scene import (
            DEFAULT_DOME_INTENSITY,
            DEFAULT_KEY_INTENSITY,
            SCHEMATIC_PATH,
            TRUE_SCALE_PATH,
            build_launcher_scene,
        )
        from skyarc.names import BODY_CART, BODY_ROCKET

        loaded = load_yaml(args.configuration)
        config = loaded.config
        layout = resolve_tube_layout(config)
        if not isinstance(layout, CurvedTubeLayout):
            raise RuntimeError("production launcher requires a schema-v3 curved layout")
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

        # Solver coordinates start at the translated-frame origin while public scene-plan
        # geometry remains global SI. The placement itself comes from the shared production
        # helper rather than being restated here, so the constructed scene and the mission
        # runner cannot start from two different initial conditions.
        reference = LaunchProfileReferenceFrame(
            layout, target_exit_speed_mps=config.launch_control.target_exit_speed_mps
        ).sample(0.0)
        initial = resolve_initial_solver_states(layout, plan, fixture, reference)
        cart_position = initial[BODY_CART].position_m
        rocket_position = initial[BODY_ROCKET].position_m
        orientation = initial[BODY_CART].orientation_wxyz
        built = build_launcher_scene(
            stage,
            config,
            plan,
            cart_position_m=cart_position,
            rocket_position_m=rocket_position,
            orientation_wxyz=orientation,
            cart_velocity_mps=initial[BODY_CART].linear_velocity_mps,
            rocket_velocity_mps=initial[BODY_ROCKET].linear_velocity_mps,
            angular_velocity_radps=initial[BODY_CART].angular_velocity_radps,
            author_visuals=True,
            key_intensity=(
                DEFAULT_KEY_INTENSITY if args.key_intensity is None else args.key_intensity
            ),
            dome_intensity=(
                DEFAULT_DOME_INTENSITY if args.dome_intensity is None else args.dome_intensity
            ),
        )

        app_utils.play()
        app.update()
        built.cart.set_world_poses(positions=[cart_position], orientations=[orientation])
        built.rocket.set_world_poses(positions=[rocket_position], orientations=[orientation])
        built.cart.set_velocities((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        built.rocket.set_velocities((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        app_utils.pause()
        settings = carb.settings.get_settings()

        captures: dict[str, object] | None = None
        if args.capture_dir is not None:
            import omni.replicator.core as rep
            from PIL import Image

            args.capture_dir.mkdir(parents=True, exist_ok=True)
            entrance = path_pose(layout, 0.0)
            exit_position = path_pose(layout, layout.length_m).position_m
            views = _camera_views(entrance, exit_position)
            requested = [name.strip() for name in args.capture_views.split(",") if name.strip()]
            unknown = sorted(set(requested) - set(views))
            if unknown:
                raise RuntimeError(f"unknown capture views {unknown}; known: {sorted(views)}")
            captures = {
                "resolution": [args.capture_width, args.capture_height],
                "settle_frames": args.capture_settle_frames,
                # Section 16.3: an image threshold is meaningless unless the renderer and
                # resolution it was produced at are pinned alongside it.
                "renderer": str(settings.get("/renderer/active")),
                "views": {},
            }
            from pxr import UsdGeom as _UsdGeom

            true_scale = _UsdGeom.Imageable(stage.GetPrimAtPath(TRUE_SCALE_PATH))
            schematic = _UsdGeom.Imageable(stage.GetPrimAtPath(SCHEMATIC_PATH))

            for name in requested:
                view = views[name]
                # The true tube is roughly a fourteenth of a pixel wide from system-scale
                # distance, so that view swaps in the exaggerated schematic band. Only one
                # set is ever visible, so no frame mixes the two scales.
                if view["schematic"]:
                    true_scale.MakeInvisible()
                    schematic.MakeVisible()
                else:
                    schematic.MakeInvisible()
                    true_scale.MakeVisible()
                camera = rep.create.camera(
                    position=view["position"], look_at=view["look_at"]
                )
                product = rep.create.render_product(
                    camera, (args.capture_width, args.capture_height)
                )
                annotator = rep.AnnotatorRegistry.get_annotator("rgb")
                annotator.attach([product])
                # The enclosed, cutaway-style interior converges more slowly than an open
                # scene, so the settle interval is rendered before the frame that counts.
                # ``app.update()`` alone does not drive a replicator render product; the
                # orchestrator is what actually renders it.
                for _ in range(max(1, args.capture_settle_frames)):
                    rep.orchestrator.step(rt_subframes=1)
                image = _as_image(
                    annotator.get_data(), args.capture_width, args.capture_height
                )
                statistics = _luminance_statistics(image)
                image_path = args.capture_dir / f"{name}.png"
                Image.fromarray(image[:, :, :3].astype("uint8")).save(image_path)
                annotator.detach()
                product.destroy()
                captures["views"][name] = {
                    "image": str(image_path.resolve()),
                    "camera_position_m": list(view["position"]),
                    "look_at_m": list(view["look_at"]),
                    # Recorded so no reader mistakes an exaggerated band for the real tube.
                    "schematic_tube": bool(view["schematic"]),
                    **statistics,
                }

        orbit: dict[str, object] | None = None
        if args.capture_orbit is not None:
            import numpy as np
            import omni.replicator.core as rep
            from PIL import Image
            from pxr import UsdGeom

            entrance = path_pose(layout, 0.0)
            exit_position = path_pose(layout, layout.length_m).position_m
            views = _camera_views(entrance, exit_position)
            if args.orbit_view not in views:
                raise RuntimeError(
                    f"unknown orbit view {args.orbit_view!r}; known: {sorted(views)}"
                )
            view = views[args.orbit_view]
            if args.orbit_view == "full_system":
                view = _path_framing(
                    layout, path_pose, args.orbit_width, args.orbit_height, args.orbit_margin
                )
            true_scale = UsdGeom.Imageable(stage.GetPrimAtPath(TRUE_SCALE_PATH))
            schematic = UsdGeom.Imageable(stage.GetPrimAtPath(SCHEMATIC_PATH))
            if view["schematic"]:
                true_scale.MakeInvisible()
                schematic.MakeVisible()
            else:
                schematic.MakeInvisible()
                true_scale.MakeVisible()

            # An authored camera rather than one replicator camera per frame: the pose is
            # then ours to set directly, one render product serves the whole sweep, and the
            # clipping range can be widened. The system-scale viewpoint sits ~76 km back,
            # which a default far plane would cut away entirely.
            camera_prim = UsdGeom.Camera.Define(stage, "/World/OrbitCamera")
            # Matches the replicator camera default the still views were framed against
            # (24 mm against a 20.955 mm aperture, about 47 degrees horizontally); only the
            # ratio matters, so this is unit-agnostic.
            camera_prim.CreateFocalLengthAttr().Set(24.0)
            camera_prim.CreateHorizontalApertureAttr().Set(20.955)
            camera_prim.CreateVerticalApertureAttr().Set(
                20.955 * args.orbit_height / args.orbit_width
            )
            camera_prim.CreateClippingRangeAttr().Set((0.1, 5.0e5))
            transform = UsdGeom.Xformable(camera_prim.GetPrim()).MakeMatrixXform()

            product = rep.create.render_product(
                "/World/OrbitCamera", (args.orbit_width, args.orbit_height)
            )
            annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            annotator.attach([product])

            frames = []
            azimuths = _orbit_azimuths(
                args.orbit_degrees, max(2, args.orbit_frames), args.orbit_mode
            )
            for index, azimuth in enumerate(azimuths):
                eye, matrix = _orbit_pose(view["position"], view["look_at"], azimuth)
                transform.Set(matrix)
                # Moving the camera resets path-traced accumulation, so every frame needs
                # the settle interval, not just the first. The first frame additionally
                # waits for the scene itself to converge.
                settle = max(1, args.capture_settle_frames)
                for _ in range(settle if index == 0 else max(1, settle // 2)):
                    rep.orchestrator.step(rt_subframes=1)
                image = _as_image(
                    annotator.get_data(), args.orbit_width, args.orbit_height
                )
                frames.append(Image.fromarray(image[:, :, :3].astype("uint8")))
            annotator.detach()
            product.destroy()

            # One shared palette taken from the mid-sweep frame. Quantising each frame
            # independently makes the palette shift from frame to frame, which shows up as
            # colour crawl across the whole image on a scene this uniformly lit.
            base = frames[len(frames) // 2].quantize(colors=192, method=Image.MEDIANCUT)
            quantized = [frame.quantize(palette=base, dither=Image.FLOYDSTEINBERG) for frame in frames]
            args.capture_orbit.parent.mkdir(parents=True, exist_ok=True)
            quantized[0].save(
                args.capture_orbit,
                save_all=True,
                append_images=quantized[1:],
                duration=args.orbit_frame_ms,
                loop=0,
                optimize=True,
            )
            orbit = {
                "path": str(args.capture_orbit.resolve()),
                "bytes": args.capture_orbit.stat().st_size,
                "view": args.orbit_view,
                "frames": len(quantized),
                "mode": args.orbit_mode,
                "camera_position_m": list(view["position"]),
                "look_at_m": list(view["look_at"]),
                "half_sweep_deg": args.orbit_degrees,
                "frame_duration_ms": args.orbit_frame_ms,
                "resolution": [args.orbit_width, args.orbit_height],
                "renderer": str(settings.get("/renderer/active")),
                "schematic_tube": bool(view["schematic"]),
                # The scene is static at t=0. Saying so in the artifact keeps the GIF from
                # being read as a mission playback, which it is not and cannot yet be.
                "motion": "camera_only_scene_static_at_t0",
                "luminance": _luminance_statistics(
                    np.asarray(frames[len(frames) // 2])
                ),
            }

        summary = {
            "schema": "vacuum_tube_production_scene_v1",
            "configuration": str(args.configuration.resolve()),
            "configuration_source_sha256": _sha256(args.configuration.resolve()),
            "configuration_resolved_sha256": loaded.resolved_sha256,
            "fixture": str(args.fixture.resolve()),
            "fixture_sha256": _sha256(args.fixture.resolve()),
            "candidate": plan.candidate,
            "coordinate_frame": plan.coordinate_frame,
            "reaction_evidence": plan.reaction_evidence,
            "backend": SimulationManager.get_active_physics_engine(),
            "device": str(SimulationManager.get_device()),
            "physics_dt_s": SimulationManager.get_physics_dt(),
            "solver_type": str(SimulationManager.get_solver_type()),
            "engine_switch_returned": bool(switched),
            "fixed_time_stepping": settings.get("/app/player/useFixedTimeStepping"),
            "rate_limit_enabled": settings.get("/app/runLoops/main/rateLimitEnabled"),
            "tube_length_m": layout.length_m,
            "tube_band_count": len(plan.tube_bands),
            "exit_track_length_m": config.tube.exit_brake_track_length_m,
            "cart_path": built.cart_path,
            "rocket_path": built.rocket_path,
            "coupling_path": built.coupling_path,
            "cradle_topology": "open_front_u",
            "rocket_shape": "cylinder_x",
            "mission_execution": "not_started_scene_construction_slice",
            "captures": captures,
            "orbit": orbit,
        }
        summary["passed"] = bool(
            summary["backend"] == "physx"
            and "cpu" in summary["device"].lower()
            and summary["fixed_time_stepping"] is True
            and summary["rate_limit_enabled"] is False
            and stage.GetPrimAtPath(built.cart_path).IsValid()
            and stage.GetPrimAtPath(built.rocket_path).IsValid()
            and stage.GetPrimAtPath(built.coupling_path).IsValid()
        )

        if args.save_usd is not None:
            args.save_usd.parent.mkdir(parents=True, exist_ok=True)
            stage.GetRootLayer().Export(str(args.save_usd.resolve()))
            summary["saved_usd"] = str(args.save_usd.resolve())
        rendered = json.dumps(summary, indent=2, sort_keys=True, allow_nan=False)
        if args.summary is not None:
            args.summary.parent.mkdir(parents=True, exist_ok=True)
            args.summary.write_text(rendered + "\n", encoding="utf-8")
        print(rendered)
        sys.stdout.flush()
        return 0 if summary["passed"] else 2
    except BaseException:
        # ``SimulationApp.close`` terminates the process, so an exception that escaped into
        # the ``finally`` below would surface as a silent exit code zero. Print it while the
        # interpreter is still alive. (The same trap bit run_mission.py first.)
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        return 3
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    raise SystemExit(main())
