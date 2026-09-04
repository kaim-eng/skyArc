"""Render a recorded mission as an animated chase-camera flythrough.

This is a *playback*, not a simulation. The mission already ran; every pose in every frame
is read out of the telemetry that run produced, so this script cannot invent motion the
physics did not produce, and it cannot disagree with the qualification artifacts.

That is also what makes it possible at all. Rigid bodies are simulated in the translated
accelerating frame of DESIGN_REVIEW section 7 while the visuals are authored in global
coordinates, so a *live* view shows the vehicle parked at the tube entrance for the whole
flight. Reconstructing the global pose after the fact -- ``x_global = x_solver + x_r(t)``,
the same identity run_mission.py asserts on reset -- sidesteps that entirely.

``SimulationApp`` is constructed before any extension-dependent import.
"""

import argparse
import csv
import json
import math
import os
import sys
import traceback
from pathlib import Path

# The physics timeline is never started here. Poses come from the record, so starting it
# would only let gravity fight the playback -- which is precisely how the first orbit
# render lost its subject between frames.


def _arguments() -> argparse.Namespace:
    project = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--configuration", type=Path, default=project / "configs" / "curved_2kms.yaml")
    parser.add_argument(
        "--fixture",
        type=Path,
        default=project / "configs" / "phase0_anti_tunneling_slab_cradle.json",
    )
    parser.add_argument(
        "--telemetry",
        type=Path,
        required=True,
        help="telemetry.csv from a completed mission, or the directory containing it.",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path)
    parser.add_argument(
        "--shot",
        choices=("chase", "separation"),
        default="chase",
        help=(
            "'chase' follows the assembly up the tube at vehicle scale. 'separation' pulls "
            "back to the whole exit region to show the cart braking to a stop on its track "
            "while the rocket departs -- a 46 km by 10 km box, at which scale the bodies "
            "are sub-pixel and are drawn as oversized markers with trails."
        ),
    )
    parser.add_argument("--sep-start-s", type=float, default=54.114)
    parser.add_argument("--sep-end-s", type=float, default=78.0)
    parser.add_argument("--sep-frames", type=int, default=64)
    parser.add_argument(
        "--tick-interval-s",
        type=float,
        default=2.0,
        help=(
            "Seconds between trail ticks. Equal-time marks are the honest way to show "
            "deceleration: the cart's bunch together as it slows, the rocket's stay evenly "
            "spaced, and neither depends on the reader trusting a caption."
        ),
    )
    parser.add_argument("--marker-radius-m", type=float, default=420.0)
    parser.add_argument("--width", type=int, default=800)
    parser.add_argument("--height", type=int, default=450)
    parser.add_argument("--frame-ms", type=int, default=80)
    parser.add_argument("--settle-frames", type=int, default=8)
    parser.add_argument(
        "--climb-frames",
        type=int,
        default=54,
        help="Frames spanning entrance to exit plane. One per second of flight by default.",
    )
    parser.add_argument(
        "--release-frames",
        type=int,
        default=26,
        help="Frames over the release and separation, played back roughly ten times slower "
        "than the climb because it is the part worth seeing.",
    )
    parser.add_argument(
        "--coast-end-s",
        type=float,
        default=56.0,
        help=(
            "Where to end. Measured separation runs 3.3 m at release, 15.9 m at t=55.0 s, "
            "121 m at 56.0 s, 357 m at 57.0 s and 724 m at 58.0 s, so the framing distance "
            "needed to hold both grows faster than the bodies stay legible. The shot ends "
            "while the divergence still reads as two vehicles rather than two pixels."
        ),
    )
    parser.add_argument(
        "--rail-drop-m",
        type=float,
        default=4.0,
        help="How far below the centerline to draw the guide rail, so it never occludes "
        "the vehicle it is carrying.",
    )
    parser.add_argument("--marker-spacing-m", type=float, default=1000.0)
    parser.add_argument("--key-intensity", type=float, default=3000.0)
    parser.add_argument("--dome-intensity", type=float, default=250.0)
    parser.add_argument("--no-overlay", action="store_true")
    return parser.parse_args()


TRAIL_WIDTH_M = 110.0
"""Thin on purpose. The trail shows the path; the equal-time ticks show the speed, and a
trail wide enough to be decorative swallows them -- at 320 m it hid both the ticks and the
brake track underneath."""

CHASE_TICK_RADIUS_M = 0.75
"""Small kilometre cue below the chase rail.

The original 9 m spheres could occupy most of the vehicle-scale frame whenever the camera
crossed one.  A 0.75 m radius remains legible across the 30--85 m chase distance without
reaching the vehicle, whose centreline is 4 m above the marker centre.
"""

COLUMNS = (
    "time_s",
    "post.cart.position_m.x",
    "post.cart.position_m.y",
    "post.cart.position_m.z",
    "post.cart.orientation.w",
    "post.cart.orientation.x",
    "post.cart.orientation.y",
    "post.cart.orientation.z",
    "post.rocket.position_m.x",
    "post.rocket.position_m.y",
    "post.rocket.position_m.z",
    "post.rocket.orientation.w",
    "post.rocket.orientation.x",
    "post.rocket.orientation.y",
    "post.rocket.orientation.z",
    "actual.rocket_axial_speed_mps",
    "observation.cart_s_m",
    "observation.coupled",
    "mission.phase",
)


def _frame_times(args) -> list[float]:
    """The shot: a long accelerating climb, then the release in near slow motion.

    Uniform sampling would spend 90% of the frames on a nearly featureless climb and blink
    past the release in one. The three segments are cut at the events the mission itself
    recorded rather than at round numbers.
    """
    if args.shot == "separation":
        # Uniform in time, deliberately. The whole point of the shot is that the two bodies
        # cover very different distances in equal intervals, and any non-uniform sampling
        # would hide exactly that.
        span = args.sep_end_s - args.sep_start_s
        return [
            args.sep_start_s + span * index / args.sep_frames
            for index in range(args.sep_frames + 1)
        ]
    exit_s = 54.114
    times = [exit_s * index / args.climb_frames for index in range(args.climb_frames)]
    times += [
        exit_s + (args.coast_end_s - exit_s) * index / args.release_frames
        for index in range(args.release_frames + 1)
    ]
    return times


def _load_samples(path: Path, wanted: list[float]) -> list[dict]:
    """One streaming pass, keeping the row nearest each requested time.

    The full-mission telemetry is 121 MB over 120,679 rows and about 200 columns. Holding
    it in memory to index it would be wasteful when fewer than a hundred rows are wanted.
    """
    if path.is_dir():
        candidates = sorted(path.rglob("telemetry.csv"))
        if not candidates:
            raise SystemExit(f"no telemetry.csv found under {path}")
        path = candidates[0]
    best: list[tuple[float, dict] | None] = [None] * len(wanted)
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        missing = [name for name in COLUMNS if name not in reader.fieldnames]
        if missing:
            raise SystemExit(f"telemetry is missing required columns: {missing}")
        for row in reader:
            time_s = float(row["time_s"])
            for index, target in enumerate(wanted):
                delta = abs(time_s - target)
                if best[index] is None or delta < best[index][0]:
                    best[index] = (delta, {name: row[name] for name in COLUMNS})
    if any(entry is None for entry in best):
        raise SystemExit("telemetry did not cover the requested time range")
    return [entry[1] for entry in best]


def _quat(row, body: str):
    return (
        float(row[f"post.{body}.orientation.w"]),
        float(row[f"post.{body}.orientation.x"]),
        float(row[f"post.{body}.orientation.y"]),
        float(row[f"post.{body}.orientation.z"]),
    )


def _world_position(row, body: str):
    """The recorded pose, which is already in global coordinates.

    The translated-frame resolution happens inside the backend adapter, below the recorder,
    so telemetry never contains solver coordinates. Confirmed against geometry: the cart at
    t=40.581 s reads (21981.3, 21004.3), and path_pose at its recorded arc length of
    30430.1 m returns (21981.2, 21004.3).
    """
    return tuple(float(row[f"post.{body}.position_m.{axis}"]) for axis in "xyz")


def _overlay(image, row, reference_speed: float, progress: float, width: int, height: int):
    """Burn the recorded state into the frame.

    A chase camera holds its subject at a fixed screen position, so acceleration is close
    to invisible without a reference. These numbers are read from the same telemetry row
    that placed the vehicle in this frame, so the caption cannot drift from the picture.
    """
    from PIL import ImageDraw, ImageFont

    def _font(size: int):
        for name in ("segoeui.ttf", "arial.ttf", "DejaVuSans.ttf"):
            try:
                return ImageFont.truetype(name, size)
            except OSError:
                continue
        return ImageFont.load_default()

    draw = ImageDraw.Draw(image)
    large = _font(max(14, height // 20))
    small = _font(max(11, height // 32))

    time_s = float(row["time_s"])
    speed = float(row["actual.rocket_axial_speed_mps"] or 0.0)
    altitude_km = _ALTITUDE_CACHE.get("value", 0.0) / 1000.0
    phase = row["mission.phase"]
    coupled = row["observation.coupled"] == "True"

    pad = max(8, width // 60)
    draw.text((pad, pad), f"t = {time_s:6.2f} s", font=large, fill=(255, 255, 255))
    draw.text(
        (pad, pad + large.size + 4),
        f"v = {speed:7.1f} m/s     h = {altitude_km:6.2f} km",
        font=small,
        fill=(220, 228, 235),
    )
    label = phase if coupled else f"{phase}  (released)"
    draw.text(
        (pad, pad + large.size + small.size + 10),
        label.replace("_", " "),
        font=small,
        fill=(255, 190, 90) if not coupled else (150, 200, 255),
    )

    # Progress along the flight, so a viewer can tell where in the mission a frame sits.
    bar_w = width - 2 * pad
    bar_y = height - pad - 6
    draw.rectangle([pad, bar_y, pad + bar_w, bar_y + 4], fill=(70, 74, 80))
    draw.rectangle(
        [pad, bar_y, pad + int(bar_w * max(0.0, min(1.0, progress))), bar_y + 4],
        fill=(90, 170, 255),
    )
    return image


_ALTITUDE_CACHE: dict[str, float] = {}


def _separation_overlay(image, row, cart, rocket, history, progress, width, height):
    """Caption the divergence with both speeds, differenced from the trail itself.

    Speeds come from consecutive recorded positions rather than from a telemetry speed
    column, because the cart's speed is the quantity on trial here and it should be the
    same number a reader could measure off the tick spacing in the picture.
    """
    import math as _math

    from PIL import Image, ImageDraw, ImageFont

    def _font(size: int):
        for name in ("segoeui.ttf", "arial.ttf", "DejaVuSans.ttf"):
            try:
                return ImageFont.truetype(name, size)
            except OSError:
                continue
        return ImageFont.load_default()

    large = _font(max(15, height // 18))
    small = _font(max(12, height // 28))
    pad = max(9, width // 70)

    # The sky is a flat mid grey and the trail colours are chosen to read against the
    # geometry, not against text. Without a backing panel the caption sits at almost the
    # same luminance as the background and is unreadable at GIF quantisation.
    panel = Image.new("RGBA", image.size, (0, 0, 0, 0))
    ImageDraw.Draw(panel).rectangle(
        [0, 0, width, pad * 2 + large.size + 3 * (small.size + 4)],
        fill=(8, 10, 14, 165),
    )
    image = Image.alpha_composite(image.convert("RGBA"), panel).convert("RGB")
    draw = ImageDraw.Draw(image)

    time_s = float(row["time_s"])
    speeds = {}
    for body, points in history.items():
        speeds[body] = (
            _math.dist(points[-1], points[-2]) / max(1e-9, _DT_CACHE.get("value", 1.0))
            if len(points) >= 2
            else 0.0
        )

    draw.text((pad, pad), f"t = {time_s:6.2f} s", font=large, fill=(255, 255, 255))
    y = pad + large.size + 6
    draw.text(
        (pad, y),
        f"cart    {speeds['cart']:7.0f} m/s   on the 25 km brake track",
        font=small,
        fill=(255, 150, 60),
    )
    draw.text(
        (pad, y + small.size + 4),
        f"rocket  {speeds['rocket']:7.0f} m/s   free flight",
        font=small,
        fill=(110, 190, 255),
    )
    draw.text(
        (pad, y + 2 * (small.size + 4)),
        f"apart   {_math.dist(cart, rocket)/1000:7.2f} km",
        font=small,
        fill=(215, 220, 228),
    )
    note = (
        "ticks every "
        f"{_TICK_CACHE.get('value', 2.0):g} s  -  markers and trails not to scale; "
        "the vehicles are metres across in a 54 km frame"
    )
    note_y = height - pad - small.size - 14
    draw.rectangle(
        [0, note_y - 5, width, height], fill=(8, 10, 14)
    )
    draw.text((pad, note_y), note, font=small, fill=(178, 183, 190))
    bar_w = width - 2 * pad
    bar_y = height - pad - 6
    draw.rectangle([pad, bar_y, pad + bar_w, bar_y + 4], fill=(70, 74, 80))
    draw.rectangle(
        [pad, bar_y, pad + int(bar_w * max(0.0, min(1.0, progress))), bar_y + 4],
        fill=(90, 170, 255),
    )
    return image


_DT_CACHE: dict[str, float] = {}
_TICK_CACHE: dict[str, float] = {}


def main() -> int:
    args = _arguments()
    isaac_path = os.environ.get("ISAAC_PATH")
    if not isaac_path:
        raise SystemExit("ISAAC_PATH is not set; launch through the target build's python.bat")
    project = Path(__file__).resolve().parents[1]
    release = Path(isaac_path).resolve()
    app = None
    try:
        times = _frame_times(args)
        samples = _load_samples(args.telemetry, times)
        print(f"loaded {len(samples)} telemetry samples", flush=True)

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
            experience=str(release / "apps" / "isaacsim.exp.full.kit"),
        )

        import omni.kit.app
        import omni.replicator.core as rep
        import isaacsim.core.experimental.utils.stage as stage_utils
        from PIL import Image
        from pxr import Gf, UsdGeom, UsdPhysics

        extension_root = project / "exts" / "skyarc"
        manager = omni.kit.app.get_app().get_extension_manager()
        manager.add_path(str(extension_root.parent.resolve()))
        if not manager.set_extension_enabled_immediate("skyarc", True):
            raise RuntimeError("failed to enable the local skyarc extension")
        if str(extension_root) not in sys.path:
            sys.path.insert(0, str(extension_root))

        from skyarc.configuration import load_yaml, resolve_tube_layout
        from skyarc.launcher.geometry import CurvedTubeLayout, path_pose
        from skyarc.launcher.path_controller import LaunchProfileReferenceFrame
        from skyarc.launcher.production import (
            build_production_scene_plan,
            load_production_fixture,
        )
        from skyarc.launcher.scene import SCHEMATIC_PATH, TRUE_SCALE_PATH, build_launcher_scene

        loaded = load_yaml(args.configuration)
        config = loaded.config
        layout = resolve_tube_layout(config)
        if not isinstance(layout, CurvedTubeLayout):
            raise RuntimeError("mission playback requires a schema-v3 curved layout")
        fixture = load_production_fixture(args.fixture)
        plan = build_production_scene_plan(config, layout, fixture)

        # Rebuilt exactly as ProductionMissionRuntime does, because the reconstruction
        # x_global = x_solver + x_r(t) is only correct against the frame the mission ran in.
        exit_track = config.tube.exit_track
        brake_distance_m = (
            exit_track.length_m
            if exit_track is not None
            else config.tube.exit_brake_track_length_m
        ) - config.cart.brake_stop_margin_m
        frame = LaunchProfileReferenceFrame(
            layout,
            target_exit_speed_mps=config.launch_control.target_exit_speed_mps,
            brake_distance_m=brake_distance_m,
        )

        # Guard the coordinate convention rather than trusting it. The reference frame is
        # the cart's intended path position, so a recorded cart pose must sit close to it
        # while still on the rail. If telemetry ever reverts to solver coordinates this gap
        # jumps to tens of kilometres, which is exactly the bug that produced a 42 km
        # altitude reading at t=40.6 s in the first cut of this script.
        probe = min(samples, key=lambda row: abs(float(row["time_s"]) - 40.0))
        probe_t = float(probe["time_s"])
        expected = frame.sample(probe_t).position_m
        actual = _world_position(probe, "cart")
        gap = math.dist(expected, actual)
        print(f"frame check at t={probe_t:.2f}s: |telemetry - reference| = {gap:.2f} m", flush=True)
        if gap > 500.0:
            raise RuntimeError(
                f"recorded cart pose sits {gap:.0f} m from the reference frame at "
                f"t={probe_t:.2f}s. Telemetry is expected in global coordinates; a gap this "
                "large means it is in solver coordinates and needs the frame offset added."
            )

        stage_utils.create_new_stage()
        stage = stage_utils.get_current_stage()
        built = build_launcher_scene(
            stage,
            config,
            plan,
            cart_position_m=(0.0, 0.0, 0.0),
            rocket_position_m=(0.0, 0.0, 0.0),
            orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
            cart_velocity_mps=(0.0, 0.0, 0.0),
            rocket_velocity_mps=(0.0, 0.0, 0.0),
            angular_velocity_radps=(0.0, 0.0, 0.0),
            author_visuals=True,
            key_intensity=args.key_intensity,
            dome_intensity=args.dome_intensity,
        )

        # Both authored tube scales are opaque solids seen from outside, so either one hides
        # the vehicle completely at chase range. The tube is replaced below by a thin rail
        # carrying the same per-stage colours, drawn clear of the vehicle.
        for scope in (TRUE_SCALE_PATH, SCHEMATIC_PATH):
            UsdGeom.Imageable(stage.GetPrimAtPath(scope)).MakeInvisible()

        rail_scope = "/World/Playback"
        UsdGeom.Xform.Define(stage, rail_scope)
        drop = Gf.Vec3f(0.0, 0.0, -float(args.rail_drop_m))
        for index, band in enumerate(plan.tube_bands):
            curve = UsdGeom.BasisCurves.Define(stage, f"{rail_scope}/Rail_{index}")
            curve.CreateTypeAttr("linear")
            curve.CreateBasisAttr("bezier")
            curve.CreateWrapAttr("nonperiodic")
            curve.CreateCurveVertexCountsAttr([len(band.points_m)])
            curve.CreatePointsAttr([Gf.Vec3f(*point) + drop for point in band.points_m])
            # Narrow on purpose. The rail runs toward the camera, so its nearest span is far
            # closer than the subject and a wide one looms over the frame.
            curve.CreateWidthsAttr([1.2] * len(band.points_m))
            curve.SetWidthsInterpolation("vertex")
            curve.CreateDisplayColorAttr([Gf.Vec3f(*band.color_rgb)])

        if args.shot == "chase":
            # The cart switches from the guided tube to the separate exit brake track at
            # release. The authored launcher scopes above are hidden, so the chase shot
            # must redraw this branch as well or the correctly tracked cart appears to
            # float off-rail in the final frames.
            # Render the straight brake branch as solid geometry. RTX turns a 25 km
            # BasisCurves hair into a broad noisy band at chase range even at its true
            # 0.2 m width; a cylinder stays a crisp sub-pixel-to-few-pixel rail.
            exit_pose = path_pose(layout, layout.length_m)
            rail_radius_m = 0.1
            # Body origins follow the mathematical centerline. Place the rail against
            # the slab underside along the exit path's local normal: half the 1.4 m cart
            # envelope plus the rail radius makes their surfaces meet.
            brake_offset = -(
                0.5 * plan.cradle.outer_height_m + rail_radius_m
            ) * Gf.Vec3d(*exit_pose.normal)
            start = Gf.Vec3d(*plan.exit_track_points_m[0]) + brake_offset
            end = Gf.Vec3d(*plan.exit_track_points_m[-1]) + brake_offset
            direction = end - start
            length = direction.GetLength()
            if length <= 0.0:
                raise RuntimeError("exit brake track must have positive length")
            brake_rail = UsdGeom.Cylinder.Define(stage, f"{rail_scope}/BrakeRail")
            brake_rail.CreateAxisAttr("X")
            brake_rail.CreateHeightAttr(length)
            brake_rail.CreateRadiusAttr(rail_radius_m)
            brake_xform = UsdGeom.Xformable(brake_rail.GetPrim())
            brake_xform.AddTranslateOp().Set(0.5 * (start + end))
            brake_xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), direction.GetNormalized()).GetQuat()
            )
            brake_rail.CreateDisplayColorAttr([Gf.Vec3f(0.32, 0.34, 0.38)])

        # Sleepers at a fixed spacing. A chase camera pins its subject in frame, so these
        # passing markers are most of what makes the acceleration legible.
        count = int(layout.length_m // args.marker_spacing_m)
        for index in range(1, count + 1):
            pose = path_pose(layout, index * args.marker_spacing_m)
            tick = UsdGeom.Sphere.Define(stage, f"{rail_scope}/Tick_{index}")
            tick.CreateRadiusAttr(CHASE_TICK_RADIUS_M)
            tick.AddTranslateOp().Set(Gf.Vec3d(*pose.position_m) + Gf.Vec3d(0.0, 0.0, -args.rail_drop_m))
            tick.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.85, 0.9)])

        body_ops = {}
        for name, path in (("cart", built.cart_path), ("rocket", built.rocket_path)):
            prim = stage.GetPrimAtPath(path)
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
            xformable = UsdGeom.Xformable(prim)
            xformable.ClearXformOpOrder()
            body_ops[name] = xformable.MakeMatrixXform()

        trails: dict[str, object] = {}
        markers: dict[str, object] = {}
        ticks: dict[str, list] = {"cart": [], "rocket": []}
        BODY_COLOUR = {"cart": (1.0, 0.55, 0.15), "rocket": (0.35, 0.75, 1.0)}
        if args.shot == "separation":
            # The vehicles are metres long inside a 46 km by 10 km frame, so at this scale
            # they are three orders of magnitude below a pixel. Hiding the real geometry and
            # drawing deliberately oversized markers is the honest choice: an invisible
            # to-scale body would just look like an empty frame, and a body scaled up
            # without saying so would misrepresent the vehicle. The caption says so.
            for path in (built.cart_path, built.rocket_path):
                UsdGeom.Imageable(stage.GetPrimAtPath(path)).MakeInvisible()
            for prim in stage.Traverse():
                name = prim.GetPath().pathString
                if name.startswith(f"{rail_scope}/Tick_") or name.startswith(f"{rail_scope}/Rail_"):
                    UsdGeom.Imageable(prim).MakeInvisible()

            # The exit brake track, drawn at its true 25 km length. This is the structure the
            # shot exists to show: it is 46% as long as the entire guided tube and does no
            # work on the payload.
            track = UsdGeom.BasisCurves.Define(stage, f"{rail_scope}/BrakeTrack")
            track.CreateTypeAttr("linear")
            track.CreateBasisAttr("bezier")
            track.CreateWrapAttr("nonperiodic")
            track.CreateCurveVertexCountsAttr([len(plan.exit_track_points_m)])
            track.CreatePointsAttr([Gf.Vec3f(*p) for p in plan.exit_track_points_m])
            track.CreateWidthsAttr([260.0] * len(plan.exit_track_points_m))
            track.SetWidthsInterpolation("vertex")
            track.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.57, 0.62)])

            # The last stretch of tube, so the exit reads as an exit rather than as the
            # arbitrary start of a line.
            tail = plan.tube_bands[-1].points_m
            approach = UsdGeom.BasisCurves.Define(stage, f"{rail_scope}/Approach")
            approach.CreateTypeAttr("linear")
            approach.CreateBasisAttr("bezier")
            approach.CreateWrapAttr("nonperiodic")
            approach.CreateCurveVertexCountsAttr([len(tail)])
            approach.CreatePointsAttr([Gf.Vec3f(*p) for p in tail])
            approach.CreateWidthsAttr([260.0] * len(tail))
            approach.SetWidthsInterpolation("vertex")
            approach.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.35, 0.2)])

            for body in ("cart", "rocket"):
                trail = UsdGeom.BasisCurves.Define(stage, f"{rail_scope}/{body}Trail")
                trail.CreateTypeAttr("linear")
                trail.CreateBasisAttr("bezier")
                trail.CreateWrapAttr("nonperiodic")
                trail.CreateDisplayColorAttr([Gf.Vec3f(*BODY_COLOUR[body])])
                # Seed with two real points from the record. A curve authored with a single
                # vertex is invalid geometry, and once Hydra has seen it in that state it
                # does not re-sync when later frames make it valid -- which is why the first
                # cut drew ticks with no line joining them.
                seed = [
                    Gf.Vec3f(*_world_position(samples[0], body)),
                    Gf.Vec3f(*_world_position(samples[min(1, len(samples) - 1)], body)),
                ]
                trail.CreateCurveVertexCountsAttr([len(seed)])
                trail.CreatePointsAttr(seed)
                trail.CreateWidthsAttr([TRAIL_WIDTH_M] * len(seed))
                trail.SetWidthsInterpolation("vertex")
                trails[body] = trail
                marker = UsdGeom.Sphere.Define(stage, f"{rail_scope}/{body}Marker")
                marker.CreateRadiusAttr(args.marker_radius_m)
                marker.CreateDisplayColorAttr([Gf.Vec3f(*BODY_COLOUR[body])])
                markers[body] = marker.AddTranslateOp()
                # One tick per interval, pre-authored and revealed as its moment arrives.
                span = args.sep_end_s - args.sep_start_s
                for slot in range(int(span / args.tick_interval_s) + 1):
                    tick = UsdGeom.Sphere.Define(
                        stage, f"{rail_scope}/{body}Tick_{slot}"
                    )
                    tick.CreateRadiusAttr(0.42 * args.marker_radius_m)
                    tick.CreateDisplayColorAttr([Gf.Vec3f(*BODY_COLOUR[body])])
                    op = tick.AddTranslateOp()
                    UsdGeom.Imageable(tick.GetPrim()).MakeInvisible()
                    ticks[body].append((tick, op))

        camera = UsdGeom.Camera.Define(stage, "/World/ChaseCamera")
        camera.CreateFocalLengthAttr(24.0)
        camera.CreateHorizontalApertureAttr(20.955)
        camera.CreateVerticalApertureAttr(20.955 * args.height / args.width)
        camera.CreateClippingRangeAttr((0.1, 5.0e5))
        camera_op = UsdGeom.Xformable(camera.GetPrim()).MakeMatrixXform()

        product = rep.create.render_product("/World/ChaseCamera", (args.width, args.height))
        annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach([product])

        target_speed = float(config.launch_control.target_exit_speed_mps)
        history: dict[str, list] = {"cart": [], "rocket": []}
        placed: dict[str, set] = {"cart": set(), "rocket": set()}
        separation_view = None
        if args.shot == "separation":
            # Frame on where the bodies actually go, read from the samples rather than
            # guessed: the cart runs 23 km down the track while the rocket departs 47.6 km,
            # so the box is about 46 km by 10 km and badly wrong if assumed.
            xs, zs = [], []
            for row in samples:
                for body in ("cart", "rocket"):
                    p = _world_position(row, body)
                    xs.append(p[0])
                    zs.append(p[2])
            pad = 2500.0
            low = (min(xs) - pad, min(zs) - pad)
            high = (max(xs) + pad, max(zs) + pad)
            centre_x = 0.5 * (low[0] + high[0])
            centre_z = 0.5 * (low[1] + high[1])
            half_h = math.radians(47.0) / 2.0
            half_v = math.atan(math.tan(half_h) * args.height / args.width)
            distance = max(
                0.5 * (high[0] - low[0]) / math.tan(half_h),
                0.5 * (high[1] - low[1]) / math.tan(half_v),
            )
            # Bias the aim upward so the subject sits below the caption panel, which covers
            # the top quarter of the frame and otherwise clips the departing rocket.
            aim_z = centre_z + 0.16 * (high[1] - low[1])
            eye = Gf.Vec3d(centre_x, -distance, aim_z)
            separation_view = (
                Gf.Matrix4d()
                .SetLookAt(eye, Gf.Vec3d(centre_x, 0.0, aim_z), Gf.Vec3d(0.0, 0.0, 1.0))
                .GetInverse()
            )
            print(
                f"separation framing: {(high[0]-low[0])/1000:.1f} km x "
                f"{(high[1]-low[1])/1000:.1f} km, camera {distance/1000:.1f} km back",
                flush=True,
            )
        if len(samples) >= 2:
            _DT_CACHE["value"] = float(samples[1]["time_s"]) - float(samples[0]["time_s"])
        _TICK_CACHE["value"] = args.tick_interval_s
        frames = []
        for index, row in enumerate(samples):
            time_s = float(row["time_s"])
            positions = {}
            for body in ("cart", "rocket"):
                # Already global. The backend adapter is constructed with the reference
                # frame and resolves x_solver -> x_global before the recorder ever sees a
                # sample, so adding the offset here double-counts it: at t=40.58 s that put
                # the vehicle at 42.0 km altitude instead of 21.0 km, off the path entirely
                # and 21 km from the rail it is supposed to be riding. Verified against
                # path_pose: telemetry (21981.3, 21004.3) == path_pose(30430.1).
                world = _world_position(row, body)
                positions[body] = world
                w, qx, qy, qz = _quat(row, body)
                matrix = Gf.Matrix4d(Gf.Rotation(Gf.Quatd(w, qx, qy, qz)), Gf.Vec3d(*world))
                body_ops[body].Set(matrix)

            cart, rocket = positions["cart"], positions["rocket"]
            _ALTITUDE_CACHE["value"] = rocket[2]
            separation = math.dist(cart, rocket)
            centre = tuple(0.5 * (cart[axis] + rocket[axis]) for axis in range(3))
            speed = float(row["actual.rocket_axial_speed_mps"] or 0.0)

            if args.shot == "separation":
                history["cart"].append(cart)
                history["rocket"].append(rocket)
                for body in ("cart", "rocket"):
                    markers[body].Set(Gf.Vec3d(*positions[body]))
                    points = history[body]
                    if len(points) >= 2:
                        # Create, not Get: the attributes do not exist until first authored,
                        # and Set on an invalid attribute fails silently -- which is why the
                        # first pass drew ticks with no trail joining them.
                        trails[body].CreateCurveVertexCountsAttr().Set([len(points)])
                        trails[body].CreatePointsAttr().Set([Gf.Vec3f(*p) for p in points])
                        trails[body].CreateWidthsAttr().Set([TRAIL_WIDTH_M] * len(points))
                        trails[body].SetWidthsInterpolation("vertex")
                    slot = int((time_s - args.sep_start_s) / args.tick_interval_s)
                    if 0 <= slot < len(ticks[body]) and slot not in placed[body]:
                        placed[body].add(slot)
                        tick, op = ticks[body][slot]
                        op.Set(Gf.Vec3d(*positions[body]))
                        UsdGeom.Imageable(tick.GetPrim()).MakeVisible()
                # Fixed camera on the whole episode's bounding box, computed once from the
                # telemetry. A camera that tracked anything would cancel the very motion the
                # shot is about.
                camera_op.Set(separation_view)
                for _ in range(max(1, args.settle_frames)):
                    rep.orchestrator.step(rt_subframes=1)
                import numpy as np

                array = np.asarray(annotator.get_data())
                if array.ndim == 1:
                    array = array.reshape(args.height, args.width, 4)
                image = Image.fromarray(array[:, :, :3].astype("uint8"))
                if not args.no_overlay:
                    image = _separation_overlay(
                        image, row, cart, rocket, history,
                        index / max(1, len(samples) - 1), args.width, args.height,
                    )
                frames.append(image)
                if index % 8 == 0:
                    print(f"  frame {index}/{len(samples)}  t={time_s:.2f}s  "
                          f"sep={separation/1000:.1f} km", flush=True)
                continue
            # Pull back as the vehicle speeds up, sub-linearly: matching distance to speed
            # exactly would cancel the apparent motion and hide the very acceleration this
            # shot exists to show. The cap is set by legibility -- the assembly is about
            # 10 m long, so past roughly 110 m it stops reading as a vehicle.
            distance = 30.0 + 55.0 * (max(0.0, speed) / target_speed) ** 0.55
            # Once the two bodies diverge, framing both is the whole point of the shot, so
            # separation takes over from speed as the binding constraint -- but only up to
            # the point where holding both makes each of them a single pixel.
            distance = min(max(distance, 2.4 * separation), 300.0)
            eye = Gf.Vec3d(centre[0], centre[1] - distance, centre[2] + 0.22 * distance)
            view = Gf.Matrix4d().SetLookAt(eye, Gf.Vec3d(*centre), Gf.Vec3d(0.0, 0.0, 1.0))
            camera_op.Set(view.GetInverse())

            for _ in range(max(1, args.settle_frames)):
                rep.orchestrator.step(rt_subframes=1)
            data = annotator.get_data()
            import numpy as np

            array = np.asarray(data)
            if array.ndim == 1:
                array = array.reshape(args.height, args.width, 4)
            image = Image.fromarray(array[:, :, :3].astype("uint8"))
            if not args.no_overlay:
                image = _overlay(
                    image, row, target_speed, index / max(1, len(samples) - 1),
                    args.width, args.height,
                )
            frames.append(image)
            if index % 10 == 0:
                print(f"  frame {index}/{len(samples)}  t={time_s:.2f}s  v={speed:.0f}", flush=True)

        annotator.detach()
        product.destroy()

        # Derive the shared palette from a montage spanning the shot, not from one frame.
        # The midpoint frame is mid-climb -- all grey and blue -- so a palette taken from it
        # alone has no entry near the amber used for the post-release caption, and that text
        # quantised to an unreadable grey.
        montage = Image.new("RGB", (args.width, args.height * 3))
        for slot, index in enumerate((0, len(frames) // 2, len(frames) - 1)):
            montage.paste(frames[index], (0, slot * args.height))
        base = montage.quantize(colors=192, method=Image.MEDIANCUT)
        quantized = [f.quantize(palette=base, dither=Image.NONE) for f in frames]
        args.output.parent.mkdir(parents=True, exist_ok=True)
        quantized[0].save(
            args.output,
            save_all=True,
            append_images=quantized[1:],
            duration=args.frame_ms,
            loop=0,
            optimize=True,
        )

        summary = {
            "schema": "skyarc_mission_playback_v1",
            "output": str(args.output.resolve()),
            "bytes": args.output.stat().st_size,
            "frames": len(quantized),
            "resolution": [args.width, args.height],
            "telemetry": str(args.telemetry.resolve()),
            "configuration": str(args.configuration.resolve()),
            "time_range_s": [float(samples[0]["time_s"]), float(samples[-1]["time_s"])],
            "motion": "replayed_from_recorded_telemetry",
            "reconstruction": "x_global = x_solver + x_reference(t)",
            "tube_hidden": True,
        }
        rendered = json.dumps(summary, indent=2, sort_keys=True)
        if args.summary is not None:
            args.summary.parent.mkdir(parents=True, exist_ok=True)
            args.summary.write_text(rendered + "\n", encoding="utf-8")
        print(rendered, flush=True)
        return 0
    except BaseException:
        # SimulationApp.close() terminates the process, so an exception escaping into the
        # finally below would surface as a silent exit code zero.
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        return 3
    finally:
        if app is not None:
            app.close()


if __name__ == "__main__":
    raise SystemExit(main())
