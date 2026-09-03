# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Isaac Sim USD scene authoring for the approved production treatment.

Import this module only after ``SimulationApp`` exists.  The package root and all pure-core
modules intentionally avoid importing it.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

from isaacsim.core.experimental.prims import RigidPrim
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics

from ..configuration.schema import ScenarioConfig
from .production import ProductionScenePlan


ROOT_PATH = "/World/VacuumTubeLauncher"
CART_PATH = f"{ROOT_PATH}/Bodies/Cradle"
ROCKET_PATH = f"{ROOT_PATH}/Bodies/Rocket"
COUPLING_PATH = f"{ROOT_PATH}/Joints/CartRocketFixedJoint"

VISUALIZATION_PATH = f"{ROOT_PATH}/Visualization"
TRUE_SCALE_PATH = f"{VISUALIZATION_PATH}/TrueScale"
SCHEMATIC_PATH = f"{VISUALIZATION_PATH}/Schematic"

DEFAULT_KEY_INTENSITY = 900.0
DEFAULT_DOME_INTENSITY = 120.0
"""Lighting the captures are exposed for.

The first captured frames came out at 0.80-0.83 mean luminance -- not black, but at the
overexposed end of any sensible band, with the dome washing the background to near white.
Section 16.3 gates mean luminance inside a configured band precisely because both ends are
failures, so these are tuned against measured captures rather than chosen by eye.
"""

SCHEMATIC_WIDTH_FRACTION = 1.0 / 300.0
"""Schematic tube width as a fraction of path length.

A 2 m tube seen from 40 km subtends about 5e-5 rad, while one pixel of a 1280-wide frame
at a ~50 degree field of view is about 6.8e-4 rad: the true tube is roughly one fourteenth
of a pixel, so *no* amount of correct geometry makes it visible at system scale. The
schematic band is therefore deliberately non-physical, authored under its own
``Schematic`` scope and invisible by default, so a system-scale view can be legible
without any frame ever implying the tube is 180 m across.
"""


@dataclass(frozen=True)
class BuiltLauncherScene:
    plan: ProductionScenePlan
    cart_path: str
    rocket_path: str
    coupling_path: str
    cart: Any
    rocket: Any
    coupling_joint: Any


def _safe_name(value: str) -> str:
    converted = "".join(character if character.isalnum() else "_" for character in value)
    return converted or "unnamed"


def _author_curve(
    stage: Any,
    path: str,
    points_m: Sequence[tuple[float, float, float]],
    *,
    width_m: float,
    color_rgb: tuple[float, float, float],
    opacity: float,
) -> None:
    curve = UsdGeom.BasisCurves.Define(stage, path)
    curve.CreateTypeAttr("linear")
    curve.CreateBasisAttr("bezier")
    curve.CreateWrapAttr("nonperiodic")
    curve.CreateCurveVertexCountsAttr([len(points_m)])
    curve.CreatePointsAttr([Gf.Vec3f(*point) for point in points_m])
    curve.CreateWidthsAttr([float(width_m)] * len(points_m))
    curve.SetWidthsInterpolation("vertex")
    curve.CreateDisplayColorAttr([Gf.Vec3f(*color_rgb)])
    curve.CreateDisplayOpacityAttr([float(opacity)])


def _author_band_set(
    stage: Any,
    scope_path: str,
    plan: ProductionScenePlan,
    *,
    tube_width_m: float,
    marker_radius_m: float,
    exit_track_width_m: float,
) -> None:
    """Author one complete set of tube bands, exit track and marker at a given scale."""
    UsdGeom.Xform.Define(stage, scope_path)
    for band in plan.tube_bands:
        _author_curve(
            stage,
            f"{scope_path}/Tube_{_safe_name(band.name)}",
            band.points_m,
            width_m=tube_width_m,
            color_rgb=band.color_rgb,
            opacity=band.opacity,
        )
    _author_curve(
        stage,
        f"{scope_path}/ExitTrack",
        plan.exit_track_points_m,
        width_m=exit_track_width_m,
        color_rgb=(0.45, 0.45, 0.48),
        opacity=1.0,
    )
    marker = UsdGeom.Sphere.Define(stage, f"{scope_path}/ExitMarker")
    marker.CreateRadiusAttr(marker_radius_m)
    marker.AddTranslateOp().Set(Gf.Vec3d(*plan.exit_marker_position_m))
    marker.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.15, 0.05)])


def author_launcher_visuals(
    stage: Any,
    plan: ProductionScenePlan,
    *,
    key_intensity: float = DEFAULT_KEY_INTENSITY,
    dome_intensity: float = DEFAULT_DOME_INTENSITY,
    schematic_width_fraction: float = SCHEMATIC_WIDTH_FRACTION,
) -> None:
    """Author tube bands at both scales, the exit track, marker and explicit lighting.

    Two band sets are authored. ``TrueScale`` carries the real tube inner diameter and is
    what a vehicle-scale view shows. ``Schematic`` carries a deliberately exaggerated width
    and starts *invisible*, so nothing renders it unless a system-scale view explicitly
    asks for it; a frame is never silently misleading about the tube's size.
    """
    UsdGeom.Xform.Define(stage, ROOT_PATH)
    UsdGeom.Xform.Define(stage, VISUALIZATION_PATH)

    _author_band_set(
        stage,
        TRUE_SCALE_PATH,
        plan,
        tube_width_m=plan.tube_inner_diameter_m,
        marker_radius_m=2.0,
        exit_track_width_m=0.2,
    )

    path_length_m = max(
        (band.end_s_m for band in plan.tube_bands), default=plan.tube_inner_diameter_m
    )
    schematic_width_m = max(
        plan.tube_inner_diameter_m, path_length_m * schematic_width_fraction
    )
    _author_band_set(
        stage,
        SCHEMATIC_PATH,
        plan,
        tube_width_m=schematic_width_m,
        marker_radius_m=2.0 * schematic_width_m,
        exit_track_width_m=0.5 * schematic_width_m,
    )
    UsdGeom.Imageable(stage.GetPrimAtPath(SCHEMATIC_PATH)).MakeInvisible()

    UsdGeom.Xform.Define(stage, f"{ROOT_PATH}/Lighting")
    key = UsdLux.DistantLight.Define(stage, f"{ROOT_PATH}/Lighting/Key")
    key.CreateIntensityAttr(key_intensity)
    key.CreateAngleAttr(1.0)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 35.0, 0.0))
    fill = UsdLux.DomeLight.Define(stage, f"{ROOT_PATH}/Lighting/Fill")
    fill.CreateIntensityAttr(dome_intensity)


def _set_transform(
    xform: UsdGeom.Xform,
    position_m: Sequence[float],
    orientation_wxyz: Sequence[float],
) -> None:
    xform.AddTranslateOp().Set(Gf.Vec3d(*position_m))
    w, x, y, z = orientation_wxyz
    xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))
    )


def _collision_cube(
    stage: Any,
    path: str,
    *,
    size_m: tuple[float, float, float],
    center_m: tuple[float, float, float],
) -> None:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3f(*center_m))
    cube.AddScaleOp().Set(Gf.Vec3f(*size_m))
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.18, 0.2, 0.24)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def author_production_bodies(
    stage: Any,
    config: ScenarioConfig,
    plan: ProductionScenePlan,
    *,
    cart_position_m: Sequence[float],
    rocket_position_m: Sequence[float],
    orientation_wxyz: Sequence[float],
    cart_velocity_mps: Sequence[float],
    rocket_velocity_mps: Sequence[float],
    angular_velocity_radps: Sequence[float],
) -> BuiltLauncherScene:
    """Author the qualified open U cradle, X-axis rocket cylinder, and fixed coupling."""
    UsdGeom.Xform.Define(stage, f"{ROOT_PATH}/Bodies")
    UsdGeom.Xform.Define(stage, f"{ROOT_PATH}/Joints")

    cradle = plan.cradle
    cart_xform = UsdGeom.Xform.Define(stage, CART_PATH)
    _set_transform(cart_xform, cart_position_m, orientation_wxyz)
    cart_prim = cart_xform.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(cart_prim)
    UsdPhysics.MassAPI.Apply(cart_prim).CreateMassAttr(cradle.mass_kg)
    UsdPhysics.RigidBodyAPI(cart_prim).CreateVelocityAttr().Set(Gf.Vec3f(*cart_velocity_mps))
    UsdPhysics.RigidBodyAPI(cart_prim).CreateAngularVelocityAttr().Set(
        Gf.Vec3f(*(float(value) * 180.0 / 3.141592653589793 for value in angular_velocity_radps))
    )
    PhysxSchema.PhysxContactReportAPI.Apply(cart_prim).CreateThresholdAttr().Set(0.0)
    PhysxSchema.PhysxRigidBodyAPI.Apply(cart_prim).CreateEnableCCDAttr(False)

    rail_height = cradle.outer_height_m - cradle.floor_thickness_m
    rail_z = -0.5 * cradle.outer_height_m + cradle.floor_thickness_m + 0.5 * rail_height
    _collision_cube(
        stage,
        f"{CART_PATH}/Floor",
        size_m=(cradle.outer_length_m, cradle.outer_width_m, cradle.floor_thickness_m),
        center_m=(0.0, 0.0, -0.5 * cradle.outer_height_m + 0.5 * cradle.floor_thickness_m),
    )
    for side, sign in (("LeftRail", -1.0), ("RightRail", 1.0)):
        _collision_cube(
            stage,
            f"{CART_PATH}/{side}",
            size_m=(cradle.outer_length_m, cradle.wall_thickness_m, rail_height),
            center_m=(
                0.0,
                sign * (0.5 * cradle.outer_width_m - 0.5 * cradle.wall_thickness_m),
                rail_z,
            ),
        )
    _collision_cube(
        stage,
        f"{CART_PATH}/RearWall",
        size_m=(cradle.wall_thickness_m, cradle.outer_width_m, rail_height),
        center_m=(-0.5 * cradle.outer_length_m + 0.5 * cradle.wall_thickness_m, 0.0, rail_z),
    )

    rocket_geometry = plan.rocket
    rocket_xform = UsdGeom.Xform.Define(stage, ROCKET_PATH)
    _set_transform(rocket_xform, rocket_position_m, orientation_wxyz)
    rocket_prim = rocket_xform.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(rocket_prim)
    UsdPhysics.MassAPI.Apply(rocket_prim).CreateMassAttr(rocket_geometry.mass_kg)
    UsdPhysics.RigidBodyAPI(rocket_prim).CreateVelocityAttr().Set(Gf.Vec3f(*rocket_velocity_mps))
    UsdPhysics.RigidBodyAPI(rocket_prim).CreateAngularVelocityAttr().Set(
        Gf.Vec3f(*(float(value) * 180.0 / 3.141592653589793 for value in angular_velocity_radps))
    )
    PhysxSchema.PhysxContactReportAPI.Apply(rocket_prim).CreateThresholdAttr().Set(0.0)
    PhysxSchema.PhysxRigidBodyAPI.Apply(rocket_prim).CreateEnableCCDAttr(False)
    cylinder = UsdGeom.Cylinder.Define(stage, f"{ROCKET_PATH}/Collision")
    cylinder.CreateAxisAttr("X")
    cylinder.CreateHeightAttr(rocket_geometry.length_m)
    cylinder.CreateRadiusAttr(0.5 * rocket_geometry.diameter_m)
    cylinder.CreateDisplayColorAttr([Gf.Vec3f(0.7, 0.72, 0.76)])
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())

    joint = UsdPhysics.FixedJoint.Define(stage, COUPLING_PATH)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(CART_PATH)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(ROCKET_PATH)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*plan.cart_to_rocket_offset_cart_m))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateCollisionEnabledAttr().Set(True)
    joint.CreateJointEnabledAttr().Set(True)

    return BuiltLauncherScene(
        plan=plan,
        cart_path=CART_PATH,
        rocket_path=ROCKET_PATH,
        coupling_path=COUPLING_PATH,
        cart=RigidPrim(CART_PATH),
        rocket=RigidPrim(ROCKET_PATH),
        coupling_joint=joint,
    )


def build_launcher_scene(
    stage: Any,
    config: ScenarioConfig,
    plan: ProductionScenePlan,
    *,
    cart_position_m: Sequence[float],
    rocket_position_m: Sequence[float],
    orientation_wxyz: Sequence[float],
    cart_velocity_mps: Sequence[float],
    rocket_velocity_mps: Sequence[float],
    angular_velocity_radps: Sequence[float],
    author_visuals: bool = True,
    key_intensity: float = DEFAULT_KEY_INTENSITY,
    dome_intensity: float = DEFAULT_DOME_INTENSITY,
) -> BuiltLauncherScene:
    if author_visuals:
        author_launcher_visuals(
            stage, plan, key_intensity=key_intensity, dome_intensity=dome_intensity
        )
    else:
        UsdGeom.Xform.Define(stage, ROOT_PATH)
    return author_production_bodies(
        stage,
        config,
        plan,
        cart_position_m=cart_position_m,
        rocket_position_m=rocket_position_m,
        orientation_wxyz=orientation_wxyz,
        cart_velocity_mps=cart_velocity_mps,
        rocket_velocity_mps=rocket_velocity_mps,
        angular_velocity_radps=angular_velocity_radps,
    )


__all__ = [
    "BuiltLauncherScene",
    "CART_PATH",
    "COUPLING_PATH",
    "DEFAULT_DOME_INTENSITY",
    "DEFAULT_KEY_INTENSITY",
    "ROCKET_PATH",
    "ROOT_PATH",
    "SCHEMATIC_PATH",
    "SCHEMATIC_WIDTH_FRACTION",
    "TRUE_SCALE_PATH",
    "VISUALIZATION_PATH",
    "author_launcher_visuals",
    "author_production_bodies",
    "build_launcher_scene",
]
