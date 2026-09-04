# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Create and frame a lit validation stage for the Stage-1-free Jupiter-C asset."""

from pathlib import Path

import carb
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade

from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.experimental.utils.app import update_app_async


for required_name in ("asset_path", "preview_stage_path"):
    if required_name not in dir():
        raise ValueError(
            f"{required_name} must be injected by isaacsim_send.py --args-json"
        )

asset = Path(asset_path).resolve()
preview_stage = Path(preview_stage_path).resolve()
if not asset.is_file():
    raise FileNotFoundError(asset)

await stage_utils.create_new_stage_async(template="empty")
stage = stage_utils.get_current_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
stage.SetDefaultPrim(world)
rocket = UsdGeom.Xform.Define(stage, "/World/JupiterCUpper")
rocket.GetPrim().GetReferences().AddReference(asset.as_posix())

ground = UsdGeom.Cylinder.Define(stage, "/World/Ground")
ground.CreateAxisAttr(UsdGeom.Tokens.z)
ground.CreateHeightAttr(0.04)
ground.CreateRadiusAttr(3.5)
ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.02))

ground_material = UsdShade.Material.Define(stage, "/World/Looks/Ground")
ground_shader = UsdShade.Shader.Define(stage, "/World/Looks/Ground/PreviewSurface")
ground_shader.CreateIdAttr("UsdPreviewSurface")
ground_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
    Gf.Vec3f(0.055, 0.065, 0.085)
)
ground_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.36)
ground_material.CreateSurfaceOutput().ConnectToSource(
    ground_shader.ConnectableAPI(), "surface"
)
UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(ground_material)

dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome.CreateIntensityAttr(450.0)
dome.CreateColorAttr(Gf.Vec3f(0.38, 0.46, 0.62))
distant = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
distant.CreateIntensityAttr(1800.0)
distant.CreateAngleAttr(1.2)
distant.AddRotateXYZOp().Set(Gf.Vec3f(315.0, 0.0, 35.0))

settings = carb.settings.get_settings()
settings.set("/rtx/rendermode", "RayTracedLighting")
settings.set("/rtx/post/tonemap/op", 4)
settings.set("/rtx/post/tonemap/filmIso", 200.0)

preview_stage.parent.mkdir(parents=True, exist_ok=True)
if not stage.GetRootLayer().Export(preview_stage.as_posix()):
    raise RuntimeError(f"Could not export preview stage: {preview_stage}")

from isaacsim.core.rendering_manager import ViewportManager
from omni.kit.viewport.utility import get_active_viewport

camera_path = str(get_active_viewport().camera_path)
ViewportManager.set_camera_view(
    camera_path,
    eye=[4.0, 4.4, 2.7],
    target=[0.0, 0.0, 1.35],
)
await update_app_async(steps=60)
print(f"Preview stage: {preview_stage}")
print(f"Camera: {camera_path}")
