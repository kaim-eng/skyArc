# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Convert the NASA Explorer/Jupiter-C GLB into a Stage-1-free visual USD asset.

The downloaded GLB is never modified.  The script writes both a complete converted USD
and an independent, Z-up USD copy with the Redstone/Jupiter-C first stage removed.  The
remaining upper vehicle is centered horizontally, placed on a small visual-only aft
interface, and normalized so that the interface base is at Z=0.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import re
from pathlib import Path

STAGE1_NODE_NAMES = (
    "jupiterc10",
    "jupiterc_1",
    "jupiterc_2",
    "jupiterc_3",
    "jupiterc_4",
    "jupiterc_5",
    "jupiterc_6",
    "jupiterc_7",
    "jupiterc_8",
    "jupiterc_9",
    "jupiterc_g",
    "pCube3 jup",
    "pCylinder1",
)


def _normalized_name(value: str) -> str:
    return re.sub(r"[^a-z0-9]", "", value.lower())


def _default_or_first_root(stage):
    root = stage.GetDefaultPrim()
    if root and root.IsValid():
        return root
    children = list(stage.GetPseudoRoot().GetChildren())
    if len(children) != 1:
        raise RuntimeError(f"Expected one asset root, found {len(children)}: {[str(p.GetPath()) for p in children]}")
    stage.SetDefaultPrim(children[0])
    return children[0]


def _world_bounds(stage, root, Usd, UsdGeom):
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    aligned = cache.ComputeWorldBound(root).ComputeAlignedRange()
    minimum = aligned.GetMin()
    maximum = aligned.GetMax()
    values = [float(v) for v in (*minimum, *maximum)]
    if not all(math.isfinite(v) and abs(v) < 1.0e30 for v in values):
        raise RuntimeError(f"Invalid bounds for {root.GetPath()}: min={minimum}, max={maximum}")
    return minimum, maximum


def _find_stage1_prims(stage):
    target_keys = {_normalized_name(name): name for name in STAGE1_NODE_NAMES}
    candidates = {key: [] for key in target_keys}
    for prim in stage.Traverse():
        key = _normalized_name(prim.GetName())
        if key in candidates:
            candidates[key].append(prim)

    missing = [target_keys[key] for key, prims in candidates.items() if not prims]
    if missing:
        raise RuntimeError(f"Converted asset is missing expected Stage 1 nodes: {missing}")

    # Asset conversion can create a mesh child with the same name as its Xform.  Remove
    # only the shallowest matching prim for each original GLB node.
    selected = []
    for key in target_keys:
        prim = min(candidates[key], key=lambda item: item.GetPath().pathElementCount)
        selected.append(prim)

    selected.sort(key=lambda item: item.GetPath().pathElementCount)
    unique = []
    for prim in selected:
        path = prim.GetPath()
        if any(path.HasPrefix(parent.GetPath()) for parent in unique):
            continue
        unique.append(prim)
    return unique


def _create_preview_material(stage, path, color, metallic, roughness, Sdf, UsdShade):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _add_aft_interface(
    stage, root, center_x, center_y, connection_z, radius, Gf, Sdf, UsdGeom, UsdShade
):
    root_path = str(root.GetPath())
    looks_path = f"{root_path}/SkyArcLooks"
    interface_path = f"{root_path}/SkyArcAftInterface"
    UsdGeom.Scope.Define(stage, looks_path)
    interface = UsdGeom.Xform.Define(stage, interface_path)

    metal = _create_preview_material(
        stage,
        f"{looks_path}/AftMetal",
        Gf.Vec3f(0.18, 0.21, 0.25),
        0.78,
        0.26,
        Sdf,
        UsdShade,
    )
    thermal = _create_preview_material(
        stage,
        f"{looks_path}/ThermalSurface",
        Gf.Vec3f(0.035, 0.04, 0.05),
        0.25,
        0.42,
        Sdf,
        UsdShade,
    )

    interface_depth = max(0.12, min(0.20, radius * 0.35))
    ring = UsdGeom.Cylinder.Define(stage, f"{interface_path}/AttachmentRing")
    ring.CreateAxisAttr(UsdGeom.Tokens.z)
    ring.CreateHeightAttr(interface_depth)
    ring.CreateRadiusAttr(radius * 1.08)
    ring.AddTranslateOp().Set(
        Gf.Vec3d(center_x, center_y, connection_z - 0.5 * interface_depth)
    )
    ring.CreateDisplayColorAttr([Gf.Vec3f(0.18, 0.21, 0.25)])
    UsdShade.MaterialBindingAPI.Apply(ring.GetPrim()).Bind(metal)

    cap_height = max(0.018, interface_depth * 0.14)
    cap = UsdGeom.Cylinder.Define(stage, f"{interface_path}/ThermalCap")
    cap.CreateAxisAttr(UsdGeom.Tokens.z)
    cap.CreateHeightAttr(cap_height)
    cap.CreateRadiusAttr(radius * 0.92)
    cap.AddTranslateOp().Set(
        Gf.Vec3d(center_x, center_y, connection_z - interface_depth + 0.5 * cap_height)
    )
    cap.CreateDisplayColorAttr([Gf.Vec3f(0.035, 0.04, 0.05)])
    UsdShade.MaterialBindingAPI.Apply(cap.GetPrim()).Bind(thermal)

    interface.GetPrim().CreateAttribute("skyarc:purpose", Sdf.ValueTypeNames.String).Set(
        "visual-only upper-stage launcher interface"
    )
    return interface_depth


def _convert_glb(source_glb: Path, full_usd: Path, app) -> None:
    import omni.kit.asset_converter
    from isaacsim.core.experimental.utils.app import enable_extension

    enable_extension("omni.kit.asset_converter")
    for _ in range(5):
        app.update()

    context = omni.kit.asset_converter.AssetConverterContext()
    context.ignore_materials = False
    context.ignore_animations = True
    context.ignore_camera = True
    context.export_preview_surface = True
    context.use_meter_as_world_unit = True
    context.create_world_as_default_root_prim = True
    context.disabling_instancing = True
    context.convert_stage_up_z = True

    def progress_callback(progress: float, total_steps: int) -> None:
        if total_steps and (progress == 0 or progress == total_steps):
            print(f"Asset conversion progress: {progress}/{total_steps}")

    converter = omni.kit.asset_converter.get_instance()
    task = converter.create_converter_task(
        source_glb.resolve().as_posix(),
        full_usd.resolve().as_posix(),
        progress_callback,
        context,
    )
    future = asyncio.ensure_future(task.wait_until_finished())
    while not future.done():
        app.update()
    success = future.result()
    if not success:
        raise RuntimeError(f"Isaac Sim asset conversion failed: {source_glb}")


def _edit_converted_asset(source_glb: Path, output_dir: Path, app) -> dict:
    from pxr import Gf, Kind, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    full_usd = output_dir / "Explorer_JupiterC_Full.usdc"
    final_usd = output_dir / "Explorer_JupiterC_NoStage1.usdc"
    manifest_path = output_dir / "Explorer_JupiterC_NoStage1.manifest.json"

    source_layer = Sdf.Layer.FindOrOpen(full_usd.as_posix())
    if not source_layer:
        raise RuntimeError(f"Could not open converted USD layer: {full_usd}")
    source_layer.Reload(force=True)
    source_stage = Usd.Stage.Open(source_layer)
    if not source_stage:
        raise RuntimeError(f"Could not open converted USD: {full_usd}")
    source_root = _default_or_first_root(source_stage)

    # The converter connects texture RGB outputs (float3) directly to
    # UsdPreviewSurface diffuseColor, whose schema type is color3f. Re-author
    # those two inputs with the schema-correct type while preserving connections.
    for prim in list(source_stage.Traverse()):
        if not prim.IsA(UsdShade.Shader):
            continue
        shader = UsdShade.Shader(prim)
        diffuse = shader.GetInput("diffuseColor")
        if not diffuse or diffuse.GetTypeName() != Sdf.ValueTypeNames.Float3:
            continue
        diffuse_attr = diffuse.GetAttr()
        connections = diffuse_attr.GetConnections()
        if not prim.RemoveProperty(diffuse_attr.GetName()):
            raise RuntimeError(f"Could not normalize shader input: {diffuse_attr.GetPath()}")
        corrected = shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
        corrected.GetAttr().SetConnections(connections)
    source_stage.GetRootLayer().Save()

    # Build a small derived layer around the complete conversion.  Referencing the
    # source keeps the imported geometry/material data intact while allowing the
    # Stage-1 prims to be deactivated non-destructively in this layer.
    final_layer = Sdf.Layer.Find(final_usd.as_posix())
    if not final_layer:
        final_layer = Sdf.Layer.CreateNew(final_usd.as_posix())
    else:
        final_layer.Clear()
    stage = Usd.Stage.Open(final_layer)
    if not stage:
        raise RuntimeError(f"Could not create derived USD: {final_usd}")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    root = UsdGeom.Xform.Define(stage, "/JupiterCUpper").GetPrim()
    stage.SetDefaultPrim(root)
    Usd.ModelAPI(root).SetKind(Kind.Tokens.assembly)
    source = UsdGeom.Xform.Define(stage, "/JupiterCUpper/Source")
    source.GetPrim().GetReferences().AddReference(
        f"./{full_usd.name}", str(source_root.GetPath())
    )
    # glTF is Y-up. Rotate -90 degrees about X so the vehicle's original -Y
    # longitudinal axis becomes +Z in the simulation stage.
    source.AddRotateXOp(opSuffix="gltfYUpToZUp").Set(-90.0)

    stage1_prims = _find_stage1_prims(stage)
    removed_paths = [str(prim.GetPath()) for prim in stage1_prims]
    for path in removed_paths:
        # Deactivate from the derived root layer so the complete conversion remains
        # intact and the composed Stage-1-free asset excludes the prims.
        stage.OverridePrim(path).SetActive(False)
        if stage.GetPrimAtPath(path).IsActive():
            raise RuntimeError(f"Failed to deactivate Stage 1 prim: {path}")

    minimum, maximum = _world_bounds(stage, root, Usd, UsdGeom)
    center_x = 0.5 * (float(minimum[0]) + float(maximum[0]))
    center_y = 0.5 * (float(minimum[1]) + float(maximum[1]))
    connection_z = float(minimum[2])
    radius = 0.5 * max(float(maximum[0] - minimum[0]), float(maximum[1] - minimum[1]))
    interface_depth = _add_aft_interface(
        stage,
        root,
        center_x,
        center_y,
        connection_z,
        radius,
        Gf,
        Sdf,
        UsdGeom,
        UsdShade,
    )

    root_xform = UsdGeom.Xformable(root)
    offset = Gf.Vec3d(-center_x, -center_y, -(connection_z - interface_depth))
    root_xform.AddTranslateOp(
        UsdGeom.XformOp.PrecisionDouble, opSuffix="skyarcBaseOffset"
    ).Set(offset)

    root.CreateAttribute("skyarc:sourceAsset", Sdf.ValueTypeNames.String).Set(source_glb.name)
    root.CreateAttribute("skyarc:removedStage", Sdf.ValueTypeNames.String).Set("Jupiter-C Stage 1")
    root.CreateAttribute("skyarc:visualOnly", Sdf.ValueTypeNames.Bool).Set(True)

    rigid_bodies = []
    collisions = []
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_bodies.append(str(prim.GetPath()))
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collisions.append(str(prim.GetPath()))
    if rigid_bodies or collisions:
        raise RuntimeError(
            f"Visual asset unexpectedly contains physics APIs: rigid={rigid_bodies}, collision={collisions}"
        )

    # Deliver a self-contained composition instead of a runtime reference to the
    # intermediate conversion. Flattening also permanently omits inactive Stage 1.
    flattened = stage.Flatten()
    stage.GetRootLayer().TransferContent(flattened)
    stage = Usd.Stage.Open(stage.GetRootLayer())
    root = _default_or_first_root(stage)
    for prim in stage.Traverse():
        prim.SetSpecifier(Sdf.SpecifierDef)
        if not prim.IsA(UsdShade.Shader):
            continue
        file_input = UsdShade.Shader(prim).GetInput("file")
        if not file_input:
            continue
        asset_value = file_input.Get()
        if not isinstance(asset_value, Sdf.AssetPath) or not asset_value.path:
            continue
        texture_name = Path(asset_value.path).name
        file_input.Set(Sdf.AssetPath(f"./textures/{texture_name}"))
    stage.GetRootLayer().Save()
    app.update()
    final_minimum, final_maximum = _world_bounds(stage, root, Usd, UsdGeom)
    retained_meshes = [str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh)]
    manifest = {
        "source_glb": source_glb.name,
        "full_converted_usd": full_usd.name,
        "stage1_free_usd": final_usd.name,
        "source_url": "https://science.nasa.gov/3d-resources/explorer-jupiter-c-rocket/",
        "source_repository": "https://github.com/nasa/NASA-3D-Resources/tree/master/3D%20Models/Explorer%20Jupiter-C%20Rocket",
        "source_author": "NASA/Michael D. Carbajal",
        "source_license": "NASA Images and Media Usage Guidelines",
        "source_license_url": "https://www.nasa.gov/nasa-brand-center/images-and-media/",
        "source_usage_notes": (
            "Acknowledge NASA as the source and do not imply NASA endorsement. "
            "NASA identifiers remain protected."
        ),
        "redistribution_status": "cleared",
        "stage_up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(stage)),
        "removed_prim_paths": removed_paths,
        "retained_mesh_count": len(retained_meshes),
        "retained_mesh_paths": retained_meshes,
        "visual_only": True,
        "physics_rigid_body_count": 0,
        "physics_collision_count": 0,
        "bounds_m": {
            "minimum": [float(v) for v in final_minimum],
            "maximum": [float(v) for v in final_maximum],
        },
    }
    manifest_path.write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8", newline="\n"
    )
    return manifest


def _prepare(args, app) -> dict:
    source_glb = Path(args.source).resolve()
    conversion_source = (
        Path(args.conversion_source).resolve() if args.conversion_source else source_glb
    )
    output_dir = Path(args.output_dir).resolve()
    if not source_glb.is_file():
        raise FileNotFoundError(source_glb)
    if not conversion_source.is_file():
        raise FileNotFoundError(conversion_source)
    output_dir.mkdir(parents=True, exist_ok=True)

    full_usd = output_dir / "Explorer_JupiterC_Full.usdc"
    _convert_glb(conversion_source, full_usd, app)
    return _edit_converted_asset(source_glb, output_dir, app)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        required=True,
        help="Source NASA GLB file (left unchanged).",
    )
    parser.add_argument(
        "--conversion-source",
        help="Optional decompressed GLB used for conversion while --source remains provenance.",
    )
    parser.add_argument(
        "--output-dir",
        default=str(Path(__file__).resolve().parents[1] / "assets" / "vehicles" / "jupiter_c"),
        help="Directory for converted and Stage-1-free USD assets.",
    )
    return parser.parse_args()


def main() -> int:
    from isaacsim import SimulationApp

    args = _parse_args()
    app = SimulationApp({"headless": True, "width": 1280, "height": 720})
    try:
        manifest = _prepare(args, app)
        print(json.dumps(manifest, indent=2))
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
