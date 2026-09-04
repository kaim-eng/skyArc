# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Extract a visual-only tapered slab from the Crawler GLB in a running Isaac Sim.

Send this script through ``isaacsim_send.py``. ``conversion_source_path`` must be a
Draco-decompressed GLB; ``source_path`` remains the untouched provenance source.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import omni.kit.asset_converter
from isaacsim.core.experimental.utils.app import enable_extension, update_app_async
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


SOURCE_SUBSET = "initialShadingGroup2"
NOSE_LENGTH_FRACTION = 1.0 / 7.0  # 0.6 m nose in the 4.2 m production slab.

for required_name in ("source_path", "conversion_source_path", "output_dir"):
    if required_name not in dir():
        raise ValueError(
            f"{required_name} must be injected by isaacsim_send.py --args-json"
        )

SOURCE = Path(source_path).resolve()
CONVERSION_SOURCE = Path(conversion_source_path).resolve()
OUTPUT_DIR = Path(output_dir).resolve()
if not SOURCE.is_file():
    raise FileNotFoundError(SOURCE)
if not CONVERSION_SOURCE.is_file():
    raise FileNotFoundError(CONVERSION_SOURCE)
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def _coordinate_key(point) -> tuple[float, float, float]:
    return tuple(round(float(value), 6) for value in point)


def _largest_component_faces(mesh: UsdGeom.Mesh, subset: UsdGeom.Subset) -> list[int]:
    """Return the largest face component, joining split UV/normal vertices by position."""
    points = mesh.GetPointsAttr().Get() or []
    counts = mesh.GetFaceVertexCountsAttr().Get() or []
    indices = mesh.GetFaceVertexIndicesAttr().Get() or []
    offsets = []
    offset = 0
    for count in counts:
        offsets.append(offset)
        offset += count

    parent = {}

    def find(key):
        parent.setdefault(key, key)
        while parent[key] != key:
            parent[key] = parent[parent[key]]
            key = parent[key]
        return key

    def union(left, right):
        left = find(left)
        right = find(right)
        if left != right:
            parent[right] = left

    keyed_faces = []
    for face_id in subset.GetIndicesAttr().Get() or []:
        start = offsets[face_id]
        point_ids = tuple(indices[start : start + counts[face_id]])
        keys = tuple(_coordinate_key(points[point_id]) for point_id in point_ids)
        keyed_faces.append((face_id, keys))
        for key in keys[1:]:
            union(keys[0], key)

    components = {}
    for face_id, keys in keyed_faces:
        components.setdefault(find(keys[0]), []).append(face_id)
    if not components:
        raise RuntimeError(f"Source subset {subset.GetPath()} has no faces")
    return max(components.values(), key=len)


enable_extension("omni.kit.asset_converter")
await update_app_async(steps=5)

context = omni.kit.asset_converter.AssetConverterContext()
context.ignore_materials = False
context.ignore_animations = True
context.ignore_camera = True
context.export_preview_surface = True
context.use_meter_as_world_unit = True
context.create_world_as_default_root_prim = True
context.disabling_instancing = True
context.convert_stage_up_z = True

full_usd = OUTPUT_DIR / "Crawler_Full.usdc"
if full_usd.exists():
    full_usd.unlink()
task = omni.kit.asset_converter.get_instance().create_converter_task(
    CONVERSION_SOURCE.as_posix(), full_usd.as_posix(), None, context
)
if not await task.wait_until_finished():
    raise RuntimeError(f"Isaac Sim asset conversion failed: {SOURCE}")

source_stage = Usd.Stage.Open(full_usd.as_posix())
if source_stage is None:
    raise RuntimeError(f"Could not open converted asset: {full_usd}")
source_mesh_prim = next(
    (prim for prim in source_stage.Traverse() if prim.IsA(UsdGeom.Mesh)), None
)
if source_mesh_prim is None:
    raise RuntimeError("Converted Crawler asset contains no mesh")
source_mesh = UsdGeom.Mesh(source_mesh_prim)
source_subset_prim = source_mesh_prim.GetChild(SOURCE_SUBSET)
if not source_subset_prim or not source_subset_prim.IsA(UsdGeom.Subset):
    raise RuntimeError(f"Converted Crawler asset is missing subset {SOURCE_SUBSET}")
selected_faces = _largest_component_faces(
    source_mesh, UsdGeom.Subset(source_subset_prim)
)
if len(selected_faces) != 668:
    raise RuntimeError(
        f"Crawler slab extraction expected 668 source faces, found {len(selected_faces)}"
    )

source_points = source_mesh.GetPointsAttr().Get() or []
source_counts = source_mesh.GetFaceVertexCountsAttr().Get() or []
source_indices = source_mesh.GetFaceVertexIndicesAttr().Get() or []
source_offsets = []
offset = 0
for count in source_counts:
    source_offsets.append(offset)
    offset += count

source_to_output = {}
output_source_points = []
output_counts = []
output_indices = []
for face_id in selected_faces:
    start = source_offsets[face_id]
    face_source_ids = source_indices[start : start + source_counts[face_id]]
    output_counts.append(len(face_source_ids))
    for source_id in face_source_ids:
        if source_id not in source_to_output:
            source_to_output[source_id] = len(output_source_points)
            output_source_points.append(source_points[source_id])
        output_indices.append(source_to_output[source_id])

source_to_world = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(
    source_mesh_prim
)
# The converter retains the source's Y-up orientation. Rotate +90 degrees around X
# into the project's Z-up frame: (x, y, z) -> (x, -z, y).
z_up_points = []
for point in output_source_points:
    world = source_to_world.Transform(Gf.Vec3d(point))
    z_up_points.append(Gf.Vec3d(world[0], -world[2], world[1]))

minimum = Gf.Vec3d(
    *(min(point[axis] for point in z_up_points) for axis in range(3))
)
maximum = Gf.Vec3d(
    *(max(point[axis] for point in z_up_points) for axis in range(3))
)
span = maximum - minimum
if any(not math.isfinite(value) or value <= 0.0 for value in span):
    raise RuntimeError(f"Invalid extracted slab bounds: {minimum}, {maximum}")
center_x = 0.5 * (minimum[0] + maximum[0])
center_y = 0.5 * (minimum[1] + maximum[1])
half_length = 0.5 * span[0]
nose_length = span[0] * NOSE_LENGTH_FRACTION
shoulder_x = half_length - nose_length
middle_z = 0.5 * span[2]

output_points = []
for point in z_up_points:
    x = point[0] - center_x
    y = point[1] - center_y
    z = point[2] - minimum[2]
    if x > shoulder_x:
        taper = max(0.0, min(1.0, (half_length - x) / nose_length))
        z = middle_z + (z - middle_z) * taper
    output_points.append(Gf.Vec3f(x, y, z))

final_usd = OUTPUT_DIR / "Crawler_Slab.usdc"
if final_usd.exists():
    final_usd.unlink()
stage = Usd.Stage.CreateNew(final_usd.as_posix())
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
root = UsdGeom.Xform.Define(stage, "/CrawlerSlab").GetPrim()
stage.SetDefaultPrim(root)
root.CreateAttribute("skyarc:visualOnly", Sdf.ValueTypeNames.Bool).Set(True)
root.CreateAttribute("skyarc:sourceAsset", Sdf.ValueTypeNames.String).Set(SOURCE.name)
root.CreateAttribute("skyarc:extraction", Sdf.ValueTypeNames.String).Set(
    f"largest coordinate-connected component of {SOURCE_SUBSET}; forward seventh tapered"
)

mesh = UsdGeom.Mesh.Define(stage, "/CrawlerSlab/Deck")
mesh.CreatePointsAttr(output_points)
mesh.CreateFaceVertexCountsAttr(output_counts)
mesh.CreateFaceVertexIndicesAttr(output_indices)
mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
mesh.CreateDoubleSidedAttr(True)
mesh.CreateExtentAttr(
    [
        Gf.Vec3f(-half_length, -0.5 * span[1], 0.0),
        Gf.Vec3f(half_length, 0.5 * span[1], span[2]),
    ]
)
mesh.CreateDisplayColorAttr([Gf.Vec3f(0.24, 0.27, 0.31)])

UsdGeom.Scope.Define(stage, "/CrawlerSlab/Looks")
material = UsdShade.Material.Define(stage, "/CrawlerSlab/Looks/DeckMetal")
shader = UsdShade.Shader.Define(
    stage, "/CrawlerSlab/Looks/DeckMetal/PreviewSurface"
)
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
    Gf.Vec3f(0.24, 0.27, 0.31)
)
shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.68)
shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.27)
material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)

rigid = [
    str(prim.GetPath())
    for prim in stage.Traverse()
    if prim.HasAPI(UsdPhysics.RigidBodyAPI)
]
collision = [
    str(prim.GetPath())
    for prim in stage.Traverse()
    if prim.HasAPI(UsdPhysics.CollisionAPI)
]
if rigid or collision:
    raise RuntimeError(
        f"Visual asset unexpectedly has physics APIs: rigid={rigid}, collision={collision}"
    )
stage.GetRootLayer().Save()
await update_app_async(steps=2)

bounds = UsdGeom.BBoxCache(
    Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
).ComputeWorldBound(root).ComputeAlignedRange()
final_minimum = bounds.GetMin()
final_maximum = bounds.GetMax()
manifest = {
    "source_glb": SOURCE.name,
    "source_sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
    "source_asset_generator": "Khronos glTF Blender I/O v4.2.57",
    "source_url": "https://science.nasa.gov/3d-resources/crawler/",
    "source_repository": (
        "https://github.com/nasa/NASA-3D-Resources/tree/master/"
        "3D%20Models/Crawler"
    ),
    "source_author": "NASA/Michael D. Carbajal",
    "source_license": "NASA Images and Media Usage Guidelines",
    "source_license_url": (
        "https://www.nasa.gov/nasa-brand-center/images-and-media/"
    ),
    "source_usage_notes": (
        "Acknowledge NASA as the source and do not imply NASA endorsement. "
        "NASA identifiers remain protected."
    ),
    "redistribution_status": "cleared",
    "derived_usd": final_usd.name,
    "extraction_subset": SOURCE_SUBSET,
    "extraction_component_rule": "largest coordinate-connected component",
    "source_face_count": len(selected_faces),
    "source_point_count": len(output_source_points),
    "forward_taper_fraction": NOSE_LENGTH_FRACTION,
    "stage_up_axis": str(UsdGeom.GetStageUpAxis(stage)),
    "meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(stage)),
    "visual_only": True,
    "physics_rigid_body_count": 0,
    "physics_collision_count": 0,
    "bounds_m": {
        "minimum": [float(value) for value in final_minimum],
        "maximum": [float(value) for value in final_maximum],
    },
}
manifest_path = OUTPUT_DIR / "Crawler_Slab.manifest.json"
manifest_path.write_text(
    json.dumps(manifest, indent=2) + "\n", encoding="utf-8", newline="\n"
)
print(json.dumps(manifest, indent=2))
