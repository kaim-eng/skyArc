# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Visual-only Jupiter-C upper-stack asset resolution and USD authoring.

The imported mesh never owns collision or mass.  It is fitted to the configured conservative
cylinder envelope while the existing X-axis cylinder remains the authoritative physics shape.
"""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import json
import math
from pathlib import Path
from typing import Any


ASSET_DIRECTORY = Path("vehicles") / "jupiter_c"
ASSET_FILENAME = "Explorer_JupiterC_NoStage1.usdc"
MANIFEST_FILENAME = "Explorer_JupiterC_NoStage1.manifest.json"


@dataclass(frozen=True)
class RocketVisualAsset:
    usd_path: Path
    manifest_path: Path
    native_length_m: float
    native_diameter_m: float
    axial_scale: float
    radial_scale: float
    redistribution_status: str


def _asset_roots() -> tuple[Path, ...]:
    extension_root = Path(__file__).resolve().parents[2]
    return (
        extension_root / "data",
        extension_root.parent.parent / "assets",
    )


def resolve_rocket_visual_asset(
    *,
    target_length_m: float,
    target_diameter_m: float,
) -> RocketVisualAsset:
    """Resolve and validate the packaged/source asset, then compute its envelope fit."""
    if not math.isfinite(target_length_m) or target_length_m <= 0.0:
        raise ValueError("target rocket visual length must be finite and positive")
    if not math.isfinite(target_diameter_m) or target_diameter_m <= 0.0:
        raise ValueError("target rocket visual diameter must be finite and positive")

    searched: list[str] = []
    for root in _asset_roots():
        directory = root / ASSET_DIRECTORY
        usd_path = directory / ASSET_FILENAME
        manifest_path = directory / MANIFEST_FILENAME
        searched.extend((str(usd_path), str(manifest_path)))
        if usd_path.is_file() and manifest_path.is_file():
            break
    else:
        raise FileNotFoundError(
            "Jupiter-C visual asset or manifest is missing; searched " + ", ".join(searched)
        )

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("visual_only") is not True:
        raise ValueError("Jupiter-C asset manifest must declare visual_only=true")
    if manifest.get("physics_rigid_body_count") != 0 or manifest.get("physics_collision_count") != 0:
        raise ValueError("Jupiter-C visual asset must not contain rigid-body or collision APIs")
    if manifest.get("stage_up_axis") != "Z" or manifest.get("meters_per_unit") != 1.0:
        raise ValueError("Jupiter-C visual asset must be Z-up and authored in metres")
    redistribution_status = manifest.get("redistribution_status")
    if redistribution_status not in {
        "cleared",
        "blocked_pending_source_and_license",
    }:
        raise ValueError("Jupiter-C asset manifest has no recognized redistribution status")
    packaged_root = (_asset_roots()[0] / ASSET_DIRECTORY).resolve()
    if (
        redistribution_status != "cleared"
        and usd_path.resolve().is_relative_to(packaged_root)
    ):
        raise ValueError(
            "packaged Jupiter-C asset is not cleared for redistribution"
        )

    bounds = manifest.get("bounds_m")
    if not isinstance(bounds, dict):
        raise ValueError("Jupiter-C asset manifest is missing bounds_m")
    minimum = bounds.get("minimum")
    maximum = bounds.get("maximum")
    if not isinstance(minimum, list) or not isinstance(maximum, list) or len(minimum) != 3 or len(maximum) != 3:
        raise ValueError("Jupiter-C asset bounds must be three-dimensional")
    spans = tuple(float(maximum[index]) - float(minimum[index]) for index in range(3))
    if any(not math.isfinite(span) or span <= 0.0 for span in spans):
        raise ValueError("Jupiter-C asset bounds must be finite and positive")

    native_diameter_m = max(spans[0], spans[1])
    native_length_m = spans[2]
    return RocketVisualAsset(
        usd_path=usd_path.resolve(),
        manifest_path=manifest_path.resolve(),
        native_length_m=native_length_m,
        native_diameter_m=native_diameter_m,
        axial_scale=target_length_m / native_length_m,
        radial_scale=target_diameter_m / native_diameter_m,
        redistribution_status=redistribution_status,
    )


def author_rocket_visual(
    stage: Any,
    parent_path: str,
    *,
    length_m: float,
    diameter_m: float,
) -> tuple[str, RocketVisualAsset]:
    """Reference the mesh below ``parent_path`` and align its native +Z axis to +X."""
    pxr = importlib.import_module("pxr")
    gf = pxr.Gf
    usd = importlib.import_module("pxr.Usd")
    usd_geom = pxr.UsdGeom
    spec = resolve_rocket_visual_asset(
        target_length_m=length_m,
        target_diameter_m=diameter_m,
    )

    visual_path = f"{parent_path}/Visual"
    offset = usd_geom.Xform.Define(stage, visual_path)
    offset.AddTranslateOp().Set(gf.Vec3d(-0.5 * length_m, 0.0, 0.0))
    axis = usd_geom.Xform.Define(stage, f"{visual_path}/AxisZToX")
    axis.AddRotateYOp().Set(90.0)
    fitted = usd_geom.Xform.Define(stage, f"{visual_path}/AxisZToX/FittedEnvelope")
    fitted.AddScaleOp().Set(
        gf.Vec3f(spec.radial_scale, spec.radial_scale, spec.axial_scale)
    )
    model_path = f"{visual_path}/AxisZToX/FittedEnvelope/Model"
    model = usd_geom.Xform.Define(stage, model_path)
    model.GetPrim().GetReferences().AddReference(spec.usd_path.as_posix())
    physics_prims: list[str] = []
    for prim in usd.PrimRange(model.GetPrim()):
        applied = tuple(str(schema) for schema in prim.GetAppliedSchemas())
        properties = tuple(str(name) for name in prim.GetPropertyNames())
        if any(schema.startswith(("Physics", "Physx")) for schema in applied) or any(
            name.startswith(("physics:", "physx")) for name in properties
        ):
            physics_prims.append(str(prim.GetPath()))
    if physics_prims:
        raise ValueError(
            "Jupiter-C visual asset contains composed physics properties: "
            + ", ".join(physics_prims)
        )
    return model_path, spec


__all__ = [
    "ASSET_DIRECTORY",
    "ASSET_FILENAME",
    "MANIFEST_FILENAME",
    "RocketVisualAsset",
    "author_rocket_visual",
    "resolve_rocket_visual_asset",
]
