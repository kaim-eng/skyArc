# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Visual-only Crawler-derived slab resolution and USD authoring."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import json
import math
from pathlib import Path
from typing import Any


ASSET_DIRECTORY = Path("vehicles") / "crawler_slab"
ASSET_FILENAME = "Crawler_Slab.usdc"
MANIFEST_FILENAME = "Crawler_Slab.manifest.json"


@dataclass(frozen=True)
class CartVisualAsset:
    usd_path: Path
    manifest_path: Path
    native_length_m: float
    native_width_m: float
    native_height_m: float
    scale_xyz: tuple[float, float, float]
    redistribution_status: str


def _asset_roots() -> tuple[Path, ...]:
    extension_root = Path(__file__).resolve().parents[2]
    return (
        extension_root / "data",
        extension_root.parent.parent / "assets",
    )


def resolve_cart_visual_asset(
    *,
    target_length_m: float,
    target_width_m: float,
    target_height_m: float,
    target_nose_length_m: float,
) -> CartVisualAsset:
    """Resolve the source-tree asset and calculate its production-envelope fit."""
    targets = (
        target_length_m,
        target_width_m,
        target_height_m,
        target_nose_length_m,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in targets):
        raise ValueError("cart visual target dimensions must be finite and positive")
    if target_nose_length_m >= target_length_m:
        raise ValueError("cart visual nose must be shorter than the slab")

    searched = []
    for root in _asset_roots():
        directory = root / ASSET_DIRECTORY
        usd_path = directory / ASSET_FILENAME
        manifest_path = directory / MANIFEST_FILENAME
        searched.extend((str(usd_path), str(manifest_path)))
        if usd_path.is_file() and manifest_path.is_file():
            break
    else:
        raise FileNotFoundError(
            "Crawler slab visual asset or manifest is missing; searched "
            + ", ".join(searched)
        )

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("visual_only") is not True:
        raise ValueError("Crawler slab manifest must declare visual_only=true")
    if (
        manifest.get("physics_rigid_body_count") != 0
        or manifest.get("physics_collision_count") != 0
    ):
        raise ValueError("Crawler slab visual must not contain physics APIs")
    if manifest.get("stage_up_axis") != "Z" or manifest.get("meters_per_unit") != 1.0:
        raise ValueError("Crawler slab visual must be Z-up and authored in metres")
    redistribution_status = manifest.get("redistribution_status")
    if redistribution_status not in {
        "cleared",
        "blocked_pending_source_and_license",
    }:
        raise ValueError("Crawler slab manifest has no recognized redistribution status")
    packaged_root = (_asset_roots()[0] / ASSET_DIRECTORY).resolve()
    if (
        redistribution_status != "cleared"
        and usd_path.resolve().is_relative_to(packaged_root)
    ):
        raise ValueError("uncleared Crawler slab visual may not be packaged")

    bounds = manifest.get("bounds_m")
    if not isinstance(bounds, dict):
        raise ValueError("Crawler slab manifest is missing bounds_m")
    minimum = bounds.get("minimum")
    maximum = bounds.get("maximum")
    if (
        not isinstance(minimum, list)
        or not isinstance(maximum, list)
        or len(minimum) != 3
        or len(maximum) != 3
    ):
        raise ValueError("Crawler slab bounds must be three-dimensional")
    spans = tuple(float(maximum[index]) - float(minimum[index]) for index in range(3))
    if any(not math.isfinite(span) or span <= 0.0 for span in spans):
        raise ValueError("Crawler slab bounds must be finite and positive")
    taper_fraction = float(manifest.get("forward_taper_fraction", float("nan")))
    target_taper_fraction = target_nose_length_m / target_length_m
    if not math.isclose(
        taper_fraction, target_taper_fraction, rel_tol=0.0, abs_tol=1e-12
    ):
        raise ValueError("Crawler slab taper does not match the production slab")

    return CartVisualAsset(
        usd_path=usd_path.resolve(),
        manifest_path=manifest_path.resolve(),
        native_length_m=spans[0],
        native_width_m=spans[1],
        native_height_m=spans[2],
        scale_xyz=(
            target_length_m / spans[0],
            target_width_m / spans[1],
            target_height_m / spans[2],
        ),
        redistribution_status=redistribution_status,
    )


def author_cart_visual(
    stage: Any,
    parent_path: str,
    *,
    length_m: float,
    width_m: float,
    height_m: float,
    nose_length_m: float,
    base_z_m: float,
) -> tuple[str, CartVisualAsset]:
    """Reference and fit the extracted slab beneath the authoritative cart body."""
    pxr = importlib.import_module("pxr")
    gf = pxr.Gf
    usd = importlib.import_module("pxr.Usd")
    usd_geom = pxr.UsdGeom
    spec = resolve_cart_visual_asset(
        target_length_m=length_m,
        target_width_m=width_m,
        target_height_m=height_m,
        target_nose_length_m=nose_length_m,
    )

    visual_path = f"{parent_path}/SlabVisual"
    visual = usd_geom.Xform.Define(stage, visual_path)
    visual.AddTranslateOp().Set(gf.Vec3d(0.0, 0.0, base_z_m))
    fitted_path = f"{visual_path}/FittedEnvelope"
    fitted = usd_geom.Xform.Define(stage, fitted_path)
    fitted.AddScaleOp().Set(gf.Vec3f(*spec.scale_xyz))
    model_path = f"{fitted_path}/Model"
    model = usd_geom.Xform.Define(stage, model_path)
    model.GetPrim().GetReferences().AddReference(spec.usd_path.as_posix())

    physics_prims = []
    for prim in usd.PrimRange(model.GetPrim()):
        applied = tuple(str(schema) for schema in prim.GetAppliedSchemas())
        properties = tuple(str(name) for name in prim.GetPropertyNames())
        if any(schema.startswith(("Physics", "Physx")) for schema in applied) or any(
            name.startswith(("physics:", "physx")) for name in properties
        ):
            physics_prims.append(str(prim.GetPath()))
    if physics_prims:
        raise ValueError(
            "Crawler slab visual contains composed physics properties: "
            + ", ".join(physics_prims)
        )
    return model_path, spec


__all__ = [
    "ASSET_DIRECTORY",
    "ASSET_FILENAME",
    "MANIFEST_FILENAME",
    "CartVisualAsset",
    "author_cart_visual",
    "resolve_cart_visual_asset",
]
