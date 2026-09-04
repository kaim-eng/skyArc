# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Live explanatory force-vector overlays, authored as non-physical USD curves."""

from __future__ import annotations

import importlib
import math
from typing import Any, Mapping, Tuple


Vec3 = Tuple[float, float, float]
OVERLAYS_PATH = "/World/VacuumTubeLauncher/Visualization/Overlays"
FORCE_COLORS: Mapping[str, Vec3] = {
    "launch": (0.1, 0.9, 1.0),
    "gravity": (0.6, 0.4, 1.0),
    "drag": (1.0, 0.35, 0.15),
    "brake": (1.0, 0.85, 0.1),
    "thrust": (0.2, 1.0, 0.25),
}


class ForceOverlay:
    def __init__(self, stage: Any, *, metres_per_newton: float = 1e-4) -> None:
        if not math.isfinite(metres_per_newton) or metres_per_newton <= 0.0:
            raise ValueError("force overlay scale must be finite and positive")
        pxr = importlib.import_module("pxr")
        self._gf = pxr.Gf
        self._usd_geom = pxr.UsdGeom
        self._stage = stage
        self._scale = metres_per_newton
        self._curves: dict[str, Any] = {}
        self._usd_geom.Xform.Define(stage, OVERLAYS_PATH)
        for name, color in FORCE_COLORS.items():
            curve = self._usd_geom.BasisCurves.Define(stage, f"{OVERLAYS_PATH}/{name.title()}")
            curve.CreateTypeAttr("linear")
            curve.CreateWrapAttr("nonperiodic")
            curve.CreateCurveVertexCountsAttr([2])
            curve.CreatePointsAttr([self._gf.Vec3f(0.0), self._gf.Vec3f(0.0)])
            curve.CreateWidthsAttr([0.08, 0.08])
            curve.CreateDisplayColorAttr([self._gf.Vec3f(*color)])
            self._curves[name] = curve

    def update(self, vectors: Mapping[str, tuple[Vec3, Vec3]]) -> None:
        for name, curve in self._curves.items():
            origin_m, force = vectors.get(
                name, ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            )
            end = tuple(origin_m[index] + self._scale * force[index] for index in range(3))
            curve.GetPointsAttr().Set([self._gf.Vec3f(*origin_m), self._gf.Vec3f(*end)])

    def set_visible(self, visible: bool) -> None:
        imageable = self._usd_geom.Imageable(self._stage.GetPrimAtPath(OVERLAYS_PATH))
        imageable.MakeVisible() if visible else imageable.MakeInvisible()


class MagneticFieldOverlay:
    """Explanatory axial field arrow; it is not a solved electromagnetic field."""

    def __init__(self, stage: Any, *, length_m: float = 8.0) -> None:
        if not math.isfinite(length_m) or length_m <= 0.0:
            raise ValueError("field-arrow length must be finite and positive")
        pxr = importlib.import_module("pxr")
        self._gf = pxr.Gf
        self._usd_geom = pxr.UsdGeom
        self._stage = stage
        self._length = length_m
        self._path = f"{OVERLAYS_PATH}/AxialMagneticField"
        curve = self._usd_geom.BasisCurves.Define(stage, self._path)
        curve.CreateTypeAttr("linear")
        curve.CreateWrapAttr("nonperiodic")
        curve.CreateCurveVertexCountsAttr([2])
        curve.CreatePointsAttr([self._gf.Vec3f(0.0), self._gf.Vec3f(0.0)])
        curve.CreateWidthsAttr([0.12, 0.12])
        curve.CreateDisplayColorAttr([self._gf.Vec3f(0.1, 0.75, 1.0)])
        self._curve = curve

    def update(self, origin_m: Vec3, tangent: Vec3) -> None:
        end = tuple(origin_m[index] + self._length * tangent[index] for index in range(3))
        self._curve.GetPointsAttr().Set([self._gf.Vec3f(*origin_m), self._gf.Vec3f(*end)])

    def set_visible(self, visible: bool) -> None:
        imageable = self._usd_geom.Imageable(self._stage.GetPrimAtPath(self._path))
        imageable.MakeVisible() if visible else imageable.MakeInvisible()


__all__ = ["FORCE_COLORS", "ForceOverlay", "MagneticFieldOverlay", "OVERLAYS_PATH"]
