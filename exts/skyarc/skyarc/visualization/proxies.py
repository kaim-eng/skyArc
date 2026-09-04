# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Global-frame visual proxies for translated-frame simulated bodies.

USD/Kit modules are resolved lazily so importing the backend-neutral package remains valid
outside a running application. Only the proxy xforms are written while physics runs; the
simulated-body transform-write invariant remains untouched.
"""

from __future__ import annotations

import importlib
from typing import Any

from ..launcher.production import ProductionScenePlan
from ..names import BODY_CART, BODY_ROCKET
from ..state import SimulationState
from .rocket_asset import author_rocket_visual


LIVE_VISUALS_PATH = "/World/VacuumTubeLauncher/Visualization/Live"


class GlobalVisualProxies:
    def __init__(self, stage: Any, plan: ProductionScenePlan, *, simulated_paths: tuple[str, str]) -> None:
        pxr = importlib.import_module("pxr")
        self._gf = pxr.Gf
        self._usd_geom = pxr.UsdGeom
        self._stage = stage
        self._usd_geom.Xform.Define(stage, LIVE_VISUALS_PATH)
        self._ops: dict[str, tuple[Any, Any]] = {}

        cart = self._usd_geom.Xform.Define(stage, f"{LIVE_VISUALS_PATH}/Cart")
        cart_body = self._usd_geom.Cube.Define(stage, f"{LIVE_VISUALS_PATH}/Cart/Body")
        cart_body.CreateSizeAttr(1.0)
        cart_body.AddScaleOp().Set(
            self._gf.Vec3f(plan.cradle.outer_length_m, plan.cradle.outer_width_m, plan.cradle.outer_height_m)
        )
        cart_body.CreateDisplayColorAttr([self._gf.Vec3f(0.18, 0.2, 0.24)])
        rocket = self._usd_geom.Xform.Define(stage, f"{LIVE_VISUALS_PATH}/Rocket")
        author_rocket_visual(
            stage,
            f"{LIVE_VISUALS_PATH}/Rocket",
            length_m=plan.rocket.length_m,
            diameter_m=plan.rocket.diameter_m,
        )
        for name, xform in ((BODY_CART, cart), (BODY_ROCKET, rocket)):
            self._ops[name] = (
                xform.AddTranslateOp(),
                xform.AddOrientOp(precision=self._usd_geom.XformOp.PrecisionFloat),
            )
        for path in simulated_paths:
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid():
                self._usd_geom.Imageable(prim).MakeInvisible()

    def update(self, state: SimulationState) -> None:
        for name, (translate, orient) in self._ops.items():
            body = state.body(name)
            translate.Set(self._gf.Vec3d(*body.position))
            w, x, y, z = body.orientation
            orient.Set(self._gf.Quatf(float(w), self._gf.Vec3f(float(x), float(y), float(z))))


__all__ = ["GlobalVisualProxies", "LIVE_VISUALS_PATH"]
