# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Global-frame visual proxies for translated-frame simulated bodies.

USD/Kit modules are resolved lazily so importing the backend-neutral package remains valid
outside a running application. Only the proxy xforms are written while physics runs; the
simulated-body transform-write invariant remains untouched.
"""

from __future__ import annotations

import importlib
import math
from typing import Any

from ..launcher.production import ProductionScenePlan
from ..names import BODY_CART, BODY_ROCKET
from ..state import SimulationState
from .cart_asset import author_cart_visual
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
        author_cart_visual(
            stage,
            f"{LIVE_VISUALS_PATH}/Cart",
            length_m=plan.cradle.outer_length_m,
            width_m=plan.cradle.outer_width_m,
            height_m=plan.cradle.slab_thickness_m,
            nose_length_m=plan.cradle.slab_nose_length_m,
            base_z_m=-0.5 * plan.cradle.outer_height_m,
        )
        rocket_radius_m = 0.5 * plan.rocket.diameter_m
        saddle_angle_rad = math.asin(
            plan.cradle.saddle_contact_offset_m / rocket_radius_m
        )
        pad_normal_offset_m = (
            0.5 * plan.cradle.saddle_pad_thickness_m + plan.initial_clearance_m
        )
        pad_center_z_m = -math.sqrt(
            rocket_radius_m**2 - plan.cradle.saddle_contact_offset_m**2
        ) - pad_normal_offset_m * math.cos(saddle_angle_rad)
        pad_center_y_m = (
            plan.cradle.saddle_contact_offset_m
            + pad_normal_offset_m * math.sin(saddle_angle_rad)
        )
        saddle_angle_deg = math.degrees(saddle_angle_rad)
        for station_index, station_x_m in enumerate(plan.cradle.saddle_stations_m):
            for side, sign in (("Left", -1.0), ("Right", 1.0)):
                pad = self._usd_geom.Cube.Define(
                    stage,
                    f"{LIVE_VISUALS_PATH}/Cart/Saddle{station_index:02d}{side}Pad",
                )
                pad.CreateSizeAttr(1.0)
                pad_xform = self._usd_geom.Xformable(pad.GetPrim())
                pad_xform.AddTranslateOp().Set(
                    self._gf.Vec3f(
                        station_x_m,
                        sign * pad_center_y_m,
                        pad_center_z_m,
                    )
                )
                pad_xform.AddRotateXYZOp().Set(
                    self._gf.Vec3f(sign * saddle_angle_deg, 0.0, 0.0)
                )
                pad_xform.AddScaleOp().Set(
                    self._gf.Vec3f(
                        plan.cradle.saddle_axial_length_m,
                        plan.cradle.saddle_pad_width_m,
                        plan.cradle.saddle_pad_thickness_m,
                    )
                )
                pad.CreateDisplayColorAttr([self._gf.Vec3f(0.22, 0.24, 0.28)])
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
