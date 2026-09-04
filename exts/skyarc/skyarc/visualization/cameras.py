# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""The seven named Section 13.3 camera definitions in global SI coordinates."""

from __future__ import annotations

import importlib
import math
from dataclasses import dataclass
from typing import Mapping, Tuple

from ..launcher.geometry import TubePath, path_pose
from ..state import SimulationState


Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class CameraView:
    name: str
    position_m: Vec3
    look_at_m: Vec3
    up: Vec3 = (0.0, 0.0, 1.0)
    schematic: bool = False
    tracking_body: str | None = None


def camera_views(layout: TubePath) -> Mapping[str, CameraView]:
    entrance = path_pose(layout, 0.0)
    exit_pose = path_pose(layout, layout.length_m)
    origin = entrance.position_m
    exit_position = exit_pose.position_m
    midpoint = tuple(0.5 * (origin[index] + exit_position[index]) for index in range(3))
    span = math.sqrt(sum((exit_position[index] - origin[index]) ** 2 for index in range(3)))
    vehicle = tuple(origin[index] + 0.8 * entrance.tangent[index] for index in range(3))
    overhead_height = max(1000.0, 1.2 * span)
    return {
        "full_system_side": CameraView(
            "full_system_side",
            (midpoint[0], midpoint[1] - 1.4 * span, midpoint[2]),
            midpoint,
            schematic=True,
        ),
        "tube_cutaway": CameraView(
            "tube_cutaway",
            (vehicle[0], vehicle[1] - 14.0, vehicle[2] + 1.5),
            vehicle,
        ),
        "cart_rocket_chase": CameraView(
            "cart_rocket_chase", (-18.0, -8.0, 5.0), (0.0, 0.0, 0.0), tracking_body="cart"
        ),
        "cart_forward": CameraView(
            "cart_forward", (-1.5, 0.0, 1.0), (20.0, 0.0, 1.0), tracking_body="cart"
        ),
        "exit_separation": CameraView(
            "exit_separation",
            (exit_position[0], exit_position[1] - 80.0, exit_position[2] + 20.0),
            exit_position,
        ),
        "rocket_chase": CameraView(
            "rocket_chase", (-25.0, -10.0, 8.0), (0.0, 0.0, 0.0), tracking_body="rocket"
        ),
        "overhead_diagnostic": CameraView(
            "overhead_diagnostic",
            (midpoint[0], midpoint[1], midpoint[2] + overhead_height),
            midpoint,
            up=(0.0, 1.0, 0.0),
            schematic=True,
        ),
    }


def tracked_view(view: CameraView, state: SimulationState) -> CameraView:
    """Translate a body-relative view without changing its authored look direction."""
    if view.tracking_body is None:
        return view
    anchor = state.body(view.tracking_body).position
    return CameraView(
        name=view.name,
        position_m=tuple(anchor[index] + view.position_m[index] for index in range(3)),
        look_at_m=tuple(anchor[index] + view.look_at_m[index] for index in range(3)),
        up=view.up,
        schematic=view.schematic,
        tracking_body=view.tracking_body,
    )


class AuthoredCameraRig:
    """Author all views and update body-relative cameras from reconstructed state."""

    def __init__(self, stage: object, views: Mapping[str, CameraView]) -> None:
        pxr = importlib.import_module("pxr")
        self._gf = pxr.Gf
        self._usd_geom = pxr.UsdGeom
        self._views = dict(views)
        self._ops = {}
        self.paths = {}
        self._usd_geom.Xform.Define(stage, "/World/VacuumTubeLauncher/Visualization/Cameras")
        for name, view in self._views.items():
            path = f"/World/VacuumTubeLauncher/Visualization/Cameras/{name}"
            camera = self._usd_geom.Camera.Define(stage, path)
            camera.CreateFocalLengthAttr(24.0)
            camera.CreateClippingRangeAttr((0.1, 5.0e5))
            self._ops[name] = self._usd_geom.Xformable(camera.GetPrim()).MakeMatrixXform()
            self.paths[name] = path
            self._set(name, view)

    def _set(self, name: str, view: CameraView) -> None:
        matrix = self._gf.Matrix4d().SetLookAt(
            self._gf.Vec3d(*view.position_m),
            self._gf.Vec3d(*view.look_at_m),
            self._gf.Vec3d(*view.up),
        )
        self._ops[name].Set(matrix.GetInverse())

    def update(self, state: SimulationState) -> None:
        for name, view in self._views.items():
            self._set(name, tracked_view(view, state))


__all__ = ["AuthoredCameraRig", "CameraView", "camera_views", "tracked_view"]
