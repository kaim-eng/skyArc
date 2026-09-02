# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Ideal analytic path guide with tangential resistance and clearance monitoring."""

from __future__ import annotations

import math
from typing import Any, Mapping

from ..components.contract import Component, ComponentDescriptor, Determinism, ScenarioContext, StepOutput
from ..effects.types import EffectBatch, Frame, Wrench
from ..events import EVENT_ABORT, Event
from ..linalg import dot, norm, scale, sub
from ..names import BODY_CART, JOINT_GUIDE, SLOT_GUIDE
from ..state import Observation, SimulationState, body_com_world
from .geometry import TubePath, guide_normal_bound_mps2, path_pose
from .launch_force import guide_resistance_force_n


class IdealPathGuide(Component):
    """Represent the pure-core straight or curved guide treatment.

    Normal motion is enforced by the backend.  This component owns only the guide's
    tangential resistance and its monitoring records; it never fabricates a second normal
    wrench on top of the constraint reaction.
    """

    def __init__(
        self,
        layout: TubePath,
        *,
        model_id: str,
        resistance_n: float,
        maximum_tracking_error_m: float,
        gravity_mps2: tuple[float, float, float] = (0.0, 0.0, -9.81),
        code_hash: str,
    ) -> None:
        if model_id not in {"ideal_prismatic_v1", "tangent_following_v1"}:
            raise ValueError(f"unsupported ideal guide model {model_id!r}")
        if not code_hash:
            raise ValueError("guide code hash may not be empty")
        if not math.isfinite(resistance_n) or resistance_n < 0.0:
            raise ValueError("guide resistance must be finite and nonnegative")
        if not math.isfinite(maximum_tracking_error_m) or maximum_tracking_error_m < 0.0:
            raise ValueError("guide tracking-error limit must be finite and nonnegative")
        self._layout = layout
        self._model_id = model_id
        self._resistance_n = resistance_n
        self._maximum_tracking_error_m = maximum_tracking_error_m
        self._gravity = gravity_mps2
        self._code_hash = code_hash
        self._peak_tracking_error_m = 0.0
        self._last_normal_load_mps2 = 0.0
        self._abort_emitted = False

    @property
    def descriptor(self) -> ComponentDescriptor:
        return ComponentDescriptor(
            slot=SLOT_GUIDE,
            model_id=self._model_id,
            model_version="1.0.0",
            parameter_schema_version="1",
            code_hash=self._code_hash,
            determinism=Determinism.DETERMINISTIC,
        )

    def prepare(self, context: ScenarioContext) -> None:
        pass

    def reset(self, initial_state: SimulationState) -> None:
        self._peak_tracking_error_m = 0.0
        self._last_normal_load_mps2 = 0.0
        self._abort_emitted = False

    def pre_step(self, observation: Observation) -> StepOutput:
        if not observation.state.joint_active.get(JOINT_GUIDE, True):
            return StepOutput.empty(SLOT_GUIDE)
        pose = path_pose(self._layout, observation.axial.s_cart_m)
        tracking_error = norm(sub(observation.cart.position, pose.position_m))
        self._peak_tracking_error_m = max(self._peak_tracking_error_m, tracking_error)
        speed = dot(observation.cart.linear_velocity, pose.tangent)
        self._last_normal_load_mps2 = guide_normal_bound_mps2(
            abs(speed),
            pose.signed_curvature_per_m,
            self._gravity,
            pose.normal,
        )
        resistance = guide_resistance_force_n(speed, self._resistance_n)
        wrench = Wrench(
            source=SLOT_GUIDE,
            body=BODY_CART,
            force_n=scale(pose.tangent, resistance),
            application_point_m=body_com_world(observation.cart),
            frame=Frame.WORLD,
        )
        events = ()
        if tracking_error > self._maximum_tracking_error_m and not self._abort_emitted:
            self._abort_emitted = True
            events = (
                Event(
                    name=EVENT_ABORT,
                    time_s=observation.time_s,
                    step_index=observation.step_index,
                    source=SLOT_GUIDE,
                    data={
                        "reason": "guide_tracking_error",
                        "tracking_error_m": tracking_error,
                        "limit_m": self._maximum_tracking_error_m,
                    },
                ),
            )
        return StepOutput(
            effects=EffectBatch(source=SLOT_GUIDE, wrenches=(wrench,)),
            events=events,
        )

    def post_step(self, state: SimulationState) -> StepOutput:
        return StepOutput.empty(SLOT_GUIDE)

    def snapshot_state(self) -> Mapping[str, Any]:
        return {
            "peak_tracking_error_m": self._peak_tracking_error_m,
            "last_normal_load_mps2": self._last_normal_load_mps2,
            "abort_emitted": self._abort_emitted,
        }
