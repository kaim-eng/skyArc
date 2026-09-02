# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Baseline releasable fixed-joint coupling component."""

from __future__ import annotations

from typing import Any, Mapping, Tuple

from ..components.contract import Component, ComponentDescriptor, Determinism, ScenarioContext, StepOutput
from ..effects.adapter import AppliedEffects, BackendAdapter
from ..effects.types import EffectBatch
from ..events import Event
from ..names import JOINT_COUPLING, PAIR_ROCKET_CRADLE, SLOT_COUPLING
from ..state import Observation, SimulationState
from .release import ReleasePhase, ReleaseTransaction


class FixedJointCoupling(Component):
    """Own the reversible fixed-joint and cradle-pair release commands."""

    def __init__(
        self,
        *,
        command_latency_s: float,
        confirmation_steps: int,
        code_hash: str,
    ) -> None:
        if not code_hash:
            raise ValueError("coupling code hash may not be empty")
        self._code_hash = code_hash
        self._release = ReleaseTransaction(
            command_latency_s=command_latency_s,
            confirmation_steps=confirmation_steps,
        )

    @property
    def descriptor(self) -> ComponentDescriptor:
        return ComponentDescriptor(
            slot=SLOT_COUPLING,
            model_id="fixed_joint_v1",
            model_version="1.1.0",
            parameter_schema_version="1",
            code_hash=self._code_hash,
            determinism=Determinism.DETERMINISTIC,
            required_backend_capabilities=("resync", "always_present_collision_pair"),
        )

    @property
    def phase(self) -> ReleasePhase:
        return self._release.phase

    @property
    def brake_eligible(self) -> bool:
        return self._release.brake_eligible

    def prepare(self, context: ScenarioContext) -> None:
        if not context.backend_capabilities.get("resync", False):
            raise ValueError("fixed_joint_v1 requires an explicit backend resync capability")
        if not context.backend_capabilities.get("always_present_collision_pair", False):
            raise ValueError("fixed_joint_v1 requires an always-present collision pair")

    def reset(self, initial_state: SimulationState) -> None:
        if not initial_state.joint_active.get(JOINT_COUPLING, False):
            raise ValueError("fixed_joint_v1 reset requires the coupling joint to be active")
        if not initial_state.collision_pair_active.get(PAIR_ROCKET_CRADLE, False):
            raise ValueError("fixed_joint_v1 reset requires the cradle collision pair to be present")
        self._release.reset()

    def request_release(
        self,
        observation: Observation,
        *,
        aft_marker_outside: bool,
    ) -> Tuple[Event, ...]:
        return self._release.request(observation, aft_marker_outside=aft_marker_outside)

    def pre_step(self, observation: Observation) -> StepOutput:
        effects, events = self._release.pre_step(observation)
        return StepOutput(effects=effects, events=events)

    def resync_after_apply(
        self,
        adapter: BackendAdapter,
        applied: AppliedEffects,
        before: SimulationState,
    ) -> Tuple[Event, ...]:
        return self._release.resync_after_apply(adapter, applied, before)

    def post_step(self, state: SimulationState) -> StepOutput:
        return StepOutput(
            effects=EffectBatch.empty(SLOT_COUPLING),
            events=self._release.post_step(state),
        )

    def snapshot_state(self) -> Mapping[str, Any]:
        return self._release.snapshot()
