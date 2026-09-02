# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Ordered, reversible cart-to-rocket release transaction.

The transaction implements the six observable steps in DESIGN_REVIEW section 10.2.  It
does not mutate a backend directly except for the explicit resynchronization boundary;
the joint change still travels through the typed effect path.  The collision pair is
present from initialization because Phase 0 rejected live activation on the target build.
"""

from __future__ import annotations

import math
from enum import Enum
from typing import Tuple

from ..effects.adapter import AppliedEffects, BackendAdapter
from ..effects.types import (
    ConstraintAction,
    ConstraintCommand,
    EffectBatch,
)
from ..events import (
    EVENT_ABORT,
    EVENT_BRAKE_ELIGIBLE,
    EVENT_RELEASE_CONFIRMED,
    EVENT_RELEASE_STEP,
    Event,
)
from ..linalg import norm, sub
from ..names import (
    BODY_CART,
    BODY_ROCKET,
    JOINT_COUPLING,
    PAIR_ROCKET_CRADLE,
    SLOT_COUPLING,
)
from ..state import Observation, SimulationState


class ReleasePhase(str, Enum):
    ATTACHED = "attached"
    LATENCY = "latency"
    COMMAND_READY = "command_ready"
    COMMAND_APPLIED = "command_applied"
    CONFIRMING = "confirming"
    RELEASED = "released"
    FAILED = "failed"


class ReleaseError(RuntimeError):
    """Raised when callers violate the transaction ordering."""


def _event(name: str, state: SimulationState, **data: object) -> Event:
    return Event(
        name=name,
        time_s=state.time_s,
        step_index=state.step_index,
        source=SLOT_COUPLING,
        data=data,
    )


def _body_state_distance(before: SimulationState, after: SimulationState, body_name: str) -> float:
    """Largest instantaneous pose/velocity discontinuity for one body."""
    first = before.body(body_name)
    second = after.body(body_name)
    return max(
        norm(sub(second.position, first.position)),
        norm(sub(second.linear_velocity, first.linear_velocity)),
        norm(sub(second.angular_velocity, first.angular_velocity)),
        math.sqrt(sum((a - b) ** 2 for a, b in zip(second.orientation, first.orientation))),
    )


class ReleaseTransaction:
    """Stateful implementation of the section 10.2 release transaction."""

    def __init__(
        self,
        *,
        command_latency_s: float,
        confirmation_steps: int,
        continuity_tolerance: float = 1e-9,
    ) -> None:
        if not math.isfinite(command_latency_s) or command_latency_s < 0.0:
            raise ValueError("release command latency must be finite and nonnegative")
        if (
            isinstance(confirmation_steps, bool)
            or not isinstance(confirmation_steps, int)
            or confirmation_steps <= 0
        ):
            raise ValueError("release confirmation steps must be a positive integer")
        if not math.isfinite(continuity_tolerance) or continuity_tolerance < 0.0:
            raise ValueError("release continuity tolerance must be finite and nonnegative")
        self._latency_s = command_latency_s
        self._confirmation_steps = confirmation_steps
        self._continuity_tolerance = continuity_tolerance
        self.reset()

    @property
    def phase(self) -> ReleasePhase:
        return self._phase

    @property
    def brake_eligible(self) -> bool:
        return self._phase is ReleasePhase.RELEASED

    @property
    def failed(self) -> bool:
        return self._phase is ReleasePhase.FAILED

    def reset(self) -> None:
        self._phase = ReleasePhase.ATTACHED
        self._request_time_s: float | None = None
        self._confirmation_count = 0

    def request(self, observation: Observation, *, aft_marker_outside: bool) -> Tuple[Event, ...]:
        """Perform step 1: verify the exit gate and begin the configured command latency."""
        if self._phase is not ReleasePhase.ATTACHED:
            return ()
        if not aft_marker_outside:
            raise ReleaseError("release requested before the rocket aft marker crossed the exit plane")
        self._request_time_s = observation.time_s
        self._phase = ReleasePhase.LATENCY
        return (
            Event(
                name=EVENT_RELEASE_STEP,
                time_s=observation.time_s,
                step_index=observation.step_index,
                source=SLOT_COUPLING,
                data={"transaction_step": 1, "action": "exit_verified"},
            ),
        )

    def pre_step(self, observation: Observation) -> tuple[EffectBatch, Tuple[Event, ...]]:
        if self._phase is ReleasePhase.LATENCY:
            assert self._request_time_s is not None
            if observation.time_s + 1e-12 >= self._request_time_s + self._latency_s:
                self._phase = ReleasePhase.COMMAND_READY
        if self._phase is not ReleasePhase.COMMAND_READY:
            return EffectBatch.empty(SLOT_COUPLING), ()
        self._phase = ReleasePhase.COMMAND_APPLIED
        return (
            EffectBatch(
                source=SLOT_COUPLING,
                constraint_commands=(
                    ConstraintCommand(
                        source=SLOT_COUPLING,
                        constraint=JOINT_COUPLING,
                        action=ConstraintAction.DISABLE,
                        bodies=(BODY_CART, BODY_ROCKET),
                    ),
                ),
            ),
            (
                Event(
                    name=EVENT_RELEASE_STEP,
                    time_s=observation.time_s,
                    step_index=observation.step_index,
                    source=SLOT_COUPLING,
                    data={"transaction_step": 2, "action": "joint_disable"},
                ),
                Event(
                    name=EVENT_RELEASE_STEP,
                    time_s=observation.time_s,
                    step_index=observation.step_index,
                    source=SLOT_COUPLING,
                    data={"transaction_step": 3, "action": "collision_pair_already_present"},
                ),
            ),
        )

    def resync_after_apply(
        self,
        adapter: BackendAdapter,
        applied: AppliedEffects,
        before: SimulationState,
    ) -> Tuple[Event, ...]:
        """Perform step 4 and reject any mutation-time state discontinuity."""
        if self._phase is not ReleasePhase.COMMAND_APPLIED:
            raise ReleaseError("release resync called before release commands were emitted")
        disabled = any(
            command.constraint == JOINT_COUPLING and command.action is ConstraintAction.DISABLE
            for command in applied.constraint_commands
        )
        pair_present = bool(before.collision_pair_active.get(PAIR_ROCKET_CRADLE, False))
        if not disabled or not pair_present or applied.collision_commands:
            self._phase = ReleasePhase.FAILED
            return (
                _event(
                    EVENT_ABORT,
                    before,
                    reason="release_commands_not_applied",
                    joint_disable_applied=disabled,
                    collision_pair_present=pair_present,
                    unexpected_collision_commands=bool(applied.collision_commands),
                ),
            )
        adapter.resync()
        synchronized = adapter.read_state()
        discontinuity = max(
            _body_state_distance(before, synchronized, BODY_CART),
            _body_state_distance(before, synchronized, BODY_ROCKET),
        )
        if discontinuity > self._continuity_tolerance:
            self._phase = ReleasePhase.FAILED
            return (
                _event(
                    EVENT_ABORT,
                    synchronized,
                    reason="release_state_discontinuity",
                    discontinuity=discontinuity,
                ),
            )
        self._phase = ReleasePhase.CONFIRMING
        return (
            _event(
                EVENT_RELEASE_STEP,
                synchronized,
                transaction_step=4,
                action="physics_resync",
            ),
        )

    def post_step(self, state: SimulationState) -> Tuple[Event, ...]:
        """Perform steps 5 and 6 after one or more integrated confirmation steps."""
        if self._phase is not ReleasePhase.CONFIRMING:
            return ()
        joint_inactive = not state.joint_active.get(JOINT_COUPLING, False)
        pair_active = bool(state.collision_pair_active.get(PAIR_ROCKET_CRADLE, False))
        if not joint_inactive or not pair_active:
            self._phase = ReleasePhase.FAILED
            return (
                _event(
                    EVENT_ABORT,
                    state,
                    reason="release_state_not_confirmed",
                    joint_inactive=joint_inactive,
                    collision_pair_active=pair_active,
                ),
            )
        self._confirmation_count += 1
        events = [
            _event(
                EVENT_RELEASE_STEP,
                state,
                transaction_step=5,
                action="post_step_confirmation",
                confirmation_count=self._confirmation_count,
            )
        ]
        if self._confirmation_count >= self._confirmation_steps:
            self._phase = ReleasePhase.RELEASED
            events.extend(
                (
                    _event(EVENT_RELEASE_CONFIRMED, state, confirmation_steps=self._confirmation_count),
                    _event(
                        EVENT_RELEASE_STEP,
                        state,
                        transaction_step=6,
                        action="brake_eligible",
                    ),
                    _event(EVENT_BRAKE_ELIGIBLE, state),
                )
            )
        return tuple(events)

    def snapshot(self) -> dict[str, object]:
        return {
            "phase": self._phase.value,
            "request_time_s": self._request_time_s,
            "confirmation_count": self._confirmation_count,
            "brake_eligible": self.brake_eligible,
        }
