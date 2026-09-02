# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Finite mission state machine with concurrent post-detach branches."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from typing import Iterable, Tuple

from .events import (
    EVENT_ABORT,
    EVENT_CART_STOPPED,
    EVENT_FLIGHT_WINDOW_COMPLETE,
    EVENT_IGNITION,
    EVENT_RELEASE_CONFIRMED,
    EVENT_SEPARATION_CONFIRMED,
    EVENT_STAGE_TRANSITION,
    EVENT_STATE_TRANSITION,
    Event,
)


class MissionPhase(str, Enum):
    IDLE = "idle"
    ARMED = "armed"
    HOLD_RELEASED = "hold_released"
    LAUNCH_STAGE = "launch_stage"
    EXIT_APPROACH = "exit_approach"
    FORCE_RAMP_DOWN = "force_ramp_down"
    ROCKET_DETACH = "rocket_detach"
    POST_DETACH = "post_detach"
    COMPLETE = "complete"
    ABORT = "abort"


class CartBranch(str, Enum):
    INACTIVE = "inactive"
    BRAKING = "braking"
    STOPPED = "stopped"


class RocketBranch(str, Enum):
    INACTIVE = "inactive"
    SEPARATION = "separation_confirmation"
    IGNITION_PENDING = "rocket_ignition"
    POWERED_FLIGHT = "powered_flight"
    WINDOW_COMPLETE = "flight_window_complete"


@dataclass(frozen=True)
class MissionState:
    phase: MissionPhase = MissionPhase.IDLE
    stage_index: int = -1
    stage_name: str = ""
    cart_branch: CartBranch = CartBranch.INACTIVE
    rocket_branch: RocketBranch = RocketBranch.INACTIVE
    abort_reason: str | None = None


class MissionStateMachine:
    """Keep one stable launch-stage type while joining two concurrent completion branches."""

    def __init__(self) -> None:
        self._state = MissionState()

    @property
    def state(self) -> MissionState:
        return self._state

    @property
    def abort_active(self) -> bool:
        return self._state.phase is MissionPhase.ABORT

    def reset(self) -> None:
        self._state = MissionState()

    def _transition(
        self,
        phase: MissionPhase,
        *,
        time_s: float,
        step_index: int,
        **changes: object,
    ) -> Event:
        before = self._state.phase
        self._state = replace(self._state, phase=phase, **changes)
        return Event(
            name=EVENT_STATE_TRANSITION,
            time_s=time_s,
            step_index=step_index,
            source="state_machine",
            data={"from": before.value, "to": phase.value},
        )

    def arm(self, *, time_s: float, step_index: int) -> Event:
        if self._state.phase is not MissionPhase.IDLE:
            raise RuntimeError("only an idle mission can be armed")
        return self._transition(MissionPhase.ARMED, time_s=time_s, step_index=step_index)

    def release_hold(self, *, time_s: float, step_index: int) -> Event:
        if self._state.phase is not MissionPhase.ARMED:
            raise RuntimeError("hold release requires an armed mission")
        return self._transition(MissionPhase.HOLD_RELEASED, time_s=time_s, step_index=step_index)

    def start_launch(self, stage_index: int, stage_name: str, *, time_s: float, step_index: int) -> Event:
        if self._state.phase is not MissionPhase.HOLD_RELEASED:
            raise RuntimeError("launch start requires the hold-released state")
        if stage_index < 0 or not stage_name:
            raise ValueError("launch stage index/name must identify a configured stage")
        return self._transition(
            MissionPhase.LAUNCH_STAGE,
            time_s=time_s,
            step_index=step_index,
            stage_index=stage_index,
            stage_name=stage_name,
        )

    def stage_transition(
        self,
        stage_index: int,
        stage_name: str,
        *,
        time_s: float,
        step_index: int,
    ) -> Event:
        if self._state.phase is not MissionPhase.LAUNCH_STAGE:
            raise RuntimeError("stage transition requires LAUNCH_STAGE")
        previous = self._state.stage_index
        self._state = replace(self._state, stage_index=stage_index, stage_name=stage_name)
        return Event(
            name=EVENT_STAGE_TRANSITION,
            time_s=time_s,
            step_index=step_index,
            source="state_machine",
            data={"from_stage_index": previous, "to_stage_index": stage_index, "stage_name": stage_name},
        )

    def exit_approach(self, *, time_s: float, step_index: int) -> Event:
        if self._state.phase not in (MissionPhase.LAUNCH_STAGE, MissionPhase.EXIT_APPROACH):
            raise RuntimeError("exit approach requires the guided launch branch")
        if self._state.phase is MissionPhase.EXIT_APPROACH:
            raise RuntimeError("exit approach was already entered")
        return self._transition(MissionPhase.EXIT_APPROACH, time_s=time_s, step_index=step_index)

    def force_ramp_down(self, *, time_s: float, step_index: int) -> Event:
        if self._state.phase is not MissionPhase.EXIT_APPROACH:
            raise RuntimeError("force ramp-down requires exit approach")
        return self._transition(MissionPhase.FORCE_RAMP_DOWN, time_s=time_s, step_index=step_index)

    def rocket_detach(self, *, time_s: float, step_index: int) -> Event:
        if self._state.phase not in (MissionPhase.EXIT_APPROACH, MissionPhase.FORCE_RAMP_DOWN):
            raise RuntimeError("rocket detach requires exit approach/ramp-down")
        return self._transition(MissionPhase.ROCKET_DETACH, time_s=time_s, step_index=step_index)

    def abort(self, reason: str, *, time_s: float, step_index: int) -> Tuple[Event, ...]:
        if self.abort_active:
            return ()
        transition = self._transition(
            MissionPhase.ABORT,
            time_s=time_s,
            step_index=step_index,
            abort_reason=reason,
        )
        return (
            transition,
            Event(
                name=EVENT_ABORT,
                time_s=time_s,
                step_index=step_index,
                source="state_machine",
                data={"reason": reason},
            ),
        )

    def _join_if_complete(self, event: Event) -> Tuple[Event, ...]:
        if (
            self._state.cart_branch is CartBranch.STOPPED
            and self._state.rocket_branch is RocketBranch.WINDOW_COMPLETE
        ):
            return (
                self._transition(
                    MissionPhase.COMPLETE,
                    time_s=event.time_s,
                    step_index=event.step_index,
                ),
            )
        return ()

    def process(self, events: Iterable[Event]) -> Tuple[Event, ...]:
        """Advance branch state from component events; abort always dominates."""
        generated = []
        for event in events:
            if self.abort_active:
                break
            if event.name == EVENT_ABORT:
                generated.extend(
                    self.abort(
                        str(event.data.get("reason", "component_abort")),
                        time_s=event.time_s,
                        step_index=event.step_index,
                    )
                )
            elif event.name == EVENT_RELEASE_CONFIRMED:
                if self._state.phase is not MissionPhase.ROCKET_DETACH:
                    generated.extend(
                        self.abort(
                            "invalid_event_order:release_confirmed",
                            time_s=event.time_s,
                            step_index=event.step_index,
                        )
                    )
                    break
                generated.append(
                    self._transition(
                        MissionPhase.POST_DETACH,
                        time_s=event.time_s,
                        step_index=event.step_index,
                        cart_branch=CartBranch.BRAKING,
                        rocket_branch=RocketBranch.SEPARATION,
                    )
                )
            elif event.name == EVENT_SEPARATION_CONFIRMED:
                if (
                    self._state.phase is not MissionPhase.POST_DETACH
                    or self._state.rocket_branch is not RocketBranch.SEPARATION
                ):
                    generated.extend(
                        self.abort(
                            "invalid_event_order:separation_confirmed",
                            time_s=event.time_s,
                            step_index=event.step_index,
                        )
                    )
                    break
                self._state = replace(self._state, rocket_branch=RocketBranch.IGNITION_PENDING)
            elif event.name == EVENT_IGNITION:
                if (
                    self._state.phase is not MissionPhase.POST_DETACH
                    or self._state.rocket_branch is not RocketBranch.IGNITION_PENDING
                ):
                    generated.extend(
                        self.abort(
                            "invalid_event_order:ignition",
                            time_s=event.time_s,
                            step_index=event.step_index,
                        )
                    )
                    break
                self._state = replace(self._state, rocket_branch=RocketBranch.POWERED_FLIGHT)
            elif event.name == EVENT_CART_STOPPED:
                if (
                    self._state.phase is not MissionPhase.POST_DETACH
                    or self._state.cart_branch is not CartBranch.BRAKING
                ):
                    generated.extend(
                        self.abort(
                            "invalid_event_order:cart_stopped",
                            time_s=event.time_s,
                            step_index=event.step_index,
                        )
                    )
                    break
                self._state = replace(self._state, cart_branch=CartBranch.STOPPED)
                generated.extend(self._join_if_complete(event))
            elif event.name == EVENT_FLIGHT_WINDOW_COMPLETE:
                if (
                    self._state.phase is not MissionPhase.POST_DETACH
                    or self._state.rocket_branch
                    not in (RocketBranch.IGNITION_PENDING, RocketBranch.POWERED_FLIGHT)
                ):
                    generated.extend(
                        self.abort(
                            "invalid_event_order:flight_window_complete",
                            time_s=event.time_s,
                            step_index=event.step_index,
                        )
                    )
                    break
                self._state = replace(self._state, rocket_branch=RocketBranch.WINDOW_COMPLETE)
                generated.extend(self._join_if_complete(event))
        return tuple(generated)

    def snapshot_state(self) -> dict[str, object]:
        return {
            "phase": self._state.phase.value,
            "stage_index": self._state.stage_index,
            "stage_name": self._state.stage_name,
            "cart_branch": self._state.cart_branch.value,
            "rocket_branch": self._state.rocket_branch.value,
            "abort_reason": self._state.abort_reason,
        }
