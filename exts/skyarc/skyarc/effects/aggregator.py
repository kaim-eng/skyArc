# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The pure pre-step effect aggregator (section 9.3).

Each physical subsystem owns its own force calculation. The aggregator checks ownership,
resolves every wrench into the world frame about each body's centre of mass, and sums
compatible effects. It applies nothing: the backend adapter alone does that.

The per-slot decomposition is retained rather than discarded after summation. Section 16.2
needs the launcher, drag, brake, thrust and resistance work terms separately, and
reconstructing them from a summed force afterwards is not possible.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Dict, Iterable, Mapping, Tuple

from ..linalg import ZERO3, Vec3, add, cross, quat_rotate, scale, sub
from ..state import SimulationState, body_com_world
from .types import CollisionPairCommand, ConstraintCommand, EffectBatch, Frame, MassUpdate, Wrench
from .validation import EffectValidationError, validate_batch


@dataclass(frozen=True)
class BodyLoad:
    """Net world-frame load on one body, about its centre of mass.

    Attributes:
        body: Body identifier.
        force_n: Net force in the world frame, newtons.
        torque_nm: Net torque about the centre of mass in the world frame, newton-metres.
        force_by_slot: Per-slot world force contributions, retained for the energy identity
            and for the force-decomposition telemetry channels.
        torque_by_slot: Per-slot world torque contributions about the body centre of mass,
            retained so rotating-body work can be attributed to the originating slot.
    """

    body: str
    force_n: Vec3 = ZERO3
    torque_nm: Vec3 = ZERO3
    force_by_slot: Mapping[str, Vec3] = field(default_factory=dict)
    torque_by_slot: Mapping[str, Vec3] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "force_by_slot", MappingProxyType(dict(self.force_by_slot)))
        object.__setattr__(self, "torque_by_slot", MappingProxyType(dict(self.torque_by_slot)))


@dataclass(frozen=True)
class AggregatedEffects:
    """The accepted effects for one step, ready for the adapter.

    This is the "accepted component effects" record of section 14. What the backend reports
    having applied is recorded separately, because a difference between the two is exactly
    the adapter error the separation exists to expose.
    """

    loads: Mapping[str, BodyLoad] = field(default_factory=dict)
    mass_updates: Tuple[MassUpdate, ...] = ()
    constraint_commands: Tuple[ConstraintCommand, ...] = ()
    collision_commands: Tuple[CollisionPairCommand, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "loads", MappingProxyType(dict(self.loads)))

    def load(self, body: str) -> BodyLoad:
        """Net load on a body, or a zero load if nothing acted on it this step."""
        return self.loads.get(body, BodyLoad(body=body))

    def slot_force(self, body: str, slot: str) -> Vec3:
        """World force contributed by one slot to one body this step."""
        return self.load(body).force_by_slot.get(slot, ZERO3)


def resolve_wrench(wrench: Wrench, state: SimulationState) -> Tuple[Vec3, Vec3]:
    """Resolve a wrench into a world force and a world torque about the body's centre of mass.

    ``Frame.BODY`` quantities are rotated by the body orientation and the application point
    is treated as a body-frame offset; ``Frame.WORLD`` quantities are used as given and the
    application point is a world position. This is the single place where that distinction
    is interpreted.
    """
    body = state.body(wrench.body)
    com = body_com_world(body)
    if wrench.frame is Frame.BODY:
        force = quat_rotate(body.orientation, wrench.force_n)
        torque = quat_rotate(body.orientation, wrench.torque_nm)
        application = add(body.position, quat_rotate(body.orientation, wrench.application_point_m))
    else:
        force = wrench.force_n
        torque = wrench.torque_nm
        application = wrench.application_point_m
    lever = sub(application, com)
    return force, add(torque, cross(lever, force))


def aggregate(
    batches: Iterable[EffectBatch],
    state: SimulationState,
    *,
    validate: bool = True,
) -> AggregatedEffects:
    """Check ownership, resolve frames, and sum compatible effects.

    Args:
        batches: One batch per component, in the pinned registration order. The order does
            not change the sum, but it does determine which conflicting command is reported
            first, so it is kept deterministic.
        state: The state the effects were computed against, used to resolve body frames and
            centres of mass.
        validate: Whether to run :func:`.validate_batch` first. Only a caller that has
            already validated should disable it.

    Returns:
        The accepted effects for this step.

    Raises:
        EffectValidationError: On any contract violation, on two mass updates for the same
            body in one step, or on two conflicting commands for the same constraint or pair.
    """
    known_bodies = tuple(state.bodies)
    forces: Dict[str, Vec3] = {}
    torques: Dict[str, Vec3] = {}
    by_slot: Dict[str, Dict[str, Vec3]] = {}
    torque_by_slot: Dict[str, Dict[str, Vec3]] = {}
    mass_updates: Dict[str, MassUpdate] = {}
    constraint_commands: Dict[str, ConstraintCommand] = {}
    collision_commands: Dict[str, CollisionPairCommand] = {}

    for batch in batches:
        if validate:
            validate_batch(batch, known_bodies)
        for wrench in batch.wrenches:
            force, torque = resolve_wrench(wrench, state)
            forces[wrench.body] = add(forces.get(wrench.body, ZERO3), force)
            torques[wrench.body] = add(torques.get(wrench.body, ZERO3), torque)
            slot_map = by_slot.setdefault(wrench.body, {})
            slot_map[batch.source] = add(slot_map.get(batch.source, ZERO3), force)
            torque_slot_map = torque_by_slot.setdefault(wrench.body, {})
            torque_slot_map[batch.source] = add(
                torque_slot_map.get(batch.source, ZERO3), torque
            )
        for update in batch.mass_updates:
            existing = mass_updates.get(update.body)
            if existing is not None:
                raise EffectValidationError(
                    f"two mass updates for body '{update.body}' in one step, from "
                    f"'{existing.source}' and '{update.source}'"
                )
            mass_updates[update.body] = update
        for command in batch.constraint_commands:
            existing_c = constraint_commands.get(command.constraint)
            if existing_c is not None and existing_c.action is not command.action:
                raise EffectValidationError(
                    f"conflicting commands for constraint '{command.constraint}': "
                    f"{existing_c.action.value} from '{existing_c.source}' and "
                    f"{command.action.value} from '{command.source}'"
                )
            constraint_commands[command.constraint] = command
        for command_p in batch.collision_commands:
            existing_p = collision_commands.get(command_p.pair)
            if existing_p is not None and existing_p.action is not command_p.action:
                raise EffectValidationError(
                    f"conflicting commands for collision pair '{command_p.pair}': "
                    f"{existing_p.action.value} from '{existing_p.source}' and "
                    f"{command_p.action.value} from '{command_p.source}'"
                )
            collision_commands[command_p.pair] = command_p

    loads = {
        body: BodyLoad(
            body=body,
            force_n=forces.get(body, ZERO3),
            torque_nm=torques.get(body, ZERO3),
            force_by_slot=dict(by_slot.get(body, {})),
            torque_by_slot=dict(torque_by_slot.get(body, {})),
        )
        for body in sorted(set(forces) | set(torques) | set(by_slot) | set(torque_by_slot))
    }
    return AggregatedEffects(
        loads=loads,
        mass_updates=tuple(mass_updates[body] for body in sorted(mass_updates)),
        constraint_commands=tuple(constraint_commands[name] for name in sorted(constraint_commands)),
        collision_commands=tuple(collision_commands[name] for name in sorted(collision_commands)),
    )


def axial_force(load: BodyLoad, axis: Vec3) -> float:
    """Signed component of a net body force along the tube axis."""
    from ..linalg import dot

    return dot(load.force_n, axis)


def axial_slot_force(load: BodyLoad, slot: str, axis: Vec3) -> float:
    """Signed component of one slot's contribution along the tube axis."""
    from ..linalg import dot

    return dot(load.force_by_slot.get(slot, ZERO3), axis)


def scaled_load(load: BodyLoad, factor: float) -> BodyLoad:
    """Scale a load and every slot contribution, used by force ramping diagnostics."""
    return BodyLoad(
        body=load.body,
        force_n=scale(load.force_n, factor),
        torque_nm=scale(load.torque_nm, factor),
        force_by_slot={slot: scale(value, factor) for slot, value in load.force_by_slot.items()},
        torque_by_slot={slot: scale(value, factor) for slot, value in load.torque_by_slot.items()},
    )
