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

"""The typed physical effects a component may return (section 5.1).

Four effect kinds exist and no others: a wrench, a mass-property update, a constraint
command, and a collision-pair command. A model that needs to do something outside this set
is asking to mutate the scene directly, which the contract forbids.

Every wrench carries an explicit :class:`Frame`. Argument order never implies a frame. The
runtime expresses the same choice with opposite polarity at different layers -- the
high-level prim API takes a ``local_frame`` flag while the underlying tensor view takes a
global-frame flag -- and confining that inversion to the adapter is only possible if the
frame travels with the effect.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from typing import Tuple

from ..linalg import ZERO3, Vec3

SI = "SI"
"""The only accepted unit system. Effects are newtons, newton-metres, metres, kilograms, seconds."""


class Frame(str, Enum):
    """Coordinate frame in which an effect's vector quantities are expressed."""

    WORLD = "world"
    BODY = "body"


class ConstraintAction(str, Enum):
    """Requested transition for a named constraint.

    Release disables the joint; it never deletes it. Deletion is a structural stage change,
    is not reversible, and would conflict with the reset obligation of section 10.2.
    """

    ENABLE = "enable"
    DISABLE = "disable"


class CollisionAction(str, Enum):
    """Requested transition for a named collision pair."""

    ENABLE = "enable"
    DISABLE = "disable"


class MomentumPolicy(str, Enum):
    """How a mass update accounts for momentum.

    ``CONSERVE`` keeps body momentum unchanged across the update, which is correct for a
    constant-mass model that merely restates its mass. ``ACCOUNTED`` declares that the model
    has itself supplied the reaction (as a propellant-flow model must), and requires an
    exhaust velocity so that the accounting is checkable rather than asserted.
    """

    CONSERVE = "conserve_momentum"
    ACCOUNTED = "accounted"


@dataclass(frozen=True)
class Wrench:
    """A force and torque applied to one body at one point.

    Attributes:
        source: Slot that produced the effect. Used for ownership checks and for the
            per-slot force decomposition that the energy identity of section 16.2 needs.
        body: Target body identifier.
        force_n: Force in newtons, expressed in ``frame``.
        torque_nm: Torque in newton-metres, expressed in ``frame``.
        application_point_m: Point of application. In ``Frame.WORLD`` this is a world
            position; in ``Frame.BODY`` it is an offset from the body origin.
        frame: Frame of ``force_n``, ``torque_nm`` and ``application_point_m``.
        units: Unit system tag; only ``"SI"`` is accepted.
    """

    source: str
    body: str
    force_n: Vec3 = ZERO3
    torque_nm: Vec3 = ZERO3
    application_point_m: Vec3 = ZERO3
    frame: Frame = Frame.WORLD
    units: str = SI

    def scaled(self, factor: float) -> "Wrench":
        """Return the same wrench with force and torque scaled, for ramping."""
        from ..linalg import scale

        return replace(self, force_n=scale(self.force_n, factor), torque_nm=scale(self.torque_nm, factor))


@dataclass(frozen=True)
class MassUpdate:
    """A change to a body's mass properties.

    Attributes:
        source: Slot that produced the effect.
        body: Target body identifier.
        mass_kg: New total mass, kilograms.
        effective_time_s: Simulation time at which the new mass takes effect. Stating it
            explicitly is what makes a later propellant curve reconcilable with the impulse
            record instead of merely plausible.
        momentum_policy: See :class:`MomentumPolicy`.
        exhaust_velocity_mps: Required when ``momentum_policy`` is ``ACCOUNTED``.
        com_offset_m: Optional new centre-of-mass offset in the body frame.
    """

    source: str
    body: str
    mass_kg: float
    effective_time_s: float
    momentum_policy: MomentumPolicy = MomentumPolicy.CONSERVE
    exhaust_velocity_mps: float | None = None
    com_offset_m: Vec3 | None = None
    units: str = SI


@dataclass(frozen=True)
class ConstraintCommand:
    """A requested transition of a named constraint between two identified bodies."""

    source: str
    constraint: str
    action: ConstraintAction
    bodies: Tuple[str, ...] = ()


@dataclass(frozen=True)
class CollisionPairCommand:
    """A requested transition of a named collision pair between two identified bodies."""

    source: str
    pair: str
    action: CollisionAction
    bodies: Tuple[str, ...] = ()


@dataclass(frozen=True)
class EffectBatch:
    """Everything one component asks for in one step.

    A batch is immutable and carries its producing slot, so that ownership can be checked
    without the aggregator having to trust a per-effect ``source`` that a component filled in.
    """

    source: str
    wrenches: Tuple[Wrench, ...] = ()
    mass_updates: Tuple[MassUpdate, ...] = ()
    constraint_commands: Tuple[ConstraintCommand, ...] = ()
    collision_commands: Tuple[CollisionPairCommand, ...] = ()

    @classmethod
    def empty(cls, source: str) -> "EffectBatch":
        """An empty batch from ``source``. The common case: most slots act in few steps."""
        return cls(source=source)

    def is_empty(self) -> bool:
        return not (self.wrenches or self.mass_updates or self.constraint_commands or self.collision_commands)

    def merged(self, other: "EffectBatch") -> "EffectBatch":
        """Concatenate two batches from the same source.

        Raises:
            ValueError: If the sources differ. Merging across slots would erase the
                ownership information the aggregator depends on.
        """
        if self.source != other.source:
            raise ValueError(f"cannot merge batches from different sources: {self.source!r} and {other.source!r}")
        return EffectBatch(
            source=self.source,
            wrenches=self.wrenches + other.wrenches,
            mass_updates=self.mass_updates + other.mass_updates,
            constraint_commands=self.constraint_commands + other.constraint_commands,
            collision_commands=self.collision_commands + other.collision_commands,
        )
