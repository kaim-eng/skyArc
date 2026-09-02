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

"""Validation of effects at the component boundary.

Section 16.1 requires effect validation to reject missing bodies, frames, application
points, units, conflicting ownership and invalid mass updates. This module is that check.
It runs before aggregation and before anything reaches the adapter, so an ill-formed effect
never becomes an applied force whose provenance has to be reconstructed afterwards.
"""

from __future__ import annotations

from typing import Any, Iterable, Mapping

from ..linalg import is_finite
from ..names import (
    COLLISION_PAIR_BODIES,
    CONSTRAINT_BODIES,
    SLOT_BODY_OWNERSHIP,
    SLOT_COLLISION_OWNERSHIP,
    SLOT_CONSTRAINT_OWNERSHIP,
    SLOT_MASS_OWNERSHIP,
)
from .types import (
    SI,
    CollisionPairCommand,
    ConstraintCommand,
    EffectBatch,
    Frame,
    MassUpdate,
    MomentumPolicy,
    Wrench,
)


class EffectValidationError(ValueError):
    """Raised when an effect violates the component/effect contract."""


def _is_finite_vec3(value: Any) -> bool:
    """Contract-safe three-vector check that never leaks ``TypeError`` to a caller."""
    try:
        return len(value) == 3 and is_finite(value)
    except (TypeError, ValueError):
        return False


def _check_units(effect_kind: str, source: str, units: str) -> None:
    if units != SI:
        raise EffectValidationError(f"{effect_kind} from '{source}' declares units {units!r}; only {SI!r} is accepted")


def validate_wrench(wrench: Wrench, known_bodies: Iterable[str]) -> None:
    """Validate one wrench against the contract.

    Raises:
        EffectValidationError: On an unknown body, a missing or unknown frame, a
            non-finite quantity, a wrong unit tag, or a slot applying force to a body it
            does not own.
    """
    known = set(known_bodies)
    if wrench.body not in known:
        raise EffectValidationError(
            f"wrench from '{wrench.source}' targets unknown body '{wrench.body}'; known: {sorted(known)}"
        )
    if not isinstance(wrench.frame, Frame):
        raise EffectValidationError(
            f"wrench from '{wrench.source}' on '{wrench.body}' has no declared frame; "
            "argument order never implies a frame"
        )
    _check_units("wrench", wrench.source, wrench.units)
    for label, value in (
        ("force_n", wrench.force_n),
        ("torque_nm", wrench.torque_nm),
        ("application_point_m", wrench.application_point_m),
    ):
        if not _is_finite_vec3(value):
            raise EffectValidationError(
                f"wrench from '{wrench.source}' on '{wrench.body}' has non-finite or malformed {label}: {value!r}"
            )
    permitted = SLOT_BODY_OWNERSHIP.get(wrench.source)
    if permitted is None:
        raise EffectValidationError(f"slot '{wrench.source}' is not permitted to emit wrenches")
    if wrench.body not in permitted:
        raise EffectValidationError(
            f"ownership violation: slot '{wrench.source}' may apply force to {sorted(permitted)}, "
            f"not to '{wrench.body}'"
        )


def validate_mass_update(update: MassUpdate, known_bodies: Iterable[str]) -> None:
    """Validate one mass update.

    Raises:
        EffectValidationError: On an unknown body, a non-positive or non-finite mass, a
            non-finite effective time, an unowned target, or an ``ACCOUNTED`` policy that
            omits the exhaust velocity that makes the accounting checkable.
    """
    known = set(known_bodies)
    if update.body not in known:
        raise EffectValidationError(
            f"mass update from '{update.source}' targets unknown body '{update.body}'; known: {sorted(known)}"
        )
    _check_units("mass update", update.source, update.units)
    if not is_finite((update.mass_kg,)) or update.mass_kg <= 0.0:
        raise EffectValidationError(
            f"mass update from '{update.source}' on '{update.body}' has invalid mass {update.mass_kg!r}"
        )
    if not is_finite((update.effective_time_s,)):
        raise EffectValidationError(
            f"mass update from '{update.source}' on '{update.body}' has non-finite effective time"
        )
    permitted = SLOT_MASS_OWNERSHIP.get(update.source, frozenset())
    if update.body not in permitted:
        raise EffectValidationError(
            f"ownership violation: slot '{update.source}' may not change the mass of '{update.body}'"
        )
    if not isinstance(update.momentum_policy, MomentumPolicy):
        raise EffectValidationError(
            f"mass update from '{update.source}' has invalid momentum policy {update.momentum_policy!r}"
        )
    if update.momentum_policy is MomentumPolicy.ACCOUNTED:
        if update.exhaust_velocity_mps is None or not is_finite((update.exhaust_velocity_mps,)):
            raise EffectValidationError(
                f"mass update from '{update.source}' declares an accounted momentum policy but supplies no "
                "finite exhaust velocity; the accounting would not be reconcilable"
            )
    elif update.exhaust_velocity_mps is not None:
        raise EffectValidationError(
            f"mass update from '{update.source}' conserves momentum but also supplies an exhaust velocity"
        )
    if update.com_offset_m is not None and not _is_finite_vec3(update.com_offset_m):
        raise EffectValidationError(f"mass update from '{update.source}' has a malformed centre-of-mass offset")


def validate_constraint_command(command: ConstraintCommand, known_bodies: Iterable[str]) -> None:
    """Validate one constraint command.

    Raises:
        EffectValidationError: If the slot does not own the constraint or the command names
            a body that does not exist.
    """
    permitted = SLOT_CONSTRAINT_OWNERSHIP.get(command.source, frozenset())
    if command.constraint not in permitted:
        raise EffectValidationError(
            f"ownership violation: slot '{command.source}' may command {sorted(permitted)}, "
            f"not constraint '{command.constraint}'"
        )
    from .types import ConstraintAction

    if not isinstance(command.action, ConstraintAction):
        raise EffectValidationError(
            f"constraint command for '{command.constraint}' has invalid action {command.action!r}"
        )
    known = set(known_bodies)
    for body in command.bodies:
        if body not in known:
            raise EffectValidationError(
                f"constraint command from '{command.source}' names unknown body '{body}'"
            )
    expected = CONSTRAINT_BODIES[command.constraint]
    if command.bodies != expected:
        raise EffectValidationError(
            f"constraint '{command.constraint}' must identify exact bodies {expected}, got {command.bodies}"
        )


def validate_collision_command(command: CollisionPairCommand, known_bodies: Iterable[str]) -> None:
    """Validate one collision-pair command.

    Raises:
        EffectValidationError: If the slot does not own the pair or the command names a
            body that does not exist.
    """
    permitted = SLOT_COLLISION_OWNERSHIP.get(command.source, frozenset())
    if command.pair not in permitted:
        raise EffectValidationError(
            f"ownership violation: slot '{command.source}' may command {sorted(permitted)}, "
            f"not collision pair '{command.pair}'"
        )
    from .types import CollisionAction

    if not isinstance(command.action, CollisionAction):
        raise EffectValidationError(
            f"collision command for '{command.pair}' has invalid action {command.action!r}"
        )
    known = set(known_bodies)
    for body in command.bodies:
        if body not in known:
            raise EffectValidationError(f"collision command from '{command.source}' names unknown body '{body}'")
    expected = COLLISION_PAIR_BODIES[command.pair]
    if command.bodies != expected:
        raise EffectValidationError(
            f"collision pair '{command.pair}' must identify exact bodies {expected}, got {command.bodies}"
        )


def validate_batch(batch: EffectBatch, known_bodies: Iterable[str]) -> None:
    """Validate every effect in a batch and its internal consistency.

    Raises:
        EffectValidationError: On any contract violation, including an effect whose own
            ``source`` disagrees with the batch it arrived in. That mismatch matters because
            ownership is checked against the batch's slot, so a divergent per-effect source
            would make the recorded provenance untrue even though the check passed.
    """
    for wrench in batch.wrenches:
        if wrench.source != batch.source:
            raise EffectValidationError(
                f"wrench source '{wrench.source}' disagrees with batch source '{batch.source}'"
            )
        validate_wrench(wrench, known_bodies)
    for update in batch.mass_updates:
        if update.source != batch.source:
            raise EffectValidationError(
                f"mass update source '{update.source}' disagrees with batch source '{batch.source}'"
            )
        validate_mass_update(update, known_bodies)
    for command in batch.constraint_commands:
        if command.source != batch.source:
            raise EffectValidationError(
                f"constraint command source '{command.source}' disagrees with batch source '{batch.source}'"
            )
        validate_constraint_command(command, known_bodies)
    for command in batch.collision_commands:
        if command.source != batch.source:
            raise EffectValidationError(
                f"collision command source '{command.source}' disagrees with batch source '{batch.source}'"
            )
        validate_collision_command(command, known_bodies)


def validate_capabilities(required: Iterable[str], available: Mapping[str, bool]) -> None:
    """Check that every capability a component declares is offered by the backend.

    Raises:
        EffectValidationError: If a required capability is absent or disabled. This is
            checked in preflight rather than at first use so that a run cannot begin and
            then discover that a mechanism it depends on is unavailable.
    """
    missing = [name for name in required if not available.get(name, False)]
    if missing:
        raise EffectValidationError(
            f"backend does not provide required capabilities: {sorted(missing)}; "
            f"available: {sorted(name for name, ok in available.items() if ok)}"
        )
