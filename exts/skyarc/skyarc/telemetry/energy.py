# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Stepwise mechanical-energy and work accounting."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping

from ..effects.adapter import AppliedEffects
from ..effects.aggregator import AggregatedEffects
from ..linalg import ZERO3, Vec3, add, dot, norm, scale, sub
from ..names import (
    BODY_ROCKET,
    SLOT_ATMOSPHERE,
    SLOT_BACKEND_ADAPTER,
    SLOT_BODY_OWNERSHIP,
    SLOT_CART_BRAKE,
    SLOT_GUIDE,
    SLOT_LAUNCH_FORCE,
    SLOT_ROCKET_AERODYNAMICS,
    SLOT_ROCKET_MOTOR,
    SLOT_SEPARATION_ACTUATOR,
)
from ..state import SimulationState, body_com_world


WORK_TERMS = (
    "launch",
    "thrust",
    "drag",
    "brake",
    "resistance",
    "separation",
    "guide_reaction",
)

SLOT_WORK_TERMS = {
    SLOT_LAUNCH_FORCE: "launch",
    SLOT_ROCKET_MOTOR: "thrust",
    SLOT_ATMOSPHERE: "drag",
    SLOT_ROCKET_AERODYNAMICS: "drag",
    SLOT_CART_BRAKE: "brake",
    SLOT_GUIDE: "resistance",
    SLOT_SEPARATION_ACTUATOR: "separation",
    SLOT_BACKEND_ADAPTER: "guide_reaction",
}
"""Every slot that may apply a wrench maps to exactly one work term.

The separation actuator is included even though the baseline ``none_v1`` applies no impulse.
Section 10.3 anticipates replacing it with a pusher, and a slot that can do work but has no
term would have that work vanish from the identity, leaving the discrepancy in the residual
with nothing to attribute it to. This is the same reasoning section 16.2 records for the
resistance term, which is not optional for exactly the same reason.

``guide_reaction`` is the backend's own term. A backend that holds the cart with a
constraint contributes nothing to it, because an ideal constraint does no work and reports
no slot force. The v0.29 force-resolved treatment substitutes a *commanded* reaction for
that constraint, and a commanded force is only approximately normal to the velocity, so it
injects or removes a small amount of energy every step. Folding that into ``resistance``
would misattribute it to the guide's tangential friction model, and leaving it out would
push it into the residual with nothing to name it -- which is exactly the disclosure the
panel asked for when it accepted a non-constraint mechanism. Per-slot torque is retained as
well, so each term is full wrench work rather than force-through-COM work only.
"""

ROTATION_TOLERANCE_RAD_S = 1e-9
"""Below this the translational identity is complete; above it, rotational energy is missing.

This is a numerical-noise threshold, not a physical limit. It is deliberately far below the
angular rates the design permits elsewhere -- the section 10.4 ignition gate allows 5 deg/s,
some eight orders of magnitude larger -- because any rotation at all makes the closure
incomplete. Exceeding it therefore marks the energy channels invalid rather than ending the
run: a rocket rotating within its own ignition gate is a valid physical state, and section 14
requires an unavailable value to be recorded as null with a validity flag, never to abort the
record that would have carried it.
"""


@dataclass(frozen=True)
class EnergySnapshot:
    kinetic_j: float
    potential_j: float
    mechanical_change_j: float
    work_j: Mapping[str, float]
    residual_j: float
    normalized_residual: float
    rocket_impulse_ns: float
    valid: bool = True
    invalid_reason: str | None = None


def rotational_closure_defect(
    state: SimulationState,
    *,
    tolerance_rad_s: float = ROTATION_TOLERANCE_RAD_S,
) -> str | None:
    """Name the first body whose rotation the translational identity cannot account for."""
    for name in sorted(state.bodies):
        rate = norm(state.bodies[name].angular_velocity)
        if rate > tolerance_rad_s:
            return (
                f"rotational kinetic energy of body {name!r} ({rate:.6g} rad/s) cannot be "
                "closed without modeled body inertia"
            )
    return None


def mechanical_energy(
    state: SimulationState,
    gravity_mps2: Vec3,
) -> tuple[float, float]:
    """Translational kinetic and gravitational potential energy of every body.

    Rotational kinetic energy is not included; :func:`rotational_closure_defect` reports
    whether omitting it makes the result incomplete.
    """
    kinetic = 0.0
    potential = 0.0
    for body in state.bodies.values():
        kinetic += 0.5 * body.mass_kg * dot(body.linear_velocity, body.linear_velocity)
        potential -= body.mass_kg * dot(gravity_mps2, body_com_world(body))
    return kinetic, potential


class EnergyAccumulator:
    """Integrate per-slot work while retaining the section 16.2 residual identity."""

    def __init__(
        self,
        initial_state: SimulationState,
        *,
        gravity_mps2: Vec3 = (0.0, 0.0, -9.81),
        normalization_floor_j: float = 1.0,
    ) -> None:
        if not math.isfinite(normalization_floor_j) or normalization_floor_j <= 0.0:
            raise ValueError("energy normalization floor must be finite and positive")
        if len(gravity_mps2) != 3 or not all(math.isfinite(value) for value in gravity_mps2):
            raise ValueError("energy gravity must be a finite three-vector")
        unmapped = sorted(
            slot
            for slot, bodies in SLOT_BODY_OWNERSHIP.items()
            if bodies and slot not in SLOT_WORK_TERMS
        )
        if unmapped:
            raise ValueError(
                f"slots {unmapped} may apply a wrench but map to no work term; their work "
                "would disappear from the section 16.2 residual with nothing to attribute it to"
            )
        self._gravity = gravity_mps2
        self._floor = normalization_floor_j
        self._initial_kinetic, self._initial_potential = mechanical_energy(initial_state, gravity_mps2)
        self._work = {name: 0.0 for name in WORK_TERMS}
        self._rocket_impulse = 0.0
        # Latching: once rotation has appeared, the accumulated change in mechanical energy
        # has already missed a rotational contribution, so every later residual stays suspect
        # even if the body stops rotating again.
        self._invalid_reason: str | None = rotational_closure_defect(initial_state)
        self._last = self._snapshot(initial_state)

    def _snapshot(self, state: SimulationState) -> EnergySnapshot:
        kinetic, potential = mechanical_energy(state, self._gravity)
        mechanical_change = (
            kinetic + potential - self._initial_kinetic - self._initial_potential
        )
        accounted_work = sum(self._work.values())
        residual = mechanical_change - accounted_work
        supplied = sum(max(0.0, value) for value in self._work.values())
        dissipated = sum(max(0.0, -value) for value in self._work.values())
        denominator = max(self._floor, supplied, dissipated)
        return EnergySnapshot(
            kinetic_j=kinetic,
            potential_j=potential,
            mechanical_change_j=mechanical_change,
            work_j=dict(self._work),
            residual_j=residual,
            normalized_residual=abs(residual) / denominator,
            rocket_impulse_ns=self._rocket_impulse,
            valid=self._invalid_reason is None,
            invalid_reason=self._invalid_reason,
        )

    @property
    def snapshot(self) -> EnergySnapshot:
        return self._last

    def update(
        self,
        before: SimulationState,
        after: SimulationState,
        applied: AggregatedEffects | AppliedEffects,
    ) -> EnergySnapshot:
        if self._invalid_reason is None:
            self._invalid_reason = rotational_closure_defect(after)
        for body_name, load in applied.loads.items():
            displacement = sub(
                body_com_world(after.body(body_name)),
                body_com_world(before.body(body_name)),
            )
            dt_s = after.time_s - before.time_s
            angular_displacement = scale(
                add(
                    before.body(body_name).angular_velocity,
                    after.body(body_name).angular_velocity,
                ),
                0.5 * dt_s,
            )
            for slot in sorted(set(load.force_by_slot) | set(load.torque_by_slot)):
                term = SLOT_WORK_TERMS.get(slot)
                if term is None:
                    # Silently skipping would move this work into the residual with no term
                    # to attribute it to, which is the failure the completeness check in
                    # __init__ exists to prevent. A slot reaching here is a wiring error.
                    raise ValueError(
                        f"slot {slot!r} applied force to {body_name!r} but maps to no work term"
                    )
                force = load.force_by_slot.get(slot, ZERO3)
                torque = load.torque_by_slot.get(slot, ZERO3)
                self._work[term] += dot(force, displacement) + dot(
                    torque, angular_displacement
                )
        if BODY_ROCKET in applied.loads:
            thrust = applied.loads[BODY_ROCKET].force_by_slot.get(SLOT_ROCKET_MOTOR, (0.0, 0.0, 0.0))
            self._rocket_impulse += norm(thrust) * (after.time_s - before.time_s)
        self._last = self._snapshot(after)
        return self._last
