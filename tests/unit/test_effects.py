# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
import unittest

import _bootstrap  # noqa: F401

from skyarc.effects import (
    CollisionAction,
    CollisionPairCommand,
    ConstraintAction,
    ConstraintCommand,
    EffectBatch,
    EffectValidationError,
    Frame,
    MassUpdate,
    MomentumPolicy,
    Wrench,
    aggregate,
    validate_batch,
)
from skyarc.effects.aggregator import resolve_wrench
from skyarc.linalg import quat_from_axis_angle
from skyarc.names import (
    BODY_CART,
    BODY_ROCKET,
    JOINT_COUPLING,
    PAIR_ROCKET_CRADLE,
    SLOT_COUPLING,
    SLOT_LAUNCH_FORCE,
    SLOT_ROCKET_MOTOR,
)
from skyarc.state import BodyState, SimulationState


def make_state(*, cart_orientation=(1.0, 0.0, 0.0, 0.0)) -> SimulationState:
    return SimulationState(
        time_s=0.0,
        step_index=0,
        dt_s=1.0 / 240.0,
        bodies={
            BODY_CART: BodyState(name=BODY_CART, orientation=cart_orientation, mass_kg=250.0),
            BODY_ROCKET: BodyState(name=BODY_ROCKET, position=(1.0, 0.0, 0.0), mass_kg=150.0),
        },
    ).frozen()


class EffectContractTests(unittest.TestCase):
    def test_body_wrench_resolves_frame_and_moment_about_com(self) -> None:
        state = make_state(cart_orientation=quat_from_axis_angle((0.0, 0.0, 1.0), math.pi / 2.0))
        wrench = Wrench(
            source=SLOT_LAUNCH_FORCE,
            body=BODY_CART,
            force_n=(1.0, 0.0, 0.0),
            application_point_m=(0.0, 1.0, 0.0),
            frame=Frame.BODY,
        )
        force, torque = resolve_wrench(wrench, state)
        self.assertAlmostEqual(force[0], 0.0, places=12)
        self.assertAlmostEqual(force[1], 1.0, places=12)
        self.assertAlmostEqual(torque[2], -1.0, places=12)

    def test_aggregation_retains_per_slot_force(self) -> None:
        state = make_state()
        batch = EffectBatch(
            source=SLOT_LAUNCH_FORCE,
            wrenches=(
                Wrench(
                    source=SLOT_LAUNCH_FORCE,
                    body=BODY_CART,
                    force_n=(10.0, 0.0, 0.0),
                    application_point_m=(0.0, 0.0, 0.0),
                ),
            ),
        )
        result = aggregate((batch,), state)
        self.assertEqual(result.load(BODY_CART).force_n, (10.0, 0.0, 0.0))
        self.assertEqual(result.slot_force(BODY_CART, SLOT_LAUNCH_FORCE), (10.0, 0.0, 0.0))

    def test_aggregated_effect_mappings_are_immutable_copies(self) -> None:
        state = make_state()
        result = aggregate(
            (
                EffectBatch(
                    source=SLOT_LAUNCH_FORCE,
                    wrenches=(
                        Wrench(
                            source=SLOT_LAUNCH_FORCE,
                            body=BODY_CART,
                            force_n=(10.0, 0.0, 0.0),
                        ),
                    ),
                ),
            ),
            state,
        )
        with self.assertRaises(TypeError):
            result.loads[BODY_ROCKET] = result.load(BODY_ROCKET)  # type: ignore[index]
        with self.assertRaises(TypeError):
            result.load(BODY_CART).force_by_slot[SLOT_LAUNCH_FORCE] = (0.0, 0.0, 0.0)  # type: ignore[index]

    def test_malformed_vector_is_a_contract_error_not_type_error(self) -> None:
        batch = EffectBatch(
            source=SLOT_LAUNCH_FORCE,
            wrenches=(
                Wrench(
                    source=SLOT_LAUNCH_FORCE,
                    body=BODY_CART,
                    application_point_m=None,  # type: ignore[arg-type]
                ),
            ),
        )
        with self.assertRaises(EffectValidationError):
            validate_batch(batch, (BODY_CART, BODY_ROCKET))

    def test_force_ownership_and_batch_source_are_enforced(self) -> None:
        wrong_body = EffectBatch(
            source=SLOT_LAUNCH_FORCE,
            wrenches=(Wrench(source=SLOT_LAUNCH_FORCE, body=BODY_ROCKET),),
        )
        with self.assertRaises(EffectValidationError):
            validate_batch(wrong_body, (BODY_CART, BODY_ROCKET))
        mismatched_source = EffectBatch(
            source=SLOT_LAUNCH_FORCE,
            wrenches=(Wrench(source=SLOT_ROCKET_MOTOR, body=BODY_CART),),
        )
        with self.assertRaises(EffectValidationError):
            validate_batch(mismatched_source, (BODY_CART, BODY_ROCKET))

    def test_mass_update_requires_valid_policy_and_accounting(self) -> None:
        invalid_policy = EffectBatch(
            source=SLOT_ROCKET_MOTOR,
            mass_updates=(
                MassUpdate(
                    source=SLOT_ROCKET_MOTOR,
                    body=BODY_ROCKET,
                    mass_kg=149.0,
                    effective_time_s=1.0,
                    momentum_policy="mystery",  # type: ignore[arg-type]
                ),
            ),
        )
        with self.assertRaises(EffectValidationError):
            validate_batch(invalid_policy, (BODY_CART, BODY_ROCKET))
        missing_exhaust = EffectBatch(
            source=SLOT_ROCKET_MOTOR,
            mass_updates=(
                MassUpdate(
                    source=SLOT_ROCKET_MOTOR,
                    body=BODY_ROCKET,
                    mass_kg=149.0,
                    effective_time_s=1.0,
                    momentum_policy=MomentumPolicy.ACCOUNTED,
                ),
            ),
        )
        with self.assertRaises(EffectValidationError):
            validate_batch(missing_exhaust, (BODY_CART, BODY_ROCKET))

    def test_constraint_and_collision_commands_require_exact_pairs(self) -> None:
        missing_body = EffectBatch(
            source=SLOT_COUPLING,
            constraint_commands=(
                ConstraintCommand(
                    source=SLOT_COUPLING,
                    constraint=JOINT_COUPLING,
                    action=ConstraintAction.DISABLE,
                    bodies=(BODY_CART,),
                ),
            ),
        )
        with self.assertRaises(EffectValidationError):
            validate_batch(missing_body, (BODY_CART, BODY_ROCKET))

        valid = EffectBatch(
            source=SLOT_COUPLING,
            constraint_commands=(
                ConstraintCommand(
                    source=SLOT_COUPLING,
                    constraint=JOINT_COUPLING,
                    action=ConstraintAction.DISABLE,
                    bodies=(BODY_CART, BODY_ROCKET),
                ),
            ),
            collision_commands=(
                CollisionPairCommand(
                    source=SLOT_COUPLING,
                    pair=PAIR_ROCKET_CRADLE,
                    action=CollisionAction.ENABLE,
                    bodies=(BODY_CART, BODY_ROCKET),
                ),
            ),
        )
        validate_batch(valid, (BODY_CART, BODY_ROCKET))


if __name__ == "__main__":
    unittest.main()
