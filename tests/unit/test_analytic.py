# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
import unittest
from pathlib import Path

import _bootstrap  # noqa: F401

from skyarc.configuration import load_yaml, resolve_tube_layout
from skyarc.configuration.schema import (
    ForcePositionPointConfig,
    LaunchControlConfig,
)
from skyarc.effects import (
    ConstraintAction,
    ConstraintCommand,
    EffectBatch,
    Frame,
    Wrench,
    aggregate,
)
from skyarc.effects.backends import AnalyticBackend
from skyarc.launcher import (
    TubeLayout,
    TubeStage,
    compute_launch_command,
    launch_ramp_factor,
    quadratic_axial_drag_force_n,
    simulate_guided_launch,
)
from skyarc.names import (
    BODY_CART,
    BODY_ROCKET,
    JOINT_COUPLING,
    JOINT_GUIDE,
    SLOT_COUPLING,
    SLOT_LAUNCH_FORCE,
)
from skyarc.state import BodyState, SimulationState


BASELINE = Path(__file__).resolve().parents[2] / "configs" / "baseline.yaml"
CURVED = Path(__file__).resolve().parents[2] / "configs" / "curved_2kms.yaml"


def straight_layout() -> TubeLayout:
    return TubeLayout(
        origin_m=(0.0, 0.0, 0.0),
        angle_deg=0.0,
        stages=(TubeStage("vacuum", 100.0, 0.0),),
        exterior_effective_density_ratio=0.0,
    )


def control(mode: str, **changes: object) -> LaunchControlConfig:
    values = dict(
        mode=mode,
        target_exit_speed_mps=50.0,
        maximum_force_n=12000.0,
        maximum_acceleration_mps2=20.0,
        force_ramp_up_distance_m=0.0,
        force_ramp_down_distance_m=0.0,
    )
    values.update(changes)
    return LaunchControlConfig(**values)  # type: ignore[arg-type]


class AnalyticBackendTests(unittest.TestCase):
    def initial_state(self) -> SimulationState:
        return SimulationState(
            time_s=0.0,
            step_index=0,
            dt_s=0.1,
            bodies={
                BODY_CART: BodyState(name=BODY_CART, mass_kg=2.0),
                BODY_ROCKET: BodyState(name=BODY_ROCKET, mass_kg=2.0),
            },
            joint_active={JOINT_COUPLING: True, JOINT_GUIDE: True},
        ).frozen()

    def test_coupled_force_uses_combined_mass_and_semi_implicit_euler(self) -> None:
        backend = AnalyticBackend(self.initial_state(), straight_layout(), gravity_mps2=(0.0, 0.0, 0.0))
        state = backend.read_state()
        batch = EffectBatch(
            source=SLOT_LAUNCH_FORCE,
            wrenches=(
                Wrench(
                    source=SLOT_LAUNCH_FORCE,
                    body=BODY_CART,
                    force_n=(400.0, 0.0, 0.0),
                    frame=Frame.WORLD,
                ),
            ),
        )
        backend.apply(aggregate((batch,), state))
        backend.step()
        result = backend.read_state()
        self.assertEqual(result.step_index, 1)
        self.assertAlmostEqual(result.body(BODY_CART).linear_velocity[0], 10.0)
        self.assertAlmostEqual(result.body(BODY_CART).position[0], 1.0)
        self.assertEqual(result.body(BODY_CART).linear_velocity, result.body(BODY_ROCKET).linear_velocity)
        self.assertEqual(result.body(BODY_CART).position, result.body(BODY_ROCKET).position)

        backend.reset()
        self.assertEqual(backend.read_state().step_index, 0)
        self.assertEqual(backend.read_state().body(BODY_CART).position, (0.0, 0.0, 0.0))

    def test_backend_rejects_torque_and_duplicate_apply(self) -> None:
        backend = AnalyticBackend(self.initial_state(), straight_layout(), gravity_mps2=(0.0, 0.0, 0.0))
        state = backend.read_state()
        torque = EffectBatch(
            source=SLOT_LAUNCH_FORCE,
            wrenches=(
                Wrench(
                    source=SLOT_LAUNCH_FORCE,
                    body=BODY_CART,
                    torque_nm=(0.0, 1.0, 0.0),
                ),
            ),
        )
        with self.assertRaisesRegex(ValueError, "translation-only"):
            backend.apply(aggregate((torque,), state))
        empty = aggregate((), state)
        backend.apply(empty)
        with self.assertRaisesRegex(RuntimeError, "already been applied"):
            backend.apply(empty)

    def test_release_switches_rocket_to_ballistic_translation(self) -> None:
        backend = AnalyticBackend(
            self.initial_state(),
            straight_layout(),
            gravity_mps2=(0.0, 0.0, -10.0),
        )
        state = backend.read_state()
        release = EffectBatch(
            source=SLOT_COUPLING,
            constraint_commands=(
                ConstraintCommand(
                    source=SLOT_COUPLING,
                    constraint=JOINT_COUPLING,
                    action=ConstraintAction.DISABLE,
                    bodies=(BODY_CART, BODY_ROCKET),
                ),
            ),
        )
        backend.apply(aggregate((release,), state))
        backend.resync()
        backend.step()
        result = backend.read_state()
        self.assertEqual(result.body(BODY_CART).position[2], 0.0)
        self.assertAlmostEqual(result.body(BODY_ROCKET).linear_velocity[2], -1.0)
        self.assertAlmostEqual(result.body(BODY_ROCKET).position[2], -0.1)
        self.assertEqual(backend.resync_count, 1)


class LauncherModelTests(unittest.TestCase):
    def test_drag_is_zero_in_vacuum_and_has_density_speed_and_wind_sign(self) -> None:
        self.assertEqual(quadratic_axial_drag_force_n(0.0, 0.3, 1.0, 100.0), 0.0)
        base = quadratic_axial_drag_force_n(1.0, 0.5, 2.0, 10.0)
        self.assertEqual(base, -50.0)
        self.assertEqual(quadratic_axial_drag_force_n(0.5, 0.5, 2.0, 20.0), 2.0 * base)
        self.assertGreater(quadratic_axial_drag_force_n(1.0, 0.5, 2.0, 10.0, 20.0), 0.0)

    def test_acceleration_ceiling_precedes_force_ceiling_and_compensates_grade(self) -> None:
        command = compute_launch_command(
            control("constant_force"),
            position_m=10.0,
            speed_mps=0.0,
            elapsed_s=1.0,
            path_length_m=100.0,
            assembly_mass_kg=400.0,
            gravity_tangent_mps2=-5.0,
            drag_force_tangent_n=-100.0,
            resistance_force_tangent_n=-50.0,
        )
        self.assertEqual(command.acceleration_command_mps2, 20.0)
        self.assertEqual(command.force_n, 10150.0)

    def test_target_mode_uses_distance_to_ramp_start_and_resultant_budget(self) -> None:
        target = control(
            "target_exit_speed",
            maximum_force_n=100000.0,
            maximum_acceleration_mps2=100.0,
            force_ramp_down_distance_m=20.0,
            maximum_resultant_load_g=10.0,
        )
        command = compute_launch_command(
            target,
            position_m=10.0,
            speed_mps=0.0,
            elapsed_s=1.0,
            path_length_m=100.0,
            assembly_mass_kg=400.0,
            gravity_tangent_mps2=0.0,
            drag_force_tangent_n=0.0,
            resistance_force_tangent_n=0.0,
            guide_normal_load_mps2=8.0 * 9.81,
        )
        self.assertAlmostEqual(command.distance_to_ramp_down_m, 70.0)
        self.assertAlmostEqual(command.target_acceleration_mps2, 2500.0 / 140.0)
        self.assertAlmostEqual(command.resultant_limit_force_n, 400.0 * 6.0 * 9.81)
        self.assertLessEqual(command.force_n, command.resultant_limit_force_n)

    def test_force_table_interpolates_and_rest_bootstrap_advances_with_time(self) -> None:
        table = control(
            "force_vs_position",
            maximum_force_n=20000.0,
            maximum_acceleration_mps2=100.0,
            force_vs_position=(
                ForcePositionPointConfig(0.0, 1000.0),
                ForcePositionPointConfig(100.0, 3000.0),
            ),
        )
        command = compute_launch_command(
            table,
            position_m=50.0,
            speed_mps=0.0,
            elapsed_s=1.0,
            path_length_m=100.0,
            assembly_mass_kg=1.0,
            gravity_tangent_mps2=0.0,
            drag_force_tangent_n=0.0,
            resistance_force_tangent_n=0.0,
        )
        self.assertEqual(command.force_n, 100.0)  # acceleration ceiling binds the 2 kN table value
        self.assertEqual(
            launch_ramp_factor(
                position_m=0.0,
                elapsed_s=0.0,
                path_length_m=100.0,
                ramp_up_distance_m=2.0,
                ramp_down_distance_m=5.0,
                maximum_acceleration_mps2=20.0,
            ),
            0.0,
        )
        self.assertGreater(
            launch_ramp_factor(
                position_m=0.0,
                elapsed_s=0.1,
                path_length_m=100.0,
                ramp_up_distance_m=2.0,
                ramp_down_distance_m=5.0,
                maximum_acceleration_mps2=20.0,
            ),
            0.0,
        )

    def test_baseline_guided_run_reaches_exit_and_energy_error_converges(self) -> None:
        loaded = load_yaml(BASELINE)
        layout = resolve_tube_layout(loaded.config)
        coarse = simulate_guided_launch(loaded.config, layout)
        fine = simulate_guided_launch(
            loaded.config,
            layout,
            dt_s=0.5 * loaded.config.simulation.physics_dt_s,
        )
        self.assertLess(abs(coarse.exit_speed_mps - 50.0) / 50.0, 0.05)
        self.assertLess(coarse.elapsed_s, loaded.config.simulation.maximum_run_time_s)

        mass = loaded.config.cart.mass_kg + loaded.config.rocket.initial_mass_kg
        def residual(result):  # type: ignore[no-untyped-def]
            body_s = layout.axial_position(result.final_state.body(BODY_CART).position)
            height = body_s * math.sin(math.radians(layout.angle_deg))
            mechanical = 0.5 * mass * result.exit_speed_mps**2 + mass * 9.81 * height
            work = result.launch_work_j + result.drag_work_j + result.resistance_work_j
            return abs(mechanical - work)

        self.assertLess(residual(fine), residual(coarse))

    def test_curved_reference_reaches_target_inside_vector_and_jerk_limits(self) -> None:
        loaded = load_yaml(CURVED)
        result = simulate_guided_launch(
            loaded.config,
            resolve_tube_layout(loaded.config),
            dt_s=0.005,
        )
        self.assertLess(abs(result.exit_speed_mps - 2000.0) / 2000.0, 0.001)
        self.assertLessEqual(
            result.peak_resultant_load_g,
            loaded.config.launch_control.maximum_resultant_load_g,
        )
        self.assertLessEqual(
            result.peak_normal_jerk_mps3,
            loaded.config.launch_control.maximum_normal_jerk_mps3,
        )

    def test_force_modes_never_deliver_more_than_the_authored_force(self) -> None:
        # The hold bias serves the motion-specified modes. Left unbounded it overrode a
        # force table that asked for less than the force needed to hold station, turning an
        # authored coast region into a hold-station region with no diagnostic.
        mass = 400.0
        gravity_tangent = -9.81 * math.sin(math.radians(45.0))
        hold_force = -mass * gravity_tangent
        below_hold = 0.25 * hold_force
        table = control(
            "force_vs_position",
            maximum_force_n=20000.0,
            maximum_acceleration_mps2=100.0,
            force_vs_position=(
                ForcePositionPointConfig(0.0, below_hold),
                ForcePositionPointConfig(90.0, below_hold),
            ),
        )
        command = compute_launch_command(
            table,
            position_m=10.0,
            speed_mps=5.0,
            elapsed_s=1.0,
            path_length_m=90.0,
            assembly_mass_kg=mass,
            gravity_tangent_mps2=gravity_tangent,
            drag_force_tangent_n=0.0,
            resistance_force_tangent_n=0.0,
        )
        self.assertLess(below_hold, hold_force)
        self.assertAlmostEqual(command.force_n, below_hold)

        # Above the hold requirement the conversion round-trips exactly, so capping changes
        # nothing and the mode still delivers precisely what the table asked for.
        above_hold = 2.0 * hold_force
        table = control(
            "force_vs_position",
            maximum_force_n=20000.0,
            maximum_acceleration_mps2=100.0,
            force_vs_position=(
                ForcePositionPointConfig(0.0, above_hold),
                ForcePositionPointConfig(90.0, above_hold),
            ),
        )
        command = compute_launch_command(
            table,
            position_m=10.0,
            speed_mps=5.0,
            elapsed_s=1.0,
            path_length_m=90.0,
            assembly_mass_kg=mass,
            gravity_tangent_mps2=gravity_tangent,
            drag_force_tangent_n=0.0,
            resistance_force_tangent_n=0.0,
        )
        self.assertAlmostEqual(command.force_n, above_hold)

    def test_reported_normal_jerk_is_the_section_7_quantity(self) -> None:
        # Differencing the guide-normal bound is not normal jerk: the bound is a max() of two
        # branches and is not differentiable where they swap. The runner must converge to the
        # preflight sweep of j_n = 2 v vdot kappa_s + v^3 dkappa_s/ds, which a finite
        # difference of the bound would not do on a low-curvature path.
        loaded = load_yaml(CURVED)
        layout = resolve_tube_layout(loaded.config)
        expected = loaded.preflight.centerline.peak_normal_jerk_mps3
        coarse = simulate_guided_launch(loaded.config, layout, dt_s=0.008)
        fine = simulate_guided_launch(loaded.config, layout, dt_s=0.002)
        self.assertLess(abs(fine.peak_normal_jerk_mps3 - expected), 0.05)
        self.assertLessEqual(
            abs(fine.peak_normal_jerk_mps3 - expected),
            abs(coarse.peak_normal_jerk_mps3 - expected) + 1e-9,
        )


if __name__ == "__main__":
    unittest.main()
