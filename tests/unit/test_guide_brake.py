# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import unittest
from pathlib import Path

import _bootstrap  # noqa: F401

from skyarc.components import ScenarioContext
from skyarc.configuration import load_yaml, resolve_tube_layout
from skyarc.events import EVENT_ABORT
from skyarc.launcher import (
    IdealPathGuide,
    TubeLayout,
    TubeStage,
    compute_brake_command,
    simulate_cart_braking,
)
from skyarc.names import BODY_CART, JOINT_GUIDE
from skyarc.state import AxialQuantities, BodyState, Observation, SimulationState


BASELINE = Path(__file__).resolve().parents[2] / "configs" / "baseline.yaml"
CURVED = Path(__file__).resolve().parents[2] / "configs" / "curved_2kms.yaml"


def layout() -> TubeLayout:
    return TubeLayout(
        origin_m=(0.0, 0.0, 0.0),
        angle_deg=0.0,
        stages=(TubeStage("vacuum", 100.0, 0.0),),
        exterior_effective_density_ratio=1.0,
    )


def guide_observation(*, lateral_offset_m: float, speed_mps: float) -> Observation:
    state = SimulationState(
        time_s=1.0,
        step_index=10,
        dt_s=0.01,
        bodies={
            BODY_CART: BodyState(
                name=BODY_CART,
                position=(0.0, lateral_offset_m, 0.0),
                linear_velocity=(speed_mps, 0.0, 0.0),
                mass_kg=250.0,
            ),
        },
        joint_active={JOINT_GUIDE: True},
    ).frozen()
    return Observation(
        source_model="test",
        time_s=state.time_s,
        step_index=state.step_index,
        dt_s=state.dt_s,
        state=state,
        axial=AxialQuantities(
            s_cart_m=0.0,
            s_rocket_m=0.0,
            marker_s_m={},
            cart_axial_velocity_mps=speed_mps,
            rocket_axial_velocity_mps=0.0,
            assembly_mass_kg=250.0,
            stage_index=0,
            stage_name="vacuum",
            effective_density_ratio=0.0,
            separation_gap_m=0.0,
            separation_rate_mps=0.0,
        ),
        coupled=False,
    )


class GuideAndBrakeTests(unittest.TestCase):
    def test_guide_applies_signed_resistance_and_emits_one_clearance_abort(self) -> None:
        guide = IdealPathGuide(
            layout(),
            model_id="ideal_prismatic_v1",
            resistance_n=10.0,
            maximum_tracking_error_m=0.1,
            gravity_mps2=(0.0, 0.0, 0.0),
            code_hash="test-guide",
        )
        observation = guide_observation(lateral_offset_m=0.2, speed_mps=5.0)
        guide.prepare(ScenarioContext(scenario_id="test"))
        guide.reset(observation.state)
        output = guide.pre_step(observation)
        self.assertEqual(output.effects.wrenches[0].force_n, (-10.0, -0.0, -0.0))
        self.assertEqual(len(output.events), 1)
        self.assertEqual(output.events[0].name, EVENT_ABORT)
        self.assertEqual(guide.pre_step(observation).events, ())
        self.assertAlmostEqual(guide.snapshot_state()["peak_tracking_error_m"], 0.2)

    def test_brake_jerk_vector_and_no_reverse_limits_compose(self) -> None:
        jerk_limited = compute_brake_command(
            speed_mps=100.0,
            remaining_control_distance_m=100.0,
            dt_s=0.1,
            mass_kg=250.0,
            force_limit_n=1e9,
            jerk_limit_mps3=10.0,
            previous_brake_acceleration_mps2=0.0,
            stopped_speed_threshold_mps=0.01,
            gravity_tangent_mps2=0.0,
            external_tangent_force_n=0.0,
            guide_normal_load_mps2=0.0,
            maximum_resultant_load_g=10.0,
        )
        self.assertAlmostEqual(jerk_limited.brake_acceleration_mps2, 1.0)
        self.assertLessEqual(jerk_limited.resultant_load_g, 10.0)

        no_reverse = compute_brake_command(
            speed_mps=1.0,
            remaining_control_distance_m=0.01,
            dt_s=1.0,
            mass_kg=1.0,
            force_limit_n=1000.0,
            jerk_limit_mps3=1000.0,
            previous_brake_acceleration_mps2=1000.0,
            stopped_speed_threshold_mps=0.01,
            gravity_tangent_mps2=0.0,
            external_tangent_force_n=0.0,
            guide_normal_load_mps2=0.0,
            maximum_resultant_load_g=None,
        )
        self.assertEqual(no_reverse.force_n, 1.0)
        held = compute_brake_command(
            speed_mps=0.005,
            remaining_control_distance_m=1.0,
            dt_s=0.1,
            mass_kg=1.0,
            force_limit_n=1.0,
            jerk_limit_mps3=1.0,
            previous_brake_acceleration_mps2=0.0,
            stopped_speed_threshold_mps=0.01,
            gravity_tangent_mps2=0.0,
            external_tangent_force_n=0.0,
            guide_normal_load_mps2=0.0,
            maximum_resultant_load_g=None,
        )
        self.assertTrue(held.held)
        self.assertAlmostEqual(held.hold_force_n, -0.05)

    def test_reference_carts_stop_inside_tracks_without_reversal(self) -> None:
        baseline = load_yaml(BASELINE)
        baseline_result = simulate_cart_braking(
            baseline.config,
            resolve_tube_layout(baseline.config),
        )
        self.assertLess(baseline_result.stop_distance_m, 35.0)
        self.assertFalse(baseline_result.reversed)

        curved = load_yaml(CURVED)
        curved_result = simulate_cart_braking(
            curved.config,
            resolve_tube_layout(curved.config),
            dt_s=0.005,
        )
        self.assertLess(curved_result.stop_distance_m, 25000.0)
        self.assertLessEqual(
            curved_result.peak_resultant_load_g,
            curved.config.cart.maximum_resultant_load_g,
        )
        self.assertFalse(curved_result.reversed)


if __name__ == "__main__":
    unittest.main()
