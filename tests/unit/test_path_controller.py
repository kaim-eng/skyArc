# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
import unittest
from pathlib import Path

import _bootstrap  # noqa: F401

from skyarc.configuration import load_yaml, resolve_tube_layout
from skyarc.launcher.geometry import TubeLayout, TubeStage, path_pose
from skyarc.launcher.path_controller import (
    ForceResolvedPathReaction,
    LaunchProfileReferenceFrame,
    PathControllerGains,
    forward_pitch_angle_rad,
    wrap_angle_rad,
)
from skyarc.linalg import ZERO3, add, cross, dot, norm, scale, sub
from skyarc.names import BODY_CART, BODY_ROCKET, JOINT_COUPLING
from skyarc.state import BodyState, SimulationState


PROJECT = Path(__file__).resolve().parents[2]
CURVED = PROJECT / "configs" / "curved_2kms.yaml"

GRAVITY = (0.0, 0.0, -9.81)
CART_MASS_KG = 250.0
ROCKET_MASS_KG = 150.0
ASSEMBLY_MASS_KG = CART_MASS_KG + ROCKET_MASS_KG


def inclined_layout(angle_deg: float = 0.0) -> TubeLayout:
    return TubeLayout(
        origin_m=(0.0, 0.0, 0.0),
        angle_deg=angle_deg,
        stages=(TubeStage(name="vacuum", length_m=500.0, effective_density_ratio=0.0),),
    )


def orientation_for(angle_deg: float) -> tuple[float, float, float, float]:
    half = 0.5 * math.radians(angle_deg)
    return (math.cos(half), 0.0, -math.sin(half), 0.0)


def attached_state(
    layout: TubeLayout,
    *,
    s_m: float,
    speed_mps: float,
    normal_offset_m: float = 0.0,
    binormal_offset_m: float = 0.0,
    coupled: bool = True,
    angular_velocity_radps: tuple[float, float, float] = ZERO3,
) -> SimulationState:
    pose = path_pose(layout, s_m)
    binormal = cross(pose.tangent, pose.normal)
    com = add(
        pose.position_m,
        add(scale(pose.normal, normal_offset_m), scale(binormal, binormal_offset_m)),
    )
    velocity = scale(pose.tangent, speed_mps)
    orientation = orientation_for(pose.inclination_deg)
    # Both bodies sit on the assembly centre of mass so the placement stays exact and the
    # test measures the reaction law rather than a lever arm.
    bodies = {
        BODY_CART: BodyState(
            name=BODY_CART,
            position=com,
            orientation=orientation,
            linear_velocity=velocity,
            angular_velocity=angular_velocity_radps,
            mass_kg=CART_MASS_KG,
        ),
        BODY_ROCKET: BodyState(
            name=BODY_ROCKET,
            position=com,
            orientation=orientation,
            linear_velocity=velocity,
            angular_velocity=angular_velocity_radps,
            mass_kg=ROCKET_MASS_KG,
        ),
    }
    return SimulationState(
        time_s=0.0,
        step_index=0,
        dt_s=0.001,
        bodies=bodies,
        joint_active={JOINT_COUPLING: coupled},
    ).frozen()


def reaction_for(layout: TubeLayout, **kwargs: float) -> ForceResolvedPathReaction:
    return ForceResolvedPathReaction(
        layout,
        coupled_pitch_inertia_kg_m2=1356.402410888672,
        cart_pitch_inertia_kg_m2=180.0,
        gravity_mps2=GRAVITY,
        **kwargs,  # type: ignore[arg-type]
    )


class GainsAndHelperTests(unittest.TestCase):
    def test_default_gains_are_the_accepted_phase0_condition(self) -> None:
        gains = PathControllerGains()
        self.assertEqual(gains.normal_kp_per_s2, 400.0)
        # Not the runner's argument default of 0.0: the accepted artifacts recorded 40.0.
        self.assertEqual(gains.normal_kd_per_s, 40.0)
        self.assertEqual(gains.attitude_kp_per_s2, 2500.0)
        self.assertEqual(gains.attitude_kd_per_s, 100.0)

    def test_gains_reject_nonfinite_and_negative_values(self) -> None:
        for field in (
            "normal_kp_per_s2",
            "normal_kd_per_s",
            "attitude_kp_per_s2",
            "attitude_kd_per_s",
        ):
            for hostile in (float("nan"), float("inf"), -1.0):
                with self.subTest(field=field, value=hostile):
                    with self.assertRaisesRegex(ValueError, "finite and nonnegative"):
                        PathControllerGains(**{field: hostile})

    def test_forward_angle_and_wrap_are_inverse_of_the_authored_orientation(self) -> None:
        for angle_deg in (-30.0, 0.0, 7.5, 15.0, 89.0):
            with self.subTest(angle_deg=angle_deg):
                measured = forward_pitch_angle_rad(orientation_for(angle_deg))
                self.assertAlmostEqual(math.degrees(measured), angle_deg, places=9)
        # The fold is half-open at ``[-pi, pi)``, so both odd multiples land on -pi.
        self.assertAlmostEqual(wrap_angle_rad(3.0 * math.pi), -math.pi, places=12)
        self.assertAlmostEqual(wrap_angle_rad(-3.0 * math.pi), -math.pi, places=12)
        self.assertAlmostEqual(wrap_angle_rad(math.radians(359.0)), math.radians(-1.0), places=12)


class ReferenceFrameTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = load_yaml(CURVED).config
        cls.layout = resolve_tube_layout(cls.config)

    def test_profile_reaches_the_target_speed_exactly_at_the_tube_exit(self) -> None:
        frame = LaunchProfileReferenceFrame(self.layout, target_exit_speed_mps=2000.0)
        # The accepted 1 ms artifact recorded this profile acceleration for the same tube.
        self.assertAlmostEqual(frame.acceleration_mps2, 36.95769665240981, places=9)
        exit_sample = frame.sample(frame.exit_time_s)
        self.assertAlmostEqual(norm(exit_sample.velocity_mps), 2000.0, places=6)
        exit_pose = path_pose(self.layout, self.layout.length_m)
        for index in range(3):
            self.assertAlmostEqual(
                exit_sample.position_m[index], exit_pose.position_m[index], places=3
            )

    def test_frame_is_inertial_after_the_exit_and_stays_velocity_continuous(self) -> None:
        frame = LaunchProfileReferenceFrame(self.layout, target_exit_speed_mps=2000.0)
        before = frame.sample(frame.exit_time_s - 1e-6)
        after = frame.sample(frame.exit_time_s + 1e-6)
        self.assertEqual(after.acceleration_mps2, ZERO3)
        for index in range(3):
            self.assertAlmostEqual(
                after.velocity_mps[index], before.velocity_mps[index], places=3
            )
            self.assertAlmostEqual(
                after.position_m[index], before.position_m[index], places=2
            )
        # After the exit the frame coasts along the exit tangent at the exit speed.
        coasted = frame.sample(frame.exit_time_s + 10.0)
        travelled = norm(sub(coasted.position_m, after.position_m))
        self.assertAlmostEqual(travelled, 2000.0 * 10.0, delta=0.01)

    def test_braking_frame_follows_the_cart_to_rest_and_then_holds(self) -> None:
        brake_distance_m = 23000.0
        frame = LaunchProfileReferenceFrame(
            self.layout, target_exit_speed_mps=2000.0, brake_distance_m=brake_distance_m
        )
        self.assertEqual(frame.brake_distance_m, brake_distance_m)
        self.assertAlmostEqual(
            frame.brake_deceleration_mps2, 2000.0**2 / (2.0 * brake_distance_m), places=9
        )
        rest_time_s = frame.rest_time_s
        self.assertIsNotNone(rest_time_s)

        exit_pose = path_pose(self.layout, self.layout.length_m)
        at_rest = frame.sample(rest_time_s)
        self.assertEqual(at_rest.velocity_mps, ZERO3)
        self.assertEqual(at_rest.acceleration_mps2, ZERO3)
        travelled_m = norm(sub(at_rest.position_m, exit_pose.position_m))
        self.assertAlmostEqual(travelled_m, brake_distance_m, delta=1e-6)

        # Holding station is the whole point: an inertial frame kept moving at the exit
        # speed and ran 34 km away from a cart that had already stopped.
        much_later = frame.sample(rest_time_s + 60.0)
        self.assertEqual(much_later.velocity_mps, ZERO3)
        for index in range(3):
            self.assertAlmostEqual(
                much_later.position_m[index], at_rest.position_m[index], places=9
            )

    def test_braking_frame_is_position_and_velocity_continuous_at_both_joins(self) -> None:
        """A step in frame velocity would report a jump the bodies never had.

        Global state is reconstructed as ``v_global = v_solver + v_r``, so ``v_r`` must be
        continuous. Acceleration may step, because it cancels in the reconstruction.
        """
        frame = LaunchProfileReferenceFrame(
            self.layout, target_exit_speed_mps=2000.0, brake_distance_m=23000.0
        )
        # The window has to be small enough that legitimate travel across it is negligible:
        # at 2 km/s even a 0.1 ms window moves the frame 0.4 m, which would swamp a real
        # discontinuity rather than expose one.
        epsilon_s = 1e-9
        for join_s in (frame.exit_time_s, frame.rest_time_s):
            with self.subTest(join_s=join_s):
                before = frame.sample(join_s - epsilon_s)
                after = frame.sample(join_s + epsilon_s)
                for index in range(3):
                    self.assertAlmostEqual(
                        after.velocity_mps[index], before.velocity_mps[index], delta=1e-5
                    )
                    self.assertAlmostEqual(
                        after.position_m[index], before.position_m[index], delta=1e-5
                    )

    def test_braking_frame_speed_decreases_monotonically_to_rest(self) -> None:
        frame = LaunchProfileReferenceFrame(
            self.layout, target_exit_speed_mps=2000.0, brake_distance_m=23000.0
        )
        previous = 2000.0 + 1.0
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            speed = norm(
                frame.sample(
                    frame.exit_time_s + fraction * (frame.rest_time_s - frame.exit_time_s)
                ).velocity_mps
            )
            self.assertLess(speed, previous)
            previous = speed
        self.assertAlmostEqual(previous, 0.0, places=9)

    def test_frame_rejects_impossible_profiles(self) -> None:
        for target, start in ((0.0, 0.0), (-5.0, 0.0), (float("nan"), 0.0)):
            with self.subTest(target=target):
                with self.assertRaises(ValueError):
                    LaunchProfileReferenceFrame(
                        self.layout, target_exit_speed_mps=target, start_s_m=start
                    )
        with self.assertRaisesRegex(ValueError, "precede the tube exit"):
            LaunchProfileReferenceFrame(
                self.layout,
                target_exit_speed_mps=2000.0,
                start_s_m=self.layout.length_m,
            )
        frame = LaunchProfileReferenceFrame(self.layout, target_exit_speed_mps=2000.0)
        with self.assertRaisesRegex(ValueError, "finite and nonnegative"):
            frame.sample(-1.0)
        self.assertIsNone(frame.brake_distance_m)
        self.assertIsNone(frame.rest_time_s)
        for hostile in (0.0, -1.0, float("nan"), float("inf")):
            with self.subTest(brake_distance_m=hostile):
                with self.assertRaisesRegex(ValueError, "brake distance"):
                    LaunchProfileReferenceFrame(
                        self.layout,
                        target_exit_speed_mps=2000.0,
                        brake_distance_m=hostile,
                    )


class GuideReactionTests(unittest.TestCase):
    def test_on_a_level_straight_path_the_reaction_only_carries_weight(self) -> None:
        layout = inclined_layout(0.0)
        state = attached_state(layout, s_m=100.0, speed_mps=300.0)
        reaction = reaction_for(layout).evaluate(state, {})
        # A level straight tube needs exactly the assembly weight held normal to the path
        # and nothing along it: the guide is not a propulsion slot.
        self.assertAlmostEqual(reaction.force_n[2], ASSEMBLY_MASS_KG * 9.81, places=6)
        self.assertAlmostEqual(reaction.force_n[0], 0.0, places=9)
        self.assertAlmostEqual(reaction.force_n[1], 0.0, places=9)
        self.assertAlmostEqual(reaction.ideal_normal_force_n, ASSEMBLY_MASS_KG * 9.81, places=6)
        self.assertAlmostEqual(reaction.tracking_error_m, 0.0, places=12)
        self.assertEqual(reaction.body, BODY_CART)
        self.assertTrue(reaction.coupled)

    def test_an_inclined_straight_path_leaves_the_tangential_component_to_the_launcher(
        self,
    ) -> None:
        layout = inclined_layout(15.0)
        state = attached_state(layout, s_m=100.0, speed_mps=300.0)
        reaction = reaction_for(layout).evaluate(state, {})
        pose = path_pose(layout, 100.0)
        self.assertAlmostEqual(dot(reaction.force_n, pose.tangent), 0.0, places=9)
        # The normal reaction carries only the gravity component perpendicular to the tube.
        self.assertAlmostEqual(
            dot(reaction.force_n, pose.normal),
            -ASSEMBLY_MASS_KG * dot(GRAVITY, pose.normal),
            places=6,
        )

    def test_an_accepted_normal_load_is_subtracted_rather_than_double_counted(self) -> None:
        layout = inclined_layout(0.0)
        state = attached_state(layout, s_m=100.0, speed_mps=300.0)
        controller = reaction_for(layout)
        unloaded = controller.evaluate(state, {})
        external = (0.0, 0.0, 500.0)
        loaded = controller.evaluate(state, {BODY_CART: external})
        self.assertAlmostEqual(loaded.force_n[2], unloaded.force_n[2] - 500.0, places=6)
        # The command a constraint would have to supply is unchanged; only the share the
        # backend has to add on top of the accepted effects moves.
        self.assertAlmostEqual(
            loaded.commanded_normal_force_n, unloaded.commanded_normal_force_n, places=9
        )

    def test_position_and_rate_feedback_have_the_documented_sign_and_magnitude(self) -> None:
        layout = inclined_layout(0.0)
        controller = reaction_for(layout)
        gains = PathControllerGains()
        centered = controller.evaluate(attached_state(layout, s_m=100.0, speed_mps=0.0), {})
        displaced = controller.evaluate(
            attached_state(layout, s_m=100.0, speed_mps=0.0, normal_offset_m=0.01), {}
        )
        self.assertAlmostEqual(
            displaced.force_n[2] - centered.force_n[2],
            -ASSEMBLY_MASS_KG * gains.normal_kp_per_s2 * 0.01,
            places=6,
        )
        self.assertAlmostEqual(displaced.normal_error_m, 0.01, places=9)
        self.assertAlmostEqual(
            displaced.tracking_error_m, abs(displaced.normal_error_m), places=12
        )

    def test_the_binormal_channel_reacts_to_out_of_plane_drift(self) -> None:
        layout = inclined_layout(0.0)
        controller = reaction_for(layout)
        reaction = controller.evaluate(
            attached_state(layout, s_m=100.0, speed_mps=0.0, binormal_offset_m=0.02), {}
        )
        self.assertAlmostEqual(abs(reaction.binormal_error_m), 0.02, places=9)
        self.assertAlmostEqual(
            abs(reaction.force_n[1]),
            ASSEMBLY_MASS_KG * PathControllerGains().normal_kp_per_s2 * 0.02,
            places=6,
        )

    def test_after_release_only_the_cart_mass_is_supported(self) -> None:
        layout = inclined_layout(0.0)
        controller = reaction_for(layout)
        released = controller.evaluate(
            attached_state(layout, s_m=100.0, speed_mps=300.0, coupled=False), {}
        )
        self.assertFalse(released.coupled)
        self.assertAlmostEqual(released.force_n[2], CART_MASS_KG * 9.81, places=6)

    def test_attitude_feedback_opposes_a_pitch_error(self) -> None:
        layout = inclined_layout(0.0)
        controller = reaction_for(layout)
        level = controller.evaluate(attached_state(layout, s_m=100.0, speed_mps=0.0), {})
        self.assertAlmostEqual(level.attitude_error_rad, 0.0, places=12)
        self.assertAlmostEqual(level.torque_nm[1], 0.0, places=9)

        pitched = attached_state(layout, s_m=100.0, speed_mps=0.0)
        nose_up = BodyState(
            name=BODY_CART,
            position=pitched.body(BODY_CART).position,
            orientation=orientation_for(1.0),
            linear_velocity=pitched.body(BODY_CART).linear_velocity,
            mass_kg=CART_MASS_KG,
        )
        state = SimulationState(
            time_s=0.0,
            step_index=0,
            dt_s=0.001,
            bodies={BODY_CART: nose_up, BODY_ROCKET: pitched.body(BODY_ROCKET)},
            joint_active={JOINT_COUPLING: True},
        ).frozen()
        reaction = controller.evaluate(state, {})
        self.assertAlmostEqual(math.degrees(reaction.attitude_error_rad), -1.0, places=9)
        # A nose-up error commands a nose-down torque about +Y.
        self.assertGreater(reaction.torque_nm[1], 0.0)

    def test_the_lever_arm_to_an_offset_cart_becomes_a_reported_torque(self) -> None:
        layout = inclined_layout(0.0)
        controller = reaction_for(layout)
        pose = path_pose(layout, 100.0)
        cart_position = sub(pose.position_m, scale(pose.tangent, 1.2225))
        rocket_position = add(cart_position, scale(pose.tangent, 3.26))
        state = SimulationState(
            time_s=0.0,
            step_index=0,
            dt_s=0.001,
            bodies={
                BODY_CART: BodyState(
                    name=BODY_CART,
                    position=cart_position,
                    orientation=orientation_for(0.0),
                    mass_kg=CART_MASS_KG,
                ),
                BODY_ROCKET: BodyState(
                    name=BODY_ROCKET,
                    position=rocket_position,
                    orientation=orientation_for(0.0),
                    mass_kg=ROCKET_MASS_KG,
                ),
            },
            joint_active={JOINT_COUPLING: True},
        ).frozen()
        reaction = controller.evaluate(state, {})
        lever = sub(reaction.application_point_m, cart_position)
        expected = cross(lever, reaction.force_n)
        # The pitch channel also contributes, so only the lever-induced components of the
        # other two axes are checked exactly.
        self.assertAlmostEqual(reaction.torque_nm[0], expected[0], places=6)
        self.assertAlmostEqual(reaction.torque_nm[2], expected[2], places=6)
        self.assertAlmostEqual(norm(lever), 1.2225, places=6)

    def test_curved_reaction_supplies_the_centripetal_force_of_the_resolved_centerline(
        self,
    ) -> None:
        config = load_yaml(CURVED).config
        layout = resolve_tube_layout(config)
        controller = ForceResolvedPathReaction(
            layout,
            coupled_pitch_inertia_kg_m2=1356.402410888672,
            cart_pitch_inertia_kg_m2=180.0,
            gravity_mps2=GRAVITY,
        )
        s_m = 0.5 * layout.length_m
        pose = path_pose(layout, s_m)
        self.assertNotAlmostEqual(pose.signed_curvature_per_m, 0.0, places=12)
        state = attached_state(layout, s_m=s_m, speed_mps=1400.0)  # type: ignore[arg-type]
        reaction = controller.evaluate(state, {})
        expected = ASSEMBLY_MASS_KG * (
            1400.0**2 * pose.signed_curvature_per_m - dot(GRAVITY, pose.normal)
        )
        self.assertAlmostEqual(reaction.ideal_normal_force_n, expected, places=3)
        self.assertAlmostEqual(dot(reaction.force_n, pose.normal), expected, places=3)

    def test_a_massless_assembly_is_rejected_rather_than_dividing_by_zero(self) -> None:
        layout = inclined_layout(0.0)
        state = SimulationState(
            time_s=0.0,
            step_index=0,
            dt_s=0.001,
            bodies={
                BODY_CART: BodyState(name=BODY_CART, mass_kg=0.0),
            },
            joint_active={JOINT_COUPLING: False},
        ).frozen()
        with self.assertRaisesRegex(ValueError, "mass must be positive"):
            reaction_for(layout).evaluate(state, {})

    def test_inertia_inputs_must_be_finite_and_positive(self) -> None:
        layout = inclined_layout(0.0)
        for coupled, cart in ((0.0, 1.0), (1.0, 0.0), (float("nan"), 1.0)):
            with self.subTest(coupled=coupled, cart=cart):
                with self.assertRaisesRegex(ValueError, "pitch inertia"):
                    ForceResolvedPathReaction(
                        layout,
                        coupled_pitch_inertia_kg_m2=coupled,
                        cart_pitch_inertia_kg_m2=cart,
                    )


if __name__ == "__main__":
    unittest.main()
