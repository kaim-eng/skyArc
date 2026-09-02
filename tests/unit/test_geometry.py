# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
import unittest

import _bootstrap  # noqa: F401

from skyarc.launcher.geometry import (
    CircularCenterlineSegment,
    ClothoidCenterlineSegment,
    CurvedTubeLayout,
    PlanarCenterline,
    RigidBodyEnvelope,
    StraightCenterlineSegment,
    TubeLayout,
    TubeStage,
    axial_position,
    cumulative_boundaries,
    effective_density_ratio,
    enumerate_boundary_crossings,
    gravity_projection,
    guide_normal_bound_mps2,
    normal_jerk_mps3,
    refine_density_stages,
    stage_index,
    swept_envelope_clearance,
    tube_axis,
    world_position,
)


class TubeGeometryTests(unittest.TestCase):
    def reference_centerline(self) -> PlanarCenterline:
        return PlanarCenterline(
            origin_m=(0.0, 0.0, 0.0),
            initial_angle_deg=45.0,
            segments=(
                StraightCenterlineSegment(20000.0),
                ClothoidCenterlineSegment(2700.0, 0.0, -1.0 / 60000.0),
                CircularCenterlineSegment(60000.0, -27.42169),
                ClothoidCenterlineSegment(2700.0, -1.0 / 60000.0, 0.0),
            ),
        )

    def test_axis_and_projection_at_several_angles(self) -> None:
        self.assertEqual(tube_axis(0.0), (1.0, 0.0, 0.0))
        self.assertAlmostEqual(tube_axis(90.0)[2], 1.0)
        axis = tube_axis(45.0)
        self.assertAlmostEqual(axis[0], math.sqrt(0.5))
        self.assertAlmostEqual(axis[2], math.sqrt(0.5))

        origin = (7.0, -2.0, 3.0)
        point = world_position(12.5, origin, axis)
        self.assertAlmostEqual(axial_position(point, origin, axis), 12.5)

    def test_cumulative_boundaries_and_stage_lookup_are_half_open(self) -> None:
        boundaries = cumulative_boundaries((2.0, 3.0, 5.0))
        self.assertEqual(boundaries, (2.0, 5.0, 10.0))
        expected = {
            -0.01: None,
            0.0: 0,
            1.999: 0,
            2.0: 1,
            4.999: 1,
            5.0: 2,
            9.999: 2,
            10.0: None,
        }
        for position, index in expected.items():
            with self.subTest(position=position):
                self.assertEqual(stage_index(position, boundaries), index)

    def test_stage_lookup_supports_arbitrary_stage_counts(self) -> None:
        for count in (1, 3, 5, 9):
            boundaries = cumulative_boundaries([1.0] * count)
            for index in range(count):
                self.assertEqual(stage_index(index + 0.5, boundaries), index)

    def test_multi_boundary_crossing_is_ordered_and_interpolated(self) -> None:
        crossings = enumerate_boundary_crossings(
            0.5,
            7.5,
            (1.0, 2.0, 5.0, 8.0),
            pre_time_s=10.0,
            post_time_s=17.0,
        )
        self.assertEqual([item.boundary_s_m for item in crossings], [1.0, 2.0, 5.0])
        self.assertEqual([item.boundary_index for item in crossings], [0, 1, 2])
        self.assertEqual([item.time_s for item in crossings], [10.5, 11.5, 14.5])
        self.assertEqual(crossings[0].from_stage_index, 0)
        self.assertEqual(crossings[0].to_stage_index, 1)

    def test_reverse_crossings_follow_travel_order(self) -> None:
        crossings = enumerate_boundary_crossings(
            8.5,
            0.5,
            (1.0, 2.0, 5.0, 8.0),
            pre_time_s=2.0,
            post_time_s=10.0,
        )
        self.assertEqual([item.boundary_s_m for item in crossings], [8.0, 5.0, 2.0, 1.0])
        self.assertTrue(all(item.direction == -1 for item in crossings))
        self.assertEqual(crossings[0].from_stage_index, None)
        self.assertEqual(crossings[0].to_stage_index, 3)
        self.assertEqual(sorted(item.time_s for item in crossings), [2.5, 5.5, 8.5, 9.5])

    def test_shared_step_endpoint_does_not_duplicate_crossing(self) -> None:
        first = enumerate_boundary_crossings(
            0.0, 2.0, (2.0, 4.0), pre_time_s=0.0, post_time_s=1.0
        )
        second = enumerate_boundary_crossings(
            2.0, 3.0, (2.0, 4.0), pre_time_s=1.0, post_time_s=2.0
        )
        self.assertEqual(len(first), 1)
        self.assertEqual(second, ())

    def test_density_profile_and_centered_blend(self) -> None:
        stages = (
            TubeStage("vacuum", 10.0, 0.0),
            TubeStage("dense", 10.0, 1.0),
        )
        self.assertEqual(effective_density_ratio(5.0, stages, exterior_ratio=0.8), 0.0)
        self.assertEqual(effective_density_ratio(15.0, stages, exterior_ratio=0.8), 1.0)
        self.assertEqual(effective_density_ratio(25.0, stages, exterior_ratio=0.8), 0.8)
        self.assertAlmostEqual(
            effective_density_ratio(10.0, stages, exterior_ratio=0.8, blend_distance_m=2.0),
            0.5,
        )
        self.assertAlmostEqual(
            effective_density_ratio(0.0, stages, exterior_ratio=0.8, blend_distance_m=2.0),
            0.4,
        )

    def test_layout_validation_and_gravity_projection(self) -> None:
        layout = TubeLayout(
            origin_m=(0.0, 0.0, 0.0),
            angle_deg=30.0,
            stages=(TubeStage("one", 4.0, 0.5),),
            boundary_blend_distance_m=1.0,
        )
        self.assertAlmostEqual(layout.length_m, 4.0)
        self.assertAlmostEqual(gravity_projection((0.0, 0.0, -9.81), layout.axis), -4.905)
        with self.assertRaises(ValueError):
            TubeLayout(
                origin_m=(0.0, 0.0, 0.0),
                angle_deg=0.0,
                stages=(TubeStage("short", 1.0, 0.0),),
                boundary_blend_distance_m=1.1,
            )

    def test_reference_curved_centerline_matches_review_geometry(self) -> None:
        centerline = self.reference_centerline()
        exit_pose = centerline.exit_pose
        self.assertAlmostEqual(centerline.length_m, 54115.9266, places=3)
        self.assertAlmostEqual(exit_pose.position_m[0], 43300.225, places=2)
        self.assertAlmostEqual(exit_pose.position_m[2], 30976.566, places=2)
        self.assertAlmostEqual(exit_pose.inclination_deg, 15.0, places=5)
        self.assertAlmostEqual(exit_pose.signed_curvature_per_m, 0.0, places=12)
        self.assertEqual(exit_pose.segment_index, 3)

    def test_exact_accumulated_endpoint_is_roundoff_safe(self) -> None:
        centerline = PlanarCenterline(
            origin_m=(0.0, 0.0, 0.0),
            initial_angle_deg=0.0,
            segments=(
                StraightCenterlineSegment(1.0),
                StraightCenterlineSegment(0.1),
            ),
        )
        self.assertEqual(centerline.length_m, 1.1)
        self.assertAlmostEqual(centerline.exit_pose.position_m[0], 1.1)
        report = swept_envelope_clearance(
            centerline,
            tube_inner_diameter_m=2.0,
            guide_clearance_m=0.0,
            bodies=(RigidBodyEnvelope("small", 0.1, 0.1),),
        )
        self.assertAlmostEqual(report.minimum_vehicle_wall_clearance_m, 0.9)

    def test_curvature_and_frame_are_continuous_at_reference_joins(self) -> None:
        centerline = self.reference_centerline()
        joins = (20000.0, 22700.0, centerline.length_m - 2700.0)
        for join in joins:
            before = centerline.pose(join - 1e-4)
            at_join = centerline.pose(join)
            after = centerline.pose(join + 1e-4)
            with self.subTest(join=join):
                self.assertAlmostEqual(before.position_m[0], at_join.position_m[0], places=3)
                self.assertAlmostEqual(after.position_m[2], at_join.position_m[2], places=3)
                self.assertAlmostEqual(before.tangent[0], after.tangent[0], places=8)
                self.assertAlmostEqual(
                    before.signed_curvature_per_m,
                    after.signed_curvature_per_m,
                    places=9,
                )

    def test_centerline_projection_recovers_arc_length(self) -> None:
        centerline = self.reference_centerline()
        expected_s = 40000.0
        pose = centerline.pose(expected_s)
        displaced = (
            pose.position_m[0] + 0.25 * pose.normal[0],
            3.0,
            pose.position_m[2] + 0.25 * pose.normal[2],
        )
        self.assertAlmostEqual(centerline.nearest_s(displaced), expected_s, places=3)

    def test_projection_extends_past_both_endpoints(self) -> None:
        # The cart continues onto the straight exit track after release and its braking
        # feedback is driven by how far past the exit plane it has travelled. A projection
        # clamped to [0, L] reports zero progress forever and freezes that feedback.
        centerline = self.reference_centerline()
        length = centerline.length_m
        for offset in (1.0, 1000.0, 23000.0):
            with self.subTest(offset=offset):
                beyond = centerline.pose(length)
                point = tuple(
                    beyond.position_m[index] + offset * beyond.tangent[index]
                    for index in range(3)
                )
                self.assertAlmostEqual(centerline.nearest_s(point), length + offset, places=3)

                before = centerline.pose(0.0)
                point = tuple(
                    before.position_m[index] - offset * before.tangent[index]
                    for index in range(3)
                )
                self.assertAlmostEqual(centerline.nearest_s(point), -offset, places=3)

    def test_projection_inverts_path_pose_outside_the_authored_interval(self) -> None:
        from skyarc.launcher.geometry import CurvedTubeLayout, path_pose

        centerline = self.reference_centerline()
        layout = CurvedTubeLayout(
            centerline=centerline,
            stages=(TubeStage("only", centerline.length_m, 0.0),),
            exterior_effective_density_ratio=0.015,
        )
        for s in (-500.0, 0.0, 0.5 * centerline.length_m, centerline.length_m, 12000.0 + centerline.length_m):
            with self.subTest(s=s):
                self.assertAlmostEqual(
                    layout.axial_position(path_pose(layout, s).position_m), s, places=3
                )

    def test_signed_normal_jerk_and_curvature_discontinuity(self) -> None:
        jerk = normal_jerk_mps3(2000.0, 0.0, 0.0, 1.0 / (60000.0 * 2700.0))
        self.assertAlmostEqual(jerk, 49.382716, places=5)
        with self.assertRaisesRegex(ValueError, "curvature is discontinuous"):
            PlanarCenterline(
                origin_m=(0.0, 0.0, 0.0),
                initial_angle_deg=45.0,
                segments=(
                    StraightCenterlineSegment(10.0),
                    CircularCenterlineSegment(60000.0, -1.0),
                ),
            )

    def test_guide_normal_bound_covers_straight_and_curved_segments(self) -> None:
        straight_normal = (-math.sqrt(0.5), 0.0, math.sqrt(0.5))
        self.assertAlmostEqual(
            guide_normal_bound_mps2(
                1000.0, 0.0, (0.0, 0.0, -9.81), straight_normal
            ),
            9.81 * math.sqrt(0.5),
        )
        self.assertAlmostEqual(
            guide_normal_bound_mps2(
                2000.0,
                -1.0 / 60000.0,
                (0.0, 0.0, -9.81),
                (0.0, 0.0, 1.0),
            ),
            2000.0**2 / 60000.0,
        )

    def test_reference_swept_envelope_has_vehicle_and_global_clearance(self) -> None:
        report = swept_envelope_clearance(
            self.reference_centerline(),
            tube_inner_diameter_m=2.0,
            guide_clearance_m=0.05,
            bodies=(
                RigidBodyEnvelope("cart", 1.25, math.hypot(0.6, 0.2)),
                RigidBodyEnvelope("rocket", 2.0, 0.25),
            ),
        )
        self.assertEqual(report.limiting_body, "cart")
        self.assertAlmostEqual(report.maximum_absolute_curvature_per_m, 1.0 / 60000.0)
        self.assertAlmostEqual(
            report.minimum_vehicle_wall_clearance_m,
            1.0 - 0.05 - math.hypot(0.6, 0.2) - 0.5 * (1.0 / 60000.0) * 1.25**2,
        )
        self.assertLess(report.polyline_chord_error_bound_m, 0.01)

    def test_swept_envelope_index_scales_with_chord_not_tube_diameter(self) -> None:
        # The former tube-diameter-sized AABB grid tried to allocate tens of millions
        # of cells for this single diagonal chord even though there is nothing to compare.
        centerline = PlanarCenterline(
            origin_m=(0.0, 0.0, 0.0),
            initial_angle_deg=45.0,
            segments=(StraightCenterlineSegment(25.0),),
        )
        report = swept_envelope_clearance(
            centerline,
            tube_inner_diameter_m=0.004,
            guide_clearance_m=0.0,
            bodies=(RigidBodyEnvelope("millimetre_body", 0.001, 0.0005),),
            maximum_polyline_spacing_m=25.0,
        )
        self.assertAlmostEqual(report.minimum_vehicle_wall_clearance_m, 0.0015)

    def test_swept_envelope_rejects_long_body_local_singularity_and_global_overlap(self) -> None:
        curved = PlanarCenterline(
            origin_m=(0.0, 0.0, 0.0),
            initial_angle_deg=0.0,
            segments=(ClothoidCenterlineSegment(1.0, 0.0, 0.1),),
        )
        with self.assertRaisesRegex(ValueError, "long.*requires radius"):
            swept_envelope_clearance(
                curved,
                tube_inner_diameter_m=2.0,
                guide_clearance_m=0.0,
                bodies=(RigidBodyEnvelope("long", 5.0, 0.1),),
            )
        singular = PlanarCenterline(
            origin_m=(0.0, 0.0, 0.0),
            initial_angle_deg=0.0,
            segments=(ClothoidCenterlineSegment(1.0, 0.0, 1.0),),
        )
        with self.assertRaisesRegex(ValueError, "local radius of curvature"):
            swept_envelope_clearance(
                singular,
                tube_inner_diameter_m=2.0,
                guide_clearance_m=0.0,
                bodies=(RigidBodyEnvelope("small", 0.1, 0.1),),
            )

        transition_length = 1.0
        curvature = -0.01
        circular_turn_rad = -1.5 * math.pi - curvature * transition_length
        crossing = PlanarCenterline(
            origin_m=(0.0, 0.0, 0.0),
            initial_angle_deg=0.0,
            segments=(
                StraightCenterlineSegment(100.0),
                ClothoidCenterlineSegment(transition_length, 0.0, curvature),
                CircularCenterlineSegment(100.0, math.degrees(circular_turn_rad)),
                ClothoidCenterlineSegment(transition_length, curvature, 0.0),
                StraightCenterlineSegment(200.0),
            ),
        )
        with self.assertRaisesRegex(ValueError, "nonlocal centerline branches"):
            swept_envelope_clearance(
                crossing,
                tube_inner_diameter_m=2.0,
                guide_clearance_m=0.0,
                bodies=(RigidBodyEnvelope("small", 0.1, 0.1),),
                maximum_polyline_spacing_m=0.5,
            )

    def test_curved_layout_uses_arc_length_for_stages_and_density(self) -> None:
        centerline = self.reference_centerline()
        stages = (
            TubeStage("vacuum", 50000.0, 0.0),
            TubeStage("transition_1", 1500.0, 0.003),
            TubeStage("transition_2", 1300.0, 0.008),
            TubeStage("exit", centerline.length_m - 52800.0, 0.015),
        )
        layout = CurvedTubeLayout(
            centerline=centerline,
            stages=stages,
            exterior_effective_density_ratio=0.015,
            boundary_blend_distance_m=40.0,
        )
        self.assertEqual(layout.stage_index(50500.0), 1)
        self.assertEqual(layout.stage_index(52000.0), 2)
        self.assertEqual(layout.stage_index(54000.0), 3)
        self.assertEqual(layout.density_ratio(50000.0), 0.0015)
        pose = layout.pose(52000.0)
        self.assertAlmostEqual(layout.axial_position(pose.position_m), 52000.0, places=3)

    def test_density_stage_refinement_preserves_length_and_samples_target(self) -> None:
        stages = (
            TubeStage("vacuum", 10.0, 0.0),
            TubeStage("exit", 10.0, 1.0),
        )
        refined = refine_density_stages(stages, 2, entrance_ratio=0.0, exit_ratio=1.0)
        self.assertEqual(len(refined), 4)
        self.assertAlmostEqual(sum(stage.length_m for stage in refined), 20.0)
        self.assertEqual([stage.name for stage in refined], [
            "vacuum.r1", "vacuum.r2", "exit.r1", "exit.r2"
        ])
        self.assertLess(refined[0].effective_density_ratio, refined[-1].effective_density_ratio)


if __name__ == "__main__":
    unittest.main()
