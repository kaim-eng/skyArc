# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tube-axis, stage, boundary-crossing, density, and gravity calculations."""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple

from ..linalg import Vec3, add, dot, is_finite, scale, sub


@dataclass(frozen=True)
class TubeStage:
    """One ordered virtual effective-density region."""

    name: str
    length_m: float
    effective_density_ratio: float

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("stage name may not be empty")
        if not math.isfinite(self.length_m) or self.length_m <= 0.0:
            raise ValueError(f"stage {self.name!r} must have a finite positive length")
        if not math.isfinite(self.effective_density_ratio) or self.effective_density_ratio < 0.0:
            raise ValueError(f"stage {self.name!r} must have a finite nonnegative density ratio")


@dataclass(frozen=True)
class StraightCenterlineSegment:
    length_m: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.length_m) or self.length_m <= 0.0:
            raise ValueError("straight centerline length must be finite and positive")


@dataclass(frozen=True)
class ClothoidCenterlineSegment:
    length_m: float
    start_curvature_per_m: float
    end_curvature_per_m: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.length_m) or self.length_m <= 0.0:
            raise ValueError("clothoid centerline length must be finite and positive")
        if not is_finite((self.start_curvature_per_m, self.end_curvature_per_m)):
            raise ValueError("clothoid curvatures must be finite")


@dataclass(frozen=True)
class CircularCenterlineSegment:
    radius_m: float
    signed_turn_deg: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.radius_m) or self.radius_m <= 0.0:
            raise ValueError("circular centerline radius must be finite and positive")
        if not math.isfinite(self.signed_turn_deg) or self.signed_turn_deg == 0.0:
            raise ValueError("circular centerline turn must be finite and nonzero")

    @property
    def length_m(self) -> float:
        return abs(math.radians(self.signed_turn_deg)) * self.radius_m

    @property
    def signed_curvature_per_m(self) -> float:
        return math.copysign(1.0 / self.radius_m, self.signed_turn_deg)


PlanarCenterlineSegment = StraightCenterlineSegment | ClothoidCenterlineSegment | CircularCenterlineSegment


@dataclass(frozen=True)
class CenterlinePose:
    s_m: float
    position_m: Vec3
    tangent: Vec3
    normal: Vec3
    inclination_deg: float
    signed_curvature_per_m: float
    curvature_rate_per_m2: float
    segment_index: int


@dataclass(frozen=True)
class _ResolvedCenterlineSegment:
    definition: PlanarCenterlineSegment
    start_s_m: float
    start_position_m: Vec3
    start_angle_rad: float
    end_s_m: float
    end_position_m: Vec3
    end_angle_rad: float


def _simpson_integral(function, upper: float) -> float:  # type: ignore[no-untyped-def]
    """Deterministic adaptive Simpson integration for clothoid displacement."""
    if upper == 0.0:
        return 0.0
    lower = 0.0
    midpoint = 0.5 * upper
    f_lower = function(lower)
    f_midpoint = function(midpoint)
    f_upper = function(upper)
    whole = upper * (f_lower + 4.0 * f_midpoint + f_upper) / 6.0
    tolerance = max(1e-10, abs(upper) * 1e-12)

    def refine(a, b, fa, fm, fb, estimate, depth):  # type: ignore[no-untyped-def]
        middle = 0.5 * (a + b)
        left_middle = 0.5 * (a + middle)
        right_middle = 0.5 * (middle + b)
        f_left_middle = function(left_middle)
        f_right_middle = function(right_middle)
        left = (middle - a) * (fa + 4.0 * f_left_middle + fm) / 6.0
        right = (b - middle) * (fm + 4.0 * f_right_middle + fb) / 6.0
        correction = left + right - estimate
        if depth <= 0 or abs(correction) <= 15.0 * tolerance:
            return left + right + correction / 15.0
        return refine(a, middle, fa, f_left_middle, fm, left, depth - 1) + refine(
            middle, b, fm, f_right_middle, fb, right, depth - 1
        )

    return refine(lower, upper, f_lower, f_midpoint, f_upper, whole, 16)


def _segment_state(
    definition: PlanarCenterlineSegment,
    distance_m: float,
    start_position_m: Vec3,
    start_angle_rad: float,
) -> tuple[Vec3, float, float, float]:
    """Return position, angle, signed curvature, and curvature rate within a segment."""
    length_m = definition.length_m
    if not math.isfinite(distance_m) or not 0.0 <= distance_m <= length_m:
        raise ValueError("segment distance must be finite and inside the segment")
    if isinstance(definition, StraightCenterlineSegment):
        position = (
            start_position_m[0] + distance_m * math.cos(start_angle_rad),
            start_position_m[1],
            start_position_m[2] + distance_m * math.sin(start_angle_rad),
        )
        return position, start_angle_rad, 0.0, 0.0
    if isinstance(definition, CircularCenterlineSegment):
        curvature = definition.signed_curvature_per_m
        angle = start_angle_rad + curvature * distance_m
        position = (
            start_position_m[0] + (math.sin(angle) - math.sin(start_angle_rad)) / curvature,
            start_position_m[1],
            start_position_m[2] + (math.cos(start_angle_rad) - math.cos(angle)) / curvature,
        )
        return position, angle, curvature, 0.0

    curvature_rate = (
        definition.end_curvature_per_m - definition.start_curvature_per_m
    ) / definition.length_m

    def angle_at(value: float) -> float:
        return (
            start_angle_rad
            + definition.start_curvature_per_m * value
            + 0.5 * curvature_rate * value * value
        )

    dx = _simpson_integral(lambda value: math.cos(angle_at(value)), distance_m)
    dz = _simpson_integral(lambda value: math.sin(angle_at(value)), distance_m)
    angle = angle_at(distance_m)
    curvature = definition.start_curvature_per_m + curvature_rate * distance_m
    return (
        (start_position_m[0] + dx, start_position_m[1], start_position_m[2] + dz),
        angle,
        curvature,
        curvature_rate,
    )


class PlanarCenterline:
    """Arc-length planar centerline with the signed-curvature convention from design v0.8."""

    def __init__(
        self,
        *,
        origin_m: Vec3,
        initial_angle_deg: float,
        segments: Sequence[PlanarCenterlineSegment],
        curvature_tolerance_per_m: float = 1e-12,
    ) -> None:
        if len(origin_m) != 3 or not is_finite(origin_m):
            raise ValueError("centerline origin must be a finite three-vector")
        if not math.isfinite(initial_angle_deg):
            raise ValueError("centerline initial angle must be finite")
        if not segments:
            raise ValueError("centerline must contain at least one segment")
        if not math.isfinite(curvature_tolerance_per_m) or curvature_tolerance_per_m < 0.0:
            raise ValueError("curvature tolerance must be finite and nonnegative")

        resolved = []
        start_s = 0.0
        start_position = tuple(float(value) for value in origin_m)
        start_angle = math.radians(initial_angle_deg)
        previous_end_curvature = 0.0
        for index, definition in enumerate(segments):
            if not isinstance(
                definition,
                (StraightCenterlineSegment, ClothoidCenterlineSegment, CircularCenterlineSegment),
            ):
                raise ValueError(f"unsupported centerline segment at index {index}")
            if isinstance(definition, StraightCenterlineSegment):
                start_curvature = end_curvature = 0.0
            elif isinstance(definition, ClothoidCenterlineSegment):
                start_curvature = definition.start_curvature_per_m
                end_curvature = definition.end_curvature_per_m
            else:
                start_curvature = end_curvature = definition.signed_curvature_per_m
            if not math.isclose(
                start_curvature,
                previous_end_curvature,
                rel_tol=0.0,
                abs_tol=curvature_tolerance_per_m,
            ):
                raise ValueError(
                    f"centerline curvature is discontinuous at segment {index}: "
                    f"{previous_end_curvature} -> {start_curvature} per metre"
                )
            end_position, end_angle, _, _ = _segment_state(
                definition, definition.length_m, start_position, start_angle
            )
            end_s = start_s + definition.length_m
            resolved.append(
                _ResolvedCenterlineSegment(
                    definition=definition,
                    start_s_m=start_s,
                    start_position_m=start_position,
                    start_angle_rad=start_angle,
                    end_s_m=end_s,
                    end_position_m=end_position,
                    end_angle_rad=end_angle,
                )
            )
            start_s = end_s
            start_position = end_position
            start_angle = end_angle
            previous_end_curvature = end_curvature
        self._origin_m = tuple(float(value) for value in origin_m)
        self._initial_angle_deg = float(initial_angle_deg)
        self._segments = tuple(resolved)
        self._ends_m = tuple(segment.end_s_m for segment in resolved)

    @property
    def length_m(self) -> float:
        return self._ends_m[-1]

    @property
    def segment_count(self) -> int:
        return len(self._segments)

    @property
    def maximum_absolute_curvature_per_m(self) -> float:
        values = []
        for resolved in self._segments:
            definition = resolved.definition
            if isinstance(definition, StraightCenterlineSegment):
                values.append(0.0)
            elif isinstance(definition, CircularCenterlineSegment):
                values.append(abs(definition.signed_curvature_per_m))
            else:
                values.append(
                    max(
                        abs(definition.start_curvature_per_m),
                        abs(definition.end_curvature_per_m),
                    )
                )
        return max(values)

    @property
    def minimum_downrange_tangent_component(self) -> float:
        """Return the exact minimum tangent-X component over every authored segment.

        The tangent angle is constant on a straight, linear on a circular arc, and
        quadratic on a clothoid.  A clothoid's angle extrema can therefore occur only
        at its endpoints or where its linearly varying curvature is zero.  The image
        of each segment's continuous angle function is the interval between those
        extrema, so checking that interval for a cosine minimum also catches arbitrarily
        narrow reversals that a fixed-count pose sample can miss.
        """

        minimum_component = 1.0
        for resolved in self._segments:
            angles = [resolved.start_angle_rad, resolved.end_angle_rad]
            definition = resolved.definition
            if isinstance(definition, ClothoidCenterlineSegment):
                curvature_rate = (
                    definition.end_curvature_per_m - definition.start_curvature_per_m
                ) / definition.length_m
                if curvature_rate != 0.0:
                    stationary_s = -definition.start_curvature_per_m / curvature_rate
                    if 0.0 < stationary_s < definition.length_m:
                        angles.append(
                            resolved.start_angle_rad
                            + definition.start_curvature_per_m * stationary_s
                            + 0.5 * curvature_rate * stationary_s**2
                        )
            lower_angle = min(angles)
            upper_angle = max(angles)
            # cos(theta) reaches -1 at pi + 2*pi*k.  If no such point is in
            # the angle interval, its minimum is attained at an endpoint.
            first_cosine_minimum = math.pi + 2.0 * math.pi * math.ceil(
                (lower_angle - math.pi) / (2.0 * math.pi)
            )
            if first_cosine_minimum <= upper_angle:
                segment_minimum = -1.0
            else:
                segment_minimum = min(math.cos(lower_angle), math.cos(upper_angle))
            minimum_component = min(minimum_component, segment_minimum)
        return minimum_component

    @property
    def exit_pose(self) -> CenterlinePose:
        return self.pose(self.length_m)

    def pose(self, s_m: float) -> CenterlinePose:
        if not math.isfinite(s_m) or not 0.0 <= s_m <= self.length_m:
            raise ValueError(f"centerline coordinate must be in [0, {self.length_m}]")
        index = min(bisect.bisect_right(self._ends_m, s_m), len(self._segments) - 1)
        resolved = self._segments[index]
        # The global range and bisection checks establish that this coordinate belongs
        # to the selected segment.  Clamp the subtraction result because accumulated
        # binary floats can otherwise turn an exact global endpoint (for example
        # 1.0 + 0.1) into a local value infinitesimally greater than the authored 0.1 m.
        local_s = min(
            resolved.definition.length_m,
            max(0.0, s_m - resolved.start_s_m),
        )
        position, angle, curvature, curvature_rate = _segment_state(
            resolved.definition,
            local_s,
            resolved.start_position_m,
            resolved.start_angle_rad,
        )
        tangent = (math.cos(angle), 0.0, math.sin(angle))
        normal = (-math.sin(angle), 0.0, math.cos(angle))
        return CenterlinePose(
            s_m=float(s_m),
            position_m=position,
            tangent=tangent,
            normal=normal,
            inclination_deg=math.degrees(angle),
            signed_curvature_per_m=curvature,
            curvature_rate_per_m2=curvature_rate,
            segment_index=index,
        )

    def sample(self, maximum_spacing_m: float) -> Tuple[CenterlinePose, ...]:
        if not math.isfinite(maximum_spacing_m) or maximum_spacing_m <= 0.0:
            raise ValueError("sample spacing must be finite and positive")
        count = max(1, math.ceil(self.length_m / maximum_spacing_m))
        return tuple(self.pose(self.length_m * index / count) for index in range(count + 1))

    def nearest_s(self, position_m: Vec3) -> float:
        """Project a point to arc length, extending past either endpoint along its tangent.

        Coordinates outside ``[0, L]`` are meaningful and have to be reported rather than
        clamped. The cart continues onto the straight exit track after release, and its
        braking feedback is driven by how far past the exit plane it has travelled; a clamped
        projection reports zero progress forever and silently freezes that feedback.
        :func:`path_pose` extrapolates along the same endpoint tangents, so the two remain
        inverses outside the authored interval as well as inside it.

        The endpoint branches assume the point lies within the validated guide envelope,
        which is the condition section 7 already places on this mapping.
        """
        if len(position_m) != 3 or not is_finite(position_m):
            raise ValueError("projection position must be a finite three-vector")
        start = self.pose(0.0)
        before_entrance = dot(sub(position_m, start.position_m), start.tangent)
        if before_entrance < 0.0:
            return before_entrance
        end = self.pose(self.length_m)
        beyond_exit = dot(sub(position_m, end.position_m), end.tangent)
        if beyond_exit > 0.0:
            return self.length_m + beyond_exit
        sample_count = max(64, 32 * len(self._segments))
        samples = [self.length_m * index / sample_count for index in range(sample_count + 1)]

        def distance_squared(s_value: float) -> float:
            point = self.pose(s_value).position_m
            delta = sub(point, position_m)
            return dot(delta, delta)

        best_index = min(range(len(samples)), key=lambda item: distance_squared(samples[item]))
        lower = samples[max(0, best_index - 1)]
        upper = samples[min(sample_count, best_index + 1)]
        ratio = 0.5 * (math.sqrt(5.0) - 1.0)
        left = upper - ratio * (upper - lower)
        right = lower + ratio * (upper - lower)
        left_value = distance_squared(left)
        right_value = distance_squared(right)
        for _ in range(48):
            if left_value <= right_value:
                upper, right, right_value = right, left, left_value
                left = upper - ratio * (upper - lower)
                left_value = distance_squared(left)
            else:
                lower, left, left_value = left, right, right_value
                right = lower + ratio * (upper - lower)
                right_value = distance_squared(right)
        return 0.5 * (lower + upper)


@dataclass(frozen=True)
class RigidBodyEnvelope:
    """Tangent-aligned rigid envelope used by the curved-tube clearance proof."""

    name: str
    half_length_m: float
    cross_section_radius_m: float

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("rigid envelope name may not be empty")
        for label, value in (
            ("half length", self.half_length_m),
            ("cross-section radius", self.cross_section_radius_m),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"rigid envelope {label} must be finite and nonnegative")


@dataclass(frozen=True)
class BodyClearanceReport:
    name: str
    cross_section_radius_m: float
    curvature_allowance_m: float
    required_radial_envelope_m: float
    minimum_wall_clearance_m: float


@dataclass(frozen=True)
class SweptEnvelopeReport:
    """Conservative tube and tangent-aligned vehicle clearance certificate."""

    tube_radius_m: float
    guide_clearance_m: float
    maximum_absolute_curvature_per_m: float
    local_curvature_margin_m: float | None
    maximum_polyline_spacing_m: float
    polyline_chord_error_bound_m: float
    required_nonlocal_centerline_separation_m: float
    body_reports: Tuple[BodyClearanceReport, ...]
    limiting_body: str
    minimum_vehicle_wall_clearance_m: float


def _cross_2d(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _point_segment_distance_2d(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    dx = end[0] - start[0]
    dz = end[1] - start[1]
    length_squared = dx * dx + dz * dz
    if length_squared == 0.0:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    fraction = max(
        0.0,
        min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dz) / length_squared),
    )
    nearest = (start[0] + fraction * dx, start[1] + fraction * dz)
    return math.hypot(point[0] - nearest[0], point[1] - nearest[1])


def _segments_intersect_2d(
    first_start: tuple[float, float],
    first_end: tuple[float, float],
    second_start: tuple[float, float],
    second_end: tuple[float, float],
) -> bool:
    tolerance = 1e-12
    orientations = (
        _cross_2d(first_start, first_end, second_start),
        _cross_2d(first_start, first_end, second_end),
        _cross_2d(second_start, second_end, first_start),
        _cross_2d(second_start, second_end, first_end),
    )
    if orientations[0] * orientations[1] < 0.0 and orientations[2] * orientations[3] < 0.0:
        return True

    def on_segment(
        point: tuple[float, float], start: tuple[float, float], end: tuple[float, float]
    ) -> bool:
        return (
            min(start[0], end[0]) - tolerance <= point[0] <= max(start[0], end[0]) + tolerance
            and min(start[1], end[1]) - tolerance <= point[1] <= max(start[1], end[1]) + tolerance
        )

    return any(
        abs(orientation) <= tolerance and on_segment(point, start, end)
        for orientation, point, start, end in (
            (orientations[0], second_start, first_start, first_end),
            (orientations[1], second_end, first_start, first_end),
            (orientations[2], first_start, second_start, second_end),
            (orientations[3], first_end, second_start, second_end),
        )
    )


def _segment_distance_2d(
    first_start: tuple[float, float],
    first_end: tuple[float, float],
    second_start: tuple[float, float],
    second_end: tuple[float, float],
) -> float:
    if _segments_intersect_2d(first_start, first_end, second_start, second_end):
        return 0.0
    return min(
        _point_segment_distance_2d(first_start, second_start, second_end),
        _point_segment_distance_2d(first_end, second_start, second_end),
        _point_segment_distance_2d(second_start, first_start, first_end),
        _point_segment_distance_2d(second_end, first_start, first_end),
    )


def _verify_nonlocal_tube_clearance(
    poses: Sequence[CenterlinePose],
    *,
    tube_radius_m: float,
    chord_error_bound_m: float,
) -> None:
    """Reject nonlocal centerline branches whose swept tube volumes may overlap."""
    required_separation = 2.0 * tube_radius_m
    detection_distance = required_separation + 2.0 * chord_error_bound_m
    maximum_chord_length = max(
        math.hypot(
            poses[index + 1].position_m[0] - poses[index].position_m[0],
            poses[index + 1].position_m[2] - poses[index].position_m[2],
        )
        for index in range(len(poses) - 1)
    )
    # A tube-diameter-sized grid makes a long diagonal chord populate every cell in
    # its two-dimensional AABB: memory then grows quadratically as tube radius shrinks.
    # A cell at least as large as the longest indexed chord keeps each chord AABB in
    # at most four cells while the detection-distance-expanded lookup remains exact.
    cell_size = max(detection_distance, maximum_chord_length, 1e-9)
    local_exclusion_m = math.pi * tube_radius_m
    cells: dict[tuple[int, int], list[int]] = {}
    segments: list[tuple[float, float, tuple[float, float], tuple[float, float]]] = []

    def cell_range(lower: float, upper: float) -> range:
        return range(math.floor(lower / cell_size), math.floor(upper / cell_size) + 1)

    for index in range(len(poses) - 1):
        start_pose = poses[index]
        end_pose = poses[index + 1]
        start = (start_pose.position_m[0], start_pose.position_m[2])
        end = (end_pose.position_m[0], end_pose.position_m[2])
        minimum_x, maximum_x = sorted((start[0], end[0]))
        minimum_z, maximum_z = sorted((start[1], end[1]))
        candidates = set()
        for x_cell in cell_range(minimum_x - detection_distance, maximum_x + detection_distance):
            for z_cell in cell_range(minimum_z - detection_distance, maximum_z + detection_distance):
                candidates.update(cells.get((x_cell, z_cell), ()))
        for candidate in candidates:
            previous_start_s, previous_end_s, previous_start, previous_end = segments[candidate]
            if start_pose.s_m - previous_end_s <= local_exclusion_m:
                continue
            distance = _segment_distance_2d(start, end, previous_start, previous_end)
            if distance <= detection_distance:
                lower_bound = max(0.0, distance - 2.0 * chord_error_bound_m)
                raise ValueError(
                    "nonlocal centerline branches violate tube swept clearance: "
                    f"s={previous_start_s:.6g}..{previous_end_s:.6g} m and "
                    f"s={start_pose.s_m:.6g}..{end_pose.s_m:.6g} m have a conservative "
                    f"separation bound {lower_bound:.6g} m, requiring more than "
                    f"{required_separation:.6g} m"
                )
        segments.append((start_pose.s_m, end_pose.s_m, start, end))
        for x_cell in cell_range(minimum_x, maximum_x):
            for z_cell in cell_range(minimum_z, maximum_z):
                cells.setdefault((x_cell, z_cell), []).append(index)


def swept_envelope_clearance(
    centerline: PlanarCenterline,
    *,
    tube_inner_diameter_m: float,
    guide_clearance_m: float,
    bodies: Sequence[RigidBodyEnvelope],
    maximum_polyline_spacing_m: float = 25.0,
) -> SweptEnvelopeReport:
    """Certify rigid-body and nonlocal tube clearance over a planar centerline.

    For a body point at longitudinal offset ``a``, the deviation between the tangent-line
    point and the centerline point at ``s+a`` is bounded by ``k_max*a^2/2``. Adding that
    to the cross-section radius produces a conservative, sampling-free rigid-envelope
    bound. Nonlocal tube overlap is checked separately with chord-error-padded polyline
    segments in a spatial index; local regularity is guaranteed by ``k_max*R < 1``.
    """
    if not isinstance(centerline, PlanarCenterline):
        raise ValueError("swept-envelope clearance requires a PlanarCenterline")
    if not math.isfinite(tube_inner_diameter_m) or tube_inner_diameter_m <= 0.0:
        raise ValueError("tube inner diameter must be finite and positive")
    if not math.isfinite(guide_clearance_m) or guide_clearance_m < 0.0:
        raise ValueError("guide clearance must be finite and nonnegative")
    if not bodies or any(not isinstance(body, RigidBodyEnvelope) for body in bodies):
        raise ValueError("swept-envelope clearance requires at least one rigid body envelope")
    if not math.isfinite(maximum_polyline_spacing_m) or maximum_polyline_spacing_m <= 0.0:
        raise ValueError("maximum polyline spacing must be finite and positive")

    tube_radius = 0.5 * tube_inner_diameter_m
    maximum_curvature = centerline.maximum_absolute_curvature_per_m
    if maximum_curvature * tube_radius >= 1.0:
        raise ValueError(
            "tube radius reaches or exceeds the local radius of curvature, so the swept tube is singular"
        )
    local_margin = (
        1.0 / maximum_curvature - tube_radius if maximum_curvature > 0.0 else None
    )
    body_reports = []
    for body in bodies:
        curvature_allowance = 0.5 * maximum_curvature * body.half_length_m**2
        required_radius = body.cross_section_radius_m + curvature_allowance
        wall_clearance = tube_radius - guide_clearance_m - required_radius
        if wall_clearance < -1e-12:
            raise ValueError(
                f"{body.name} swept envelope requires radius {required_radius:.6g} m plus "
                f"{guide_clearance_m:.6g} m guide clearance inside a {tube_radius:.6g} m tube radius"
            )
        body_reports.append(
            BodyClearanceReport(
                name=body.name,
                cross_section_radius_m=body.cross_section_radius_m,
                curvature_allowance_m=curvature_allowance,
                required_radial_envelope_m=required_radius,
                minimum_wall_clearance_m=max(0.0, wall_clearance),
            )
        )

    error_budget = max(1e-9, min(0.01, 0.005 * tube_radius))
    curvature_spacing = (
        math.sqrt(2.0 * error_budget / maximum_curvature)
        if maximum_curvature > 0.0
        else maximum_polyline_spacing_m
    )
    spacing = min(maximum_polyline_spacing_m, curvature_spacing)
    poses = centerline.sample(spacing)
    actual_spacing = max(
        poses[index + 1].s_m - poses[index].s_m for index in range(len(poses) - 1)
    )
    # For any bounded-curvature segment, integrating the maximum tangent deviation
    # gives the conservative k*ds^2/2 distance bound. The familiar circular-arc
    # sagitta k*ds^2/8 is tighter but is not a proof for an arbitrary clothoid/S-curve.
    chord_error = 0.5 * maximum_curvature * actual_spacing**2
    _verify_nonlocal_tube_clearance(
        poses,
        tube_radius_m=tube_radius,
        chord_error_bound_m=chord_error,
    )
    limiting = min(body_reports, key=lambda report: report.minimum_wall_clearance_m)
    return SweptEnvelopeReport(
        tube_radius_m=tube_radius,
        guide_clearance_m=guide_clearance_m,
        maximum_absolute_curvature_per_m=maximum_curvature,
        local_curvature_margin_m=local_margin,
        maximum_polyline_spacing_m=actual_spacing,
        polyline_chord_error_bound_m=chord_error,
        required_nonlocal_centerline_separation_m=2.0 * tube_radius,
        body_reports=tuple(body_reports),
        limiting_body=limiting.name,
        minimum_vehicle_wall_clearance_m=limiting.minimum_wall_clearance_m,
    )


def normal_jerk_mps3(
    speed_mps: float,
    tangential_acceleration_mps2: float,
    signed_curvature_per_m: float,
    curvature_rate_per_m2: float,
) -> float:
    """Signed normal jerk for an arc-length path and scalar speed profile."""
    values = (
        speed_mps,
        tangential_acceleration_mps2,
        signed_curvature_per_m,
        curvature_rate_per_m2,
    )
    if not is_finite(values):
        raise ValueError("normal-jerk inputs must be finite")
    return (
        2.0 * speed_mps * tangential_acceleration_mps2 * signed_curvature_per_m
        + speed_mps**3 * curvature_rate_per_m2
    )


def guide_normal_bound_mps2(
    speed_mps: float,
    signed_curvature_per_m: float,
    gravity_mps2: Vec3,
    normal: Vec3,
) -> float:
    """Two-sided v0.9 bound for non-gravitational guide-normal acceleration."""
    if not is_finite((speed_mps, signed_curvature_per_m)):
        raise ValueError("guide-normal speed and curvature must be finite")
    if speed_mps < 0.0:
        raise ValueError("guide-normal speed must be nonnegative")
    if any(len(value) != 3 or not is_finite(value) for value in (gravity_mps2, normal)):
        raise ValueError("gravity and normal must be finite three-vectors")
    normal_length = math.sqrt(dot(normal, normal))
    if not math.isclose(normal_length, 1.0, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("guide normal must be normalized")
    curvature_acceleration = speed_mps * speed_mps * signed_curvature_per_m
    gravity_normal = dot(gravity_mps2, normal)
    return max(
        abs(curvature_acceleration),
        abs(curvature_acceleration - gravity_normal),
    )


@dataclass(frozen=True)
class CurvedTubeLayout:
    """Schema-v3 centerline and atmosphere stages sharing one arc-length coordinate."""

    centerline: PlanarCenterline
    stages: Tuple[TubeStage, ...]
    exterior_effective_density_ratio: float
    boundary_blend_distance_m: float = 0.0

    def __post_init__(self) -> None:
        if not isinstance(self.centerline, PlanarCenterline):
            raise ValueError("curved layout requires a PlanarCenterline")
        if not self.stages or any(not isinstance(stage, TubeStage) for stage in self.stages):
            raise ValueError("curved layout requires at least one valid TubeStage")
        if not math.isclose(
            sum(stage.length_m for stage in self.stages),
            self.centerline.length_m,
            rel_tol=1e-9,
            abs_tol=1e-3,
        ):
            raise ValueError("stage lengths must sum to centerline arc length")
        if (
            not math.isfinite(self.exterior_effective_density_ratio)
            or self.exterior_effective_density_ratio < 0.0
        ):
            raise ValueError("exterior effective-density ratio must be finite and nonnegative")
        if not math.isfinite(self.boundary_blend_distance_m) or self.boundary_blend_distance_m < 0.0:
            raise ValueError("boundary blend distance must be finite and nonnegative")
        if self.boundary_blend_distance_m > min(stage.length_m for stage in self.stages):
            raise ValueError("boundary blend distance may not exceed the shortest stage length")

    @property
    def length_m(self) -> float:
        return self.centerline.length_m

    @property
    def boundaries_m(self) -> Tuple[float, ...]:
        return cumulative_boundaries(stage.length_m for stage in self.stages)

    def pose(self, s_m: float) -> CenterlinePose:
        return self.centerline.pose(s_m)

    def axial_position(self, position_m: Vec3) -> float:
        return self.centerline.nearest_s(position_m)

    def stage_index(self, s_m: float) -> int | None:
        return stage_index(s_m, self.boundaries_m)

    def density_ratio(self, s_m: float) -> float:
        return effective_density_ratio(
            s_m,
            self.stages,
            exterior_ratio=self.exterior_effective_density_ratio,
            blend_distance_m=self.boundary_blend_distance_m,
        )


@dataclass(frozen=True)
class TubeLayout:
    """Validated straight-tube geometry shared by all axial consumers."""

    origin_m: Vec3
    angle_deg: float
    stages: Tuple[TubeStage, ...]
    exterior_effective_density_ratio: float = 1.0
    boundary_blend_distance_m: float = 0.0

    def __post_init__(self) -> None:
        if len(self.origin_m) != 3 or not is_finite(self.origin_m):
            raise ValueError("tube origin must be a finite three-vector")
        if not math.isfinite(self.angle_deg):
            raise ValueError("tube angle must be finite")
        if not self.stages:
            raise ValueError("at least one tube stage is required")
        if any(not isinstance(stage, TubeStage) for stage in self.stages):
            raise ValueError("all stages must be TubeStage values")
        if (
            not math.isfinite(self.exterior_effective_density_ratio)
            or self.exterior_effective_density_ratio < 0.0
        ):
            raise ValueError("exterior effective-density ratio must be finite and nonnegative")
        if not math.isfinite(self.boundary_blend_distance_m) or self.boundary_blend_distance_m < 0.0:
            raise ValueError("boundary blend distance must be finite and nonnegative")
        if self.boundary_blend_distance_m > min(stage.length_m for stage in self.stages):
            raise ValueError("boundary blend distance may not exceed the shortest stage length")

    @property
    def axis(self) -> Vec3:
        return tube_axis(self.angle_deg)

    @property
    def boundaries_m(self) -> Tuple[float, ...]:
        return cumulative_boundaries(stage.length_m for stage in self.stages)

    @property
    def length_m(self) -> float:
        return self.boundaries_m[-1]

    def axial_position(self, position_m: Vec3) -> float:
        return axial_position(position_m, self.origin_m, self.axis)

    def stage_index(self, s_m: float) -> int | None:
        return stage_index(s_m, self.boundaries_m)

    def density_ratio(self, s_m: float) -> float:
        return effective_density_ratio(
            s_m,
            self.stages,
            exterior_ratio=self.exterior_effective_density_ratio,
            blend_distance_m=self.boundary_blend_distance_m,
        )


TubePath = TubeLayout | CurvedTubeLayout


def path_pose(layout: TubePath, s_m: float) -> CenterlinePose:
    """Resolve a straight or curved path pose, extrapolating its endpoint tangents.

    The atmosphere and analytic backend must agree on one local frame even immediately
    before the entrance and on the separate straight exit track.  Curved centerlines are
    authored only over the atmospheric tube, so coordinates outside that interval extend
    the corresponding zero-curvature endpoint tangent instead of asking the centerline to
    evaluate outside its domain.
    """
    if not math.isfinite(s_m):
        raise ValueError("path coordinate must be finite")
    if isinstance(layout, TubeLayout):
        angle_rad = math.radians(layout.angle_deg)
        return CenterlinePose(
            s_m=float(s_m),
            position_m=world_position(s_m, layout.origin_m, layout.axis),
            tangent=layout.axis,
            normal=(-math.sin(angle_rad), 0.0, math.cos(angle_rad)),
            inclination_deg=layout.angle_deg,
            signed_curvature_per_m=0.0,
            curvature_rate_per_m2=0.0,
            segment_index=0,
        )

    endpoint_s = 0.0 if s_m < 0.0 else layout.length_m
    if 0.0 <= s_m <= layout.length_m:
        return layout.pose(s_m)
    endpoint = layout.pose(endpoint_s)
    offset = s_m - endpoint_s
    return CenterlinePose(
        s_m=float(s_m),
        position_m=add(endpoint.position_m, scale(endpoint.tangent, offset)),
        tangent=endpoint.tangent,
        normal=endpoint.normal,
        inclination_deg=endpoint.inclination_deg,
        signed_curvature_per_m=0.0,
        curvature_rate_per_m2=0.0,
        segment_index=endpoint.segment_index,
    )


@dataclass(frozen=True)
class BoundaryCrossing:
    """One cumulative stage boundary crossed during a physics step."""

    boundary_index: int
    boundary_s_m: float
    time_s: float
    direction: int
    from_stage_index: int | None
    to_stage_index: int | None


def tube_axis(angle_deg: float) -> Vec3:
    """Unit tube axis in the world X-Z plane for an elevation angle in degrees."""
    if not math.isfinite(angle_deg):
        raise ValueError("tube angle must be finite")
    angle_rad = math.radians(angle_deg)
    return (math.cos(angle_rad), 0.0, math.sin(angle_rad))


def axial_position(position_m: Vec3, origin_m: Vec3, axis: Vec3) -> float:
    """Project a world position into the tube coordinate ``s``."""
    if any(len(value) != 3 or not is_finite(value) for value in (position_m, origin_m, axis)):
        raise ValueError("position, origin, and axis must be finite three-vectors")
    axis_length = math.sqrt(dot(axis, axis))
    if not math.isclose(axis_length, 1.0, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError(f"tube axis must be normalized, got length {axis_length}")
    return dot(sub(position_m, origin_m), axis)


def world_position(s_m: float, origin_m: Vec3, axis: Vec3) -> Vec3:
    """Map an axial coordinate back to the tube centerline in world space."""
    if not math.isfinite(s_m):
        raise ValueError("axial coordinate must be finite")
    # Reuse the normalization and vector validation at an arbitrary point.
    axial_position(origin_m, origin_m, axis)
    return add(origin_m, scale(axis, s_m))


def axial_velocity(velocity_mps: Vec3, axis: Vec3) -> float:
    """Signed component of a world velocity along the tube axis."""
    return axial_position(velocity_mps, (0.0, 0.0, 0.0), axis)


def cumulative_boundaries(lengths_m: Iterable[float]) -> Tuple[float, ...]:
    """Cumulative ends of every stage, including the tube exit as the final value."""
    total = 0.0
    boundaries = []
    for index, length in enumerate(lengths_m):
        if not math.isfinite(length) or length <= 0.0:
            raise ValueError(f"stage length at index {index} must be finite and positive")
        total += float(length)
        boundaries.append(total)
    if not boundaries:
        raise ValueError("at least one stage length is required")
    return tuple(boundaries)


def _validate_boundaries(boundaries_m: Sequence[float]) -> Tuple[float, ...]:
    boundaries = tuple(float(value) for value in boundaries_m)
    if not boundaries:
        raise ValueError("at least one cumulative boundary is required")
    previous = 0.0
    for index, boundary in enumerate(boundaries):
        if not math.isfinite(boundary) or boundary <= previous:
            raise ValueError(
                f"cumulative boundaries must be finite and strictly increasing; index {index} is {boundary}"
            )
        previous = boundary
    return boundaries


def stage_index(s_m: float, boundaries_m: Sequence[float]) -> int | None:
    """Return the active stage index, or ``None`` before the entrance/at or beyond exit.

    An exact internal boundary belongs to the stage on its forward side. This half-open
    convention makes each axial location unambiguous: stage intervals are ``[start, end)``.
    """
    if not math.isfinite(s_m):
        raise ValueError("axial coordinate must be finite")
    boundaries = _validate_boundaries(boundaries_m)
    if s_m < 0.0 or s_m >= boundaries[-1]:
        return None
    return bisect.bisect_right(boundaries, s_m)


def enumerate_boundary_crossings(
    pre_s_m: float,
    post_s_m: float,
    boundaries_m: Sequence[float],
    *,
    pre_time_s: float,
    post_time_s: float,
) -> Tuple[BoundaryCrossing, ...]:
    """Enumerate all stage-end crossings in travel order with interpolated times.

    The segment start is excluded and its end is included. Consecutive step segments can
    therefore share an endpoint without emitting the same boundary twice.
    """
    if not is_finite((pre_s_m, post_s_m, pre_time_s, post_time_s)):
        raise ValueError("crossing coordinates and times must be finite")
    if post_time_s < pre_time_s:
        raise ValueError("post-step time may not precede pre-step time")
    boundaries = _validate_boundaries(boundaries_m)
    delta_s = post_s_m - pre_s_m
    if delta_s == 0.0:
        return ()
    if post_time_s == pre_time_s:
        raise ValueError("a nonzero displacement requires a positive time interval")

    if delta_s > 0.0:
        selected = ((index, boundary) for index, boundary in enumerate(boundaries) if pre_s_m < boundary <= post_s_m)
        direction = 1
    else:
        selected = (
            (index, boundary)
            for index, boundary in reversed(tuple(enumerate(boundaries)))
            if post_s_m <= boundary < pre_s_m
        )
        direction = -1

    crossings = []
    for index, boundary in selected:
        fraction = (boundary - pre_s_m) / delta_s
        crossing_time = pre_time_s + fraction * (post_time_s - pre_time_s)
        if direction > 0:
            from_stage = index
            to_stage = index + 1 if index + 1 < len(boundaries) else None
        else:
            from_stage = index + 1 if index + 1 < len(boundaries) else None
            to_stage = index
        crossings.append(
            BoundaryCrossing(
                boundary_index=index,
                boundary_s_m=boundary,
                time_s=crossing_time,
                direction=direction,
                from_stage_index=from_stage,
                to_stage_index=to_stage,
            )
        )
    return tuple(crossings)


def effective_density_ratio(
    s_m: float,
    stages: Sequence[TubeStage],
    *,
    exterior_ratio: float,
    blend_distance_m: float = 0.0,
) -> float:
    """Piecewise stage ratio with an optional centered linear boundary blend."""
    if not math.isfinite(s_m):
        raise ValueError("axial coordinate must be finite")
    if not stages:
        raise ValueError("at least one stage is required")
    if any(not isinstance(stage, TubeStage) for stage in stages):
        raise ValueError("all stages must be TubeStage values")
    if not math.isfinite(exterior_ratio) or exterior_ratio < 0.0:
        raise ValueError("exterior density ratio must be finite and nonnegative")
    if not math.isfinite(blend_distance_m) or blend_distance_m < 0.0:
        raise ValueError("blend distance must be finite and nonnegative")
    if blend_distance_m > min(stage.length_m for stage in stages):
        raise ValueError("blend distance may not exceed the shortest stage length")

    boundaries = cumulative_boundaries(stage.length_m for stage in stages)
    ratios = (exterior_ratio,) + tuple(stage.effective_density_ratio for stage in stages) + (exterior_ratio,)
    all_boundaries = (0.0,) + boundaries

    if blend_distance_m > 0.0:
        half_width = 0.5 * blend_distance_m
        for index, boundary in enumerate(all_boundaries):
            if boundary - half_width <= s_m <= boundary + half_width:
                fraction = (s_m - (boundary - half_width)) / blend_distance_m
                return ratios[index] + fraction * (ratios[index + 1] - ratios[index])

    active = stage_index(s_m, boundaries)
    return exterior_ratio if active is None else stages[active].effective_density_ratio


def refine_density_stages(
    stages: Sequence[TubeStage],
    refinement_factor: int,
    *,
    entrance_ratio: float | None = None,
    exit_ratio: float | None = None,
) -> Tuple[TubeStage, ...]:
    """Deterministically refine the piecewise-linear density target from design v0.8."""
    if not stages or any(not isinstance(stage, TubeStage) for stage in stages):
        raise ValueError("density refinement requires at least one valid stage")
    if isinstance(refinement_factor, bool) or not isinstance(refinement_factor, int):
        raise ValueError("refinement factor must be an integer")
    if refinement_factor < 2:
        raise ValueError("refinement factor must be at least 2")
    entrance = stages[0].effective_density_ratio if entrance_ratio is None else entrance_ratio
    exit_value = stages[-1].effective_density_ratio if exit_ratio is None else exit_ratio
    if not math.isfinite(entrance) or entrance < 0.0:
        raise ValueError("entrance density ratio must be finite and nonnegative")
    if not math.isfinite(exit_value) or exit_value < 0.0:
        raise ValueError("exit density ratio must be finite and nonnegative")

    boundaries = cumulative_boundaries(stage.length_m for stage in stages)
    starts = (0.0,) + boundaries[:-1]
    knots = [(0.0, float(entrance))]
    knots.extend(
        (
            start + 0.5 * stage.length_m,
            stage.effective_density_ratio,
        )
        for start, stage in zip(starts, stages)
    )
    knots.append((boundaries[-1], float(exit_value)))

    def interpolated_ratio(s_m: float) -> float:
        right_index = bisect.bisect_right([item[0] for item in knots], s_m)
        if right_index <= 0:
            return knots[0][1]
        if right_index >= len(knots):
            return knots[-1][1]
        left_s, left_ratio = knots[right_index - 1]
        right_s, right_ratio = knots[right_index]
        fraction = (s_m - left_s) / (right_s - left_s)
        return left_ratio + fraction * (right_ratio - left_ratio)

    refined = []
    for start, stage in zip(starts, stages):
        refined_length = stage.length_m / refinement_factor
        for sub_index in range(refinement_factor):
            midpoint = start + (sub_index + 0.5) * refined_length
            refined.append(
                TubeStage(
                    name=f"{stage.name}.r{sub_index + 1}",
                    length_m=refined_length,
                    effective_density_ratio=interpolated_ratio(midpoint),
                )
            )
    return tuple(refined)


def gravity_projection(gravity_mps2: Vec3, axis: Vec3) -> float:
    """Signed gravitational acceleration along the tube axis."""
    if any(len(value) != 3 or not is_finite(value) for value in (gravity_mps2, axis)):
        raise ValueError("gravity and axis must be finite three-vectors")
    # Validate that callers do not silently scale gravity with a non-unit direction.
    axial_position((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), axis)
    return dot(gravity_mps2, axis)
