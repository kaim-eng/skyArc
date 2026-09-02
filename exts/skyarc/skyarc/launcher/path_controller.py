# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Backend-side force-resolved path guide and its translated accelerating frame.

DESIGN_REVIEW v0.29 accepted the *force-resolved path controller* as the production
curved-guide mechanism.  It is explicitly **not** a solver constraint: the normal reaction
is commanded from the resolved centerline rather than read back from a joint.  Section 5.1
keeps that fact structural -- ``IdealPathGuide`` owns only the guide's tangential
resistance and never fabricates a normal wrench, because normal motion is the *backend's*
responsibility.  The analytic backend discharges it by construction (it integrates along
the path); PhysX has no path joint, so the reaction has to be computed and applied.

This module is that computation, kept in the backend-neutral core so it is unit-testable
without Kit and so the one place that decides the reaction is not buried inside an
Isaac-only module.  ``effects/backends/isaac.py`` is its only production caller; it reports
the result through :class:`~..effects.adapter.AppliedEffects` under
``SLOT_BACKEND_ADAPTER`` so that the applied load never silently exceeds the accepted load
without an attributable slot behind the difference.

The formulas reproduce the Phase 0 qualified controller in
``standalone/qualify_curved_guide.py`` exactly when the accepted external load is purely
tangential, which is the case the evidence covers.  The generalization is that the accepted
normal and binormal load is now subtracted before the reaction is sized, so that a future
component emitting an off-axis wrench cannot be double-counted or silently cancelled.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Tuple

from ..linalg import ZERO3, Vec3, add, cross, dot, quat_rotate, scale, sub
from ..names import BODY_CART, BODY_ROCKET, JOINT_COUPLING
from ..state import SimulationState, body_com_world
from .geometry import TubePath, path_pose


@dataclass(frozen=True)
class TranslatedFrameState:
    """Global state of the non-rotating solver origin at one simulation instant.

    ``x_global = x_solver + position_m``, ``v_global = v_solver + velocity_mps``, and each
    body carries the exact uniform fictitious force ``-m * acceleration_mps2``.  Orientation
    is untouched and body separations are invariant, which is what keeps collision and the
    fixed-joint anchor unaffected by the frame.
    """

    position_m: Vec3 = ZERO3
    velocity_mps: Vec3 = ZERO3
    acceleration_mps2: Vec3 = ZERO3

    def __post_init__(self) -> None:
        for name in ("position_m", "velocity_mps", "acceleration_mps2"):
            value = getattr(self, name)
            if len(value) != 3 or not all(math.isfinite(component) for component in value):
                raise ValueError(f"reference frame {name} must be a finite three-vector")


@dataclass(frozen=True)
class PathControllerGains:
    """Feedback gains for the force-resolved guide.

    The defaults are the gains the accepted Phase 0 curved-guide artifacts were produced
    with; they are part of that result's identity and ``tests/unit/test_phase0_runner.py``
    pins them as literals.  Note that the derivative gain is 40.0 per second and *not* the
    ``qualify_curved_guide.py`` argument default of zero -- the accepted runs passed it
    explicitly, and production must reproduce the qualified condition rather than the
    runner's convenience default.  Overriding any of these produces a new condition rather
    than a variation of the qualified one.
    """

    normal_kp_per_s2: float = 400.0
    normal_kd_per_s: float = 40.0
    attitude_kp_per_s2: float = 2500.0
    attitude_kd_per_s: float = 100.0

    def __post_init__(self) -> None:
        for name in (
            "normal_kp_per_s2",
            "normal_kd_per_s",
            "attitude_kp_per_s2",
            "attitude_kd_per_s",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"path controller gain {name} must be finite and nonnegative")


@dataclass(frozen=True)
class GuideReaction:
    """One step of commanded guide reaction, resolved about the reacting body's COM."""

    body: str
    force_n: Vec3
    torque_nm: Vec3
    application_point_m: Vec3
    s_m: float
    normal_error_m: float
    binormal_error_m: float
    tracking_error_m: float
    attitude_error_rad: float
    coupled: bool
    ideal_normal_force_n: float
    commanded_normal_force_n: float


def forward_pitch_angle_rad(orientation_wxyz: Tuple[float, float, float, float]) -> float:
    """Elevation of a body's local +X axis in the world X-Z plane."""
    forward = quat_rotate(orientation_wxyz, (1.0, 0.0, 0.0))
    return math.atan2(forward[2], forward[0])


def wrap_angle_rad(value: float) -> float:
    """Fold an angle into ``[-pi, pi)`` so an attitude error never wraps the long way."""
    return (value + math.pi) % (2.0 * math.pi) - math.pi


class LaunchProfileReferenceFrame:
    """Translated accelerating frame following the commanded uniform launch profile.

    Direct global float32 coordinates were rejected by a matched Phase 0 control: at 25 km
    the reported pose rate and the tensor velocity disagreed by roughly 14 m/s, which
    exhausted the 0.317531 m cart clearance.  The frame exists to keep solver coordinates
    small while every public quantity stays global SI.

    The reference is the *commanded* uniform-acceleration profile, not the measured
    trajectory: an analytic reference has an exact analytic acceleration, and a fictitious
    force derived from a finite-differenced measurement would feed integration noise back
    into the bodies it is supposed to be neutral for.  The mission's launch controller
    compensates drag, grade and resistance, so the assembly does not track this profile
    exactly; the residual appears as a bounded, measurable solver-coordinate offset rather
    than as a modelling error.

    Past the tube exit the frame follows the cart, not the rocket.  Only one of the two can
    be tracked -- after release they separate by tens of kilometres -- and the cart is the
    body that carries a geometric budget: the guide holds it to ``tube.guide_clearance_m``,
    0.05 m on the reference curve, while the rocket is ballistic and constrained by nothing.

    An inertial continuation was tried first and is not viable.  It leaves the frame
    coasting at the exit speed while the cart decelerates to rest, so the offset between
    them grows without bound: a complete mission measured 34,046.9 m of solver offset and
    aborted on ``guide_tracking_error`` six seconds *after* the cart had stopped, with peak
    tracking 0.0495851 m against the 0.05 m limit.  Float32 resolution at 34 km is about
    4 mm, roughly eight percent of the whole tracking budget, and it only worsens; the run
    was going to abort eventually regardless of controller tuning.

    ``brake_distance_m`` therefore extends the profile with a constant deceleration from the
    exit speed to rest over that distance, and the frame then holds station.  It stays
    analytic, which is the point: the fictitious force needs an exact acceleration, and one
    finite-differenced from the measured cart would feed integration noise back into the
    bodies it is supposed to be neutral for.  The cart's own brake is jerk-limited rather
    than constant-deceleration, so the two differ by a few percent, and the residual shows
    up as a bounded solver offset instead of an unbounded one.

    Both position and velocity are continuous across the two joins.  That is required, not
    cosmetic: global state is reconstructed as ``v_global = v_solver + v_r``, so a step in
    ``v_r`` would report a velocity jump the bodies never had.  Acceleration may and does
    step, because it cancels exactly in the reconstruction and never reaches global state.

    Omitting ``brake_distance_m`` retains the inertial continuation, which is what a
    launch-only evidence run wants and what
    :attr:`~..effects.backends.isaac.IsaacPhysxBackend.peak_solver_offset_m` measured to
    reject it for complete missions.
    """

    def __init__(
        self,
        layout: TubePath,
        *,
        target_exit_speed_mps: float,
        start_s_m: float = 0.0,
        brake_distance_m: float | None = None,
    ) -> None:
        if not math.isfinite(target_exit_speed_mps) or target_exit_speed_mps <= 0.0:
            raise ValueError("reference profile target exit speed must be finite and positive")
        if not math.isfinite(start_s_m) or start_s_m < 0.0:
            raise ValueError("reference profile start coordinate must be finite and nonnegative")
        remaining_m = layout.length_m - start_s_m
        if remaining_m <= 0.0:
            raise ValueError("reference profile start coordinate must precede the tube exit")
        if brake_distance_m is not None and (
            not math.isfinite(brake_distance_m) or brake_distance_m <= 0.0
        ):
            raise ValueError("reference frame brake distance must be finite and positive")
        self._layout = layout
        self._start_s_m = float(start_s_m)
        self._target_speed_mps = float(target_exit_speed_mps)
        self._acceleration_mps2 = target_exit_speed_mps**2 / (2.0 * remaining_m)
        self._exit_time_s = target_exit_speed_mps / self._acceleration_mps2
        self._exit_pose = path_pose(layout, layout.length_m)
        self._brake_distance_m = None if brake_distance_m is None else float(brake_distance_m)
        if self._brake_distance_m is None:
            self._brake_deceleration_mps2 = 0.0
            self._brake_duration_s = 0.0
        else:
            self._brake_deceleration_mps2 = target_exit_speed_mps**2 / (
                2.0 * self._brake_distance_m
            )
            self._brake_duration_s = target_exit_speed_mps / self._brake_deceleration_mps2

    @property
    def acceleration_mps2(self) -> float:
        return self._acceleration_mps2

    @property
    def exit_time_s(self) -> float:
        return self._exit_time_s

    @property
    def brake_distance_m(self) -> float | None:
        return self._brake_distance_m

    @property
    def brake_deceleration_mps2(self) -> float:
        """Constant deceleration applied past the exit; zero when coasting inertially."""
        return self._brake_deceleration_mps2

    @property
    def rest_time_s(self) -> float | None:
        """Time at which the frame comes to rest, or ``None`` when it coasts forever."""
        if self._brake_distance_m is None:
            return None
        return self._exit_time_s + self._brake_duration_s

    def sample(self, time_s: float) -> TranslatedFrameState:
        if not math.isfinite(time_s) or time_s < 0.0:
            raise ValueError("reference frame time must be finite and nonnegative")
        if time_s <= self._exit_time_s:
            speed = self._acceleration_mps2 * time_s
            s_m = self._start_s_m + 0.5 * self._acceleration_mps2 * time_s * time_s
            pose = path_pose(self._layout, min(s_m, self._layout.length_m))
            return TranslatedFrameState(
                position_m=pose.position_m,
                velocity_mps=scale(pose.tangent, speed),
                acceleration_mps2=add(
                    scale(pose.tangent, self._acceleration_mps2),
                    scale(pose.normal, speed * speed * pose.signed_curvature_per_m),
                ),
            )
        elapsed_s = time_s - self._exit_time_s
        if self._brake_distance_m is None:
            # Inertial continuation. Retained for launch-only evidence; rejected for
            # complete missions, where it runs away from the decelerating cart.
            coasted_s = self._target_speed_mps * elapsed_s
            return TranslatedFrameState(
                position_m=add(
                    self._exit_pose.position_m, scale(self._exit_pose.tangent, coasted_s)
                ),
                velocity_mps=scale(self._exit_pose.tangent, self._target_speed_mps),
                acceleration_mps2=ZERO3,
            )
        if elapsed_s >= self._brake_duration_s:
            # At rest on the exit track. The cart is stationary here too, so the offset
            # between them stops growing instead of increasing at the exit speed forever.
            return TranslatedFrameState(
                position_m=add(
                    self._exit_pose.position_m,
                    scale(self._exit_pose.tangent, self._brake_distance_m),
                ),
                velocity_mps=ZERO3,
                acceleration_mps2=ZERO3,
            )
        speed = self._target_speed_mps - self._brake_deceleration_mps2 * elapsed_s
        travelled_m = (
            self._target_speed_mps * elapsed_s
            - 0.5 * self._brake_deceleration_mps2 * elapsed_s * elapsed_s
        )
        return TranslatedFrameState(
            position_m=add(
                self._exit_pose.position_m, scale(self._exit_pose.tangent, travelled_m)
            ),
            velocity_mps=scale(self._exit_pose.tangent, speed),
            acceleration_mps2=scale(self._exit_pose.tangent, -self._brake_deceleration_mps2),
        )


class ForceResolvedPathReaction:
    """Command the guide reaction that a path constraint would otherwise supply.

    While the coupling joint is active the attached assembly is treated as one rigid body:
    the reaction is sized from the assembly mass and applied at the assembly centre of mass.
    Applying it at the cart centre of mass instead would add an ``r x F`` pitch moment that
    a path guide does not own and that the attitude command would then have to fight.  After
    release only the cart remains guided; the rocket is free and receives no reaction.
    """

    def __init__(
        self,
        layout: TubePath,
        *,
        coupled_pitch_inertia_kg_m2: float,
        cart_pitch_inertia_kg_m2: float,
        gains: PathControllerGains = PathControllerGains(),
        gravity_mps2: Vec3 = (0.0, 0.0, -9.81),
    ) -> None:
        for label, value in (
            ("coupled", coupled_pitch_inertia_kg_m2),
            ("cart", cart_pitch_inertia_kg_m2),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{label} pitch inertia must be finite and positive")
        if len(gravity_mps2) != 3 or not all(math.isfinite(value) for value in gravity_mps2):
            raise ValueError("path controller gravity must be a finite three-vector")
        self._layout = layout
        self._gains = gains
        self._gravity = tuple(float(value) for value in gravity_mps2)
        self._coupled_inertia = float(coupled_pitch_inertia_kg_m2)
        self._cart_inertia = float(cart_pitch_inertia_kg_m2)

    @property
    def gains(self) -> PathControllerGains:
        return self._gains

    def evaluate(
        self,
        state: SimulationState,
        external_force_n: Mapping[str, Vec3],
    ) -> GuideReaction:
        """Return the reaction on the cart for one pre-step.

        Args:
            state: Global SI latent state, already reconstructed out of the solver frame.
            external_force_n: Accepted net world force per body for this step.  It is
                subtracted in the normal and binormal directions so the reaction supplies
                exactly the balance the centerline requires, and its tangential part feeds
                the attitude feed-forward.
        """
        cart = state.body(BODY_CART)
        coupled = bool(state.joint_active.get(JOINT_COUPLING, False)) and BODY_ROCKET in state.bodies
        members = (BODY_CART, BODY_ROCKET) if coupled else (BODY_CART,)

        mass_kg = 0.0
        weighted_position: Vec3 = ZERO3
        weighted_velocity: Vec3 = ZERO3
        external: Vec3 = ZERO3
        for name in members:
            body = state.body(name)
            mass_kg += body.mass_kg
            weighted_position = add(weighted_position, scale(body_com_world(body), body.mass_kg))
            weighted_velocity = add(weighted_velocity, scale(body.linear_velocity, body.mass_kg))
            external = add(external, external_force_n.get(name, ZERO3))
        if mass_kg <= 0.0:
            raise ValueError("guided assembly mass must be positive")
        com_m = scale(weighted_position, 1.0 / mass_kg)
        velocity_mps = scale(weighted_velocity, 1.0 / mass_kg)

        s_m = self._layout.axial_position(com_m)
        pose = path_pose(self._layout, s_m)
        tangent = pose.tangent
        normal = pose.normal
        binormal = cross(tangent, normal)

        offset = sub(com_m, pose.position_m)
        normal_error_m = dot(offset, normal)
        binormal_error_m = dot(offset, binormal)
        tangential_speed = dot(velocity_mps, tangent)
        normal_speed = dot(velocity_mps, normal)
        binormal_speed = dot(velocity_mps, binormal)

        gravity_t = dot(self._gravity, tangent)
        gravity_n = dot(self._gravity, normal)
        gravity_b = dot(self._gravity, binormal)
        curvature = pose.signed_curvature_per_m

        # The centripetal term is what an ideal constraint would supply; the two feedback
        # terms are the controller correction that a constraint would not need.  They are
        # reported separately so the correction can be gated instead of hidden inside the
        # command it modifies.
        ideal_normal_force_n = mass_kg * (tangential_speed**2 * curvature - gravity_n)
        commanded_normal_force_n = ideal_normal_force_n + mass_kg * (
            -self._gains.normal_kp_per_s2 * normal_error_m
            - self._gains.normal_kd_per_s * normal_speed
        )
        normal_force_n = commanded_normal_force_n - dot(external, normal)
        binormal_force_n = (
            mass_kg
            * (
                -self._gains.normal_kp_per_s2 * binormal_error_m
                - self._gains.normal_kd_per_s * binormal_speed
                - gravity_b
            )
            - dot(external, binormal)
        )
        force_n = add(scale(normal, normal_force_n), scale(binormal, binormal_force_n))

        tangential_acceleration_mps2 = dot(external, tangent) / mass_kg + gravity_t
        target_rate_rad_s = -tangential_speed * curvature
        target_angular_acceleration_rad_s2 = -(
            tangential_acceleration_mps2 * curvature
            + tangential_speed**2 * pose.curvature_rate_per_m2
        )
        attitude_error_rad = wrap_angle_rad(
            math.radians(pose.inclination_deg) - forward_pitch_angle_rad(cart.orientation)
        )
        inertia = self._coupled_inertia if coupled else self._cart_inertia
        pitch_torque_nm = inertia * (
            target_angular_acceleration_rad_s2
            - self._gains.attitude_kp_per_s2 * attitude_error_rad
            + self._gains.attitude_kd_per_s * (target_rate_rad_s - cart.angular_velocity[1])
        )

        # Resolve about the cart's centre of mass, which is where the adapter applies it.
        lever = sub(com_m, body_com_world(cart))
        torque_nm = add((0.0, pitch_torque_nm, 0.0), cross(lever, force_n))
        return GuideReaction(
            body=BODY_CART,
            force_n=force_n,
            torque_nm=torque_nm,
            application_point_m=com_m,
            s_m=s_m,
            normal_error_m=normal_error_m,
            binormal_error_m=binormal_error_m,
            tracking_error_m=math.hypot(normal_error_m, binormal_error_m),
            attitude_error_rad=attitude_error_rad,
            coupled=coupled,
            ideal_normal_force_n=ideal_normal_force_n,
            commanded_normal_force_n=commanded_normal_force_n,
        )


__all__ = [
    "ForceResolvedPathReaction",
    "GuideReaction",
    "LaunchProfileReferenceFrame",
    "PathControllerGains",
    "TranslatedFrameState",
    "forward_pitch_angle_rad",
    "wrap_angle_rad",
]
