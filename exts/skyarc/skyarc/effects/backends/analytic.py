# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Deterministic translation-only reference backend.

This adapter exists to exercise the same accepted-effect boundary as Isaac Sim in controlled
contact-free cases.  It uses semi-implicit Euler, keeps the cart on the resolved path, keeps
the rocket at a fixed path offset while the coupling joint is active, and switches the rocket
to three-dimensional ballistic translation after release.  It is a numerical oracle and test
fixture, not a replacement for measured guide/contact physics.
"""

from __future__ import annotations

import math
from dataclasses import replace
from types import MappingProxyType
from typing import Dict

from ...launcher.geometry import TubePath, path_pose
from ...linalg import ZERO3, Vec3, add, dot, is_finite, norm, scale
from ...names import BODY_CART, BODY_ROCKET, JOINT_COUPLING, JOINT_GUIDE
from ...state import BodyState, SimulationState
from ..adapter import AppliedEffects, BackendCapabilities
from ..aggregator import AggregatedEffects
from ..types import CollisionAction, ConstraintAction, MomentumPolicy


def _path_orientation(inclination_deg: float) -> tuple[float, float, float, float]:
    """Quaternion rotating local +X onto the X-Z path tangent."""
    half = -0.5 * math.radians(inclination_deg)
    return (math.cos(half), 0.0, math.sin(half), 0.0)


def _finite_body(body: BodyState) -> None:
    values = (
        *body.position,
        *body.orientation,
        *body.linear_velocity,
        *body.angular_velocity,
        body.mass_kg,
        *body.com_offset_m,
    )
    if not body.name or not is_finite(values) or body.mass_kg <= 0.0:
        raise ValueError(f"body {body.name!r} must have finite state and positive mass")
    quaternion_norm = math.sqrt(sum(value * value for value in body.orientation))
    if not math.isclose(quaternion_norm, 1.0, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError(f"body {body.name!r} orientation must be normalized")


class AnalyticBackend:
    """Pure-Python :class:`BackendAdapter` for controlled contact-free trajectories."""

    def __init__(
        self,
        initial_state: SimulationState,
        layout: TubePath,
        *,
        gravity_mps2: Vec3 = (0.0, 0.0, -9.81),
    ) -> None:
        if not math.isfinite(initial_state.dt_s) or initial_state.dt_s <= 0.0:
            raise ValueError("analytic backend timestep must be finite and positive")
        if len(gravity_mps2) != 3 or not is_finite(gravity_mps2):
            raise ValueError("analytic backend gravity must be a finite three-vector")
        if not initial_state.bodies:
            raise ValueError("analytic backend requires at least one body")
        for body in initial_state.bodies.values():
            _finite_body(body)
        if BODY_ROCKET in initial_state.bodies and BODY_CART not in initial_state.bodies:
            raise ValueError("an analytic rocket scenario must also contain the cart")

        self._layout = layout
        self._gravity = tuple(float(value) for value in gravity_mps2)
        self._initial_state = initial_state.frozen()
        self._capabilities = BackendCapabilities(
            backend="analytic",
            device="cpu",
            features={
                "deterministic": True,
                "fixed_time_step": True,
                "resync": True,
                "always_present_collision_pair": True,
                "translation_only": True,
                "contact_reporting": False,
                "ccd": False,
            },
        )
        self._pending: AggregatedEffects | None = None
        self._resync_count = 0
        self._reset_from_initial()

    @property
    def capabilities(self) -> BackendCapabilities:
        return self._capabilities

    @property
    def resync_count(self) -> int:
        return self._resync_count

    def path_coordinate(self, body: str) -> float:
        """Return the adapter's exact guided coordinate without geometric reprojection."""
        try:
            return self._s_by_body[body]
        except KeyError:
            raise KeyError(f"unknown analytic body {body!r}") from None

    def _reset_from_initial(self) -> None:
        self._bodies: Dict[str, BodyState] = dict(self._initial_state.bodies)
        self._joint_active = dict(self._initial_state.joint_active)
        self._collision_pair_active = dict(self._initial_state.collision_pair_active)
        self._s_by_body = {
            name: self._layout.axial_position(body.position)
            for name, body in self._bodies.items()
        }
        self._coupled_offset_m = (
            self._s_by_body.get(BODY_ROCKET, 0.0) - self._s_by_body.get(BODY_CART, 0.0)
        )
        self._time_s = self._initial_state.time_s
        self._step_index = self._initial_state.step_index
        self._pending = None
        self._resync_count = 0

    def read_state(self) -> SimulationState:
        return SimulationState(
            time_s=self._time_s,
            step_index=self._step_index,
            dt_s=self._initial_state.dt_s,
            bodies=MappingProxyType(dict(self._bodies)),
            contacts=MappingProxyType({}),
            joint_active=MappingProxyType(dict(self._joint_active)),
            collision_pair_active=MappingProxyType(dict(self._collision_pair_active)),
        )

    def apply(self, effects: AggregatedEffects) -> AppliedEffects:
        if self._pending is not None:
            raise RuntimeError("analytic effects have already been applied for this step")
        for load in effects.loads.values():
            if norm(load.torque_nm) > 1e-12:
                raise ValueError("analytic backend is translation-only and rejects nonzero torque")

        for update in effects.mass_updates:
            if update.effective_time_s > self._time_s + 1e-12:
                raise ValueError("analytic backend does not queue future mass updates")
            body = self._bodies[update.body]
            velocity = body.linear_velocity
            if update.momentum_policy is MomentumPolicy.CONSERVE:
                velocity = scale(velocity, body.mass_kg / update.mass_kg)
            self._bodies[update.body] = replace(
                body,
                mass_kg=update.mass_kg,
                linear_velocity=velocity,
                com_offset_m=body.com_offset_m if update.com_offset_m is None else update.com_offset_m,
            )
        for command in effects.constraint_commands:
            self._joint_active[command.constraint] = command.action is ConstraintAction.ENABLE
        for command in effects.collision_commands:
            self._collision_pair_active[command.pair] = command.action is CollisionAction.ENABLE

        self._pending = effects
        return AppliedEffects.exactly(effects)

    def _integrate_guided_body(self, name: str, force_n: Vec3, dt_s: float) -> None:
        body = self._bodies[name]
        s_m = self._s_by_body[name]
        pose = path_pose(self._layout, s_m)
        speed_mps = dot(body.linear_velocity, pose.tangent)
        acceleration_mps2 = dot(force_n, pose.tangent) / body.mass_kg + dot(
            self._gravity, pose.tangent
        )
        new_speed = speed_mps + acceleration_mps2 * dt_s
        new_s = s_m + new_speed * dt_s
        new_pose = path_pose(self._layout, new_s)
        self._s_by_body[name] = new_s
        self._bodies[name] = replace(
            body,
            position=new_pose.position_m,
            orientation=_path_orientation(new_pose.inclination_deg),
            linear_velocity=scale(new_pose.tangent, new_speed),
            angular_velocity=ZERO3,
        )

    def _integrate_coupled(self, effects: AggregatedEffects, dt_s: float) -> None:
        cart = self._bodies[BODY_CART]
        rocket = self._bodies[BODY_ROCKET]
        cart_s = self._s_by_body[BODY_CART]
        pose = path_pose(self._layout, cart_s)
        combined_mass = cart.mass_kg + rocket.mass_kg
        total_force = add(
            effects.load(BODY_CART).force_n,
            effects.load(BODY_ROCKET).force_n,
        )
        shared_speed = dot(cart.linear_velocity, pose.tangent)
        acceleration = dot(total_force, pose.tangent) / combined_mass + dot(
            self._gravity, pose.tangent
        )
        new_speed = shared_speed + acceleration * dt_s
        new_cart_s = cart_s + new_speed * dt_s
        new_rocket_s = new_cart_s + self._coupled_offset_m
        for name, new_s in ((BODY_CART, new_cart_s), (BODY_ROCKET, new_rocket_s)):
            body = self._bodies[name]
            new_pose = path_pose(self._layout, new_s)
            self._s_by_body[name] = new_s
            self._bodies[name] = replace(
                body,
                position=new_pose.position_m,
                orientation=_path_orientation(new_pose.inclination_deg),
                linear_velocity=scale(new_pose.tangent, new_speed),
                angular_velocity=ZERO3,
            )

    def _integrate_free_body(self, name: str, force_n: Vec3, dt_s: float) -> None:
        body = self._bodies[name]
        acceleration = add(scale(force_n, 1.0 / body.mass_kg), self._gravity)
        velocity = add(body.linear_velocity, scale(acceleration, dt_s))
        self._bodies[name] = replace(
            body,
            position=add(body.position, scale(velocity, dt_s)),
            linear_velocity=velocity,
        )

    def step(self) -> None:
        effects = self._pending or AggregatedEffects()
        dt_s = self._initial_state.dt_s
        coupling_active = bool(self._joint_active.get(JOINT_COUPLING, False))
        guide_active = bool(self._joint_active.get(JOINT_GUIDE, True))
        if coupling_active and BODY_ROCKET in self._bodies:
            if not guide_active:
                raise RuntimeError("analytic coupled motion requires the cart guide")
            self._integrate_coupled(effects, dt_s)
        else:
            for name in tuple(self._bodies):
                force = effects.load(name).force_n
                if name == BODY_CART and guide_active:
                    self._integrate_guided_body(name, force, dt_s)
                else:
                    self._integrate_free_body(name, force, dt_s)
        self._time_s += dt_s
        self._step_index += 1
        self._pending = None

    def resync(self) -> None:
        self._resync_count += 1

    def reset(self) -> None:
        self._reset_from_initial()
