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

"""Immutable simulator state, sensing output, and the record separation between them.

Section 5.1 gives every model immutable state and observations; section 14 requires each
step to record latent ground truth and the controller observation as *separate* things so
that model error, sensing error and control error stay distinguishable. Both are therefore
represented here by distinct types: :class:`SimulationState` is what the backend reports,
:class:`Observation` is what a controller is allowed to see.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping, Tuple

from .linalg import IDENTITY_QUAT, ZERO3, Quat, Vec3


@dataclass(frozen=True)
class BodyState:
    """Rigid-body state as reported by the backend, in SI units and the world frame.

    Attributes:
        name: Stable body identifier from :mod:`.names`.
        position: World position of the body origin, metres.
        orientation: World orientation, quaternion ``(w, x, y, z)``.
        linear_velocity: World linear velocity of the centre of mass, m/s.
        angular_velocity: World angular velocity, rad/s.
        mass_kg: Current mass. Evolves only through an owned mass update.
        com_offset_m: Centre-of-mass offset from the body origin, expressed in the body frame.
    """

    name: str
    position: Vec3 = ZERO3
    orientation: Quat = IDENTITY_QUAT
    linear_velocity: Vec3 = ZERO3
    angular_velocity: Vec3 = ZERO3
    mass_kg: float = 1.0
    com_offset_m: Vec3 = ZERO3


@dataclass(frozen=True)
class ContactReport:
    """A contact quantity for one named pair.

    Contact quantities are recorded as impulses (section 10.4). The runtime returns force or
    impulse from the same call depending on a time-scaling argument and nothing in the value
    distinguishes them, so ``time_scaling`` is carried with the value rather than assumed.
    """

    pair: str
    impulse_ns: Vec3 = ZERO3
    time_scaling: str = "impulse"
    active: bool = False

    @property
    def magnitude_ns(self) -> float:
        from .linalg import norm

        return norm(self.impulse_ns)


@dataclass(frozen=True)
class SimulationState:
    """Latent ground truth for one instant, as read back from the backend adapter."""

    time_s: float
    step_index: int
    dt_s: float
    bodies: Mapping[str, BodyState]
    contacts: Mapping[str, ContactReport] = field(default_factory=dict)
    joint_active: Mapping[str, bool] = field(default_factory=dict)
    collision_pair_active: Mapping[str, bool] = field(default_factory=dict)

    def body(self, name: str) -> BodyState:
        """Return the named body state.

        Raises:
            KeyError: If the body is not present in this state.
        """
        try:
            return self.bodies[name]
        except KeyError:
            raise KeyError(f"unknown body '{name}'; known bodies: {sorted(self.bodies)}") from None

    def contact(self, pair: str) -> ContactReport:
        """Return the contact report for a pair, or an inactive zero report if absent."""
        return self.contacts.get(pair, ContactReport(pair=pair))

    def frozen(self) -> "SimulationState":
        """Return an equivalent state whose mappings cannot be mutated by a component."""
        return SimulationState(
            time_s=self.time_s,
            step_index=self.step_index,
            dt_s=self.dt_s,
            bodies=MappingProxyType(dict(self.bodies)),
            contacts=MappingProxyType(dict(self.contacts)),
            joint_active=MappingProxyType(dict(self.joint_active)),
            collision_pair_active=MappingProxyType(dict(self.collision_pair_active)),
        )


@dataclass(frozen=True)
class AxialQuantities:
    """Tube-frame quantities derived from a state, in the axial coordinate ``s`` of section 7.

    Every consumer of position uses ``s`` rather than a world component, which is what keeps
    stage lookup, exit detection and telemetry independent of the configured tube angle.
    """

    s_cart_m: float
    s_rocket_m: float
    marker_s_m: Mapping[str, float]
    cart_axial_velocity_mps: float
    rocket_axial_velocity_mps: float
    assembly_mass_kg: float
    stage_index: int
    stage_name: str
    effective_density_ratio: float
    separation_gap_m: float
    separation_rate_mps: float

    def marker(self, name: str) -> float:
        """Return the axial location of a named marker.

        Raises:
            KeyError: If the marker was not configured.
        """
        try:
            return self.marker_s_m[name]
        except KeyError:
            raise KeyError(f"unknown marker '{name}'; known markers: {sorted(self.marker_s_m)}") from None


@dataclass(frozen=True)
class Observation:
    """What a controller is permitted to see.

    The baseline ``ground_truth`` observer copies selected post-step state without noise.
    A later observer may add rate, latency, noise or estimator state; validation keeps the
    latent :class:`SimulationState` regardless, so the two never have to be reconstructed
    from one another.
    """

    source_model: str
    time_s: float
    step_index: int
    dt_s: float
    state: SimulationState
    axial: AxialQuantities
    coupled: bool
    latency_steps: int = 0

    @property
    def cart(self) -> BodyState:
        from .names import BODY_CART

        return self.state.body(BODY_CART)

    @property
    def rocket(self) -> BodyState:
        from .names import BODY_ROCKET

        return self.state.body(BODY_ROCKET)


@dataclass(frozen=True)
class MarkerSpec:
    """A named point rigidly attached to a body.

    Attributes:
        name: Marker identifier from :mod:`.names`.
        body: Body the marker is attached to.
        offset_m: Offset from the body origin, expressed in the *body* frame.
    """

    name: str
    body: str
    offset_m: Vec3 = ZERO3


def marker_world_position(spec: MarkerSpec, state: SimulationState) -> Vec3:
    """Resolve a marker to a world position using the body's current pose."""
    from .linalg import add, quat_rotate

    body = state.body(spec.body)
    return add(body.position, quat_rotate(body.orientation, spec.offset_m))


def body_com_world(body: BodyState) -> Vec3:
    """World position of a body's centre of mass."""
    from .linalg import add, quat_rotate

    return add(body.position, quat_rotate(body.orientation, body.com_offset_m))


def combined_mass(state: SimulationState, bodies: Tuple[str, ...]) -> float:
    """Total mass of the named bodies."""
    return sum(state.body(name).mass_kg for name in bodies)
