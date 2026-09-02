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

"""Stable identifiers for bodies, joints, collision pairs, markers and component slots.

Section 5.1 requires that every effect identify its target body, that constraint and
collision commands identify the exact bodies and requested transition, and that separation
be measured between *named* envelopes. Those names are therefore centralized here rather
than spelled as literals at each use site, so that a rename cannot silently desynchronize
an effect, an ownership rule, a telemetry column and a manifest entry.
"""

from __future__ import annotations

# --- rigid bodies -----------------------------------------------------------------------

BODY_CART = "cart"
BODY_ROCKET = "rocket"

ALL_BODIES = (BODY_CART, BODY_ROCKET)

# --- constraints ------------------------------------------------------------------------

JOINT_COUPLING = "coupling_fixed_joint"
"""Releasable cart-to-rocket fixed joint (section 10.2). Disabled on release, never deleted."""

JOINT_GUIDE = "cart_prismatic_guide"
"""Prismatic guide that constrains the cart to the tube axis and then to the exit track."""

# --- collision pairs --------------------------------------------------------------------

PAIR_ROCKET_CRADLE = "rocket_cradle"
"""Rocket-to-cradle pair. Suppressed while the coupling joint is active (section 10.2)."""

# --- named markers ----------------------------------------------------------------------

MARKER_ROCKET_AFT = "rocket_aft"
"""Aft clearance marker. Controls tube exit only; never reused as a clearance surrogate."""

MARKER_ROCKET_STAGNATION = "rocket_stagnation"
"""Leading stagnation marker at which the active effective density is sampled (section 8)."""

MARKER_ASSEMBLY_EXIT = "assembly_exit"
"""Marker whose exit-plane crossing defines exit speed for section 16.4."""

MARKER_CART_CRADLE_FRONT = "cart_cradle_front"
"""Forward face of the open-front cradle; one side of the separation envelope pair."""

ALL_MARKERS = (
    MARKER_ROCKET_AFT,
    MARKER_ROCKET_STAGNATION,
    MARKER_ASSEMBLY_EXIT,
    MARKER_CART_CRADLE_FRONT,
)

MARKER_BODY_OWNERSHIP = {
    MARKER_ROCKET_AFT: BODY_ROCKET,
    MARKER_ROCKET_STAGNATION: BODY_ROCKET,
    MARKER_ASSEMBLY_EXIT: BODY_ROCKET,
    MARKER_CART_CRADLE_FRONT: BODY_CART,
}
"""Body on which each baseline marker must be authored.

The marker names carry physical roles rather than being interchangeable labels. Sampling
the stagnation marker on the cart, for example, would silently select the wrong density,
and placing the cradle-front marker on the rocket would invalidate the separation geometry.
"""

# --- component slots --------------------------------------------------------------------

SLOT_LAUNCH_FORCE = "launch_force"
SLOT_ATMOSPHERE = "atmosphere"
SLOT_GUIDE = "guide"
SLOT_COUPLING = "coupling"
SLOT_SEPARATION_ACTUATOR = "separation_actuator"
SLOT_CART_BRAKE = "cart_brake"
SLOT_ROCKET_MOTOR = "rocket_motor"
SLOT_ROCKET_AERODYNAMICS = "rocket_aerodynamics"
SLOT_OBSERVER = "observer"
SLOT_BACKEND_ADAPTER = "backend_adapter"
SLOT_TELEMETRY_SINK = "telemetry_sink"

ALL_SLOTS = (
    SLOT_LAUNCH_FORCE,
    SLOT_ATMOSPHERE,
    SLOT_GUIDE,
    SLOT_COUPLING,
    SLOT_SEPARATION_ACTUATOR,
    SLOT_CART_BRAKE,
    SLOT_ROCKET_MOTOR,
    SLOT_ROCKET_AERODYNAMICS,
    SLOT_OBSERVER,
    SLOT_BACKEND_ADAPTER,
    SLOT_TELEMETRY_SINK,
)

PHYSICAL_SLOTS = (
    SLOT_LAUNCH_FORCE,
    SLOT_ATMOSPHERE,
    SLOT_GUIDE,
    SLOT_COUPLING,
    SLOT_SEPARATION_ACTUATOR,
    SLOT_CART_BRAKE,
    SLOT_ROCKET_MOTOR,
    SLOT_ROCKET_AERODYNAMICS,
)
"""Slots permitted to emit physical effects.

The observer is excluded on purpose: section 5.1 keeps sensing outside the physical effect
path so that observation error and actuation error remain separable in the record.
"""

SLOT_BODY_OWNERSHIP = {
    SLOT_LAUNCH_FORCE: frozenset({BODY_CART}),
    SLOT_ATMOSPHERE: frozenset({BODY_CART, BODY_ROCKET}),
    SLOT_GUIDE: frozenset({BODY_CART}),
    SLOT_COUPLING: frozenset(),
    SLOT_SEPARATION_ACTUATOR: frozenset({BODY_CART, BODY_ROCKET}),
    SLOT_CART_BRAKE: frozenset({BODY_CART}),
    SLOT_ROCKET_MOTOR: frozenset({BODY_ROCKET}),
    SLOT_ROCKET_AERODYNAMICS: frozenset({BODY_ROCKET}),
}
"""Which bodies each slot may apply a wrench to.

This is the executable form of the force-ownership rule in section 9.3. The coupling slot
owns no wrench at all: it acts only through constraint and collision-pair commands, which
is what keeps a future piston or ejection model out of the electromagnetic slot.
"""

SLOT_CONSTRAINT_OWNERSHIP = {
    SLOT_COUPLING: frozenset({JOINT_COUPLING}),
    SLOT_GUIDE: frozenset({JOINT_GUIDE}),
}

SLOT_COLLISION_OWNERSHIP = {
    SLOT_COUPLING: frozenset({PAIR_ROCKET_CRADLE}),
}

SLOT_MASS_OWNERSHIP = {
    SLOT_ROCKET_MOTOR: frozenset({BODY_ROCKET}),
}
"""Only the propulsion slot may evolve a body mass, and only the rocket's.

The baseline motor is constant-mass; the rule exists so that a later propellant-depletion
model inherits a checked path rather than needing a new one.
"""

CONSTRAINT_BODIES = {
    JOINT_COUPLING: (BODY_CART, BODY_ROCKET),
    JOINT_GUIDE: (BODY_CART,),
}
"""Exact dynamic-body identities carried by each constraint command.

The guide's other endpoint is the authored world/track, so only the cart is a named rigid
body. The coupling has two named rigid-body endpoints and preserves their canonical order.
"""

COLLISION_PAIR_BODIES = {
    PAIR_ROCKET_CRADLE: (BODY_CART, BODY_ROCKET),
}
"""Exact dynamic-body identities carried by each collision-pair command."""
