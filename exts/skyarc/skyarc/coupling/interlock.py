# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Fail-closed seven-gate rocket ignition interlock."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Tuple

from ..configuration.schema import IgnitionConfig
from ..launcher.geometry import TubePath
from ..linalg import is_finite, norm
from ..names import JOINT_COUPLING, MARKER_ROCKET_AFT, PAIR_ROCKET_CRADLE
from ..state import MarkerSpec, Observation


@dataclass(frozen=True)
class InterlockDecision:
    allowed: bool
    blocked_gates: Tuple[str, ...]
    gate_status: Mapping[str, bool]


class IgnitionInterlock:
    """Evaluate all ignition gates from one immutable observation."""

    GATE_ORDER = (
        "attachment_released",
        "rocket_outside_tube",
        "clearance_confirmed",
        "ignition_delay_elapsed",
        "no_active_collision",
        "finite_bounded_rocket_state",
        "no_abort",
    )

    def __init__(
        self,
        criteria: IgnitionConfig,
        layout: TubePath,
        markers: Mapping[str, MarkerSpec],
    ) -> None:
        if MARKER_ROCKET_AFT not in markers:
            raise ValueError("ignition interlock requires the rocket aft marker")
        self._criteria = criteria
        self._layout = layout
        self._markers = dict(markers)

    def evaluate(
        self,
        observation: Observation,
        *,
        release_time_s: float | None,
        separation_confirmed: bool,
        abort_active: bool,
    ) -> InterlockDecision:
        rocket = observation.rocket
        rocket_values = (
            *rocket.position,
            *rocket.orientation,
            *rocket.linear_velocity,
            *rocket.angular_velocity,
            rocket.mass_kg,
        )
        quaternion_norm = math.sqrt(sum(value * value for value in rocket.orientation))
        state_valid = (
            is_finite(rocket_values)
            and rocket.mass_kg > 0.0
            and math.isclose(quaternion_norm, 1.0, rel_tol=0.0, abs_tol=1e-6)
            and math.degrees(norm(rocket.angular_velocity))
            <= self._criteria.maximum_angular_rate_deg_s + 1e-12
        )
        delay_elapsed = (
            release_time_s is not None
            and observation.time_s + 1e-12 >= release_time_s + self._criteria.delay_s
        )
        gate_status = {
            "attachment_released": not observation.state.joint_active.get(JOINT_COUPLING, False),
            "rocket_outside_tube": observation.axial.marker(MARKER_ROCKET_AFT)
            >= self._layout.length_m,
            "clearance_confirmed": separation_confirmed
            and observation.axial.separation_gap_m >= self._criteria.minimum_cart_clearance_m,
            "ignition_delay_elapsed": delay_elapsed,
            "no_active_collision": not observation.state.contact(PAIR_ROCKET_CRADLE).active,
            "finite_bounded_rocket_state": state_valid,
            "no_abort": not abort_active,
        }
        blocked = tuple(name for name in self.GATE_ORDER if not gate_status[name])
        return InterlockDecision(
            allowed=not blocked,
            blocked_gates=blocked,
            gate_status=gate_status,
        )
