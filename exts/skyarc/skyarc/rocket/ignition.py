# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Trajectory-conditioned ignition timing, separate from the safety interlock."""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping

from ..configuration.schema import IgnitionTriggerConfig
from ..state import Observation


@dataclass(frozen=True)
class IgnitionTriggerDecision:
    ready: bool
    condition_status: Mapping[str, bool]
    measurements: Mapping[str, float]


class TrajectoryIgnitionTrigger:
    """Decide when to ignite from immutable global-SI rocket state.

    This object deliberately knows nothing about coupling, clearance, collision, or
    abort state.  Those remain fail-closed responsibilities of ``IgnitionInterlock``.
    """

    def __init__(self, config: IgnitionTriggerConfig) -> None:
        self._config = config

    def evaluate(self, observation: Observation) -> IgnitionTriggerDecision:
        if self._config.model == "safety_gates_only_v1":
            return IgnitionTriggerDecision(
                ready=True,
                condition_status=MappingProxyType({}),
                measurements=MappingProxyType({}),
            )
        if self._config.model != "trajectory_thresholds_v1":
            raise ValueError(f"unsupported ignition trigger model {self._config.model!r}")

        rocket = observation.rocket
        altitude_m = rocket.position[2]
        vertical_speed_mps = rocket.linear_velocity[2]
        horizontal_speed_mps = math.hypot(
            rocket.linear_velocity[0], rocket.linear_velocity[1]
        )
        flight_path_angle_deg = math.degrees(
            math.atan2(vertical_speed_mps, horizontal_speed_mps)
        )
        measurements = {
            "altitude_m": altitude_m,
            "flight_path_angle_deg": flight_path_angle_deg,
            "vertical_speed_mps": vertical_speed_mps,
        }
        finite = all(math.isfinite(value) for value in measurements.values())
        status: dict[str, bool] = {}
        if self._config.minimum_altitude_m is not None:
            status["minimum_altitude"] = (
                finite and altitude_m + 1e-12 >= self._config.minimum_altitude_m
            )
        if self._config.maximum_flight_path_angle_deg is not None:
            status["maximum_flight_path_angle"] = (
                finite
                and flight_path_angle_deg
                <= self._config.maximum_flight_path_angle_deg + 1e-12
            )
        if self._config.maximum_vertical_speed_mps is not None:
            status["maximum_vertical_speed"] = (
                finite
                and vertical_speed_mps <= self._config.maximum_vertical_speed_mps + 1e-12
            )
        return IgnitionTriggerDecision(
            ready=bool(status) and all(status.values()),
            condition_status=MappingProxyType(status),
            measurements=MappingProxyType(measurements),
        )
