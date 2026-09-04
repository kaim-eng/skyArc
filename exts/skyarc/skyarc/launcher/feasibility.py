# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Parametric upper-stage feasibility screen for the launcher's delivered state.

The project's question is whether the launcher can replace a first stage, not how a second
stage performs.  Modelling a real upper stage would mean variable mass, a thrust profile and
a steering law -- a large amount of machinery whose output, for this question, collapses to
one number: does the stage have enough delta-v left.  So the upper stage is represented here
by the four parameters that determine that number, and the launcher run is scored against
them.

This is deliberately a *screen*, not a trajectory.  The required delta-v is computed as a
single impulsive energy raise applied at the handoff point:

    v_needed = sqrt(2 * (epsilon_target + mu / r_handoff)),   dv = v_needed - v_handoff

It therefore ignores where in the orbit the impulse is applied, any plane change, finite-burn
losses and the shape of the transfer. The measured handoff vector now supplies a distinct
alignment penalty; the remaining ``assumed_unmodeled_loss_mps`` is named separately rather
than conflated with that measurable term. Two configurations are comparable under this
screen; an absolute margin near zero is not a prediction that the vehicle reaches orbit.

The seam for replacing it is :class:`Stage2Constraint.model`.  A later ``full_stage_v1``
becomes a variable-mass component in the effect path -- ``MomentumPolicy.ACCOUNTED``,
``SLOT_MASS_OWNERSHIP`` and the validation that rejects accounted mass flow without an
exhaust velocity already exist for it -- and the same :class:`DeliveredState` interface then
carries a measured insertion state instead of feeding an estimate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


STANDARD_GRAVITY_MPS2 = 9.80665
"""Standard gravity used for the ``Isp -> exhaust velocity`` conversion (CGPM definition)."""

EARTH_GRAVITATIONAL_PARAMETER_M3_S2 = 3.986004418e14
"""WGS-84 geocentric gravitational constant."""

EARTH_MEAN_RADIUS_M = 6371008.8
"""Mean radius. The launcher's altitudes are scene ``+Z``, so a mean radius is the
consistent choice: the scene carries no geodetic latitude that an ellipsoidal radius could
be evaluated at, and claiming one would be false precision against a 20 km altitude scale.
"""


@dataclass(frozen=True)
class Stage2Constraint:
    """The upper stage reduced to what determines its delta-v."""

    model: str = "parametric_deltav_v2"
    specific_impulse_s: float = 350.0
    propellant_mass_fraction: float = 0.85
    target_orbit_altitude_m: float = 200000.0
    assumed_unmodeled_loss_mps: float = 500.0

    def __post_init__(self) -> None:
        if self.model != "parametric_deltav_v2":
            raise ValueError(f"unsupported stage-2 constraint model {self.model!r}")
        for label, value in (
            ("specific_impulse_s", self.specific_impulse_s),
            ("target_orbit_altitude_m", self.target_orbit_altitude_m),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"stage-2 {label} must be finite and positive")
        if (
            not math.isfinite(self.assumed_unmodeled_loss_mps)
            or self.assumed_unmodeled_loss_mps < 0.0
        ):
            raise ValueError("stage-2 assumed unmodeled loss must be finite and nonnegative")
        # A fraction of exactly 1 is a vehicle with no dry mass; the logarithm diverges and
        # the screen would report an unbounded margin for a physically impossible stage.
        if not math.isfinite(self.propellant_mass_fraction) or not (
            0.0 < self.propellant_mass_fraction < 1.0
        ):
            raise ValueError("stage-2 propellant mass fraction must lie strictly in (0, 1)")

    @property
    def exhaust_velocity_mps(self) -> float:
        return STANDARD_GRAVITY_MPS2 * self.specific_impulse_s

    @property
    def delta_v_available_mps(self) -> float:
        """Ideal Tsiolkovsky delta-v for the declared mass fraction."""
        return self.exhaust_velocity_mps * math.log(
            1.0 / (1.0 - self.propellant_mass_fraction)
        )


@dataclass(frozen=True)
class DeliveredState:
    """What the launcher hands over, at the handoff point the screen is evaluated at.

    This is the interface a full upper-stage model would consume unchanged, which is why it
    carries the geometry of the state rather than a single scalar.
    """

    time_s: float
    altitude_m: float
    speed_mps: float
    flight_path_angle_deg: float
    downrange_m: float = 0.0

    def __post_init__(self) -> None:
        values = (
            self.time_s,
            self.altitude_m,
            self.speed_mps,
            self.flight_path_angle_deg,
            self.downrange_m,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("delivered state values must be finite")
        if self.speed_mps < 0.0:
            raise ValueError("delivered speed may not be negative")
        if self.altitude_m <= -EARTH_MEAN_RADIUS_M:
            raise ValueError("delivered altitude places the vehicle at or below the geocentre")


@dataclass(frozen=True)
class Stage2Budget:
    """Result of the screen. ``margin_mps`` is the number the criterion gates on."""

    model: str
    handoff_time_s: float
    handoff_altitude_m: float
    handoff_downrange_m: float
    handoff_speed_mps: float
    handoff_flight_path_angle_deg: float
    target_orbit_altitude_m: float
    target_orbit_speed_mps: float
    ideal_energy_raise_mps: float
    measured_alignment_loss_mps: float
    delta_v_required_mps: float
    delta_v_available_mps: float
    assumed_unmodeled_loss_mps: float
    margin_mps: float

    def to_dict(self) -> dict[str, object]:
        return {
            "model": self.model,
            "handoff_time_s": self.handoff_time_s,
            "handoff_altitude_m": self.handoff_altitude_m,
            "handoff_downrange_m": self.handoff_downrange_m,
            "handoff_speed_mps": self.handoff_speed_mps,
            "handoff_flight_path_angle_deg": self.handoff_flight_path_angle_deg,
            "target_orbit_altitude_m": self.target_orbit_altitude_m,
            "target_orbit_speed_mps": self.target_orbit_speed_mps,
            "ideal_energy_raise_mps": self.ideal_energy_raise_mps,
            "measured_alignment_loss_mps": self.measured_alignment_loss_mps,
            "delta_v_required_mps": self.delta_v_required_mps,
            "delta_v_available_mps": self.delta_v_available_mps,
            "assumed_unmodeled_loss_mps": self.assumed_unmodeled_loss_mps,
            "margin_mps": self.margin_mps,
        }


def circular_orbit_speed_mps(
    altitude_m: float,
    *,
    gravitational_parameter_m3_s2: float = EARTH_GRAVITATIONAL_PARAMETER_M3_S2,
    body_radius_m: float = EARTH_MEAN_RADIUS_M,
) -> float:
    """Circular orbital speed at an altitude above the body's mean radius."""
    radius_m = body_radius_m + altitude_m
    if radius_m <= 0.0:
        raise ValueError("orbital radius must be positive")
    return math.sqrt(gravitational_parameter_m3_s2 / radius_m)


def evaluate_stage2(
    delivered: DeliveredState,
    constraint: Stage2Constraint,
    *,
    gravitational_parameter_m3_s2: float = EARTH_GRAVITATIONAL_PARAMETER_M3_S2,
    body_radius_m: float = EARTH_MEAN_RADIUS_M,
) -> Stage2Budget:
    """Score a delivered state against the parametric upper stage.

    The required delta-v is the impulsive energy raise from the handoff state to the target
    circular orbit's specific energy. The measured handoff flight-path angle supplies the
    vector-alignment correction, and the explicitly assumed residual covers only effects
    the unsimulated upper stage cannot measure here.
    """
    radius_m = body_radius_m + delivered.altitude_m
    if radius_m <= 0.0:
        raise ValueError("handoff radius must be positive")
    target_radius_m = body_radius_m + constraint.target_orbit_altitude_m
    if target_radius_m <= radius_m:
        raise ValueError(
            "the target orbit altitude must lie above the handoff altitude for this screen"
        )

    target_specific_energy = -gravitational_parameter_m3_s2 / (2.0 * target_radius_m)
    speed_needed_squared = 2.0 * (
        target_specific_energy + gravitational_parameter_m3_s2 / radius_m
    )
    if speed_needed_squared <= 0.0:
        raise ValueError("target orbit is unreachable from the handoff radius")
    speed_needed_mps = math.sqrt(speed_needed_squared)

    # The old scalar screen credited all handoff speed, including its vertical component.
    # Use the measured flight-path angle to compute the instantaneous vector correction to
    # a horizontal target velocity. This is still an impulsive screen, but the trajectory
    # penalty is now measured rather than buried in the allowance.
    gamma_rad = math.radians(delivered.flight_path_angle_deg)
    horizontal_speed_mps = delivered.speed_mps * math.cos(gamma_rad)
    vertical_speed_mps = delivered.speed_mps * math.sin(gamma_rad)
    ideal_energy_raise_mps = max(0.0, speed_needed_mps - delivered.speed_mps)
    vector_impulse_mps = math.hypot(
        max(0.0, speed_needed_mps - horizontal_speed_mps),
        vertical_speed_mps,
    )
    measured_alignment_loss_mps = max(0.0, vector_impulse_mps - ideal_energy_raise_mps)
    delta_v_required_mps = (
        ideal_energy_raise_mps
        + measured_alignment_loss_mps
        + constraint.assumed_unmodeled_loss_mps
    )
    delta_v_available_mps = constraint.delta_v_available_mps
    return Stage2Budget(
        model=constraint.model,
        handoff_time_s=delivered.time_s,
        handoff_altitude_m=delivered.altitude_m,
        handoff_downrange_m=delivered.downrange_m,
        handoff_speed_mps=delivered.speed_mps,
        handoff_flight_path_angle_deg=delivered.flight_path_angle_deg,
        target_orbit_altitude_m=constraint.target_orbit_altitude_m,
        target_orbit_speed_mps=circular_orbit_speed_mps(
            constraint.target_orbit_altitude_m,
            gravitational_parameter_m3_s2=gravitational_parameter_m3_s2,
            body_radius_m=body_radius_m,
        ),
        ideal_energy_raise_mps=ideal_energy_raise_mps,
        measured_alignment_loss_mps=measured_alignment_loss_mps,
        delta_v_required_mps=delta_v_required_mps,
        delta_v_available_mps=delta_v_available_mps,
        assumed_unmodeled_loss_mps=constraint.assumed_unmodeled_loss_mps,
        margin_mps=delta_v_available_mps - delta_v_required_mps,
    )


__all__ = [
    "EARTH_GRAVITATIONAL_PARAMETER_M3_S2",
    "EARTH_MEAN_RADIUS_M",
    "STANDARD_GRAVITY_MPS2",
    "DeliveredState",
    "Stage2Budget",
    "Stage2Constraint",
    "circular_orbit_speed_mps",
    "evaluate_stage2",
]
