# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Turn cart mass from an input assumption into a sized output.

Until this module existed, ``cart.mass_kg`` was a declared number -- the docs call it "the
250 kg reference cart" -- and nothing in the simulation resisted changing it.  Set it to
25 kg and every figure improves: less launch force, less brake energy, a higher payload
energy fraction, and no modelled consequence anywhere.  A sweep over an input like that can
only ever conclude "lighter is better", which is not an answer.  The physics that actually
sets cart mass -- how much drive hardware a given thrust needs, how much structure a given
load needs -- was simply absent.

That absence matters more than it first appears, because the cart is accelerated to the
full exit speed and lifted to the full exit altitude alongside the payload.  At the 250 kg
reference against a 150 kg rocket, **62.5% of the launcher's kinetic output goes into the
cart**, and all of it has to be taken back out again through the exit brake track.  Cart
mass is therefore upstream of both the energy efficiency of the launcher and the length of
the structure needed to stop the cart.

The model closes on itself, which is its whole point.  The drive must accelerate its own
mass, so drive mass depends on total mass, which contains cart mass:

    m_cart = m_drive + m_guide + m_structure + m_fixed

    m_drive = max( (m_cart + m_payload) * (a_launch + g sin(theta)),
                   m_cart * a_brake ) / sigma_drive

The ``max`` is the part that is easy to miss.  Braking runs back through the same magnetic
interface that did the accelerating, and it acts on the cart *alone* but at the full brake
limit, so at the reference launcher it is the larger of the two by roughly an order of
magnitude.  Sizing the drive from launch thrust only -- which this module did in its first
draft -- silently under-builds it.

That coupling produces the model's sharpest result.  Drive mass is ``F / sigma`` and the
brake force is ``m_cart * a_brake``, so the drive's share of the cart is ``a_brake /
sigma``: **the specific thrust is a hard ceiling on brake deceleration**, reached when the
cart is entirely drive hardware.  Cart mass cancels out of that ratio, so a lighter cart
buys no extra deceleration -- it is the wrong lever.  Worse, pushing the brake limit up
makes the cart *heavier*, which raises the brake force, which demands more drive.  A
configuration can therefore fail by divergence, and :class:`CartSizingError` reports that
rather than returning a plausible-looking number.

**The coefficients are order-of-magnitude engineering estimates, not validated data.**  They
are stated as specific quantities -- newtons of thrust per kilogram of drive hardware, and
so on -- precisely so that a reader can disagree with a number instead of with a result.
The presets below carry their derivations.  Nothing here is measured by the simulation, and
this module deliberately imports nothing from the rest of the package: like
``feasibility.py``, it is a screen that a detailed model can later replace at the
:attr:`CartArchitecture.model` seam.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


STANDARD_GRAVITY_MPS2 = 9.80665
"""Standard gravity, used for load factors quoted in G."""


@dataclass(frozen=True)
class CartArchitecture:
    """How the cart is built, reduced to the four numbers that set its mass.

    ``drive_specific_thrust_n_per_kg`` is the parameter that distinguishes the candidate
    architectures from one another, and it is where the decisive design choice lives: which
    half of the linear motor carries the heavy, expensive, active elements.  Put them in the
    track -- which is fixed, and reused on every launch -- and the moving element can be
    close to passive.
    """

    model: str = "specific_thrust_v1"
    drive_specific_thrust_n_per_kg: float = 220.0
    guide_specific_load_n_per_kg: float = 3000.0
    structure_specific_load_n_per_kg: float = 1500.0
    fixed_mass_kg: float = 8.0
    safety_factor: float = 1.5

    def __post_init__(self) -> None:
        if self.model != "specific_thrust_v1":
            raise ValueError(f"unsupported cart architecture model {self.model!r}")
        for label, value in (
            ("drive_specific_thrust_n_per_kg", self.drive_specific_thrust_n_per_kg),
            ("guide_specific_load_n_per_kg", self.guide_specific_load_n_per_kg),
            ("structure_specific_load_n_per_kg", self.structure_specific_load_n_per_kg),
            ("safety_factor", self.safety_factor),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"cart architecture {label} must be finite and positive")
        if not math.isfinite(self.fixed_mass_kg) or self.fixed_mass_kg < 0.0:
            raise ValueError("cart architecture fixed mass must be finite and nonnegative")


PERMANENT_MAGNET_CART = CartArchitecture(
    drive_specific_thrust_n_per_kg=220.0,
)
"""Halbach array on the cart, coils in the track.

Roughly 30 kN/m^2 of thrust at a practical gap, against an NdFeB array about 18 mm thick at
7500 kg/m^3, so about 135 kg/m^2 of moving hardware: 30000 / 135 is near 220 N/kg.  This is
the architecture the 250 kg reference cart is consistent with, which is worth saying
plainly -- 250 kg is a defensible number for a permanent-magnet cart, not a careless one.
"""

INDUCTION_PLATE_CART = CartArchitecture(
    drive_specific_thrust_n_per_kg=1100.0,
)
"""Aluminium reaction plate on the cart, everything active in the track.

A linear induction drive develops less thrust per unit area -- call it 15 kN/m^2 -- but the
moving element is a 5 mm aluminium plate at about 13.5 kg/m^2, so 15000 / 13.5 is near
1100 N/kg, five times the permanent-magnet figure.  The cost is efficiency and track-side
power, which this screen does not model; it sizes mass, not electricity.
"""

THIN_PLATE_CART = CartArchitecture(
    drive_specific_thrust_n_per_kg=2600.0,
)
"""A reaction plate thinned to the skin depth at launch speed.

The plate in :data:`INDUCTION_PLATE_CART` is 5 mm because that is a conventional figure at
conventional speeds.  It is the wrong thickness here.  Slip frequency scales with speed, and
at 2000 m/s against a half-metre pole pitch the electrical frequency is of order 2 kHz,
where the skin depth in aluminium -- ``sqrt(2 rho / omega mu)`` with rho = 2.8e-8 ohm m -- is
about 1.9 mm.  Conductor thicker than that carries little of the induced current and is
close to dead mass.

Sizing the plate to roughly 2.5 mm gives about 6.8 kg/m^2, and at a conservative 18 kN/m^2
that is near 2600 N/kg: a ceiling of about 265 G rather than 112 G.  The high speed that
makes this launcher hard is, for once, what makes the brake interface light.

This is the architecture implied by treating the released cart as "just a piece of metal",
and it is the only preset here under which a 100 G brake closes.  Two caveats travel with
it.  The force density is an estimate at a speed where linear-machine data is thin.  And it
must be **regenerative**: see :func:`evaluate_brake` on why a passive eddy-current brake
cannot survive its own dissipation.
"""

SUPERCONDUCTING_CART = CartArchitecture(
    drive_specific_thrust_n_per_kg=600.0,
    fixed_mass_kg=25.0,
)
"""Superconducting coils on the cart.

High force density, but the cryostat is both mass and a fixed overhead that does not shrink
with thrust, which is why the fixed term rises rather than the specific thrust simply being
set high.
"""


@dataclass(frozen=True)
class CartDuty:
    """What the cart has to survive, and over what geometry.

    Everything here is either read from the scenario configuration or derived from it, so a
    duty cycle can be built for any candidate launcher without touching this module.
    """

    payload_mass_kg: float
    exit_speed_mps: float
    launch_length_m: float
    exit_altitude_m: float
    maximum_inclination_deg: float = 45.0
    design_resultant_g: float = 10.0
    design_normal_g: float = 10.0
    brake_limit_g: float = 10.0
    brake_jerk_limit_mps3: float = 50.0

    def __post_init__(self) -> None:
        for label, value in (
            ("payload_mass_kg", self.payload_mass_kg),
            ("exit_speed_mps", self.exit_speed_mps),
            ("launch_length_m", self.launch_length_m),
            ("design_resultant_g", self.design_resultant_g),
            ("design_normal_g", self.design_normal_g),
            ("brake_limit_g", self.brake_limit_g),
            ("brake_jerk_limit_mps3", self.brake_jerk_limit_mps3),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"cart duty {label} must be finite and positive")
        if not math.isfinite(self.exit_altitude_m):
            raise ValueError("cart duty exit altitude must be finite")
        if not -90.0 < self.maximum_inclination_deg < 90.0:
            raise ValueError("cart duty maximum inclination must lie strictly within +/-90 deg")

    @property
    def launch_acceleration_mps2(self) -> float:
        """Uniform acceleration reaching the exit speed over the guided length.

        The reference scenario is built this way and runs at a measured 36.97 m/s^2 against
        the 36.96 m/s^2 this returns, so the uniform assumption is not costing anything at
        the reference point.  A profiled launch would raise the peak above this.
        """
        return self.exit_speed_mps**2 / (2.0 * self.launch_length_m)

    @property
    def gravity_along_path_mps2(self) -> float:
        """Gravity's component along the steepest part of the climb.

        Taken at the maximum inclination because peak thrust sizes the drive, and the
        steepest section is where the drive works hardest against gravity.
        """
        return STANDARD_GRAVITY_MPS2 * math.sin(math.radians(self.maximum_inclination_deg))


@dataclass(frozen=True)
class CartMassBudget:
    """A sized cart, with the breakdown that explains the total."""

    model: str
    cart_mass_kg: float
    drive_mass_kg: float
    guide_mass_kg: float
    structure_mass_kg: float
    fixed_mass_kg: float
    total_accelerated_mass_kg: float
    peak_thrust_n: float
    brake_case_binding: bool
    closure_factor: float
    payload_energy_fraction: float
    kinetic_energy_total_j: float
    cart_kinetic_energy_j: float

    @property
    def payload_mass_ratio(self) -> float:
        """Payload mass over cart mass. Below 1 means the cart outweighs what it launches."""
        return (self.total_accelerated_mass_kg - self.cart_mass_kg) / self.cart_mass_kg

    def to_dict(self) -> dict[str, object]:
        return {
            "model": self.model,
            "cart_mass_kg": self.cart_mass_kg,
            "drive_mass_kg": self.drive_mass_kg,
            "guide_mass_kg": self.guide_mass_kg,
            "structure_mass_kg": self.structure_mass_kg,
            "fixed_mass_kg": self.fixed_mass_kg,
            "total_accelerated_mass_kg": self.total_accelerated_mass_kg,
            "peak_thrust_n": self.peak_thrust_n,
            "brake_case_binding": self.brake_case_binding,
            "closure_factor": self.closure_factor,
            "payload_energy_fraction": self.payload_energy_fraction,
            "payload_mass_ratio": self.payload_mass_ratio,
            "kinetic_energy_total_j": self.kinetic_energy_total_j,
            "cart_kinetic_energy_j": self.cart_kinetic_energy_j,
        }


@dataclass(frozen=True)
class BrakeBudget:
    """What stopping the cart costs, in structure and in power."""

    deceleration_mps2: float
    ideal_track_m: float
    ramped_track_m: float
    stop_time_s: float
    peak_force_n: float
    peak_power_w: float
    dissipated_energy_j: float
    exit_structure_fraction: float
    adiabatic_temperature_rise_k: float
    regeneration_required: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "deceleration_mps2": self.deceleration_mps2,
            "ideal_track_m": self.ideal_track_m,
            "ramped_track_m": self.ramped_track_m,
            "stop_time_s": self.stop_time_s,
            "peak_force_n": self.peak_force_n,
            "peak_power_w": self.peak_power_w,
            "dissipated_energy_j": self.dissipated_energy_j,
            "exit_structure_fraction": self.exit_structure_fraction,
            "adiabatic_temperature_rise_k": self.adiabatic_temperature_rise_k,
            "regeneration_required": self.regeneration_required,
        }


class CartSizingError(ValueError):
    """Raised when a duty and architecture admit no finite cart."""


def size_cart(architecture: CartArchitecture, duty: CartDuty) -> CartMassBudget:
    """Solve the cart's mass closure for a given architecture and duty.

    Two load cases size the drive, and which one binds is a real design finding rather than
    a detail:

    *Launch* accelerates cart and payload together, so it demands
    ``(m_cart + m_payload) * (a_launch + g sin theta)``.

    *Braking* acts on the cart alone -- the payload has left -- but at the full brake limit,
    so it demands ``m_cart * a_brake``.  Because the cart is decelerated through the same
    magnetic interface that accelerated it, this force passes through the same hardware, and
    at the reference launcher the brake case is roughly an order of magnitude larger.  A
    model that sized the drive from launch alone would quietly under-build it.

    The consequence is worth stating in the open: since drive mass is ``F / sigma`` and the
    brake force is ``m_cart * a_brake``, the drive fraction of the cart is ``a_brake /
    sigma``.  **The specific thrust is therefore a hard ceiling on brake deceleration** --
    at ``a_brake = sigma`` the cart is entirely drive hardware.  Cart mass does not enter
    that ratio at all, so a lighter cart buys no additional deceleration.

    The ``max`` over the two cases makes the closure piecewise, so it is solved by fixed
    point rather than by the closed form.  Divergence is reported rather than returned as a
    plausible-looking number.
    """
    launch_demand_mps2 = duty.launch_acceleration_mps2 + duty.gravity_along_path_mps2
    normal_demand_mps2 = (
        duty.design_normal_g * STANDARD_GRAVITY_MPS2 * architecture.safety_factor
    )
    brake_demand_mps2 = duty.brake_limit_g * STANDARD_GRAVITY_MPS2

    # The cradle reacts the payload's design load only. It does not carry the cart's own
    # drive or guide hardware, which react into the track directly, and it is empty during
    # braking, so the brake limit never sizes it.
    structure_mass_kg = (
        duty.payload_mass_kg
        * duty.design_resultant_g
        * STANDARD_GRAVITY_MPS2
        * architecture.safety_factor
    ) / architecture.structure_specific_load_n_per_kg
    additive_kg = structure_mass_kg + architecture.fixed_mass_kg

    # The guide is sized by the launch case: the exit track is straight, so a braking cart
    # carries no curvature load, only gravity normal to a 15-degree grade.
    def _iterate(cart_mass_kg: float) -> float:
        total_kg = cart_mass_kg + duty.payload_mass_kg
        thrust_n = max(total_kg * launch_demand_mps2, cart_mass_kg * brake_demand_mps2)
        drive_kg = thrust_n / architecture.drive_specific_thrust_n_per_kg
        guide_kg = (total_kg * normal_demand_mps2) / architecture.guide_specific_load_n_per_kg
        return drive_kg + guide_kg + additive_kg

    if brake_demand_mps2 >= architecture.drive_specific_thrust_n_per_kg:
        raise CartSizingError(
            f"a {duty.brake_limit_g:.0f} G brake demands "
            f"{brake_demand_mps2:.0f} N per kg of cart, but the drive delivers only "
            f"{architecture.drive_specific_thrust_n_per_kg:.0f} N/kg, so the interface "
            "needed to stop the cart would outweigh the cart. This ceiling is set by the "
            "drive's specific thrust and cannot be bought back by making the cart lighter."
        )

    cart_mass_kg = duty.payload_mass_kg
    for _ in range(200):
        updated = _iterate(cart_mass_kg)
        if not math.isfinite(updated) or updated > 1.0e9:
            break
        if abs(updated - cart_mass_kg) <= 1.0e-12 * max(1.0, updated):
            cart_mass_kg = updated
            break
        cart_mass_kg = updated
    else:  # pragma: no cover - guarded by the ceiling check above
        raise CartSizingError("cart mass closure did not converge")
    if not math.isfinite(cart_mass_kg) or cart_mass_kg <= 0.0 or cart_mass_kg > 1.0e9:
        raise CartSizingError(
            "no finite cart closes this duty; the hardware needed to accelerate and stop "
            "the cart grows faster than the cart itself"
        )

    total_mass_kg = cart_mass_kg + duty.payload_mass_kg
    peak_thrust_n = max(
        total_mass_kg * launch_demand_mps2, cart_mass_kg * brake_demand_mps2
    )
    drive_mass_kg = peak_thrust_n / architecture.drive_specific_thrust_n_per_kg
    guide_mass_kg = (
        total_mass_kg * normal_demand_mps2
    ) / architecture.guide_specific_load_n_per_kg
    kappa = (
        peak_thrust_n / architecture.drive_specific_thrust_n_per_kg
        + total_mass_kg * normal_demand_mps2 / architecture.guide_specific_load_n_per_kg
    ) / total_mass_kg

    kinetic_total_j = 0.5 * total_mass_kg * duty.exit_speed_mps**2
    cart_kinetic_j = 0.5 * cart_mass_kg * duty.exit_speed_mps**2
    # Potential energy is counted too: the cart is lifted to the exit altitude as surely as
    # the payload is, and at 31 km that is not a rounding term.
    potential_total_j = total_mass_kg * STANDARD_GRAVITY_MPS2 * duty.exit_altitude_m
    payload_share_j = (
        0.5 * duty.payload_mass_kg * duty.exit_speed_mps**2
        + duty.payload_mass_kg * STANDARD_GRAVITY_MPS2 * duty.exit_altitude_m
    )
    delivered_total_j = kinetic_total_j + potential_total_j

    return CartMassBudget(
        model=architecture.model,
        cart_mass_kg=cart_mass_kg,
        drive_mass_kg=drive_mass_kg,
        guide_mass_kg=guide_mass_kg,
        structure_mass_kg=structure_mass_kg,
        fixed_mass_kg=architecture.fixed_mass_kg,
        total_accelerated_mass_kg=total_mass_kg,
        peak_thrust_n=peak_thrust_n,
        brake_case_binding=cart_mass_kg * brake_demand_mps2 > total_mass_kg * launch_demand_mps2,
        closure_factor=kappa,
        payload_energy_fraction=(
            payload_share_j / delivered_total_j if delivered_total_j > 0.0 else 0.0
        ),
        kinetic_energy_total_j=kinetic_total_j,
        cart_kinetic_energy_j=cart_kinetic_j,
    )


def evaluate_brake(
    cart_mass_kg: float,
    duty: CartDuty,
    *,
    specific_heat_j_per_kg_k: float = 900.0,
    melting_rise_k: float = 640.0,
) -> BrakeBudget:
    """Size the exit brake track for a cart of a given mass.

    Stopping distance is ``v^2 / 2a`` and **mass does not appear in it**: a lighter cart
    needs the same track at the same deceleration.  Mass sets the force, the power and the
    energy; only the deceleration limit sets the length.  Both are reported because they
    trade against each other -- shortening the track means a higher instantaneous power the
    brake section has to handle.

    ``ramped_track_m`` adds the distance lost while the brake ramps in under its jerk limit,
    which is the difference between an idealised figure and a realisable one.  Both are
    computed without drag or grade; the qualified reference run measures 23.0 km against
    this model's 24.4 km at the same 9.01 G, the difference being the drag and the uphill
    grade that both help the real cart stop.
    """
    if not math.isfinite(cart_mass_kg) or cart_mass_kg <= 0.0:
        raise ValueError("cart mass must be finite and positive")

    deceleration_mps2 = duty.brake_limit_g * STANDARD_GRAVITY_MPS2
    speed = duty.exit_speed_mps
    ideal_track_m = speed**2 / (2.0 * deceleration_mps2)

    ramp_time_s = deceleration_mps2 / duty.brake_jerk_limit_mps3
    speed_lost_in_ramp = 0.5 * deceleration_mps2 * ramp_time_s
    if speed_lost_in_ramp >= speed:
        # The cart stops before the brake has even reached full force, so the ramp integral
        # is the whole story and the constant-deceleration tail does not exist.
        ramp_time_s = math.sqrt(2.0 * speed / duty.brake_jerk_limit_mps3)
        ramped_track_m = speed * ramp_time_s - (duty.brake_jerk_limit_mps3 / 6.0) * ramp_time_s**3
        stop_time_s = ramp_time_s
    else:
        ramp_distance_m = (
            speed * ramp_time_s - (duty.brake_jerk_limit_mps3 / 6.0) * ramp_time_s**3
        )
        remaining_speed = speed - speed_lost_in_ramp
        ramped_track_m = ramp_distance_m + remaining_speed**2 / (2.0 * deceleration_mps2)
        stop_time_s = ramp_time_s + remaining_speed / deceleration_mps2

    # If the brake were passive -- an eddy-current plate dumping its energy as heat in the
    # cart -- the cart would have to absorb its own kinetic energy. That is 0.5 v^2 per
    # kilogram regardless of mass, which at 2000 m/s is 2.0 MJ/kg. Aluminium needs about
    # 0.97 MJ/kg to reach its melting point from cold, so the cart would not merely
    # overheat, it would be destroyed several times over. The brake therefore has to be
    # regenerative, returning energy to the track rather than dissipating it on board.
    energy_j = 0.5 * cart_mass_kg * speed**2
    temperature_rise_k = (0.5 * speed**2) / specific_heat_j_per_kg_k

    return BrakeBudget(
        deceleration_mps2=deceleration_mps2,
        ideal_track_m=ideal_track_m,
        ramped_track_m=ramped_track_m,
        stop_time_s=stop_time_s,
        peak_force_n=cart_mass_kg * deceleration_mps2,
        peak_power_w=cart_mass_kg * deceleration_mps2 * speed,
        dissipated_energy_j=energy_j,
        adiabatic_temperature_rise_k=temperature_rise_k,
        regeneration_required=temperature_rise_k > melting_rise_k,
        # What fraction of the total elevated structure exists only to stop the cart. The
        # guided length does work on the payload; the brake track does none.
        exit_structure_fraction=ramped_track_m / (duty.launch_length_m + ramped_track_m),
    )


__all__ = [
    "INDUCTION_PLATE_CART",
    "PERMANENT_MAGNET_CART",
    "THIN_PLATE_CART",
    "STANDARD_GRAVITY_MPS2",
    "SUPERCONDUCTING_CART",
    "BrakeBudget",
    "CartArchitecture",
    "CartDuty",
    "CartMassBudget",
    "CartSizingError",
    "evaluate_brake",
    "size_cart",
]


