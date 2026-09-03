# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
import unittest

import _bootstrap  # noqa: F401

from skyarc.launcher.cart_sizing import (
    INDUCTION_PLATE_CART,
    PERMANENT_MAGNET_CART,
    STANDARD_GRAVITY_MPS2,
    CartArchitecture,
    CartDuty,
    CartSizingError,
    evaluate_brake,
    size_cart,
)


# The reference scenario in configs/curved_2kms.yaml, so the sized result can be compared
# against the 250 kg cart the qualified mission actually flew.
REFERENCE_DUTY = CartDuty(
    payload_mass_kg=150.0,
    exit_speed_mps=2000.0,
    launch_length_m=54115.92661767223,
    exit_altitude_m=30976.6,
    maximum_inclination_deg=45.0,
    design_resultant_g=10.0,
    design_normal_g=10.0,
    brake_limit_g=10.0,
    brake_jerk_limit_mps3=50.0,
)


class DutyTests(unittest.TestCase):
    def test_uniform_acceleration_matches_the_flown_mission(self) -> None:
        """Against the value the qualified mission actually commanded, not a rounded one.

        ``command.launch_acceleration_mps2`` in the full-mission telemetry reads
        36.957696652423245 m/s^2 at the first step, and the closed form here agrees to
        eleven significant figures. The model needs no fitting to reproduce the reference
        launcher, which is the only reason its extrapolations are worth reading.
        """
        self.assertAlmostEqual(
            REFERENCE_DUTY.launch_acceleration_mps2, 36.957696652423245, places=9
        )

    def test_gravity_term_is_taken_at_the_steepest_section(self) -> None:
        self.assertAlmostEqual(
            REFERENCE_DUTY.gravity_along_path_mps2,
            STANDARD_GRAVITY_MPS2 * math.sin(math.radians(45.0)),
            places=9,
        )

    def test_inclination_beyond_vertical_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            CartDuty(
                payload_mass_kg=150.0,
                exit_speed_mps=2000.0,
                launch_length_m=54115.9,
                exit_altitude_m=30976.6,
                maximum_inclination_deg=90.0,
            )


class ClosureTests(unittest.TestCase):
    def test_the_solved_mass_satisfies_the_equation_it_came_from(self) -> None:
        """The closure is the model. Verify the answer actually closes."""
        budget = size_cart(PERMANENT_MAGNET_CART, REFERENCE_DUTY)
        reconstructed = (
            budget.drive_mass_kg
            + budget.guide_mass_kg
            + budget.structure_mass_kg
            + budget.fixed_mass_kg
        )
        self.assertAlmostEqual(reconstructed, budget.cart_mass_kg, places=6)
        self.assertAlmostEqual(
            budget.total_accelerated_mass_kg,
            budget.cart_mass_kg + REFERENCE_DUTY.payload_mass_kg,
            places=9,
        )

    def test_peak_thrust_is_consistent_with_the_sized_mass(self) -> None:
        budget = size_cart(PERMANENT_MAGNET_CART, REFERENCE_DUTY)
        expected = budget.total_accelerated_mass_kg * (
            REFERENCE_DUTY.launch_acceleration_mps2 + REFERENCE_DUTY.gravity_along_path_mps2
        )
        self.assertAlmostEqual(budget.peak_thrust_n, expected, places=6)

    def test_a_drive_too_weak_to_lift_itself_is_refused_not_approximated(self) -> None:
        # Specific thrust below the acceleration demand makes kappa exceed 1, at which point
        # the closed form would return a *negative* mass and look like a small cart.
        hopeless = CartArchitecture(drive_specific_thrust_n_per_kg=20.0)
        with self.assertRaises(CartSizingError):
            size_cart(hopeless, REFERENCE_DUTY)

    def test_closure_factor_stays_below_one_for_the_presets(self) -> None:
        for architecture in (PERMANENT_MAGNET_CART, INDUCTION_PLATE_CART):
            budget = size_cart(architecture, REFERENCE_DUTY)
            self.assertLess(budget.closure_factor, 1.0)
            self.assertGreater(budget.cart_mass_kg, 0.0)


class ArchitectureTests(unittest.TestCase):
    def test_passive_cart_is_substantially_lighter_than_permanent_magnet(self) -> None:
        magnet = size_cart(PERMANENT_MAGNET_CART, REFERENCE_DUTY)
        induction = size_cart(INDUCTION_PLATE_CART, REFERENCE_DUTY)
        self.assertLess(induction.cart_mass_kg, magnet.cart_mass_kg)
        # Moving the active elements into the track is the decisive choice, not a marginal
        # one; if this ever falls below a third the presets have drifted apart.
        self.assertLess(induction.cart_mass_kg, 0.67 * magnet.cart_mass_kg)

    def test_lighter_cart_raises_the_payload_energy_fraction(self) -> None:
        magnet = size_cart(PERMANENT_MAGNET_CART, REFERENCE_DUTY)
        induction = size_cart(INDUCTION_PLATE_CART, REFERENCE_DUTY)
        self.assertGreater(
            induction.payload_energy_fraction, magnet.payload_energy_fraction
        )

    def test_payload_mass_ratio_below_one_means_the_cart_outweighs_the_rocket(self) -> None:
        magnet = size_cart(PERMANENT_MAGNET_CART, REFERENCE_DUTY)
        self.assertAlmostEqual(
            magnet.payload_mass_ratio,
            REFERENCE_DUTY.payload_mass_kg / magnet.cart_mass_kg,
            places=9,
        )


class BrakeTests(unittest.TestCase):
    def test_track_length_is_independent_of_cart_mass(self) -> None:
        """The point that makes cart mass and track length separate problems."""
        light = evaluate_brake(40.0, REFERENCE_DUTY)
        heavy = evaluate_brake(250.0, REFERENCE_DUTY)
        self.assertAlmostEqual(light.ideal_track_m, heavy.ideal_track_m, places=9)
        self.assertAlmostEqual(light.ramped_track_m, heavy.ramped_track_m, places=9)
        # Force, power and energy do scale with mass, in proportion.
        self.assertAlmostEqual(heavy.peak_force_n / light.peak_force_n, 6.25, places=9)
        self.assertAlmostEqual(heavy.dissipated_energy_j / light.dissipated_energy_j, 6.25, places=9)

    def test_ideal_track_is_the_textbook_result(self) -> None:
        brake = evaluate_brake(250.0, REFERENCE_DUTY)
        self.assertAlmostEqual(
            brake.ideal_track_m,
            2000.0**2 / (2.0 * 10.0 * STANDARD_GRAVITY_MPS2),
            places=6,
        )

    def test_jerk_ramp_lengthens_the_track(self) -> None:
        brake = evaluate_brake(250.0, REFERENCE_DUTY)
        self.assertGreater(brake.ramped_track_m, brake.ideal_track_m)

    def test_raising_the_limit_shortens_the_track_and_raises_the_power(self) -> None:
        gentle = evaluate_brake(250.0, REFERENCE_DUTY)
        hard = evaluate_brake(
            250.0,
            CartDuty(
                payload_mass_kg=150.0,
                exit_speed_mps=2000.0,
                launch_length_m=54115.92661767223,
                exit_altitude_m=30976.6,
                brake_limit_g=50.0,
            ),
        )
        # Distance goes as 1/a, so a five-fold limit is a five-fold shorter ideal track.
        self.assertAlmostEqual(gentle.ideal_track_m / hard.ideal_track_m, 5.0, places=6)
        self.assertAlmostEqual(hard.peak_power_w / gentle.peak_power_w, 5.0, places=6)
        self.assertLess(hard.exit_structure_fraction, gentle.exit_structure_fraction)

    def test_energy_is_the_carts_kinetic_energy(self) -> None:
        budget = size_cart(PERMANENT_MAGNET_CART, REFERENCE_DUTY)
        brake = evaluate_brake(budget.cart_mass_kg, REFERENCE_DUTY)
        self.assertAlmostEqual(
            brake.dissipated_energy_j, budget.cart_kinetic_energy_j, places=6
        )

    def test_a_brake_that_stops_during_its_own_ramp_is_handled(self) -> None:
        # A slow vehicle under a high limit never reaches constant deceleration.
        duty = CartDuty(
            payload_mass_kg=150.0,
            exit_speed_mps=5.0,
            launch_length_m=1000.0,
            exit_altitude_m=100.0,
            brake_limit_g=50.0,
            brake_jerk_limit_mps3=1.0,
        )
        brake = evaluate_brake(100.0, duty)
        self.assertGreater(brake.ramped_track_m, 0.0)
        self.assertTrue(math.isfinite(brake.stop_time_s))


if __name__ == "__main__":
    unittest.main()
