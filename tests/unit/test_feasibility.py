# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
import unittest

import _bootstrap  # noqa: F401

from skyarc.launcher.feasibility import (
    EARTH_GRAVITATIONAL_PARAMETER_M3_S2,
    EARTH_MEAN_RADIUS_M,
    STANDARD_GRAVITY_MPS2,
    DeliveredState,
    Stage2Constraint,
    circular_orbit_speed_mps,
    evaluate_stage2,
)


# The state the completed production mission actually handed over at stage-2 ignition.
MEASURED_HANDOFF = DeliveredState(
    time_s=54.679,
    altitude_m=31267.0,
    speed_mps=1991.87,
    flight_path_angle_deg=14.847,
    downrange_m=0.0,
)


class ConstraintTests(unittest.TestCase):
    def test_available_delta_v_is_tsiolkovsky_over_the_declared_fraction(self) -> None:
        constraint = Stage2Constraint(specific_impulse_s=350.0, propellant_mass_fraction=0.85)
        self.assertAlmostEqual(
            constraint.exhaust_velocity_mps, STANDARD_GRAVITY_MPS2 * 350.0, places=9
        )
        self.assertAlmostEqual(
            constraint.delta_v_available_mps,
            STANDARD_GRAVITY_MPS2 * 350.0 * math.log(1.0 / 0.15),
            places=6,
        )

    def test_a_dry_massless_stage_is_rejected_rather_than_reported_as_infinite(self) -> None:
        for fraction in (0.0, 1.0, 1.5, -0.1, float("nan")):
            with self.subTest(propellant_mass_fraction=fraction):
                with self.assertRaisesRegex(ValueError, "propellant mass fraction"):
                    Stage2Constraint(propellant_mass_fraction=fraction)

    def test_constraint_rejects_nonpositive_and_nonfinite_inputs(self) -> None:
        with self.assertRaisesRegex(ValueError, "specific_impulse_s"):
            Stage2Constraint(specific_impulse_s=0.0)
        with self.assertRaisesRegex(ValueError, "target_orbit_altitude_m"):
            Stage2Constraint(target_orbit_altitude_m=-1.0)
        with self.assertRaisesRegex(ValueError, "assumed unmodeled loss"):
            Stage2Constraint(assumed_unmodeled_loss_mps=-1.0)
        with self.assertRaisesRegex(ValueError, "unsupported stage-2 constraint model"):
            Stage2Constraint(model="full_stage_v1")


class OrbitSpeedTests(unittest.TestCase):
    def test_circular_speed_matches_the_textbook_low_orbit_value(self) -> None:
        # ~7.79 km/s at 200 km is the standard figure; this pins the constants together.
        self.assertAlmostEqual(circular_orbit_speed_mps(200000.0), 7788.0, delta=2.0)
        self.assertAlmostEqual(
            circular_orbit_speed_mps(0.0),
            math.sqrt(EARTH_GRAVITATIONAL_PARAMETER_M3_S2 / EARTH_MEAN_RADIUS_M),
            places=6,
        )

    def test_speed_decreases_with_altitude(self) -> None:
        self.assertGreater(circular_orbit_speed_mps(200000.0), circular_orbit_speed_mps(400000.0))


class BudgetTests(unittest.TestCase):
    def test_measured_handoff_reproduces_the_hand_calculation(self) -> None:
        constraint = Stage2Constraint(
            specific_impulse_s=350.0,
            propellant_mass_fraction=0.85,
            target_orbit_altitude_m=200000.0,
            assumed_unmodeled_loss_mps=500.0,
        )
        budget = evaluate_stage2(MEASURED_HANDOFF, constraint)
        # Roughly 6.0 km/s of energy raise before the declared allowance.
        self.assertAlmostEqual(budget.ideal_energy_raise_mps, 6000.0, delta=60.0)
        self.assertGreater(budget.measured_alignment_loss_mps, 0.0)
        self.assertAlmostEqual(budget.delta_v_available_mps, 6512.0, delta=15.0)
        self.assertAlmostEqual(
            budget.margin_mps,
            budget.delta_v_available_mps - budget.delta_v_required_mps,
            places=9,
        )
        self.assertAlmostEqual(budget.target_orbit_speed_mps, 7788.0, delta=2.0)
        self.assertEqual(budget.model, "parametric_deltav_v2")
        self.assertEqual(budget.handoff_time_s, MEASURED_HANDOFF.time_s)

    def test_margin_ranks_the_configurations_the_way_the_hand_table_did(self) -> None:
        """Isp 350/0.80 fails, 350/0.85 is marginal, 450/0.85 passes comfortably."""
        def margin(isp: float, fraction: float) -> float:
            return evaluate_stage2(
                MEASURED_HANDOFF,
                Stage2Constraint(
                    specific_impulse_s=isp,
                    propellant_mass_fraction=fraction,
                    target_orbit_altitude_m=200000.0,
                    assumed_unmodeled_loss_mps=500.0,
                ),
            ).margin_mps

        poor = margin(350.0, 0.80)
        marginal = margin(350.0, 0.85)
        generous = margin(450.0, 0.85)
        self.assertLess(poor, 0.0)
        self.assertLess(poor, marginal)
        self.assertLess(marginal, generous)
        self.assertGreater(generous, 1000.0)

    def test_a_faster_or_higher_handoff_never_needs_more_delta_v(self) -> None:
        """The screen must be monotone, or it cannot rank launcher configurations."""
        constraint = Stage2Constraint()
        base = evaluate_stage2(MEASURED_HANDOFF, constraint).delta_v_required_mps
        for field, delta in (("speed_mps", 100.0), ("altitude_m", 5000.0)):
            with self.subTest(improved=field):
                better = DeliveredState(
                    time_s=MEASURED_HANDOFF.time_s,
                    altitude_m=MEASURED_HANDOFF.altitude_m
                    + (delta if field == "altitude_m" else 0.0),
                    speed_mps=MEASURED_HANDOFF.speed_mps
                    + (delta if field == "speed_mps" else 0.0),
                    flight_path_angle_deg=MEASURED_HANDOFF.flight_path_angle_deg,
                )
                self.assertLess(evaluate_stage2(better, constraint).delta_v_required_mps, base)

    def test_measured_alignment_and_assumed_loss_are_reported_separately(self) -> None:
        """Even an already-sufficient handoff keeps the remaining assumption visible."""
        constraint = Stage2Constraint(
            assumed_unmodeled_loss_mps=500.0,
            target_orbit_altitude_m=200000.0,
        )
        orbital = DeliveredState(
            time_s=0.0, altitude_m=31267.0, speed_mps=20000.0, flight_path_angle_deg=0.0
        )
        budget = evaluate_stage2(orbital, constraint)
        self.assertAlmostEqual(budget.delta_v_required_mps, 500.0, places=9)
        self.assertEqual(budget.measured_alignment_loss_mps, 0.0)
        self.assertEqual(budget.assumed_unmodeled_loss_mps, 500.0)

    def test_a_target_below_the_handoff_is_rejected_rather_than_scored(self) -> None:
        constraint = Stage2Constraint(target_orbit_altitude_m=10000.0)
        with self.assertRaisesRegex(ValueError, "above the handoff altitude"):
            evaluate_stage2(MEASURED_HANDOFF, constraint)

    def test_delivered_state_rejects_nonfinite_and_impossible_values(self) -> None:
        with self.assertRaisesRegex(ValueError, "must be finite"):
            DeliveredState(
                time_s=0.0, altitude_m=float("nan"), speed_mps=1.0, flight_path_angle_deg=0.0
            )
        with self.assertRaisesRegex(ValueError, "speed may not be negative"):
            DeliveredState(
                time_s=0.0, altitude_m=1000.0, speed_mps=-1.0, flight_path_angle_deg=0.0
            )
        with self.assertRaisesRegex(ValueError, "geocentre"):
            DeliveredState(
                time_s=0.0,
                altitude_m=-EARTH_MEAN_RADIUS_M,
                speed_mps=1.0,
                flight_path_angle_deg=0.0,
            )


class ConfigurationIntegrationTests(unittest.TestCase):
    """The authored block must reach the screen with the same admissibility rules."""

    def test_reference_configuration_declares_a_loadable_constraint(self) -> None:
        from pathlib import Path

        from skyarc.configuration import load_yaml

        project = Path(__file__).resolve().parents[2]
        config = load_yaml(project / "configs" / "curved_2kms.yaml").config
        constraint = config.stage2_constraint
        self.assertIsNotNone(constraint)
        self.assertEqual(constraint.model, "parametric_deltav_v2")
        budget = evaluate_stage2(
            MEASURED_HANDOFF,
            Stage2Constraint(
                model=constraint.model,
                specific_impulse_s=constraint.specific_impulse_s,
                propellant_mass_fraction=constraint.propellant_mass_fraction,
                target_orbit_altitude_m=constraint.target_orbit_altitude_m,
                assumed_unmodeled_loss_mps=constraint.assumed_unmodeled_loss_mps,
            ),
        )
        self.assertAlmostEqual(budget.margin_mps, -78.0, delta=30.0)

    def test_the_baseline_declares_no_upper_stage(self) -> None:
        """Absent must mean 'do not score', not 'score with defaults'."""
        from pathlib import Path

        from skyarc.configuration import load_yaml

        project = Path(__file__).resolve().parents[2]
        self.assertIsNone(load_yaml(project / "configs" / "baseline.yaml").config.stage2_constraint)

    def test_preflight_rejects_an_inadmissible_constraint_at_load_time(self) -> None:
        """The config layer must not accept what the screen would later refuse."""
        import tempfile
        from pathlib import Path

        from skyarc.configuration import load_yaml
        from skyarc.configuration.errors import ConfigurationError

        project = Path(__file__).resolve().parents[2]
        source = (project / "configs" / "curved_2kms.yaml").read_text(encoding="utf-8")
        hostile = source.replace(
            "propellant_mass_fraction: 0.85", "propellant_mass_fraction: 1.0"
        )
        self.assertNotEqual(hostile, source)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "hostile.yaml"
            path.write_text(hostile, encoding="utf-8")
            with self.assertRaisesRegex(ConfigurationError, "stage2_constraint"):
                load_yaml(path)


if __name__ == "__main__":
    unittest.main()
