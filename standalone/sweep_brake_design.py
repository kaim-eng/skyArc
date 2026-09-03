# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Sweep cart architecture, brake load and jerk for the 2 km/s reference mission.

This is an analytic design study, not qualification evidence.  It deliberately leaves the
qualified mission configuration untouched and uses the conservative level-track/no-drag
brake calculation in :mod:`skyarc.launcher.cart_sizing`.  The configured stop margin is
then added explicitly.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[1]
EXTENSION_ROOT = PROJECT_ROOT / "exts" / "skyarc"
if str(EXTENSION_ROOT) not in sys.path:
    sys.path.insert(0, str(EXTENSION_ROOT))

from skyarc.configuration.loader import load_yaml  # noqa: E402
from skyarc.configuration.validation import resolve_centerline  # noqa: E402
from skyarc.launcher.cart_sizing import (  # noqa: E402
    INDUCTION_PLATE_CART,
    PERMANENT_MAGNET_CART,
    STANDARD_GRAVITY_MPS2,
    SUPERCONDUCTING_CART,
    THIN_PLATE_CART,
    CartArchitecture,
    CartDuty,
    CartSizingError,
    evaluate_brake,
    size_cart,
)


SCHEMA = "skyarc_brake_design_sweep_v1"
DEFAULT_G_VALUES = (10.0, 15.0, 20.0, 25.0, 30.0, 40.0, 50.0, 60.0, 75.0, 100.0)
DEFAULT_JERK_VALUES_MPS3 = (50.0, 100.0, 200.0, 300.0, 500.0, 1000.0)

# Lower is preferred.  The ordering is an explicit engineering-risk judgment, not an
# output of the mass model.  A conventional aluminium induction plate is preferred over
# cryogenic or skin-depth-optimized hardware when the permanent-magnet cart cannot meet
# the track-length target.
ARCHITECTURE_RISK_RANK = {
    "permanent_magnet": 0,
    "induction_plate": 1,
    "superconducting": 2,
    "thin_plate": 3,
}
ARCHITECTURES: Mapping[str, CartArchitecture] = {
    "permanent_magnet": PERMANENT_MAGNET_CART,
    "induction_plate": INDUCTION_PLATE_CART,
    "superconducting": SUPERCONDUCTING_CART,
    "thin_plate": THIN_PLATE_CART,
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _parse_values(text: str) -> tuple[float, ...]:
    try:
        values = tuple(float(item.strip()) for item in text.split(",") if item.strip())
    except ValueError as exc:
        raise argparse.ArgumentTypeError("values must be comma-separated numbers") from exc
    if not values or any(not math.isfinite(value) or value <= 0.0 for value in values):
        raise argparse.ArgumentTypeError("values must contain finite positive numbers")
    return values


def reference_duty(config_path: Path) -> tuple[CartDuty, Mapping[str, Any]]:
    loaded = load_yaml(config_path)
    config = loaded.config
    centerline = resolve_centerline(config)
    poses = centerline.sample(100.0)
    maximum_inclination_deg = max(abs(pose.inclination_deg) for pose in poses)
    exit_altitude_m = centerline.pose(centerline.length_m).position_m[2]
    design_resultant_g = config.launch_control.maximum_resultant_load_g
    design_normal_g = config.cart.maximum_resultant_load_g
    if design_resultant_g is None or design_normal_g is None:
        raise ValueError("brake sweep requires explicit launch and cart resultant-load limits")
    duty = CartDuty(
        payload_mass_kg=config.rocket.initial_mass_kg,
        exit_speed_mps=config.launch_control.target_exit_speed_mps,
        launch_length_m=centerline.length_m,
        exit_altitude_m=exit_altitude_m,
        maximum_inclination_deg=maximum_inclination_deg,
        design_resultant_g=design_resultant_g,
        design_normal_g=design_normal_g,
        brake_limit_g=config.cart.brake_force_limit_n
        / (config.cart.mass_kg * STANDARD_GRAVITY_MPS2),
        brake_jerk_limit_mps3=config.cart.brake_jerk_limit_mps3,
    )
    try:
        source_name = str(config_path.relative_to(PROJECT_ROOT)).replace("\\", "/")
    except ValueError:
        source_name = str(config_path)
    metadata = {
        "source": source_name,
        "source_sha256": loaded.source_sha256,
        "resolved_sha256": loaded.resolved_sha256,
        "configured_cart_mass_kg": config.cart.mass_kg,
        "configured_brake_force_limit_n": config.cart.brake_force_limit_n,
        "configured_brake_resultant_limit_g": config.cart.maximum_resultant_load_g,
        "configured_brake_jerk_limit_mps3": config.cart.brake_jerk_limit_mps3,
        "configured_track_length_m": config.tube.exit_brake_track_length_m,
        "configured_stop_margin_m": config.cart.brake_stop_margin_m,
        "exit_track_grade_deg": config.tube.exit_track_grade_deg,
    }
    return duty, metadata


def evaluate_case(
    architecture_name: str,
    architecture: CartArchitecture,
    base_duty: CartDuty,
    *,
    brake_limit_g: float,
    jerk_limit_mps3: float,
    stop_margin_m: float,
    regeneration_efficiency: float,
) -> dict[str, Any]:
    case_id = f"{architecture_name}__{brake_limit_g:g}g__{jerk_limit_mps3:g}j"
    duty = replace(
        base_duty,
        brake_limit_g=brake_limit_g,
        brake_jerk_limit_mps3=jerk_limit_mps3,
    )
    common: dict[str, Any] = {
        "case_id": case_id,
        "architecture": architecture_name,
        "architecture_risk_rank": ARCHITECTURE_RISK_RANK[architecture_name],
        "brake_limit_g": brake_limit_g,
        "jerk_limit_mps3": jerk_limit_mps3,
    }
    try:
        mass = size_cart(architecture, duty)
    except CartSizingError as exc:
        return {**common, "status": "infeasible_mass_closure", "reason": str(exc)}

    brake = evaluate_brake(mass.cart_mass_kg, duty)
    recoverable_energy_j = brake.dissipated_energy_j
    return {
        **common,
        "status": "feasible",
        "cart_mass_kg": mass.cart_mass_kg,
        "drive_mass_kg": mass.drive_mass_kg,
        "guide_mass_kg": mass.guide_mass_kg,
        "structure_mass_kg": mass.structure_mass_kg,
        "closure_factor": mass.closure_factor,
        "brake_case_binding": mass.brake_case_binding,
        "payload_energy_fraction": mass.payload_energy_fraction,
        "ideal_track_m": brake.ideal_track_m,
        "active_track_m": brake.ramped_track_m,
        "stop_margin_m": stop_margin_m,
        "total_track_m": brake.ramped_track_m + stop_margin_m,
        "brake_ramp_time_s": brake.deceleration_mps2 / jerk_limit_mps3,
        "stop_time_s": brake.stop_time_s,
        "peak_brake_force_n": brake.peak_force_n,
        "peak_brake_power_w": brake.peak_power_w,
        "recoverable_brake_energy_j": recoverable_energy_j,
        "assumed_grid_return_energy_j": recoverable_energy_j * regeneration_efficiency,
        "assumed_regeneration_loss_j": recoverable_energy_j
        * (1.0 - regeneration_efficiency),
        "regeneration_required": brake.regeneration_required,
    }


def _reference_case(
    duty: CartDuty,
    metadata: Mapping[str, Any],
    regeneration_efficiency: float,
) -> dict[str, Any]:
    brake = evaluate_brake(float(metadata["configured_cart_mass_kg"]), duty)
    energy = brake.dissipated_energy_j
    return {
        "cart_mass_kg": metadata["configured_cart_mass_kg"],
        "brake_limit_g_from_force": duty.brake_limit_g,
        "jerk_limit_mps3": duty.brake_jerk_limit_mps3,
        "active_track_m": brake.ramped_track_m,
        "total_track_m": brake.ramped_track_m + float(metadata["configured_stop_margin_m"]),
        "configured_track_m": metadata["configured_track_length_m"],
        "peak_brake_force_n": brake.peak_force_n,
        "peak_brake_power_w": brake.peak_power_w,
        "recoverable_brake_energy_j": energy,
        "assumed_grid_return_energy_j": energy * regeneration_efficiency,
        "model_note": "conservative level-track/no-drag screen; the recorded 15-degree mission stops in 23.0 km",
    }


def select_recommendation(
    cases: Iterable[Mapping[str, Any]],
    *,
    target_total_track_m: float,
    maximum_cart_mass_kg: float,
) -> Mapping[str, Any]:
    eligible = [
        case
        for case in cases
        if case["status"] == "feasible"
        and float(case["total_track_m"]) <= target_total_track_m
        and float(case["cart_mass_kg"]) <= maximum_cart_mass_kg
    ]
    if not eligible:
        raise ValueError("no swept brake design meets the track and cart-mass targets")
    return min(
        eligible,
        key=lambda case: (
            int(case["architecture_risk_rank"]),
            float(case["brake_limit_g"]),
            float(case["jerk_limit_mps3"]),
            float(case["peak_brake_power_w"]),
        ),
    )


def architecture_frontier(
    cases: Sequence[Mapping[str, Any]], target_total_track_m: float
) -> list[dict[str, Any]]:
    output = []
    for architecture_name in ARCHITECTURES:
        feasible = [
            case
            for case in cases
            if case["architecture"] == architecture_name and case["status"] == "feasible"
        ]
        meeting = [
            case for case in feasible if float(case["total_track_m"]) <= target_total_track_m
        ]
        if meeting:
            selected = min(
                meeting,
                key=lambda case: (
                    float(case["brake_limit_g"]),
                    float(case["jerk_limit_mps3"]),
                ),
            )
            target_met = True
        else:
            selected = min(feasible, key=lambda case: float(case["total_track_m"]))
            target_met = False
        output.append({**selected, "target_met": target_met})
    return output


def build_sweep(
    config_path: Path,
    *,
    g_values: Sequence[float] = DEFAULT_G_VALUES,
    jerk_values_mps3: Sequence[float] = DEFAULT_JERK_VALUES_MPS3,
    target_total_track_m: float = 10_000.0,
    regeneration_efficiency: float = 0.90,
) -> dict[str, Any]:
    if not math.isfinite(target_total_track_m) or target_total_track_m <= 0.0:
        raise ValueError("target track length must be positive")
    if not math.isfinite(regeneration_efficiency) or not 0.0 <= regeneration_efficiency <= 1.0:
        raise ValueError("regeneration efficiency must lie in [0, 1]")
    if not g_values or any(not math.isfinite(value) or value <= 0.0 for value in g_values):
        raise ValueError("G values must contain finite positive numbers")
    if not jerk_values_mps3 or any(
        not math.isfinite(value) or value <= 0.0 for value in jerk_values_mps3
    ):
        raise ValueError("jerk values must contain finite positive numbers")
    duty, config_metadata = reference_duty(config_path)
    stop_margin_m = float(config_metadata["configured_stop_margin_m"])
    cases = [
        evaluate_case(
            name,
            architecture,
            duty,
            brake_limit_g=g_value,
            jerk_limit_mps3=jerk_value,
            stop_margin_m=stop_margin_m,
            regeneration_efficiency=regeneration_efficiency,
        )
        for name, architecture in ARCHITECTURES.items()
        for g_value in g_values
        for jerk_value in jerk_values_mps3
    ]
    maximum_cart_mass_kg = float(config_metadata["configured_cart_mass_kg"])
    recommendation = select_recommendation(
        cases,
        target_total_track_m=target_total_track_m,
        maximum_cart_mass_kg=maximum_cart_mass_kg,
    )
    return {
        "schema": SCHEMA,
        "runner_sha256": _sha256(Path(__file__)),
        "configuration": dict(config_metadata),
        "reference_duty": asdict(duty),
        "architectures": {
            name: {
                **asdict(architecture),
                "risk_rank": ARCHITECTURE_RISK_RANK[name],
            }
            for name, architecture in ARCHITECTURES.items()
        },
        "assumptions": {
            "brake_track_model": "level_track_no_drag_conservative_v1",
            "stop_margin_m": stop_margin_m,
            "regeneration_efficiency": regeneration_efficiency,
            "regenerative_braking_required": True,
            "coefficients": "order_of_magnitude_not_validated_hardware_data",
        },
        "selection_policy": {
            "target_total_track_m": target_total_track_m,
            "maximum_cart_mass_kg": maximum_cart_mass_kg,
            "order": [
                "architecture_risk_rank",
                "brake_limit_g",
                "jerk_limit_mps3",
                "peak_brake_power_w",
            ],
        },
        "reference_configuration_case": _reference_case(
            duty, config_metadata, regeneration_efficiency
        ),
        "recommendation_case_id": recommendation["case_id"],
        "recommendation": dict(recommendation),
        "architecture_frontier": architecture_frontier(cases, target_total_track_m),
        "case_count": len(cases),
        "cases": cases,
    }


def _fmt(value: float, divisor: float = 1.0, digits: int = 2) -> str:
    return f"{value / divisor:.{digits}f}"


def render_markdown(sweep: Mapping[str, Any]) -> str:
    recommendation = sweep["recommendation"]
    reference = sweep["reference_configuration_case"]
    policy = sweep["selection_policy"]
    assumptions = sweep["assumptions"]
    lines = [
        "# skyArc brake design trade study",
        "",
        "This is an analytic design screen, not qualification evidence. The qualified mission",
        "configuration is unchanged.",
        "",
        "## Decision",
        "",
        f"Select **{str(recommendation['architecture']).replace('_', ' ')}**, "
        f"**{_fmt(float(recommendation['brake_limit_g']), digits=0)} G**, and "
        f"**{_fmt(float(recommendation['jerk_limit_mps3']), digits=0)} m/s³** as the next "
        "design point to validate.",
        "",
        "| Quantity | Current configured cart | Selected design point |",
        "|---|---:|---:|",
        f"| Cart mass | {_fmt(float(reference['cart_mass_kg']))} kg | {_fmt(float(recommendation['cart_mass_kg']))} kg |",
        f"| Active stopping track | {_fmt(float(reference['active_track_m']), 1000.0)} km | {_fmt(float(recommendation['active_track_m']), 1000.0)} km |",
        f"| Total track with margin | {_fmt(float(reference['total_track_m']), 1000.0)} km | {_fmt(float(recommendation['total_track_m']), 1000.0)} km |",
        f"| Peak brake force | {_fmt(float(reference['peak_brake_force_n']), 1000.0)} kN | {_fmt(float(recommendation['peak_brake_force_n']), 1000.0)} kN |",
        f"| Peak brake power | {_fmt(float(reference['peak_brake_power_w']), 1.0e6)} MW | {_fmt(float(recommendation['peak_brake_power_w']), 1.0e6)} MW |",
        f"| Recoverable cart energy | {_fmt(float(reference['recoverable_brake_energy_j']), 1.0e6)} MJ | {_fmt(float(recommendation['recoverable_brake_energy_j']), 1.0e6)} MJ |",
        f"| Grid return at {100.0 * float(assumptions['regeneration_efficiency']):.0f}% | {_fmt(float(reference['assumed_grid_return_energy_j']), 1.0e6)} MJ | {_fmt(float(recommendation['assumed_grid_return_energy_j']), 1.0e6)} MJ |",
        f"| Payload energy fraction | n/a | {100.0 * float(recommendation['payload_energy_fraction']):.1f}% |",
        "",
        "The selected point is the lowest-G, lowest-jerk conventional induction-plate case",
        f"that meets the **{float(policy['target_total_track_m']) / 1000.0:g} km** total-track target.",
        "It keeps all active equipment track-side and avoids both a moving cryostat and the",
        "less-certain skin-depth-thinned plate assumption.",
        "",
        "## Architecture frontier",
        "",
        "| Architecture | Meets target | G | Jerk (m/s³) | Cart (kg) | Total track (km) | Peak power (MW) | Recoverable energy (MJ) |",
        "|---|:---:|---:|---:|---:|---:|---:|---:|",
    ]
    for case in sweep["architecture_frontier"]:
        lines.append(
            f"| {str(case['architecture']).replace('_', ' ')} | "
            f"{'yes' if case['target_met'] else 'no'} | "
            f"{_fmt(float(case['brake_limit_g']), digits=0)} | "
            f"{_fmt(float(case['jerk_limit_mps3']), digits=0)} | "
            f"{_fmt(float(case['cart_mass_kg']))} | "
            f"{_fmt(float(case['total_track_m']), 1000.0)} | "
            f"{_fmt(float(case['peak_brake_power_w']), 1.0e6)} | "
            f"{_fmt(float(case['recoverable_brake_energy_j']), 1.0e6)} |"
        )
    lines.extend(
        [
            "",
            "## Limits before changing the mission",
            "",
            "- The force-density, guide-load and structural coefficients are order-of-magnitude estimates.",
            "- The stopping calculation gives no credit for the actual 15-degree uphill grade or aerodynamic drag.",
            "- Peak power is the instantaneous mechanical power at brake entry; power electronics, sectioning, thermal paths and grid acceptance are not modeled.",
            "- Thirty-G operation is applied only to the empty cart after release. It still needs a structural and guide-load validation before becoming a mission requirement.",
            "- Regeneration is mandatory. The reported return assumes the declared efficiency; it is not a measured electrical result.",
            "",
            f"Full machine-readable sweep: `{sweep['case_count']}` cases in `artifacts/design/brake_design_sweep.json`.",
            "",
        ]
    )
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "configs" / "curved_2kms.yaml",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "artifacts" / "design" / "brake_design_sweep.json",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=PROJECT_ROOT / "BRAKE_TRADE_STUDY.md",
    )
    parser.add_argument("--g-values", type=_parse_values, default=DEFAULT_G_VALUES)
    parser.add_argument(
        "--jerk-values-mps3", type=_parse_values, default=DEFAULT_JERK_VALUES_MPS3
    )
    parser.add_argument("--target-track-km", type=float, default=10.0)
    parser.add_argument("--regeneration-efficiency", type=float, default=0.90)
    args = parser.parse_args(argv)

    sweep = build_sweep(
        args.config.resolve(),
        g_values=args.g_values,
        jerk_values_mps3=args.jerk_values_mps3,
        target_total_track_m=args.target_track_km * 1000.0,
        regeneration_efficiency=args.regeneration_efficiency,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(sweep, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    args.report.write_text(render_markdown(sweep), encoding="utf-8")
    print(json.dumps(sweep["recommendation"], indent=2, sort_keys=True))
    print(f"wrote {args.output}")
    print(f"wrote {args.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
