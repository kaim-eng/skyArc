# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Stable end-of-run summary record."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Mapping

from ..launcher.feasibility import DeliveredState, Stage2Budget
from .energy import EnergySnapshot


@dataclass(frozen=True)
class RunSummary:
    schema_version: str
    termination_reason: str
    mission_phase: str
    elapsed_s: float
    physics_steps: int
    telemetry_samples: int
    event_count: int
    target_exit_speed_mps: float | None
    actual_exit_speed_mps: float | None
    exit_speed_relative_error: float | None
    peak_resultant_load_g: float
    maximum_separation_gap_m: float
    rocket_impulse_ns: float
    energy_residual_j: float | None
    normalized_energy_residual: float | None
    energy_closure_valid: bool
    energy_closure_defect: str | None
    first_event_time_s: Mapping[str, float]
    apogee_time_s: float
    apogee_altitude_m: float
    handoff_time_s: float | None
    handoff_altitude_m: float | None
    handoff_downrange_m: float | None
    handoff_speed_mps: float | None
    handoff_flight_path_angle_deg: float | None
    pre_handoff_rocket_drag_loss_mps: float
    stage2_ideal_energy_raise_mps: float | None
    stage2_measured_alignment_loss_mps: float | None
    stage2_delta_v_required_mps: float | None
    stage2_delta_v_available_mps: float | None
    stage2_assumed_unmodeled_loss_mps: float | None
    stage2_margin_mps: float | None

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def build_summary(
    *,
    termination_reason: str,
    mission_phase: str,
    elapsed_s: float,
    physics_steps: int,
    telemetry_samples: int,
    event_count: int,
    target_exit_speed_mps: float | None,
    actual_exit_speed_mps: float | None,
    peak_resultant_load_g: float,
    maximum_separation_gap_m: float,
    energy: EnergySnapshot,
    first_event_time_s: Mapping[str, float],
    apogee_time_s: float,
    apogee_altitude_m: float,
    pre_handoff_rocket_drag_loss_mps: float,
    delivered_state: DeliveredState | None = None,
    stage2_budget: Stage2Budget | None = None,
) -> RunSummary:
    relative_error = None
    if (
        target_exit_speed_mps is not None
        and target_exit_speed_mps > 0.0
        and actual_exit_speed_mps is not None
    ):
        relative_error = abs(actual_exit_speed_mps - target_exit_speed_mps) / target_exit_speed_mps
    return RunSummary(
        schema_version="run_summary_v2",
        termination_reason=termination_reason,
        mission_phase=mission_phase,
        elapsed_s=elapsed_s,
        physics_steps=physics_steps,
        telemetry_samples=telemetry_samples,
        event_count=event_count,
        target_exit_speed_mps=target_exit_speed_mps,
        actual_exit_speed_mps=actual_exit_speed_mps,
        exit_speed_relative_error=relative_error,
        peak_resultant_load_g=peak_resultant_load_g,
        maximum_separation_gap_m=maximum_separation_gap_m,
        rocket_impulse_ns=energy.rocket_impulse_ns,
        # A summary that reported an incomplete residual as a number would let a run be
        # judged against the section 16.2 gate on a figure that omits rotational energy.
        energy_residual_j=energy.residual_j if energy.valid else None,
        normalized_energy_residual=energy.normalized_residual if energy.valid else None,
        energy_closure_valid=energy.valid,
        energy_closure_defect=energy.invalid_reason,
        first_event_time_s=dict(first_event_time_s),
        apogee_time_s=apogee_time_s,
        apogee_altitude_m=apogee_altitude_m,
        handoff_time_s=None if delivered_state is None else delivered_state.time_s,
        handoff_altitude_m=None if delivered_state is None else delivered_state.altitude_m,
        handoff_downrange_m=None if delivered_state is None else delivered_state.downrange_m,
        handoff_speed_mps=None if delivered_state is None else delivered_state.speed_mps,
        handoff_flight_path_angle_deg=(
            None if delivered_state is None else delivered_state.flight_path_angle_deg
        ),
        pre_handoff_rocket_drag_loss_mps=pre_handoff_rocket_drag_loss_mps,
        stage2_ideal_energy_raise_mps=(
            None if stage2_budget is None else stage2_budget.ideal_energy_raise_mps
        ),
        stage2_measured_alignment_loss_mps=(
            None if stage2_budget is None else stage2_budget.measured_alignment_loss_mps
        ),
        stage2_delta_v_required_mps=(
            None if stage2_budget is None else stage2_budget.delta_v_required_mps
        ),
        stage2_delta_v_available_mps=(
            None if stage2_budget is None else stage2_budget.delta_v_available_mps
        ),
        stage2_assumed_unmodeled_loss_mps=(
            None if stage2_budget is None else stage2_budget.assumed_unmodeled_loss_mps
        ),
        stage2_margin_mps=None if stage2_budget is None else stage2_budget.margin_mps,
    )
