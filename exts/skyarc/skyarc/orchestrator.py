# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Backend-neutral mission step orchestration.

The ordering is deliberately explicit: read, observe, state-machine gates, component
pre-step, validate/aggregate, adapter apply, release resync when required, integrate,
read back, boundary detection, post-step, and event processing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Mapping, Tuple

from .components.contract import Component, ScenarioContext, StepOutput
from .components.observers import GroundTruthObserver
from .coupling import (
    FixedJointCoupling,
    IgnitionInterlock,
    NoneSeparationActuator,
    ReleasePhase,
    SeparationMonitor,
    measure_separation,
)
from .coupling.interlock import InterlockDecision
from .configuration.schema import ScenarioConfig
from .effects.adapter import BackendAdapter
from .effects.aggregator import aggregate
from .experiments.criteria import resolve_evidence_window
from .experiments.hashing import builtin_package_code_identity
from .launcher.atmosphere import DensityDragModel
from .launcher.brake import ForceLimitedCartBrake
from .launcher.guide import IdealPathGuide
from .launcher.launch_force import STANDARD_GRAVITY_MPS2, AbstractAxialLaunchForce
from .rocket.aerodynamics import QuadraticPointDrag
from .events import (
    EVENT_ABORT,
    EVENT_EXIT_PLANE_CROSSED,
    EVENT_FLIGHT_WINDOW_COMPLETE,
    EVENT_FORCE_RAMP_DOWN,
    EVENT_IGNITION,
    EVENT_RELEASE_CONFIRMED,
    EVENT_SEPARATION_CONFIRMED,
    Event,
)
from .launcher.geometry import TubePath, enumerate_boundary_crossings
from .names import (
    JOINT_COUPLING,
    MARKER_ASSEMBLY_EXIT,
    MARKER_ROCKET_AFT,
    MARKER_ROCKET_STAGNATION,
    SLOT_ATMOSPHERE,
    SLOT_CART_BRAKE,
    SLOT_COUPLING,
    SLOT_GUIDE,
    SLOT_LAUNCH_FORCE,
    SLOT_OBSERVER,
    SLOT_ROCKET_AERODYNAMICS,
    SLOT_ROCKET_MOTOR,
    SLOT_SEPARATION_ACTUATOR,
)
from .rocket.motor import ConstantMassThrustMotor
from .state import MarkerSpec, Observation, SimulationState
from .state_machine import MissionPhase, MissionState, MissionStateMachine
from .telemetry.recorder import StepTelemetryInput, TelemetrySink


@dataclass(frozen=True)
class MissionComponents:
    launch_force: Component
    atmosphere: Component
    guide: Component
    coupling: FixedJointCoupling
    separation_actuator: Component
    cart_brake: Component
    rocket_motor: ConstantMassThrustMotor
    rocket_aerodynamics: Component

    def all(self) -> Tuple[Component, ...]:
        return (
            self.launch_force,
            self.atmosphere,
            self.guide,
            self.coupling,
            self.separation_actuator,
            self.cart_brake,
            self.rocket_motor,
            self.rocket_aerodynamics,
        )


@dataclass(frozen=True)
class MissionRunResult:
    mission_state: MissionState
    final_state: SimulationState
    events: Tuple[Event, ...]
    steps: int
    telemetry_summary: object | None = None


class SimulationOrchestrator:
    """Run the common component/effect path through release and concurrent branches."""

    def __init__(
        self,
        *,
        adapter: BackendAdapter,
        layout: TubePath,
        markers: Mapping[str, MarkerSpec],
        observer: GroundTruthObserver,
        components: MissionComponents,
        separation_monitor: SeparationMonitor,
        ignition_interlock: IgnitionInterlock,
        free_flight_duration_s: float,
        maximum_run_time_s: float,
        scenario_id: str = "analytic_mission",
        telemetry_sink: TelemetrySink | None = None,
    ) -> None:
        if not math.isfinite(free_flight_duration_s) or free_flight_duration_s < 0.0:
            raise ValueError("free-flight duration must be finite and nonnegative")
        if not math.isfinite(maximum_run_time_s) or maximum_run_time_s <= 0.0:
            raise ValueError("maximum run time must be finite and positive")
        missing = sorted(
            {MARKER_ASSEMBLY_EXIT, MARKER_ROCKET_AFT, MARKER_ROCKET_STAGNATION} - set(markers)
        )
        if missing:
            raise ValueError(f"orchestrator is missing required markers: {missing}")
        self._adapter = adapter
        self._layout = layout
        self._markers = dict(markers)
        self._observer = observer
        self._components = components
        self._separation = separation_monitor
        self._interlock = ignition_interlock
        self._free_flight_duration_s = free_flight_duration_s
        self._maximum_run_time_s = maximum_run_time_s
        self._machine = MissionStateMachine()
        self._events: list[Event] = []
        self._separation_confirmed_time_s: float | None = None
        self._flight_window_emitted = False
        self._release_confirmed = False
        self._last_interlock_decision: InterlockDecision | None = None
        self._started = False
        self._telemetry_sink = telemetry_sink
        self._telemetry_event_cursor = 0

        context = ScenarioContext(
            scenario_id=scenario_id,
            markers=self._markers,
            backend_capabilities=adapter.capabilities.features,
        )
        initial = adapter.read_state()
        observer.prepare(context)
        observer.reset(initial)
        for component in components.all():
            component.prepare(context)
            component.reset(initial)

    @property
    def mission_state(self) -> MissionState:
        return self._machine.state

    @property
    def events(self) -> Tuple[Event, ...]:
        return tuple(self._events)

    def _record(self, events: Iterable[Event], *, process: bool = True) -> None:
        items = tuple(events)
        if not items:
            return
        self._events.extend(items)
        if process:
            self._events.extend(self._machine.process(items))

    def _observe(self, state: SimulationState) -> Observation:
        measurement = measure_separation(state, self._layout, self._markers)
        return self._observer.observe(
            state,
            coupled=bool(state.joint_active.get(JOINT_COUPLING, False)),
            separation_gap_m=measurement.gap_m,
            separation_rate_mps=measurement.relative_speed_mps,
        )

    def start(self) -> None:
        if self._started:
            return
        state = self._adapter.read_state()
        observation = self._observe(state)
        stage_index = observation.axial.stage_index
        if stage_index < 0:
            stage_index = 0
        stage_name = self._layout.stages[stage_index].name
        self._record((self._machine.arm(time_s=state.time_s, step_index=state.step_index),), process=False)
        self._record(
            (self._machine.release_hold(time_s=state.time_s, step_index=state.step_index),),
            process=False,
        )
        self._record(
            (
                self._machine.start_launch(
                    stage_index,
                    stage_name,
                    time_s=state.time_s,
                    step_index=state.step_index,
                ),
            ),
            process=False,
        )
        self._started = True

    def reset(self) -> None:
        """Reconstruct backend/component state and clear all mission-owned history."""
        if self._telemetry_sink is not None:
            raise RuntimeError("a reset replay requires a new telemetry run instance")
        self._adapter.reset()
        initial = self._adapter.read_state()
        self._observer.reset(initial)
        for component in self._components.all():
            component.reset(initial)
        self._separation.reset()
        self._machine.reset()
        self._events.clear()
        self._separation_confirmed_time_s = None
        self._flight_window_emitted = False
        self._release_confirmed = False
        self._last_interlock_decision = None
        self._started = False
        self._telemetry_event_cursor = 0

    @property
    def last_interlock_decision(self) -> InterlockDecision | None:
        """Most recent ignition adjudication, retained so telemetry can record blocked gates."""
        return self._last_interlock_decision

    @staticmethod
    def _outputs_have_release_commands(outputs: Iterable[StepOutput]) -> bool:
        return any(
            output.effects.constraint_commands or output.effects.collision_commands
            for output in outputs
        )

    def _pre_step_outputs(self, observation: Observation) -> Tuple[StepOutput, ...]:
        coupling_output = self._components.coupling.pre_step(observation)
        if coupling_output.effects.constraint_commands or coupling_output.effects.collision_commands:
            # The mutation/confirmation step is intentionally force-free.  Applying an
            # attached-assembly equivalent wrench after disabling the joint would put the
            # entire wrench on the cart and create a release impulse mismatch.
            return (coupling_output,)
        outputs: list[StepOutput] = [coupling_output, self._components.guide.pre_step(observation)]
        if observation.coupled:
            outputs.extend(
                (
                    self._components.launch_force.pre_step(observation),
                    self._components.atmosphere.pre_step(observation),
                )
            )
        else:
            outputs.extend(
                (
                    self._components.atmosphere.pre_step(observation),
                    self._components.separation_actuator.pre_step(observation),
                    self._components.rocket_aerodynamics.pre_step(observation),
                    self._components.rocket_motor.pre_step(observation),
                )
            )
            if self._components.coupling.brake_eligible:
                outputs.append(self._components.cart_brake.pre_step(observation))
        return tuple(outputs)

    def _flush_telemetry_events(self) -> None:
        if self._telemetry_sink is None:
            return
        pending = self._events[self._telemetry_event_cursor :]
        if pending:
            self._telemetry_sink.record_events(pending)
            self._telemetry_event_cursor = len(self._events)

    def _detect_guided_events(self, before: Observation, after: Observation) -> None:
        crossings = enumerate_boundary_crossings(
            before.axial.marker(MARKER_ROCKET_STAGNATION),
            after.axial.marker(MARKER_ROCKET_STAGNATION),
            self._layout.boundaries_m,
            pre_time_s=before.time_s,
            post_time_s=after.time_s,
        )
        for crossing in crossings:
            if crossing.to_stage_index is not None and self._machine.state.phase is MissionPhase.LAUNCH_STAGE:
                self._record(
                    (
                        self._machine.stage_transition(
                            crossing.to_stage_index,
                            self._layout.stages[crossing.to_stage_index].name,
                            time_s=crossing.time_s,
                            step_index=after.step_index,
                        ),
                    ),
                    process=False,
                )
            elif crossing.to_stage_index is None and self._machine.state.phase is MissionPhase.LAUNCH_STAGE:
                self._record(
                    (
                        self._machine.exit_approach(
                            time_s=crossing.time_s,
                            step_index=after.step_index,
                        ),
                        Event(
                            name=EVENT_EXIT_PLANE_CROSSED,
                            time_s=crossing.time_s,
                            step_index=after.step_index,
                            source="orchestrator",
                            data={"marker": MARKER_ROCKET_STAGNATION},
                        ),
                    ),
                    process=False,
                )

        if (
            before.axial.marker(MARKER_ASSEMBLY_EXIT) < self._layout.length_m
            <= after.axial.marker(MARKER_ASSEMBLY_EXIT)
        ):
            fraction = (
                (self._layout.length_m - before.axial.marker(MARKER_ASSEMBLY_EXIT))
                / (
                    after.axial.marker(MARKER_ASSEMBLY_EXIT)
                    - before.axial.marker(MARKER_ASSEMBLY_EXIT)
                )
            )
            crossing_time = before.time_s + fraction * (after.time_s - before.time_s)
            self._record(
                (
                    Event(
                        name=EVENT_EXIT_PLANE_CROSSED,
                        time_s=crossing_time,
                        step_index=after.step_index,
                        source="orchestrator",
                        data={"marker": MARKER_ASSEMBLY_EXIT},
                    ),
                ),
                process=False,
            )
            if self._machine.state.phase is MissionPhase.LAUNCH_STAGE:
                self._record(
                    (
                        self._machine.exit_approach(
                            time_s=crossing_time,
                            step_index=after.step_index,
                        ),
                    ),
                    process=False,
                )

        if (
            before.axial.marker(MARKER_ROCKET_AFT) < self._layout.length_m
            <= after.axial.marker(MARKER_ROCKET_AFT)
            and self._components.coupling.phase is ReleasePhase.ATTACHED
        ):
            if self._machine.state.phase is MissionPhase.LAUNCH_STAGE:
                self._record(
                    (
                        self._machine.exit_approach(
                            time_s=after.time_s,
                            step_index=after.step_index,
                        ),
                    ),
                    process=False,
                )
            if self._machine.state.phase is MissionPhase.EXIT_APPROACH:
                self._record(
                    (
                        self._machine.force_ramp_down(
                            time_s=after.time_s,
                            step_index=after.step_index,
                        ),
                        Event(
                            name=EVENT_FORCE_RAMP_DOWN,
                            time_s=after.time_s,
                            step_index=after.step_index,
                            source="orchestrator",
                        ),
                    ),
                    process=False,
                )
            self._record(
                (
                    self._machine.rocket_detach(
                        time_s=after.time_s,
                        step_index=after.step_index,
                    ),
                ),
                process=False,
            )
            self._record(
                self._components.coupling.request_release(
                    after,
                    aft_marker_outside=True,
                ),
                process=False,
            )

    def _post_release_events(self, state: SimulationState, observation: Observation) -> None:
        measurement = measure_separation(state, self._layout, self._markers)
        release_events = self._components.coupling.post_step(state).events
        self._record(release_events)
        if any(event.name == EVENT_RELEASE_CONFIRMED for event in release_events):
            self._release_confirmed = True
            self._separation.begin(state)
        separation_events = self._separation.update(state, measurement)
        self._record(separation_events)
        if any(event.name == EVENT_SEPARATION_CONFIRMED for event in separation_events):
            self._separation_confirmed_time_s = state.time_s

        # The interlock is evaluated on every post-release step, from the monitor's actual
        # confirmation flag. Gating entry on `status.confirmed` and then passing a literal
        # True would make the clearance gate incapable of failing on that input, so a monitor
        # that wrongly confirmed would bypass the interlock rather than be caught by it.
        # Section 10.4 is a conjunction that the interlock alone is meant to adjudicate.
        if self._release_confirmed and not self._components.rocket_motor.ignited:
            decision = self._interlock.evaluate(
                observation,
                release_time_s=self._separation.status.release_time_s,
                separation_confirmed=self._separation.status.confirmed,
                abort_active=self._machine.abort_active,
            )
            self._last_interlock_decision = decision
            if decision.allowed:
                self._record(self._components.rocket_motor.command_ignition(observation, decision))

        if (
            self._separation_confirmed_time_s is not None
            and not self._flight_window_emitted
            and state.time_s + 1e-12
            >= self._separation_confirmed_time_s + self._free_flight_duration_s
        ):
            self._flight_window_emitted = True
            self._record(
                (
                    Event(
                        name=EVENT_FLIGHT_WINDOW_COMPLETE,
                        time_s=state.time_s,
                        step_index=state.step_index,
                        source="orchestrator",
                        data={"duration_s": self._free_flight_duration_s},
                    ),
                )
            )

    def step(self) -> MissionState:
        self.start()
        before_state = self._adapter.read_state()
        if before_state.time_s >= self._maximum_run_time_s - 1e-12:
            self._record(
                self._machine.abort(
                    "maximum_run_time_exceeded",
                    time_s=before_state.time_s,
                    step_index=before_state.step_index,
                ),
                process=False,
            )
            self._flush_telemetry_events()
            return self._machine.state
        before = self._observe(before_state)
        outputs = self._pre_step_outputs(before)
        diagnostics = []
        for output in outputs:
            # Locate the component by output source to apply the common provenance check.
            for component in self._components.all():
                if component.descriptor.slot == output.effects.source:
                    component.validate_output(output)
                    break
            self._record(output.events)
            diagnostics.extend(output.diagnostics)
        if self._machine.abort_active:
            if self._telemetry_sink is not None:
                self._telemetry_sink.record_diagnostics(
                    diagnostics,
                    time_s=before_state.time_s,
                    step_index=before_state.step_index,
                )
            self._flush_telemetry_events()
            return self._machine.state
        command_snapshots = {
            component.descriptor.slot: component.snapshot_state()
            for component in self._components.all()
        }
        accepted = aggregate((output.effects for output in outputs), before_state)
        applied = self._adapter.apply(accepted)
        if self._outputs_have_release_commands(outputs):
            self._record(
                self._components.coupling.resync_after_apply(
                    self._adapter,
                    applied,
                    before_state,
                )
            )
        self._adapter.step()
        after_state = self._adapter.read_state()
        after = self._observe(after_state)
        self._detect_guided_events(before, after)

        post_events = []
        for component in self._components.all():
            if component is self._components.coupling:
                continue
            output = component.post_step(after_state)
            component.validate_output(output)
            if not output.effects.is_empty():
                raise RuntimeError("post-step physical effects must be queued explicitly for the next pre-step")
            post_events.extend(output.events)
            diagnostics.extend(output.diagnostics)
        self._record(post_events)
        self._post_release_events(after_state, after)
        if self._telemetry_sink is not None:
            self._telemetry_sink.record_step(
                StepTelemetryInput(
                    pre_state=before_state,
                    observation=before,
                    command_snapshots=command_snapshots,
                    accepted_effects=accepted,
                    applied_effects=applied,
                    post_state=after_state,
                    post_observation=after,
                    mission_state=self._machine.state,
                    interlock_allowed=(
                        None
                        if self._last_interlock_decision is None
                        else self._last_interlock_decision.allowed
                    ),
                    abort_active=self._machine.abort_active,
                )
            )
            self._telemetry_sink.record_diagnostics(
                diagnostics,
                time_s=after_state.time_s,
                step_index=after_state.step_index,
            )
        self._flush_telemetry_events()
        return self._machine.state

    def run(self) -> MissionRunResult:
        self.start()
        while self._machine.state.phase not in (MissionPhase.COMPLETE, MissionPhase.ABORT):
            self.step()
        final = self._adapter.read_state()
        self._flush_telemetry_events()
        telemetry_summary = None
        if self._telemetry_sink is not None:
            termination_reason = (
                self._machine.state.abort_reason
                if self._machine.state.phase is MissionPhase.ABORT
                else "complete"
            )
            telemetry_summary = self._telemetry_sink.finalize(
                termination_reason=termination_reason or "abort",
                mission_phase=self._machine.state.phase.value,
            )
        return MissionRunResult(
            mission_state=self._machine.state,
            final_state=final,
            events=tuple(self._events),
            steps=final.step_index,
            telemetry_summary=telemetry_summary,
        )


def marker_specs(config: ScenarioConfig) -> Mapping[str, MarkerSpec]:
    """Resolve the configured marker table into the runtime marker specification."""
    return {
        name: MarkerSpec(name, marker.body, marker.offset_m)
        for name, marker in config.markers.items()
    }


def build_mission(
    config: ScenarioConfig,
    layout: TubePath,
    adapter: BackendAdapter,
    *,
    free_flight_duration_s: float | None = None,
    component_code_hashes: Mapping[str, str] | None = None,
    gravity_mps2: Tuple[float, float, float] = (0.0, 0.0, -STANDARD_GRAVITY_MPS2),
    telemetry_sink: TelemetrySink | None = None,
) -> SimulationOrchestrator:
    """Assemble the baseline component set for one scenario against one adapter.

    The wiring is a single fact about the scenario schema, so it belongs here rather than
    being restated by the standalone runner, the extension, and each evidence harness. Those
    three copies would drift, and a slot wired differently in one of them is exactly the kind
    of difference the ablation manifest of section 14.1 is supposed to make visible.

    The default code identity is the conservative declared closure of the complete
    backend-neutral package plus its resolved external versions. A caller may provide
    narrower, packaging-generated per-slot closures, but must provide every selected slot.
    """
    if free_flight_duration_s is None:
        free_flight_duration_s = resolve_evidence_window(config).duration_s
    selected_slots = (
        SLOT_LAUNCH_FORCE,
        SLOT_ATMOSPHERE,
        SLOT_GUIDE,
        SLOT_COUPLING,
        SLOT_SEPARATION_ACTUATOR,
        SLOT_CART_BRAKE,
        SLOT_ROCKET_MOTOR,
        SLOT_ROCKET_AERODYNAMICS,
        SLOT_OBSERVER,
    )
    if component_code_hashes is None:
        package_hash = builtin_package_code_identity().sha256
        code_hashes = {slot: package_hash for slot in selected_slots}
    else:
        missing_hashes = sorted(set(selected_slots) - set(component_code_hashes))
        extra_hashes = sorted(set(component_code_hashes) - set(selected_slots))
        if missing_hashes or extra_hashes:
            raise ValueError(
                f"component code-hash inventory mismatch; missing={missing_hashes}, extra={extra_hashes}"
            )
        code_hashes = dict(component_code_hashes)
        if any(
            not isinstance(value, str)
            or len(value) != 64
            or any(character not in "0123456789abcdef" for character in value)
            for value in code_hashes.values()
        ):
            raise ValueError("component code hashes must be lower-case SHA-256 digests")
    markers = marker_specs(config)
    exterior_ratio = config.tube.exterior_effective_density_ratio
    reference_density = config.simulation.reference_density_kg_m3
    components = MissionComponents(
        launch_force=AbstractAxialLaunchForce(
            layout,
            config.launch_control,
            config.guided_phase_aerodynamics,
            reference_density_kg_m3=reference_density,
            guide_resistance_n=config.cart.guide_resistance_n,
            gravity_mps2=gravity_mps2,
            code_hash=code_hashes[SLOT_LAUNCH_FORCE],
        ),
        atmosphere=DensityDragModel(
            layout,
            config.guided_phase_aerodynamics,
            reference_density_kg_m3=reference_density,
            cart=config.cart,
            exterior_atmosphere=config.tube.exterior_atmosphere,
            code_hash=code_hashes[SLOT_ATMOSPHERE],
        ),
        guide=IdealPathGuide(
            layout,
            model_id=config.models.guide,
            resistance_n=config.cart.guide_resistance_n,
            maximum_tracking_error_m=config.tube.guide_clearance_m,
            gravity_mps2=gravity_mps2,
            code_hash=code_hashes[SLOT_GUIDE],
        ),
        coupling=FixedJointCoupling(
            command_latency_s=config.simulation.release_command_latency_s,
            confirmation_steps=config.simulation.release_confirmation_steps,
            code_hash=code_hashes[SLOT_COUPLING],
        ),
        separation_actuator=NoneSeparationActuator(
            code_hash=code_hashes[SLOT_SEPARATION_ACTUATOR]
        ),
        cart_brake=ForceLimitedCartBrake(
            layout,
            config.cart,
            exit_track_length_m=(
                config.tube.exit_track.length_m
                if config.tube.exit_track is not None
                else config.tube.exit_brake_track_length_m
            ),
            reference_density_kg_m3=reference_density,
            exterior_density_ratio=exterior_ratio,
            exterior_atmosphere=config.tube.exterior_atmosphere,
            gravity_mps2=gravity_mps2,
            code_hash=code_hashes[SLOT_CART_BRAKE],
        ),
        rocket_motor=ConstantMassThrustMotor(
            config.rocket.motor, code_hash=code_hashes[SLOT_ROCKET_MOTOR]
        ),
        rocket_aerodynamics=QuadraticPointDrag(
            config.rocket,
            reference_density_kg_m3=reference_density,
            exterior_density_ratio=exterior_ratio,
            exterior_atmosphere=config.tube.exterior_atmosphere,
            code_hash=code_hashes[SLOT_ROCKET_AERODYNAMICS],
        ),
    )
    return SimulationOrchestrator(
        adapter=adapter,
        layout=layout,
        markers=markers,
        observer=GroundTruthObserver(
            layout, markers, code_hash=code_hashes[SLOT_OBSERVER]
        ),
        components=components,
        separation_monitor=SeparationMonitor(config.rocket.ignition),
        ignition_interlock=IgnitionInterlock(config.rocket.ignition, layout, markers),
        free_flight_duration_s=free_flight_duration_s,
        maximum_run_time_s=config.simulation.maximum_run_time_s,
        scenario_id=config.experiment.condition_id,
        telemetry_sink=telemetry_sink,
    )
