# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
import unittest
from dataclasses import replace
from pathlib import Path

import _bootstrap  # noqa: F401

from skyarc.components import ScenarioContext
from skyarc.configuration import load_yaml, resolve_tube_layout
from skyarc.configuration.schema import IgnitionConfig, MotorConfig
from skyarc.coupling import (
    FixedJointCoupling,
    IgnitionInterlock,
    InterlockDecision,
    ReleaseError,
    ReleasePhase,
    SeparationMonitor,
    measure_separation,
)
from skyarc.effects import EffectBatch, aggregate
from skyarc.effects.backends import AnalyticBackend
from skyarc.events import (
    EVENT_ABORT,
    EVENT_BRAKE_ELIGIBLE,
    EVENT_CART_STOPPED,
    EVENT_FLIGHT_WINDOW_COMPLETE,
    EVENT_IGNITION,
    EVENT_INTERLOCK_BLOCKED,
    EVENT_RECONTACT,
    EVENT_RELEASE_CONFIRMED,
    EVENT_RELEASE_STEP,
    EVENT_SEPARATION_CONFIRMED,
    Event,
)
from skyarc.launcher import TubeLayout, TubeStage, path_pose
from skyarc.names import (
    BODY_CART,
    BODY_ROCKET,
    JOINT_COUPLING,
    JOINT_GUIDE,
    MARKER_ASSEMBLY_EXIT,
    MARKER_CART_CRADLE_FRONT,
    MARKER_ROCKET_AFT,
    MARKER_ROCKET_STAGNATION,
    PAIR_ROCKET_CRADLE,
)
from skyarc.orchestrator import build_mission
from skyarc.rocket import ConstantMassThrustMotor
from skyarc.state import (
    AxialQuantities,
    BodyState,
    ContactReport,
    MarkerSpec,
    Observation,
    SimulationState,
)
from skyarc.state_machine import (
    CartBranch,
    MissionPhase,
    MissionStateMachine,
    RocketBranch,
)


BASELINE = Path(__file__).resolve().parents[2] / "configs" / "baseline.yaml"


def layout() -> TubeLayout:
    return TubeLayout(
        origin_m=(0.0, 0.0, 0.0),
        angle_deg=0.0,
        stages=(TubeStage("vacuum", 10.0, 0.0),),
        exterior_effective_density_ratio=0.0,
    )


def markers() -> dict[str, MarkerSpec]:
    return {
        MARKER_ROCKET_AFT: MarkerSpec(MARKER_ROCKET_AFT, BODY_ROCKET, (-2.0, 0.0, 0.0)),
        MARKER_ROCKET_STAGNATION: MarkerSpec(
            MARKER_ROCKET_STAGNATION, BODY_ROCKET, (2.0, 0.0, 0.0)
        ),
        MARKER_ASSEMBLY_EXIT: MarkerSpec(
            MARKER_ASSEMBLY_EXIT, BODY_ROCKET, (2.0, 0.0, 0.0)
        ),
        MARKER_CART_CRADLE_FRONT: MarkerSpec(
            MARKER_CART_CRADLE_FRONT, BODY_CART, (1.25, 0.0, 0.0)
        ),
    }


def state_at(
    time_s: float,
    *,
    cart_x: float = 12.0,
    rocket_x: float = 12.0,
    cart_speed: float = 10.0,
    rocket_speed: float = 10.0,
    coupled: bool = True,
    contact: ContactReport | None = None,
) -> SimulationState:
    return SimulationState(
        time_s=time_s,
        step_index=round(time_s * 10),
        dt_s=0.1,
        bodies={
            BODY_CART: BodyState(
                name=BODY_CART,
                position=(cart_x, 0.0, 0.0),
                linear_velocity=(cart_speed, 0.0, 0.0),
                mass_kg=250.0,
            ),
            BODY_ROCKET: BodyState(
                name=BODY_ROCKET,
                position=(rocket_x, 0.0, 0.0),
                linear_velocity=(rocket_speed, 0.0, 0.0),
                mass_kg=150.0,
            ),
        },
        contacts={} if contact is None else {PAIR_ROCKET_CRADLE: contact},
        joint_active={JOINT_COUPLING: coupled, JOINT_GUIDE: True},
        collision_pair_active={PAIR_ROCKET_CRADLE: True},
    ).frozen()


def observe(state: SimulationState, *, gap: float = 0.0, rate: float = 0.0) -> Observation:
    stage = layout().stage_index(state.body(BODY_ROCKET).position[0] + 2.0)
    return Observation(
        source_model="test",
        time_s=state.time_s,
        step_index=state.step_index,
        dt_s=state.dt_s,
        state=state,
        axial=AxialQuantities(
            s_cart_m=state.body(BODY_CART).position[0],
            s_rocket_m=state.body(BODY_ROCKET).position[0],
            marker_s_m={
                MARKER_ROCKET_AFT: state.body(BODY_ROCKET).position[0] - 2.0,
                MARKER_ROCKET_STAGNATION: state.body(BODY_ROCKET).position[0] + 2.0,
                MARKER_ASSEMBLY_EXIT: state.body(BODY_ROCKET).position[0] + 2.0,
            },
            cart_axial_velocity_mps=state.body(BODY_CART).linear_velocity[0],
            rocket_axial_velocity_mps=state.body(BODY_ROCKET).linear_velocity[0],
            assembly_mass_kg=400.0 if state.joint_active.get(JOINT_COUPLING, False) else 250.0,
            stage_index=-1 if stage is None else stage,
            stage_name="exterior" if stage is None else "vacuum",
            effective_density_ratio=0.0,
            separation_gap_m=gap,
            separation_rate_mps=rate,
        ),
        coupled=state.joint_active.get(JOINT_COUPLING, False),
    )


def ignition() -> IgnitionConfig:
    return IgnitionConfig(
        delay_s=0.25,
        minimum_cart_clearance_m=3.0,
        minimum_relative_speed_mps=0.1,
        no_recontact_dwell_s=0.1,
        separation_timeout_s=2.0,
        maximum_contact_impulse_ns=25.0,
        maximum_angular_rate_deg_s=5.0,
    )


class ReleaseTransactionTests(unittest.TestCase):
    def test_coupling_requires_startup_authored_collision_pair(self) -> None:
        initial = replace(
            state_at(0.0),
            collision_pair_active={PAIR_ROCKET_CRADLE: False},
        ).frozen()
        coupling = FixedJointCoupling(
            command_latency_s=0.0,
            confirmation_steps=1,
            code_hash="coupling-test",
        )
        with self.assertRaisesRegex(ValueError, "always-present collision pair"):
            coupling.prepare(
                ScenarioContext(
                    scenario_id="release",
                    backend_capabilities={"resync": True},
                )
            )
        with self.assertRaisesRegex(ValueError, "collision pair to be present"):
            coupling.reset(initial)

    def test_six_step_release_is_ordered_resynced_and_brake_gated(self) -> None:
        initial = state_at(0.0)
        backend = AnalyticBackend(initial, layout(), gravity_mps2=(0.0, 0.0, 0.0))
        coupling = FixedJointCoupling(
            command_latency_s=0.0,
            confirmation_steps=1,
            code_hash="coupling-test",
        )
        coupling.prepare(
            ScenarioContext(
                scenario_id="release",
                backend_capabilities=backend.capabilities.features,
            )
        )
        coupling.reset(initial)
        with self.assertRaises(ReleaseError):
            coupling.request_release(observe(initial), aft_marker_outside=False)
        events = list(coupling.request_release(observe(initial), aft_marker_outside=True))
        self.assertFalse(coupling.brake_eligible)

        output = coupling.pre_step(observe(initial))
        self.assertEqual(output.effects.collision_commands, ())
        events.extend(output.events)
        accepted = aggregate((output.effects,), initial)
        applied = backend.apply(accepted)
        events.extend(coupling.resync_after_apply(backend, applied, initial))
        self.assertEqual(backend.read_state().body(BODY_ROCKET), initial.body(BODY_ROCKET))
        self.assertEqual(backend.resync_count, 1)
        backend.step()
        events.extend(coupling.post_step(backend.read_state()).events)

        self.assertEqual(coupling.phase, ReleasePhase.RELEASED)
        self.assertTrue(coupling.brake_eligible)
        self.assertFalse(backend.read_state().joint_active[JOINT_COUPLING])
        self.assertTrue(backend.read_state().collision_pair_active[PAIR_ROCKET_CRADLE])
        steps = [event.data["transaction_step"] for event in events if event.name == EVENT_RELEASE_STEP]
        self.assertEqual(steps, [1, 2, 3, 4, 5, 6])
        self.assertEqual(sum(event.name == EVENT_RELEASE_CONFIRMED for event in events), 1)
        self.assertEqual(sum(event.name == EVENT_BRAKE_ELIGIBLE for event in events), 1)


class SeparationAndInterlockTests(unittest.TestCase):
    def test_named_envelope_gap_confirmation_and_recontact_abort(self) -> None:
        released = state_at(0.0, cart_x=10.0, rocket_x=10.0, coupled=False)
        measurement = measure_separation(released, layout(), markers())
        self.assertAlmostEqual(measurement.gap_m, -3.25)
        monitor = SeparationMonitor(ignition())
        monitor.begin(released)
        clear = state_at(
            0.2,
            cart_x=10.0,
            rocket_x=16.5,
            cart_speed=5.0,
            rocket_speed=10.0,
            coupled=False,
        )
        events = monitor.update(clear, measure_separation(clear, layout(), markers()))
        self.assertEqual([event.name for event in events], [EVENT_SEPARATION_CONFIRMED])
        self.assertTrue(monitor.status.confirmed)

        recontact = replace(
            clear,
            time_s=0.3,
            step_index=3,
            contacts={
                PAIR_ROCKET_CRADLE: ContactReport(
                    pair=PAIR_ROCKET_CRADLE,
                    impulse_ns=(1.0, 0.0, 0.0),
                    active=True,
                )
            },
        ).frozen()
        events = monitor.update(recontact, measure_separation(recontact, layout(), markers()))
        self.assertEqual([event.name for event in events], [EVENT_RECONTACT, EVENT_ABORT])
        self.assertTrue(monitor.status.failed)

    def test_all_seven_ignition_gates_fail_closed(self) -> None:
        clear_state = state_at(
            0.5,
            cart_x=10.0,
            rocket_x=16.5,
            cart_speed=5.0,
            rocket_speed=10.0,
            coupled=False,
        )
        interlock = IgnitionInterlock(ignition(), layout(), markers())
        clear_observation = observe(clear_state, gap=3.25, rate=5.0)
        decision = interlock.evaluate(
            clear_observation,
            release_time_s=0.0,
            separation_confirmed=True,
            abort_active=False,
        )
        self.assertTrue(decision.allowed)
        self.assertEqual(tuple(decision.gate_status), IgnitionInterlock.GATE_ORDER)

        cases = {
            "attachment_released": observe(state_at(0.5), gap=3.25, rate=5.0),
            "rocket_outside_tube": observe(
                state_at(0.5, cart_x=5.0, rocket_x=5.0, coupled=False), gap=3.25, rate=5.0
            ),
            "clearance_confirmed": observe(clear_state, gap=2.9, rate=5.0),
            "ignition_delay_elapsed": clear_observation,
            "no_active_collision": observe(
                replace(
                    clear_state,
                    contacts={PAIR_ROCKET_CRADLE: ContactReport(PAIR_ROCKET_CRADLE, active=True)},
                ).frozen(),
                gap=3.25,
                rate=5.0,
            ),
            "finite_bounded_rocket_state": observe(
                replace(
                    clear_state,
                    bodies={
                        **clear_state.bodies,
                        BODY_ROCKET: replace(
                            clear_state.body(BODY_ROCKET), angular_velocity=(0.0, 0.0, math.radians(6.0))
                        ),
                    },
                ).frozen(),
                gap=3.25,
                rate=5.0,
            ),
            "no_abort": clear_observation,
        }
        for gate, observation in cases.items():
            with self.subTest(gate=gate):
                checked = interlock.evaluate(
                    observation,
                    release_time_s=0.4 if gate == "ignition_delay_elapsed" else 0.0,
                    separation_confirmed=gate != "clearance_confirmed",
                    abort_active=gate == "no_abort",
                )
                self.assertFalse(checked.allowed)
                self.assertIn(gate, checked.blocked_gates)


class RocketAndStateMachineTests(unittest.TestCase):
    def test_motor_is_interlock_gated_and_acts_only_on_rocket(self) -> None:
        released = state_at(0.5, coupled=False, cart_speed=0.0, rocket_speed=0.0)
        observation = observe(released, gap=3.25, rate=1.0)
        motor = ConstantMassThrustMotor(
            MotorConfig(model="constant", thrust_n=1500.0, burn_duration_s=1.0),
            code_hash="motor-test",
        )
        motor.prepare(ScenarioContext(scenario_id="motor"))
        motor.reset(released)
        blocked = InterlockDecision(False, ("attachment_released",), {"attachment_released": False})
        self.assertEqual(motor.command_ignition(observation, blocked)[0].name, EVENT_INTERLOCK_BLOCKED)
        self.assertTrue(motor.pre_step(observation).effects.is_empty())

        allowed = InterlockDecision(True, (), {})
        self.assertEqual(motor.command_ignition(observation, allowed)[0].name, EVENT_IGNITION)
        output = motor.pre_step(observation)
        backend = AnalyticBackend(released, layout(), gravity_mps2=(0.0, 0.0, 0.0))
        backend.apply(aggregate((output.effects,), released))
        backend.step()
        result = backend.read_state()
        self.assertEqual(result.body(BODY_CART).linear_velocity, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(result.body(BODY_ROCKET).linear_velocity[0], 1.0)

    def test_post_detach_branches_join_only_after_both_finish_and_abort_dominates(self) -> None:
        machine = MissionStateMachine()
        machine.arm(time_s=0.0, step_index=0)
        machine.release_hold(time_s=0.0, step_index=0)
        machine.start_launch(0, "vacuum", time_s=0.0, step_index=0)
        machine.exit_approach(time_s=1.0, step_index=1)
        machine.rocket_detach(time_s=1.0, step_index=1)
        machine.process((Event(EVENT_RELEASE_CONFIRMED, 1.1, 2, "coupling"),))
        self.assertEqual(machine.state.cart_branch, CartBranch.BRAKING)
        self.assertEqual(machine.state.rocket_branch, RocketBranch.SEPARATION)
        machine.process((Event(EVENT_SEPARATION_CONFIRMED, 1.2, 3, "separation_actuator"),))
        machine.process((Event(EVENT_IGNITION, 1.3, 4, "rocket_motor"),))
        machine.process((Event(EVENT_CART_STOPPED, 2.0, 5, "cart_brake"),))
        self.assertNotEqual(machine.state.phase, MissionPhase.COMPLETE)
        machine.process((Event(EVENT_FLIGHT_WINDOW_COMPLETE, 3.0, 6, "orchestrator"),))
        self.assertEqual(machine.state.phase, MissionPhase.COMPLETE)

        aborted = MissionStateMachine()
        aborted.arm(time_s=0.0, step_index=0)
        aborted.process((Event(EVENT_ABORT, 0.1, 1, "guide", {"reason": "limit"}),))
        aborted.process((Event(EVENT_CART_STOPPED, 0.2, 2, "cart_brake"),))
        self.assertEqual(aborted.state.phase, MissionPhase.ABORT)

        out_of_order = MissionStateMachine()
        out_of_order.arm(time_s=0.0, step_index=0)
        out_of_order.process((Event(EVENT_IGNITION, 0.1, 1, "rocket_motor"),))
        self.assertEqual(out_of_order.state.phase, MissionPhase.ABORT)
        self.assertEqual(out_of_order.state.abort_reason, "invalid_event_order:ignition")


class CompleteAnalyticMissionTests(unittest.TestCase):
    def test_baseline_release_separation_ignition_and_concurrent_completion(self) -> None:
        loaded = load_yaml(BASELINE)
        config = loaded.config
        tube = resolve_tube_layout(config)
        initial_pose = path_pose(tube, 0.0)
        half_angle = -0.5 * math.radians(initial_pose.inclination_deg)
        orientation = (math.cos(half_angle), 0.0, math.sin(half_angle), 0.0)
        initial = SimulationState(
            time_s=0.0,
            step_index=0,
            dt_s=config.simulation.physics_dt_s,
            bodies={
                BODY_CART: BodyState(
                    name=BODY_CART,
                    position=initial_pose.position_m,
                    orientation=orientation,
                    mass_kg=config.cart.mass_kg,
                ),
                BODY_ROCKET: BodyState(
                    name=BODY_ROCKET,
                    position=initial_pose.position_m,
                    orientation=orientation,
                    mass_kg=config.rocket.initial_mass_kg,
                ),
            },
            joint_active={JOINT_COUPLING: True, JOINT_GUIDE: True},
            collision_pair_active={PAIR_ROCKET_CRADLE: True},
        ).frozen()
        # The production factory is the wiring under test. Restating it here would let the
        # suite pass against a component set no shipped entry point ever assembles.
        backend = AnalyticBackend(initial, tube)
        orchestrator = build_mission(config, tube, backend, free_flight_duration_s=0.5)
        result = orchestrator.run()
        self.assertIsNotNone(orchestrator.last_interlock_decision)
        self.assertEqual(result.mission_state.phase, MissionPhase.COMPLETE)
        names = [event.name for event in result.events]
        self.assertIn(EVENT_RELEASE_CONFIRMED, names)
        self.assertIn(EVENT_SEPARATION_CONFIRMED, names)
        self.assertIn(EVENT_IGNITION, names)
        self.assertIn(EVENT_CART_STOPPED, names)
        self.assertIn(EVENT_FLIGHT_WINDOW_COMPLETE, names)
        self.assertLess(names.index(EVENT_RELEASE_CONFIRMED), names.index(EVENT_SEPARATION_CONFIRMED))
        self.assertLess(names.index(EVENT_SEPARATION_CONFIRMED), names.index(EVENT_IGNITION))
        release_steps = [
            event.data["transaction_step"]
            for event in result.events
            if event.name == EVENT_RELEASE_STEP
        ]
        self.assertEqual(release_steps, [1, 2, 3, 4, 5, 6])
        self.assertGreater(
            result.final_state.body(BODY_ROCKET).position[0],
            result.final_state.body(BODY_CART).position[0],
        )
        self.assertLess(
            math.sqrt(sum(value * value for value in result.final_state.body(BODY_CART).linear_velocity)),
            1e-9,
        )

        first_names = [event.name for event in result.events]
        first_bodies = dict(result.final_state.bodies)
        orchestrator.reset()
        repeated = orchestrator.run()
        self.assertEqual(repeated.mission_state.phase, MissionPhase.COMPLETE)
        self.assertEqual([event.name for event in repeated.events], first_names)
        self.assertEqual(dict(repeated.final_state.bodies), first_bodies)
        self.assertEqual(backend.resync_count, 1)

    def test_interlock_adjudicates_from_the_monitors_real_confirmation_flag(self) -> None:
        # Gating entry to the interlock on the monitor's own confirmation and then passing a
        # literal True made the clearance gate incapable of failing on that input, so a
        # monitor that wrongly confirmed would have bypassed the interlock instead of being
        # caught by it. The gate has to be able to block, and to do so for real reasons.
        loaded = load_yaml(BASELINE)
        config = loaded.config
        tube = resolve_tube_layout(config)
        initial_pose = path_pose(tube, 0.0)
        half_angle = -0.5 * math.radians(initial_pose.inclination_deg)
        orientation = (math.cos(half_angle), 0.0, math.sin(half_angle), 0.0)
        initial = SimulationState(
            time_s=0.0,
            step_index=0,
            dt_s=config.simulation.physics_dt_s,
            bodies={
                BODY_CART: BodyState(
                    name=BODY_CART,
                    position=initial_pose.position_m,
                    orientation=orientation,
                    mass_kg=config.cart.mass_kg,
                ),
                BODY_ROCKET: BodyState(
                    name=BODY_ROCKET,
                    position=initial_pose.position_m,
                    orientation=orientation,
                    mass_kg=config.rocket.initial_mass_kg,
                ),
            },
            joint_active={JOINT_COUPLING: True, JOINT_GUIDE: True},
            collision_pair_active={PAIR_ROCKET_CRADLE: True},
        ).frozen()
        backend = AnalyticBackend(initial, tube)
        orchestrator = build_mission(config, tube, backend, free_flight_duration_s=0.5)
        orchestrator.start()
        blocked_counts: dict[str, int] = {}
        adjudicated = 0
        while orchestrator.mission_state.phase not in (MissionPhase.COMPLETE, MissionPhase.ABORT):
            orchestrator.step()
            decision = orchestrator.last_interlock_decision
            if decision is None:
                continue
            adjudicated += 1
            for gate in decision.blocked_gates:
                blocked_counts[gate] = blocked_counts.get(gate, 0) + 1

        self.assertEqual(orchestrator.mission_state.phase, MissionPhase.COMPLETE)
        # Adjudication starts at release confirmation, not at separation confirmation.
        self.assertGreater(adjudicated, 0)
        self.assertGreater(blocked_counts.get("clearance_confirmed", 0), 0)
        self.assertGreater(blocked_counts.get("ignition_delay_elapsed", 0), 0)
        # Gates that no nominal run can trip must never have blocked spuriously.
        for gate in ("no_active_collision", "finite_bounded_rocket_state", "no_abort"):
            self.assertEqual(blocked_counts.get(gate, 0), 0)


if __name__ == "__main__":
    unittest.main()
