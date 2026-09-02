# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Kit integration coverage for the production adapter boundary.

The pure suite already proves the mission ordering, the release transaction and the guide
reaction law.  What it cannot prove is that the *boundary* behaves: that solver state
reconstructs into global SI, that the applied load differs from the accepted load by
exactly the backend's own slot, that a resync does not advance time, and that a stop/rebuild
reset returns the authored state.  Those are the cases here, and each one needs a real
PhysX scene.

Every test builds its own stage.  Sharing one would let a released joint or an accumulated
solver offset leak from one case into the next, which is precisely the class of defect the
reset test exists to detect.
"""

from __future__ import annotations

import math
from pathlib import Path

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.kit.test
from isaacsim.core.simulation_manager import SimulationManager
from pxr import UsdPhysics

from ..components.contract import ScenarioContext
from ..components.observers import GroundTruthObserver
from ..configuration import load_yaml, resolve_tube_layout
from ..coupling import FixedJointCoupling, measure_separation
from ..effects.aggregator import aggregate
from ..effects.types import (
    CollisionAction,
    CollisionPairCommand,
    ConstraintAction,
    ConstraintCommand,
    EffectBatch,
    Frame,
    MassUpdate,
    MomentumPolicy,
    Wrench,
)
from ..events import EVENT_ABORT, EVENT_RELEASE_CONFIRMED
from ..launcher.geometry import CurvedTubeLayout, path_pose
from ..orchestrator import marker_specs
from ..launcher.production import (
    build_production_scene_plan,
    load_production_fixture,
    resolve_initial_solver_states,
)
from ..launcher.production_runtime import ProductionMissionRuntime
from ..linalg import add, dot, norm, scale, sub
from ..names import (
    BODY_CART,
    BODY_ROCKET,
    JOINT_COUPLING,
    PAIR_ROCKET_CRADLE,
    SLOT_BACKEND_ADAPTER,
    SLOT_LAUNCH_FORCE,
    SLOT_ROCKET_MOTOR,
)
from ..state_machine import MissionPhase


PROJECT_ROOT = Path(__file__).resolve().parents[4]
CONFIGURATION = PROJECT_ROOT / "configs" / "curved_2kms.yaml"
FIXTURE = PROJECT_ROOT / "configs" / "phase0_anti_tunneling_open_cradle.json"
PHYSICS_SCENE_PATH = "/World/PhysicsScene"
GRAVITY_MPS2 = (0.0, 0.0, -9.81)

GUIDED_STEPS = 200
"""Steps taken by the guided cases.

Long enough to leave the settling transient and to accumulate any systematic tracking
drift, short enough that the whole Kit suite stays inside a normal test timeout. Full
54,116-step profiles belong to the standalone evidence runner, not here.
"""


class ProductionAdapterTestBase(omni.kit.test.AsyncTestCase):
    """Build one production scene, adapter and mission per test."""

    async def setUp(self) -> None:
        self.assertTrue(CONFIGURATION.is_file(), f"missing configuration: {CONFIGURATION}")
        loaded = load_yaml(CONFIGURATION)
        self.config = loaded.config
        self.layout = resolve_tube_layout(self.config)
        self.assertIsInstance(self.layout, CurvedTubeLayout)
        self.fixture = load_production_fixture(FIXTURE)
        self.plan = build_production_scene_plan(self.config, self.layout, self.fixture)

        stage_utils.create_new_stage()
        await app_utils.update_app_async()
        self.stage = stage_utils.get_current_stage()
        scene = UsdPhysics.Scene.Define(self.stage, PHYSICS_SCENE_PATH)
        scene.CreateGravityDirectionAttr().Set((0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(-GRAVITY_MPS2[2])
        SimulationManager.set_default_physics_scene(PHYSICS_SCENE_PATH)
        SimulationManager.switch_physics_engine(self.plan.backend, verbose=False)
        SimulationManager.setup_simulation(
            dt=self.config.simulation.physics_dt_s, device=self.plan.device
        )
        SimulationManager.enable_ccd(False, physics_scene=PHYSICS_SCENE_PATH)

        self.runtime = ProductionMissionRuntime(
            self.stage,
            self.config,
            self.layout,
            self.fixture,
            self.plan,
            gravity_mps2=GRAVITY_MPS2,
            author_visuals=False,
            update=omni.kit.app.get_app().update,
        )
        self.backend = self.runtime.backend
        self.mission = self.runtime.mission

    async def tearDown(self) -> None:
        app_utils.stop()
        await app_utils.update_app_async()
        self.runtime = None
        self.backend = None
        self.mission = None
        stage_utils.create_new_stage()
        await app_utils.update_app_async()

    def assembly_centre_of_mass(self, state) -> tuple[float, float, float]:
        cart = state.body(BODY_CART)
        rocket = state.body(BODY_ROCKET)
        total = cart.mass_kg + rocket.mass_kg
        return scale(
            add(scale(cart.position, cart.mass_kg), scale(rocket.position, rocket.mass_kg)),
            1.0 / total,
        )


class TestProductionAdapterBoundary(ProductionAdapterTestBase):
    async def test_selected_runtime_and_capabilities_are_the_phase0_condition(self) -> None:
        capabilities = self.backend.capabilities
        self.assertEqual(capabilities.backend, "physx")
        self.assertEqual(capabilities.device, "cpu")
        self.assertTrue(capabilities.supports("resync"))
        self.assertTrue(capabilities.supports("always_present_collision_pair"))
        self.assertTrue(capabilities.supports("translated_accelerating_frame"))
        self.assertTrue(capabilities.supports("commanded_path_reaction"))
        # The whole v0.29 disposition rests on this being false; a backend that claimed a
        # solver constraint reaction would silently re-qualify hardware contact loads.
        self.assertFalse(capabilities.supports("solver_constraint_reaction"))
        self.assertEqual(SimulationManager.get_active_physics_engine(), "physx")
        self.assertIn("cpu", str(SimulationManager.get_device()).lower())

    async def test_initial_state_reconstructs_global_si_on_the_centerline(self) -> None:
        state = self.backend.read_state()
        self.assertEqual(state.time_s, 0.0)
        self.assertEqual(state.step_index, 0)
        self.assertTrue(state.joint_active[JOINT_COUPLING])
        self.assertTrue(state.collision_pair_active[PAIR_ROCKET_CRADLE])

        entrance = path_pose(self.layout, 0.0)
        cart_s_m = self.layout.axial_position(state.body(BODY_CART).position)
        self.assertLess(cart_s_m, 0.0)
        com_offset_m = norm(sub(self.assembly_centre_of_mass(state), entrance.position_m))
        self.assertLess(
            com_offset_m,
            1e-3,
            f"assembly centre of mass off the qualified entrance by {com_offset_m} m",
        )
        spacing_m = norm(
            sub(state.body(BODY_ROCKET).position, state.body(BODY_CART).position)
        )
        self.assertAlmostEqual(spacing_m, self.plan.cart_to_rocket_offset_m, delta=1e-3)
        # Solver coordinates are near the origin even though the global pose is not; that
        # is the entire point of the translated frame.
        self.assertLess(self.backend.peak_solver_offset_m, 10.0)
        self.assertGreater(norm(entrance.position_m) + spacing_m, 0.0)

    async def test_applied_load_exceeds_accepted_only_by_the_backend_slot(self) -> None:
        state = self.backend.read_state()
        accepted = aggregate(
            (
                EffectBatch(
                    source=SLOT_LAUNCH_FORCE,
                    wrenches=(
                        Wrench(
                            source=SLOT_LAUNCH_FORCE,
                            body=BODY_CART,
                            force_n=scale(path_pose(self.layout, 0.0).tangent, 1234.0),
                            application_point_m=state.body(BODY_CART).position,
                            frame=Frame.WORLD,
                        ),
                    ),
                ),
            ),
            state,
        )
        applied = self.backend.apply(accepted)
        reaction = self.backend.last_guide_reaction
        self.assertIsNotNone(reaction)
        self.assertEqual(reaction.body, BODY_CART)

        cart_applied = applied.loads[BODY_CART]
        difference = sub(cart_applied.force_n, accepted.load(BODY_CART).force_n)
        for index in range(3):
            self.assertAlmostEqual(difference[index], reaction.force_n[index], delta=1e-6)
            self.assertAlmostEqual(
                cart_applied.force_by_slot[SLOT_BACKEND_ADAPTER][index],
                reaction.force_n[index],
                delta=1e-9,
            )
        self.assertEqual(
            cart_applied.force_by_slot[SLOT_LAUNCH_FORCE],
            accepted.load(BODY_CART).force_by_slot[SLOT_LAUNCH_FORCE],
        )
        # The rocket is carried by the joint; the backend adds nothing of its own to it.
        rocket_applied = applied.loads.get(BODY_ROCKET)
        if rocket_applied is not None:
            self.assertNotIn(SLOT_BACKEND_ADAPTER, rocket_applied.force_by_slot)

    async def test_live_collision_filter_mutation_is_refused(self) -> None:
        state = self.backend.read_state()
        accepted = aggregate(
            (
                EffectBatch(
                    source="coupling",
                    collision_commands=(
                        CollisionPairCommand(
                            source="coupling",
                            pair=PAIR_ROCKET_CRADLE,
                            action=CollisionAction.DISABLE,
                            bodies=(BODY_CART, BODY_ROCKET),
                        ),
                    ),
                ),
            ),
            state,
        )
        with self.assertRaises(ValueError):
            self.backend.apply(accepted)

    async def test_resync_does_not_advance_the_simulation(self) -> None:
        before = self.backend.read_state()
        self.backend.resync()
        after = self.backend.read_state()
        self.assertEqual(after.time_s, before.time_s)
        self.assertEqual(after.step_index, before.step_index)
        for body in (BODY_CART, BODY_ROCKET):
            first = before.body(body)
            second = after.body(body)
            # The release transaction rejects any discontinuity above 1e-9, so an
            # approximate match here would not actually be enough to release at speed.
            self.assertLessEqual(norm(sub(second.position, first.position)), 1e-9)
            self.assertLessEqual(
                norm(sub(second.linear_velocity, first.linear_velocity)), 1e-9
            )

    async def test_conserved_mass_update_preserves_global_linear_momentum(self) -> None:
        for _ in range(5):
            self.mission.step()
        before = self.backend.read_state().body(BODY_ROCKET)
        new_mass_kg = 0.5 * before.mass_kg
        update = MassUpdate(
            source=SLOT_ROCKET_MOTOR,
            body=BODY_ROCKET,
            mass_kg=new_mass_kg,
            effective_time_s=self.backend.read_state().time_s,
            momentum_policy=MomentumPolicy.CONSERVE,
        )
        effects = aggregate(
            (EffectBatch(source=SLOT_ROCKET_MOTOR, mass_updates=(update,)),),
            self.backend.read_state(),
        )
        self.backend.apply(effects)
        after = self.backend.read_state().body(BODY_ROCKET)
        for index in range(3):
            self.assertAlmostEqual(
                after.mass_kg * after.linear_velocity[index],
                before.mass_kg * before.linear_velocity[index],
                delta=1e-5,
            )


class TestProductionGuidedStepping(ProductionAdapterTestBase):
    async def test_guided_steps_hold_the_assembly_inside_the_guide_clearance(self) -> None:
        peak_tracking_error_m = 0.0
        peak_attitude_error_deg = 0.0
        for _ in range(GUIDED_STEPS):
            self.mission.step()
            reaction = self.backend.last_guide_reaction
            self.assertIsNotNone(reaction)
            peak_tracking_error_m = max(peak_tracking_error_m, reaction.tracking_error_m)
            peak_attitude_error_deg = max(
                peak_attitude_error_deg, abs(math.degrees(reaction.attitude_error_rad))
            )
        state = self.backend.read_state()
        self.assertEqual(state.step_index, GUIDED_STEPS)
        self.assertAlmostEqual(
            state.time_s, GUIDED_STEPS * self.config.simulation.physics_dt_s, places=9
        )
        self.assertNotEqual(self.mission.mission_state.phase, MissionPhase.ABORT)
        self.assertLess(peak_tracking_error_m, self.config.tube.guide_clearance_m)
        self.assertLess(peak_attitude_error_deg, 1.0)

        # The launcher, not the guide, supplies downrange motion, and it must actually do
        # so. The entrance is a 45-degree straight, so a launcher that never engaged would
        # not merely be slow: the assembly would roll backwards at -g sin(45) and this
        # would be negative.
        entrance = path_pose(self.layout, 0.0)
        speed_mps = dot(state.body(BODY_CART).linear_velocity, entrance.tangent)
        expected_mps = (
            self.runtime.reference_frame.acceleration_mps2
            * GUIDED_STEPS
            * self.config.simulation.physics_dt_s
        )
        self.assertGreater(speed_mps, 0.0)
        self.assertAlmostEqual(speed_mps, expected_mps, delta=0.02 * expected_mps)

    async def test_reset_restores_the_authored_state_and_clears_mission_history(self) -> None:
        for _ in range(20):
            self.mission.step()
        self.assertGreater(self.backend.read_state().step_index, 0)

        self.mission.reset()
        restored = self.backend.read_state()
        self.assertEqual(restored.time_s, 0.0)
        self.assertEqual(restored.step_index, 0)
        self.assertEqual(self.mission.events, ())
        self.assertTrue(restored.joint_active[JOINT_COUPLING])
        self.assertTrue(restored.collision_pair_active[PAIR_ROCKET_CRADLE])

        reference = self.runtime.reference_frame.sample(0.0)
        authored = resolve_initial_solver_states(self.layout, self.plan, self.fixture, reference)
        for body in (BODY_CART, BODY_ROCKET):
            expected = add(authored[body].position_m, reference.position_m)
            error_m = norm(sub(restored.body(body).position, expected))
            # The same float32 round-trip allowance the Phase 0 reset probe used.
            self.assertLessEqual(error_m, 1e-5, f"{body} reset position error {error_m} m")
            self.assertLessEqual(norm(restored.body(body).linear_velocity), 1e-5)

        # A reset run must be able to step again rather than merely look restored.
        self.mission.step()
        self.assertEqual(self.backend.read_state().step_index, 1)


class TestProductionReleaseTransaction(ProductionAdapterTestBase):
    """Drive the section 10.2 release transaction against the real PhysX scene.

    The orchestrator releases when the rocket aft marker crosses the exit plane, roughly
    54 seconds and 54 km into the mission. Waiting for that here would make the Kit suite
    an hour long, so this probe asserts the exit gate itself and drives the transaction at
    the tube entrance. What it is actually testing is the *adapter boundary* under a
    constraint mutation -- specifically that a resync performed while forces are already
    pending neither advances the simulation nor discards them -- which is the one boundary
    the pure suite cannot reach and the guided-stepping case never triggers.
    """

    CODE_HASH = "0" * 64

    def launch_batch(self, state) -> EffectBatch:
        """A launcher-sized tangential wrench, so the resync happens with forces pending."""
        pose = path_pose(self.layout, self.layout.axial_position(state.body(BODY_CART).position))
        assembly_mass_kg = (
            state.body(BODY_CART).mass_kg + state.body(BODY_ROCKET).mass_kg
        )
        return EffectBatch(
            source=SLOT_LAUNCH_FORCE,
            wrenches=(
                Wrench(
                    source=SLOT_LAUNCH_FORCE,
                    body=BODY_CART,
                    force_n=scale(
                        pose.tangent,
                        assembly_mass_kg
                        * (
                            self.runtime.reference_frame.acceleration_mps2
                            - dot(GRAVITY_MPS2, pose.tangent)
                        ),
                    ),
                    application_point_m=state.body(BODY_CART).position,
                    frame=Frame.WORLD,
                ),
            ),
        )

    async def test_release_resync_is_continuous_and_the_solver_consumes_the_disable(
        self,
    ) -> None:
        markers = marker_specs(self.config)
        observer = GroundTruthObserver(self.layout, markers, code_hash=self.CODE_HASH)
        coupling = FixedJointCoupling(
            command_latency_s=self.config.simulation.release_command_latency_s,
            confirmation_steps=self.config.simulation.release_confirmation_steps,
            code_hash=self.CODE_HASH,
        )
        context = ScenarioContext(
            scenario_id="kit_release_probe",
            markers=markers,
            backend_capabilities=self.backend.capabilities.features,
        )
        observer.prepare(context)
        coupling.prepare(context)
        initial = self.backend.read_state()
        observer.reset(initial)
        coupling.reset(initial)

        def observe(state):
            measurement = measure_separation(state, self.layout, markers)
            return observer.observe(
                state,
                coupled=bool(state.joint_active[JOINT_COUPLING]),
                separation_gap_m=measurement.gap_m,
                separation_rate_mps=measurement.relative_speed_mps,
            )

        # Settle briefly under a real load so the release step is not the first step.
        for _ in range(20):
            state = self.backend.read_state()
            self.backend.apply(aggregate((self.launch_batch(state),), state))
            self.backend.step()

        before = self.backend.read_state()
        attached_spacing_m = norm(
            sub(before.body(BODY_ROCKET).position, before.body(BODY_CART).position)
        )
        observation = observe(before)
        request_events = coupling.request_release(observation, aft_marker_outside=True)
        self.assertTrue(request_events)

        output = coupling.pre_step(observation)
        self.assertTrue(output.effects.constraint_commands)
        self.assertFalse(output.effects.collision_commands)
        accepted = aggregate((output.effects, self.launch_batch(before)), before)
        applied = self.backend.apply(accepted)

        events = coupling.resync_after_apply(self.backend, applied, before)
        names = [event.name for event in events]
        # The transaction aborts on any mutation-time discontinuity above 1e-9 m, so an
        # abort here is the failure this whole probe exists to detect.
        self.assertNotIn(EVENT_ABORT, names, f"release aborted: {[e.data for e in events]}")

        self.backend.step()
        after = self.backend.read_state()
        self.assertFalse(after.joint_active[JOINT_COUPLING])
        self.assertTrue(after.collision_pair_active[PAIR_ROCKET_CRADLE])
        confirmation = coupling.post_step(after).events
        self.assertIn(EVENT_RELEASE_CONFIRMED, [event.name for event in confirmation])
        self.assertTrue(coupling.brake_eligible)

        # A disabled joint the solver never consumed would hold the two bodies at the
        # authored spacing forever, so the released rocket must actually drift.
        for _ in range(100):
            state = self.backend.read_state()
            self.backend.apply(aggregate((self.launch_batch(state),), state))
            self.backend.step()
        released = self.backend.read_state()
        released_spacing_m = norm(
            sub(released.body(BODY_ROCKET).position, released.body(BODY_CART).position)
        )
        self.assertGreater(abs(released_spacing_m - attached_spacing_m), 0.01)

    async def test_the_reaction_resizes_to_the_cart_once_the_joint_is_inactive(self) -> None:
        """After release the guide holds the cart, not the cart plus the rocket.

        A reaction that stayed assembly-sized would push a 250 kg cart with 400 kg of
        support for the whole braking phase, which is a large systematic error rather than
        a transient one.
        """
        state = self.backend.read_state()
        self.backend.apply(aggregate((self.launch_batch(state),), state))
        attached = self.backend.last_guide_reaction
        self.assertIsNotNone(attached)
        self.assertTrue(attached.coupled)
        self.backend.step()

        disable = EffectBatch(
            source="coupling",
            constraint_commands=(
                ConstraintCommand(
                    source="coupling",
                    constraint=JOINT_COUPLING,
                    action=ConstraintAction.DISABLE,
                    bodies=(BODY_CART, BODY_ROCKET),
                ),
            ),
        )
        state = self.backend.read_state()
        self.backend.apply(aggregate((disable,), state))
        self.backend.resync()
        self.backend.step()

        state = self.backend.read_state()
        self.assertFalse(state.joint_active[JOINT_COUPLING])
        self.backend.apply(aggregate((self.launch_batch(state),), state))
        released = self.backend.last_guide_reaction
        self.assertIsNotNone(released)
        self.assertFalse(released.coupled)
        cart_mass_kg = state.body(BODY_CART).mass_kg
        assembly_mass_kg = cart_mass_kg + state.body(BODY_ROCKET).mass_kg
        self.assertAlmostEqual(
            released.ideal_normal_force_n / attached.ideal_normal_force_n,
            cart_mass_kg / assembly_mass_kg,
            delta=0.02,
        )


__all__ = [
    "TestProductionAdapterBoundary",
    "TestProductionGuidedStepping",
    "TestProductionReleaseTransaction",
]
