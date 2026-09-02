# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import unittest

import _bootstrap  # noqa: F401

from test_component_adapter import DummyComponent
from skyarc.components import (
    ComponentRegistry,
    GroundTruthObserver,
    RegistryError,
    ScenarioContext,
)
from skyarc.launcher import TubeLayout, TubeStage
from skyarc.names import (
    BODY_CART,
    BODY_ROCKET,
    MARKER_ROCKET_STAGNATION,
    SLOT_LAUNCH_FORCE,
)
from skyarc.state import BodyState, MarkerSpec, SimulationState


class RegistryAndObserverTests(unittest.TestCase):
    def test_registry_resolves_and_rejects_duplicates(self) -> None:
        registry = ComponentRegistry()
        registry.register(SLOT_LAUNCH_FORCE, "dummy_v1", lambda parameters: DummyComponent())
        component = registry.resolve(SLOT_LAUNCH_FORCE, "dummy_v1", {"force": 12.0})
        self.assertEqual(component.descriptor.model_id, "dummy_v1")
        with self.assertRaises(RegistryError):
            registry.register(SLOT_LAUNCH_FORCE, "dummy_v1", lambda parameters: DummyComponent())
        with self.assertRaises(RegistryError):
            registry.resolve(SLOT_LAUNCH_FORCE, "missing")

    def test_registry_deep_freezes_factory_parameters(self) -> None:
        registry = ComponentRegistry()
        received = []

        def factory(parameters):  # type: ignore[no-untyped-def]
            received.append(parameters)
            return DummyComponent()

        registry.register(SLOT_LAUNCH_FORCE, "dummy_v1", factory)
        source = {"curve": [1.0, 2.0]}
        registry.resolve(SLOT_LAUNCH_FORCE, "dummy_v1", source)
        source["curve"][0] = 99.0
        self.assertEqual(received[0]["curve"], (1.0, 2.0))
        with self.assertRaises(TypeError):
            received[0]["curve"][0] = 3.0

    def test_ground_truth_observer_uses_marker_density_and_keeps_latent_state_distinct(self) -> None:
        layout = TubeLayout(
            origin_m=(0.0, 0.0, 0.0),
            angle_deg=0.0,
            stages=(
                TubeStage("vacuum", 10.0, 0.0),
                TubeStage("dense", 10.0, 0.5),
            ),
        )
        marker = MarkerSpec(
            name=MARKER_ROCKET_STAGNATION,
            body=BODY_ROCKET,
            offset_m=(2.0, 0.0, 0.0),
        )
        observer = GroundTruthObserver(
            layout,
            {MARKER_ROCKET_STAGNATION: marker},
            code_hash="observer-hash",
        )
        context = ScenarioContext(
            scenario_id="baseline",
            markers={MARKER_ROCKET_STAGNATION: marker},
        )
        observer.prepare(context)
        state = SimulationState(
            time_s=1.0,
            step_index=240,
            dt_s=1.0 / 240.0,
            bodies={
                BODY_CART: BodyState(
                    name=BODY_CART,
                    position=(8.0, 0.0, 0.0),
                    linear_velocity=(4.0, 0.0, 0.0),
                    mass_kg=250.0,
                ),
                BODY_ROCKET: BodyState(
                    name=BODY_ROCKET,
                    position=(9.0, 0.0, 0.0),
                    linear_velocity=(4.0, 0.0, 0.0),
                    mass_kg=150.0,
                ),
            },
        )
        observation = observer.observe(
            state,
            coupled=True,
            separation_gap_m=-0.2,
            separation_rate_mps=0.0,
        )
        self.assertIsNot(observation.state, state)
        self.assertEqual(observation.axial.stage_index, 1)
        self.assertEqual(observation.axial.stage_name, "dense")
        self.assertEqual(observation.axial.effective_density_ratio, 0.5)
        self.assertEqual(observation.axial.assembly_mass_kg, 400.0)
        self.assertEqual(observation.axial.marker(MARKER_ROCKET_STAGNATION), 11.0)
        self.assertEqual(observer.snapshot_state()["last_step_index"], 240)

    def test_observer_rejects_a_context_that_omits_its_markers(self) -> None:
        layout = TubeLayout(
            origin_m=(0.0, 0.0, 0.0),
            angle_deg=0.0,
            stages=(TubeStage("only", 10.0, 0.0),),
        )
        marker = MarkerSpec(name=MARKER_ROCKET_STAGNATION, body=BODY_ROCKET, offset_m=(2.0, 0.0, 0.0))
        observer = GroundTruthObserver(layout, {MARKER_ROCKET_STAGNATION: marker}, code_hash="h")
        # An empty marker table is the worst case, not an exemption from the check.
        with self.assertRaisesRegex(ValueError, "absent from scenario context"):
            observer.prepare(ScenarioContext(scenario_id="baseline", markers={}))
        with self.assertRaisesRegex(ValueError, "absent from scenario context"):
            observer.prepare(
                ScenarioContext(
                    scenario_id="baseline",
                    markers={"unrelated": MarkerSpec(name="unrelated", body=BODY_CART)},
                )
            )
        with self.assertRaisesRegex(ValueError, "disagree with scenario context"):
            observer.prepare(
                ScenarioContext(
                    scenario_id="baseline",
                    markers={
                        MARKER_ROCKET_STAGNATION: MarkerSpec(
                            name=MARKER_ROCKET_STAGNATION,
                            body=BODY_ROCKET,
                            offset_m=(3.0, 0.0, 0.0),
                        )
                    },
                )
            )


if __name__ == "__main__":
    unittest.main()
