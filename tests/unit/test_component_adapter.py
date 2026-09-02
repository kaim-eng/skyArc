# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import unittest

import _bootstrap  # noqa: F401

from skyarc.components import (
    Component,
    ComponentDescriptor,
    Determinism,
    ScenarioContext,
    StepOutput,
)
from skyarc.effects import (
    AppliedEffects,
    BackendAdapter,
    BackendCapabilities,
    EffectBatch,
    aggregate,
)
from skyarc.names import BODY_CART, SLOT_LAUNCH_FORCE, SLOT_ROCKET_MOTOR
from skyarc.state import BodyState, SimulationState


def make_state() -> SimulationState:
    return SimulationState(
        time_s=0.0,
        step_index=0,
        dt_s=0.01,
        bodies={BODY_CART: BodyState(name=BODY_CART)},
    ).frozen()


class DummyComponent(Component):
    @property
    def descriptor(self) -> ComponentDescriptor:
        return ComponentDescriptor(
            slot=SLOT_LAUNCH_FORCE,
            model_id="dummy_v1",
            model_version="1.0.0",
            parameter_schema_version="1",
            code_hash="abc123",
            determinism=Determinism.DETERMINISTIC,
        )

    def prepare(self, context: ScenarioContext) -> None:
        self.context = context

    def reset(self, initial_state: SimulationState) -> None:
        self.state = initial_state

    def pre_step(self, observation):  # type: ignore[no-untyped-def]
        return StepOutput.empty(SLOT_LAUNCH_FORCE)

    def post_step(self, state: SimulationState) -> StepOutput:
        return StepOutput.empty(SLOT_LAUNCH_FORCE)

    def snapshot_state(self):  # type: ignore[no-untyped-def]
        return {"ready": True}


class DummyAdapter:
    def __init__(self) -> None:
        self._state = make_state()
        self._capabilities = BackendCapabilities("analytic", "cpu", {"resync": True})

    @property
    def capabilities(self) -> BackendCapabilities:
        return self._capabilities

    def read_state(self) -> SimulationState:
        return self._state

    def apply(self, effects):  # type: ignore[no-untyped-def]
        return AppliedEffects.exactly(effects)

    def step(self) -> None:
        pass

    def resync(self) -> None:
        pass

    def reset(self) -> None:
        pass


class ComponentAndAdapterTests(unittest.TestCase):
    def test_context_is_immutable_copy(self) -> None:
        parameters = {"force_n": 10.0}
        context = ScenarioContext(scenario_id="baseline", parameters=parameters)
        parameters["force_n"] = 99.0
        self.assertEqual(context.parameters["force_n"], 10.0)
        with self.assertRaises(TypeError):
            context.parameters["force_n"] = 1.0  # type: ignore[index]

    def test_descriptor_and_output_provenance(self) -> None:
        component = DummyComponent()
        component.validate_output(StepOutput.empty(SLOT_LAUNCH_FORCE))
        with self.assertRaises(ValueError):
            component.validate_output(StepOutput(effects=EffectBatch.empty(SLOT_ROCKET_MOTOR)))

    def test_invalid_descriptor_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            ComponentDescriptor(
                slot="not_a_slot",
                model_id="x",
                model_version="1",
                parameter_schema_version="1",
                code_hash="hash",
            )

    def test_adapter_protocol_and_applied_effect_record(self) -> None:
        adapter = DummyAdapter()
        self.assertIsInstance(adapter, BackendAdapter)
        accepted = aggregate((), adapter.read_state())
        applied = adapter.apply(accepted)
        self.assertEqual(dict(applied.loads), {})
        self.assertTrue(adapter.capabilities.supports("resync"))
        self.assertFalse(adapter.capabilities.supports("ccd"))


if __name__ == "__main__":
    unittest.main()
