# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Deterministic slot/model resolution for replaceable components."""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Callable, Dict, Mapping

from ..names import ALL_SLOTS
from .contract import Component, _deep_freeze


class RegistryError(ValueError):
    """Raised for duplicate, unknown, or dishonest model registrations."""


ComponentFactory = Callable[[Mapping[str, object]], Component]


@dataclass(frozen=True)
class ModelRegistration:
    slot: str
    model_id: str
    factory: ComponentFactory


class ComponentRegistry:
    """A mutable setup-time registry that resolves to immutable component instances."""

    def __init__(self) -> None:
        self._registrations: Dict[tuple[str, str], ModelRegistration] = {}

    def register(self, slot: str, model_id: str, factory: ComponentFactory) -> None:
        if slot not in ALL_SLOTS:
            raise RegistryError(f"unknown component slot {slot!r}")
        if not model_id:
            raise RegistryError("model_id may not be empty")
        if not callable(factory):
            raise RegistryError("component factory must be callable")
        key = (slot, model_id)
        if key in self._registrations:
            raise RegistryError(f"model {model_id!r} is already registered for slot {slot!r}")
        self._registrations[key] = ModelRegistration(slot=slot, model_id=model_id, factory=factory)

    def resolve(
        self,
        slot: str,
        model_id: str,
        parameters: Mapping[str, object] | None = None,
    ) -> Component:
        try:
            registration = self._registrations[(slot, model_id)]
        except KeyError:
            available = sorted(
                candidate_model
                for candidate_slot, candidate_model in self._registrations
                if candidate_slot == slot
            )
            raise RegistryError(
                f"model {model_id!r} is not registered for slot {slot!r}; available: {available}"
            ) from None
        frozen_parameters = _deep_freeze(parameters or {})
        component = registration.factory(frozen_parameters)
        descriptor = component.descriptor
        if descriptor.slot != slot or descriptor.model_id != model_id:
            raise RegistryError(
                f"factory registered as {slot}/{model_id} returned descriptor "
                f"{descriptor.slot}/{descriptor.model_id}"
            )
        return component

    def models_for_slot(self, slot: str) -> tuple[str, ...]:
        return tuple(
            sorted(model_id for candidate_slot, model_id in self._registrations if candidate_slot == slot)
        )

    @property
    def registrations(self) -> Mapping[tuple[str, str], ModelRegistration]:
        return MappingProxyType(dict(self._registrations))
