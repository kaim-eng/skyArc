# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Thin Kit control panel that delegates every action to the common mission owner."""

from __future__ import annotations

import importlib
from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class LauncherUiActions:
    load_configuration: Callable[[str], None]
    generate_scene: Callable[[], None]
    arm_and_launch: Callable[[], None]
    toggle_pause: Callable[[], None]
    single_step: Callable[[], None]
    abort: Callable[[], None]
    reset: Callable[[], None]
    export: Callable[[], None]
    select_camera: Callable[[str], None]
    show_forces: Callable[[bool], None]
    show_fields: Callable[[bool], None]


class LauncherControlPanel:
    """View/controller only; it never advances physics except through supplied actions."""

    def __init__(self, actions: LauncherUiActions, camera_names: tuple[str, ...]) -> None:
        ui = importlib.import_module("omni.ui")
        self._ui = ui
        self._actions = actions
        self._window = ui.Window("skyArc Launcher", width=380, height=520)
        with self._window.frame:
            with ui.VStack(spacing=6):
                ui.Label("Configuration")
                self._configuration = ui.StringField()
                ui.Button(
                    "Load",
                    clicked_fn=lambda: actions.load_configuration(
                        self._configuration.model.get_value_as_string()
                    ),
                )
                ui.Button("Generate scene", clicked_fn=actions.generate_scene)
                with ui.HStack():
                    ui.Button("Arm + launch", clicked_fn=actions.arm_and_launch)
                    ui.Button("Pause", clicked_fn=actions.toggle_pause)
                    ui.Button("Step", clicked_fn=actions.single_step)
                with ui.HStack():
                    ui.Button("Abort", clicked_fn=actions.abort)
                    ui.Button("Reset", clicked_fn=actions.reset)
                    ui.Button("Export", clicked_fn=actions.export)
                ui.Label("Camera")
                for name in camera_names:
                    ui.Button(name.replace("_", " ").title(), clicked_fn=lambda n=name: actions.select_camera(n))
                ui.CheckBox(model=ui.SimpleBoolModel(True)).model.add_value_changed_fn(
                    lambda model: actions.show_forces(model.get_value_as_bool())
                )
                ui.Label("Force vectors")
                ui.CheckBox(model=ui.SimpleBoolModel(True)).model.add_value_changed_fn(
                    lambda model: actions.show_fields(model.get_value_as_bool())
                )
                ui.Label("Magnetic-field graphics")
                self._status = ui.Label("Idle")

    def set_status(self, text: str) -> None:
        self._status.text = text

    def destroy(self) -> None:
        self._window.visible = False
        self._window = None


__all__ = ["LauncherControlPanel", "LauncherUiActions"]
