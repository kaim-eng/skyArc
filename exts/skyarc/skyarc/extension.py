# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Kit extension entrypoint for the production vacuum-tube launcher layer."""

from __future__ import annotations

import carb
import omni.ext


class VacuumTubeLauncherExtension(omni.ext.IExt):
    """Keep extension startup side-effect free; runners explicitly build each scene."""

    def on_startup(self, extension_id: str) -> None:
        self._extension_id = extension_id
        carb.log_info(
            "Vacuum Tube Launcher extension enabled; production scenes are built explicitly "
            "from a validated schema-v3 configuration"
        )

    def on_shutdown(self) -> None:
        carb.log_info("Vacuum Tube Launcher extension disabled")
        self._extension_id = None


__all__ = ["VacuumTubeLauncherExtension"]
