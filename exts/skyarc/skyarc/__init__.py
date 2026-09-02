# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Configurable vacuum-tube electromagnetic launch simulation.

This package is deliberately split into a backend-neutral core and a thin Isaac Sim layer.

Nothing imported at the top level of this package may import Isaac Sim, ``omni``, ``numpy``
or ``warp``. The core (``configuration``, ``components``, ``effects``, ``launcher``,
``rocket``, ``coupling``, ``experiments``, ``telemetry``, ``state_machine``, ``orchestrator``)
is pure Python so that the unit suite in ``tests/unit`` can exercise it without a Kit
application, as required by section 15 of the design review.

The Isaac Sim layer lives in ``effects.backends.isaac``, ``launcher.scene`` and
``visualization``; those modules import Isaac Sim and are only loaded by the extension or
by the standalone runner after the application exists.
"""

__version__ = "0.1.0"
