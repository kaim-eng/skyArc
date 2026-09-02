# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Make the pure `skyarc` package importable for the unit suite.

The extension root is inserted directly. Under the previous
``isaacsim.examples.vacuum_tube_launcher`` namespace this had to point at a nested
``isaacsim/examples`` directory and was shadowed by the bundled runtime's own `isaacsim`
package outside a Kit application; owning the top-level name removes that hazard entirely.
"""

from __future__ import annotations

import sys
from pathlib import Path


EXTENSION_PACKAGE_PARENT = Path(__file__).resolve().parents[2] / "exts" / "skyarc"
sys.path.insert(0, str(EXTENSION_PACKAGE_PARENT))
