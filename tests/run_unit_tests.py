# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Convenience entry point for the pure stdlib unit suite."""

from __future__ import annotations

import unittest
from pathlib import Path


if __name__ == "__main__":
    suite = unittest.defaultTestLoader.discover(str(Path(__file__).resolve().parent / "unit"))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(0 if result.wasSuccessful() else 1)
