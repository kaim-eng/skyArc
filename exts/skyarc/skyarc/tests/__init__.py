# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Kit integration tests for the production vacuum-tube launcher layer.

These require a running Kit application and are therefore separate from the pure stdlib
suite under ``tests/unit`` at the repository root.  They are also excluded from every
component's declared source closure: a test-only edit must not read as a behavioural change
in the section 14.1 manifest.

Only the discovery flag is set here.  ``omni.kit.test`` imports the individual test modules
itself, so enabling the extension outside a test run does not drag ``omni.kit.test`` in.
"""

scan_for_test_modules = True
