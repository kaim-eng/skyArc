# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Configuration failure type used before any scene is created."""


class ConfigurationError(ValueError):
    """Raised when schema parsing or cross-field preflight fails."""
