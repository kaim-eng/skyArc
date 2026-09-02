# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Access-order-independent named pseudo-random streams."""

from __future__ import annotations

import hashlib
import hmac
import random
from types import MappingProxyType
from typing import Iterable, Mapping


class RandomStreamError(ValueError):
    """Raised for ambiguous or undeclared random stream use."""


class NamedRandomStreams:
    """Derive one stable seed per declared name from a master seed.

    Python's process-randomized ``hash`` is deliberately not used. Adding or accessing one
    stream cannot advance another stream, which keeps paired conditions paired.
    """

    schema_version = "named_random_streams_v1"

    def __init__(self, master_seed: int, names: Iterable[str]) -> None:
        if isinstance(master_seed, bool) or not isinstance(master_seed, int):
            raise RandomStreamError("master seed must be an integer")
        declared = tuple(names)
        if any(not isinstance(name, str) or not name.strip() for name in declared):
            raise RandomStreamError("random stream names must be non-empty strings")
        if len(set(declared)) != len(declared):
            raise RandomStreamError("random stream names may not be duplicated")
        self.master_seed = master_seed
        key = f"{self.schema_version}:{master_seed}".encode("utf-8")
        self._seeds = {
            name: int.from_bytes(hmac.new(key, name.encode("utf-8"), hashlib.sha256).digest()[:16], "big")
            for name in sorted(declared)
        }
        self._streams: dict[str, random.Random] = {}

    @property
    def seeds(self) -> Mapping[str, int]:
        return MappingProxyType(dict(self._seeds))

    def stream(self, name: str) -> random.Random:
        if name not in self._seeds:
            raise RandomStreamError(f"random stream {name!r} was not declared")
        if name not in self._streams:
            self._streams[name] = random.Random(self._seeds[name])
        return self._streams[name]

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "master_seed": self.master_seed,
            "streams": dict(self._seeds),
        }
