# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Canonical value and declared dependency-closure hashing."""

from __future__ import annotations

import hashlib
import json
import math
import platform
from functools import lru_cache
from dataclasses import dataclass, fields, is_dataclass
from enum import Enum
from pathlib import Path
from types import MappingProxyType
from typing import Any, Iterable, Mapping


class HashingError(ValueError):
    """Raised when an identity input cannot be hashed without ambiguity."""


def canonical_value(value: Any) -> Any:
    """Return a JSON-safe value with stable ordering and no non-finite numbers."""
    if is_dataclass(value) and not isinstance(value, type):
        # ``dataclasses.asdict`` deep-copies leaves and cannot copy MappingProxyType,
        # which is intentionally used throughout the immutable runtime contracts.
        return canonical_value({field.name: getattr(value, field.name) for field in fields(value)})
    if isinstance(value, Enum):
        return canonical_value(value.value)
    if isinstance(value, Mapping):
        if any(not isinstance(key, str) for key in value):
            raise HashingError("canonical mappings must use string keys")
        return {key: canonical_value(value[key]) for key in sorted(value)}
    if isinstance(value, (tuple, list)):
        return [canonical_value(item) for item in value]
    if isinstance(value, (set, frozenset)):
        normalized = [canonical_value(item) for item in value]
        return sorted(normalized, key=canonical_json)
    if isinstance(value, float):
        if not math.isfinite(value):
            raise HashingError("canonical values may not contain NaN or infinity")
        return value
    if value is None or isinstance(value, (str, int, bool)):
        return value
    raise HashingError(f"unsupported canonical value type {type(value).__name__}")


def canonical_json(value: Any) -> str:
    return json.dumps(
        canonical_value(value),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )


def sha256_value(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class CodeIdentity:
    """Auditable hash of source files plus resolved external dependency versions."""

    sha256: str
    file_sha256: Mapping[str, str]
    external_versions: Mapping[str, str]

    def __post_init__(self) -> None:
        object.__setattr__(self, "file_sha256", MappingProxyType(dict(self.file_sha256)))
        object.__setattr__(self, "external_versions", MappingProxyType(dict(self.external_versions)))

    def to_dict(self) -> dict[str, object]:
        return {
            "sha256": self.sha256,
            "files": dict(self.file_sha256),
            "external_versions": dict(self.external_versions),
        }


def hash_dependency_closure(
    project_root: str | Path,
    *,
    component_sources: Iterable[str | Path],
    dependency_sources: Iterable[str | Path] = (),
    external_versions: Mapping[str, str] | None = None,
) -> CodeIdentity:
    """Hash an explicitly declared, project-contained source dependency closure.

    The component sources and every shared intra-project dependency are separate inputs so
    callers cannot mistake a single-file hash for the Section 14.1 code identity. Paths are
    stored relative to ``project_root`` and sorted before hashing.
    """
    root = Path(project_root).resolve()
    component_paths = tuple(component_sources)
    if not component_paths:
        raise HashingError("a code identity requires at least one component source")
    declared = component_paths + tuple(dependency_sources)
    records: dict[str, str] = {}
    for declared_path in declared:
        candidate = Path(declared_path)
        path = (root / candidate).resolve() if not candidate.is_absolute() else candidate.resolve()
        try:
            relative = path.relative_to(root).as_posix()
        except ValueError:
            raise HashingError(f"declared source {path} escapes project root {root}") from None
        if relative in records:
            raise HashingError(f"declared source {relative!r} appears more than once")
        if not path.is_file():
            raise HashingError(f"declared source {relative!r} is not a file")
        records[relative] = hashlib.sha256(path.read_bytes()).hexdigest()

    versions = dict(external_versions or {})
    if any(
        not isinstance(name, str)
        or not name.strip()
        or not isinstance(version, str)
        or not version.strip()
        for name, version in versions.items()
    ):
        raise HashingError("external dependency names and resolved versions must be non-empty strings")
    payload = {
        "schema_version": "dependency_closure_v1",
        "files": records,
        "external_versions": versions,
    }
    return CodeIdentity(
        sha256=sha256_value(payload),
        file_sha256={name: records[name] for name in sorted(records)},
        external_versions={name: versions[name] for name in sorted(versions)},
    )


EXCLUDED_CLOSURE_DIRECTORIES = frozenset({"tests", "__pycache__"})
"""Directories whose contents are not part of any component's behavioral closure.

Test sources are excluded deliberately. Section 15 places the Kit integration tests inside
this package, and including them would make editing a test change every component's code
hash -- so a test-only change would read as a behavioral change in the manifest, and the
paired contrasts of section 14.1 spanning that change would be refused or misattributed.
"""


def interpreter_version() -> str:
    """Resolved interpreter identity, recorded as an external dependency of the closure.

    The pure core depends on the standard library, whose numeric formatting, hashing and
    serialization behavior are version-dependent, so the interpreter is an input to what the
    code does. Inside Kit it is pinned transitively by the Isaac Sim build, but that field is
    a caller-supplied string that nothing validates, and a pure-core evidence run outside Kit
    can execute under any interpreter. Reading it from the running process cannot disagree
    with reality the way a declared value can.
    """
    return f"{platform.python_implementation()} {platform.python_version()}"


@lru_cache(maxsize=1)
def builtin_package_code_identity() -> CodeIdentity:
    """Return the conservative declared closure used by built-in mission components.

    Until per-model closures are generated during packaging, every built-in declares the
    whole backend-neutral package. This is intentionally conservative: an unrelated core
    change may invalidate more identities than necessary, but a shared behavioral change
    can never leave a stale component hash behind.

    The result is cached for the process. A fresh evidence run therefore always resolves it
    from the current sources; a long-lived session that hot-reloads edited sources must call
    ``builtin_package_code_identity.cache_clear()`` or it will keep reporting the identity of
    the code it started with.
    """
    package_root = Path(__file__).resolve().parents[1]
    sources = tuple(
        sorted(
            relative
            for relative in (
                path.relative_to(package_root) for path in package_root.rglob("*.py")
            )
            if not (set(relative.parts[:-1]) & EXCLUDED_CLOSURE_DIRECTORIES)
        )
    )
    if not sources:
        raise HashingError("the built-in closure resolved no package sources")
    try:
        import yaml

        yaml_version = str(yaml.__version__)
    except (ImportError, AttributeError) as exc:
        raise HashingError("the built-in closure cannot resolve its PyYAML version") from exc
    return hash_dependency_closure(
        package_root,
        # The whole package is one declared closure; the split is required by the two-argument
        # signature, which exists so a caller cannot pass a single file and call it an identity.
        component_sources=(sources[0],),
        dependency_sources=sources[1:],
        external_versions={"PyYAML": yaml_version, "python": interpreter_version()},
    )
