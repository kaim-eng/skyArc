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

"""Minimal pure-Python vector and quaternion helpers.

The core of this project must import without ``numpy`` (section 12 of the design review
notes that the numerical array package is supplied by a bundled dependency rather than by
the interpreter, and is therefore unavailable to the pure unit suite). The quantities
involved are three-vectors and single quaternions, so a small module of tuple operations is
sufficient and keeps the core dependency-free.

Quaternions are ``(w, x, y, z)``. That is the convention of the rigid-body prim boundary
described in section 14; the underlying physics tensor view returns the scalar component
last and the adapter is the only place allowed to reorder it.
"""

from __future__ import annotations

import math
from typing import Iterable, Sequence, Tuple

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]

ZERO3: Vec3 = (0.0, 0.0, 0.0)
IDENTITY_QUAT: Quat = (1.0, 0.0, 0.0, 0.0)


def vec3(values: Iterable[float]) -> Vec3:
    """Coerce an iterable of three numbers into a ``Vec3``.

    Raises:
        ValueError: If the iterable does not hold exactly three values.
    """
    items = tuple(float(v) for v in values)
    if len(items) != 3:
        raise ValueError(f"expected 3 components, got {len(items)}")
    return items  # type: ignore[return-value]


def quat(values: Iterable[float]) -> Quat:
    """Coerce an iterable of four numbers into a ``(w, x, y, z)`` quaternion."""
    items = tuple(float(v) for v in values)
    if len(items) != 4:
        raise ValueError(f"expected 4 components, got {len(items)}")
    return items  # type: ignore[return-value]


def add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Vec3, k: float) -> Vec3:
    return (a[0] * k, a[1] * k, a[2] * k)


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a: Vec3) -> float:
    return math.sqrt(dot(a, a))


def normalize(a: Vec3) -> Vec3:
    """Return the unit vector along ``a``.

    Raises:
        ValueError: If ``a`` has (near) zero length and no direction can be defined.
    """
    length = norm(a)
    if length <= 1e-12:
        raise ValueError("cannot normalize a zero-length vector")
    return scale(a, 1.0 / length)


def sum_vectors(vectors: Iterable[Vec3]) -> Vec3:
    total: Vec3 = ZERO3
    for v in vectors:
        total = add(total, v)
    return total


def is_finite(values: Sequence[float]) -> bool:
    """Whether every component is a finite float (no NaN, no infinity)."""
    return all(math.isfinite(float(v)) for v in values)


def clamp(value: float, low: float, high: float) -> float:
    """Clamp ``value`` into ``[low, high]``.

    Raises:
        ValueError: If the interval is empty.
    """
    if low > high:
        raise ValueError(f"empty clamp interval [{low}, {high}]")
    return low if value < low else (high if value > high else value)


def quat_conjugate(q: Quat) -> Quat:
    return (q[0], -q[1], -q[2], -q[3])


def quat_multiply(a: Quat, b: Quat) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_rotate(q: Quat, v: Vec3) -> Vec3:
    """Rotate ``v`` by the ``(w, x, y, z)`` quaternion ``q``."""
    w, x, y, z = q
    u: Vec3 = (x, y, z)
    # v' = v + 2w (u x v) + 2 (u x (u x v))
    uv = cross(u, v)
    uuv = cross(u, uv)
    return add(v, add(scale(uv, 2.0 * w), scale(uuv, 2.0)))


def quat_rotate_inverse(q: Quat, v: Vec3) -> Vec3:
    """Rotate ``v`` by the inverse of ``q``."""
    return quat_rotate(quat_conjugate(q), v)


def quat_from_axis_angle(axis: Vec3, angle_rad: float) -> Quat:
    """Build a ``(w, x, y, z)`` quaternion from an axis and an angle in radians."""
    unit = normalize(axis)
    half = 0.5 * angle_rad
    s = math.sin(half)
    return (math.cos(half), unit[0] * s, unit[1] * s, unit[2] * s)


def quat_wxyz_from_xyzw(q: Sequence[float]) -> Quat:
    """Reorder a scalar-last quaternion into the ``(w, x, y, z)`` convention.

    Only the backend adapter may call this. Section 5.1 confines the frame and ordering
    inversions between runtime layers to the adapter precisely so that a component cannot
    introduce a sign or ordering error that no ownership check would catch.
    """
    x, y, z, w = (float(v) for v in q)
    return (w, x, y, z)


def quat_xyzw_from_wxyz(q: Quat) -> Tuple[float, float, float, float]:
    """Reorder a ``(w, x, y, z)`` quaternion into the scalar-last convention."""
    w, x, y, z = q
    return (x, y, z, w)
