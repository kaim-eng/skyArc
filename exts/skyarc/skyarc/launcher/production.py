# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Pure production-scene planning for the selected schema-v3 treatment.

This module deliberately contains no Isaac Sim imports.  It turns the resolved launcher
geometry and the separately qualified rocket/cradle fixture into immutable scene data so
the USD authoring layer, evidence harness, and unit tests consume one definition.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence, Tuple

from ..configuration.schema import ScenarioConfig
from ..linalg import ZERO3, Quat, Vec3, add, norm, scale, sub
from ..names import BODY_CART, BODY_ROCKET
from .geometry import TubePath, path_pose
from .path_controller import TranslatedFrameState


@dataclass(frozen=True)
class OpenCradleGeometry:
    mass_kg: float
    outer_length_m: float
    outer_width_m: float
    outer_height_m: float
    wall_thickness_m: float
    floor_thickness_m: float


@dataclass(frozen=True)
class RocketGeometry:
    mass_kg: float
    length_m: float
    diameter_m: float


@dataclass(frozen=True)
class ProductionFixture:
    source_path: str
    pair_name: str
    initial_clearance_m: float
    rocket: RocketGeometry
    cradle: OpenCradleGeometry


@dataclass(frozen=True)
class CurveBand:
    name: str
    start_s_m: float
    end_s_m: float
    points_m: Tuple[Vec3, ...]
    color_rgb: Vec3
    opacity: float


@dataclass(frozen=True)
class ProductionScenePlan:
    candidate: str
    coordinate_frame: str
    backend: str
    device: str
    tube_inner_diameter_m: float
    tube_bands: Tuple[CurveBand, ...]
    exit_track_points_m: Tuple[Vec3, ...]
    exit_marker_position_m: Vec3
    cradle: OpenCradleGeometry
    rocket: RocketGeometry
    initial_clearance_m: float
    cart_to_rocket_offset_cart_m: Vec3
    reaction_evidence: str

    @property
    def cart_to_rocket_offset_m(self) -> float:
        """Three-dimensional body-centre separation used by mass properties."""
        return norm(self.cart_to_rocket_offset_cart_m)


def _unique_object(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    return value


def _exact_keys(value: Mapping[str, Any], expected: set[str], name: str) -> None:
    actual = set(value)
    if actual != expected:
        raise ValueError(
            f"{name} keys must be exactly {sorted(expected)}, got {sorted(actual)}"
        )


def _positive_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def load_production_fixture(path: str | Path) -> ProductionFixture:
    """Load and strictly validate the qualified cylinder/open-front-cradle fixture."""
    source = Path(path).resolve()
    data = json.loads(source.read_text(encoding="utf-8"), object_pairs_hook=_unique_object)
    root = _mapping(data, "fixture")
    _exact_keys(
        root,
        {"schema", "pair_name", "impact_case", "initial_clearance_m", "rocket", "cradle"},
        "fixture",
    )
    if root["schema"] != "vacuum_tube_anti_tunneling_fixture_v1":
        raise ValueError("unsupported production fixture schema")
    if root["pair_name"] != "rocket_cradle":
        raise ValueError("production fixture pair_name must be 'rocket_cradle'")
    if root["impact_case"] != "axial_rear_wall":
        raise ValueError("production fixture impact_case must be 'axial_rear_wall'")

    rocket_data = _mapping(root["rocket"], "fixture.rocket")
    _exact_keys(
        rocket_data,
        {"shape", "axis", "mass_kg", "length_m", "diameter_m"},
        "fixture.rocket",
    )
    if rocket_data["shape"] != "cylinder" or rocket_data["axis"] != "X":
        raise ValueError("production rocket must be an X-axis cylinder")
    rocket = RocketGeometry(
        mass_kg=_positive_number(rocket_data["mass_kg"], "fixture.rocket.mass_kg"),
        length_m=_positive_number(rocket_data["length_m"], "fixture.rocket.length_m"),
        diameter_m=_positive_number(rocket_data["diameter_m"], "fixture.rocket.diameter_m"),
    )

    cradle_data = _mapping(root["cradle"], "fixture.cradle")
    _exact_keys(
        cradle_data,
        {
            "topology",
            "mass_kg",
            "outer_length_m",
            "outer_width_m",
            "outer_height_m",
            "wall_thickness_m",
            "floor_thickness_m",
        },
        "fixture.cradle",
    )
    if cradle_data["topology"] != "open_front_u":
        raise ValueError("production cradle must use open_front_u topology")
    cradle = OpenCradleGeometry(
        mass_kg=_positive_number(cradle_data["mass_kg"], "fixture.cradle.mass_kg"),
        outer_length_m=_positive_number(
            cradle_data["outer_length_m"], "fixture.cradle.outer_length_m"
        ),
        outer_width_m=_positive_number(
            cradle_data["outer_width_m"], "fixture.cradle.outer_width_m"
        ),
        outer_height_m=_positive_number(
            cradle_data["outer_height_m"], "fixture.cradle.outer_height_m"
        ),
        wall_thickness_m=_positive_number(
            cradle_data["wall_thickness_m"], "fixture.cradle.wall_thickness_m"
        ),
        floor_thickness_m=_positive_number(
            cradle_data["floor_thickness_m"], "fixture.cradle.floor_thickness_m"
        ),
    )
    if 2.0 * cradle.wall_thickness_m >= cradle.outer_width_m:
        raise ValueError("production cradle side walls leave no open interior")
    if cradle.floor_thickness_m >= cradle.outer_height_m:
        raise ValueError("production cradle floor leaves no open interior")
    if rocket.diameter_m >= cradle.outer_width_m - 2.0 * cradle.wall_thickness_m:
        raise ValueError("production rocket does not fit between cradle side walls")
    floor_clearance_m = (
        0.5 * (cradle.outer_height_m - rocket.diameter_m)
        - cradle.floor_thickness_m
    )
    if floor_clearance_m <= 0.0:
        raise ValueError("production rocket intersects the cradle floor")

    return ProductionFixture(
        source_path=str(source),
        pair_name="rocket_cradle",
        initial_clearance_m=_positive_number(
            root["initial_clearance_m"], "fixture.initial_clearance_m"
        ),
        rocket=rocket,
        cradle=cradle,
    )


def validate_fixture_against_scenario(
    fixture: ProductionFixture,
    config: ScenarioConfig,
) -> None:
    """Fail if the independently qualified fixture differs from scenario mass/size fields."""
    comparisons = (
        ("rocket mass", fixture.rocket.mass_kg, config.rocket.initial_mass_kg),
        ("rocket length", fixture.rocket.length_m, config.rocket.length_m),
        ("rocket diameter", fixture.rocket.diameter_m, config.rocket.diameter_m),
        ("cradle mass", fixture.cradle.mass_kg, config.cart.mass_kg),
        ("cradle length", fixture.cradle.outer_length_m, config.cart.length_m),
        ("cradle width", fixture.cradle.outer_width_m, config.cart.width_m),
        ("cradle height", fixture.cradle.outer_height_m, config.cart.height_m),
    )
    for label, qualified, configured in comparisons:
        if not math.isclose(qualified, configured, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError(
                f"production fixture {label} {qualified} does not match scenario {configured}"
            )
    if not any(pair.name == fixture.pair_name for pair in config.tube.anti_tunneling_pairs):
        raise ValueError("scenario does not name the qualified rocket_cradle collision pair")


def _sample_interval(
    layout: TubePath,
    start_s_m: float,
    end_s_m: float,
    maximum_spacing_m: float,
) -> Tuple[Vec3, ...]:
    count = max(1, math.ceil((end_s_m - start_s_m) / maximum_spacing_m))
    return tuple(
        path_pose(layout, start_s_m + (end_s_m - start_s_m) * index / count).position_m
        for index in range(count + 1)
    )


def build_production_scene_plan(
    config: ScenarioConfig,
    layout: TubePath,
    fixture: ProductionFixture,
    *,
    maximum_curve_spacing_m: float = 100.0,
) -> ProductionScenePlan:
    """Build the single source of truth consumed by USD scene authoring."""
    if config.schema_version != 3:
        raise ValueError("the approved production treatment requires schema version 3")
    if config.models.guide != "tangent_following_v1":
        raise ValueError("the approved production treatment requires tangent_following_v1")
    if not math.isfinite(maximum_curve_spacing_m) or maximum_curve_spacing_m <= 0.0:
        raise ValueError("maximum curve spacing must be finite and positive")
    validate_fixture_against_scenario(fixture, config)

    bands = []
    start_s_m = 0.0
    for index, stage in enumerate(config.tube.stages):
        end_s_m = start_s_m + stage.length_m
        if index == len(config.tube.stages) - 1:
            if not math.isclose(end_s_m, layout.length_m, rel_tol=0.0, abs_tol=1e-3):
                raise ValueError("resolved stage lengths do not cover the production centerline")
            # The authored decimal stage total and the analytic arc length differ by
            # 0.118 mm.  The final visualization band owns that rounding residue so the
            # exit marker and tube endpoint remain exactly the same resolved pose.
            end_s_m = layout.length_m
        bands.append(
            CurveBand(
                name=stage.name,
                start_s_m=start_s_m,
                end_s_m=end_s_m,
                points_m=_sample_interval(
                    layout, start_s_m, end_s_m, maximum_curve_spacing_m
                ),
                color_rgb=stage.color_rgb,
                opacity=stage.opacity,
            )
        )
        start_s_m = end_s_m
    if start_s_m != layout.length_m:
        raise ValueError("resolved stage lengths do not cover the production centerline")

    exit_pose = path_pose(layout, layout.length_m)
    exit_length = config.tube.exit_brake_track_length_m
    exit_end = add(exit_pose.position_m, scale(exit_pose.tangent, exit_length))
    # Nest the rocket in the independently qualified open-cradle geometry. Its aft cap
    # clears the rear wall by the declared gap, while its axis stays on the cart/tube
    # centreline. The full-length U-cradle leaves positive floor clearance for passive
    # separation and contains both rocket caps at release; a shallow or short tray makes
    # the vehicle look top-mounted and lets gravity drive the rocket onto its front lip,
    # violating the ignition angular-rate interlock.
    rear_wall_forward_face_x_m = (
        -0.5 * fixture.cradle.outer_length_m + fixture.cradle.wall_thickness_m
    )
    rocket_axial_offset_m = (
        rear_wall_forward_face_x_m
        + 0.5 * fixture.rocket.length_m
        + fixture.initial_clearance_m
    )
    rocket_normal_offset_m = 0.0
    return ProductionScenePlan(
        candidate="force_resolved_path_controller_v1",
        coordinate_frame="translated_accelerating_v1",
        backend="physx",
        device="cpu",
        tube_inner_diameter_m=config.tube.inner_diameter_m,
        tube_bands=tuple(bands),
        exit_track_points_m=(exit_pose.position_m, exit_end),
        exit_marker_position_m=exit_pose.position_m,
        cradle=fixture.cradle,
        rocket=fixture.rocket,
        initial_clearance_m=fixture.initial_clearance_m,
        cart_to_rocket_offset_cart_m=(
            rocket_axial_offset_m,
            0.0,
            rocket_normal_offset_m,
        ),
        reaction_evidence=(
            "commanded_and_backend_reconstructed_not_solver_constraint_reaction"
        ),
    )


@dataclass(frozen=True)
class InitialRigidState:
    """Authored solver-frame state for one body, reproduced exactly by every reset."""

    position_m: Vec3
    orientation_wxyz: Quat
    linear_velocity_mps: Vec3
    angular_velocity_radps: Vec3


def resolve_initial_solver_states(
    layout: TubePath,
    plan: ProductionScenePlan,
    fixture: ProductionFixture,
    reference: TranslatedFrameState,
) -> Mapping[str, InitialRigidState]:
    """Place the attached assembly at rest with its centre of mass at the tube entrance.

    This is the initial condition used by the Phase 0 curved-guide qualification. Launch
    control uses assembly centre-of-mass progress, so the cart may extend a mass-weighted
    distance into the straight entrance lead-in without disabling the launcher.

    The result is expressed in solver coordinates, which is what the scene is authored in
    and what a reset re-authors; every public quantity elsewhere stays global SI.
    """
    start = path_pose(layout, 0.0)
    if abs(start.signed_curvature_per_m) > 1e-12:
        raise ValueError(
            "the production start placement assumes a straight tube entrance; a curved "
            "entrance would leave the assembly centre of mass off the centerline by the "
            "chord sagitta of the fixed-joint spacing"
        )
    half_angle_rad = 0.5 * math.radians(start.inclination_deg)
    orientation: Quat = (math.cos(half_angle_rad), 0.0, -math.sin(half_angle_rad), 0.0)
    assembly_mass_kg = fixture.cradle.mass_kg + fixture.rocket.mass_kg
    axial_offset_m, binormal_offset_m, normal_offset_m = plan.cart_to_rocket_offset_cart_m
    world_offset_m = add(
        add(scale(start.tangent, axial_offset_m), (0.0, binormal_offset_m, 0.0)),
        scale(start.normal, normal_offset_m),
    )
    rocket_mass_fraction = fixture.rocket.mass_kg / assembly_mass_kg
    global_cart_m = sub(start.position_m, scale(world_offset_m, rocket_mass_fraction))
    global_rocket_m = add(global_cart_m, world_offset_m)
    solver_velocity = sub(ZERO3, reference.velocity_mps)
    return {
        BODY_CART: InitialRigidState(
            position_m=sub(global_cart_m, reference.position_m),
            orientation_wxyz=orientation,
            linear_velocity_mps=solver_velocity,
            angular_velocity_radps=ZERO3,
        ),
        BODY_ROCKET: InitialRigidState(
            position_m=sub(global_rocket_m, reference.position_m),
            orientation_wxyz=orientation,
            linear_velocity_mps=solver_velocity,
            angular_velocity_radps=ZERO3,
        ),
    }


def combined_pitch_inertia_kg_m2(
    cart_inertia: Sequence[float],
    rocket_inertia: Sequence[float],
    *,
    cart_mass_kg: float,
    rocket_mass_kg: float,
    offset_m: float,
) -> float:
    """Pitch inertia of the attached assembly about its own centre of mass.

    Each body contributes its own ``I_yy`` plus the parallel-axis term of the reduced mass
    over the fixed-joint separation.  The attitude command is scaled by this, so reading it
    from the solver's own tensors rather than restating a literal keeps a mass-property
    change in the fixture from silently detuning the controller.
    """
    if len(cart_inertia) != 9 or len(rocket_inertia) != 9:
        raise RuntimeError("expected one 3x3 inertia tensor for each production body")
    total_mass_kg = cart_mass_kg + rocket_mass_kg
    if total_mass_kg <= 0.0:
        raise ValueError("assembly mass must be positive")
    return (
        float(cart_inertia[4])
        + float(rocket_inertia[4])
        + cart_mass_kg * rocket_mass_kg / total_mass_kg * offset_m**2
    )


__all__ = [
    "CurveBand",
    "InitialRigidState",
    "OpenCradleGeometry",
    "ProductionFixture",
    "ProductionScenePlan",
    "RocketGeometry",
    "build_production_scene_plan",
    "combined_pitch_inertia_kg_m2",
    "load_production_fixture",
    "resolve_initial_solver_states",
    "validate_fixture_against_scenario",
]
