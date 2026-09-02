# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Qualified CPU PhysX adapter with translated accelerating-frame reconstruction.

Import this module only after ``SimulationApp`` exists.  It is the only production module
allowed to translate accepted body loads into ``RigidPrim`` mutations.

Two things happen here that no component may do.  The first is the coordinate translation:
solver state is reconstructed into global SI and every body carries the exact uniform
fictitious force of the accelerating reference frame.  The second is the guide reaction.
PhysX exposes no path or spline joint, so the reaction a constraint would supply has to be
commanded; DESIGN_REVIEW v0.29 accepted that non-constraint treatment for system-level
simulation.  The reaction is computed by the backend-neutral
:mod:`...launcher.path_controller` and reported through
:class:`~..adapter.AppliedEffects` under ``SLOT_BACKEND_ADAPTER``, so the difference between
the accepted and applied load is always itemized by slot rather than being an unattributed
excess.
"""

from __future__ import annotations

from types import MappingProxyType
from typing import Any, Callable, Mapping

from isaacsim.core.simulation_manager import SimulationManager

from ...launcher.path_controller import (
    ForceResolvedPathReaction,
    GuideReaction,
    TranslatedFrameState,
)
from ...linalg import ZERO3, Vec3, add, norm, scale, sub
from ...names import SLOT_BACKEND_ADAPTER
from ...state import BodyState, ContactReport, SimulationState
from ..adapter import AppliedEffects, BackendCapabilities
from ..aggregator import AggregatedEffects, BodyLoad
from ..types import CollisionAction, ConstraintAction, MomentumPolicy


def flatten_numbers(value: Any) -> list[float]:
    """Flatten a backend tensor value into plain Python floats.

    The tensor API returns Warp arrays, which expose ``shape`` but raise on item indexing,
    so a shape-driven walk breaks on exactly the values this boundary receives. Converting
    through ``numpy()`` before ``tolist()`` is the one reading that works for Warp arrays,
    NumPy arrays and plain sequences alike, and it is the same conversion the Phase 0
    evidence runners use.
    """
    if hasattr(value, "numpy"):
        value = value.numpy()
    if hasattr(value, "tolist"):
        value = value.tolist()
    if value is None:
        return []
    if isinstance(value, (int, float)):
        return [float(value)]
    flattened: list[float] = []
    for item in value:
        flattened.extend(flatten_numbers(item))
    return flattened


def _fixed_width(value: Any, width: int, label: str) -> tuple[float, ...]:
    values = flatten_numbers(value)
    if len(values) != width:
        raise RuntimeError(f"expected {label}, got {values!r}")
    return tuple(values)


def _vector(value: Any) -> Vec3:
    return _fixed_width(value, 3, "three-vector")  # type: ignore[return-value]


def _quaternion(value: Any) -> tuple[float, float, float, float]:
    return _fixed_width(value, 4, "quaternion")  # type: ignore[return-value]


class IsaacPhysxBackend:
    """Production :class:`BackendAdapter` for the selected PhysX/CPU condition."""

    def __init__(
        self,
        *,
        bodies: Mapping[str, Any],
        masses_kg: Mapping[str, float],
        constraints: Mapping[str, Any],
        collision_pair_active: Mapping[str, bool],
        dt_s: float,
        reference_frame: Callable[[float], TranslatedFrameState],
        guide_reaction: ForceResolvedPathReaction | None = None,
        contact_readers: Mapping[str, Callable[[float], ContactReport]] | None = None,
        resync_callback: Callable[[], None] | None = None,
        reset_callback: Callable[[], None] | None = None,
    ) -> None:
        if set(bodies) != set(masses_kg):
            raise ValueError("Isaac body and mass mappings must have identical keys")
        if not bodies:
            raise ValueError("Isaac adapter requires at least one body")
        if dt_s <= 0.0:
            raise ValueError("Isaac adapter timestep must be positive")
        if SimulationManager.get_active_physics_engine() != "physx":
            raise RuntimeError("production adapter requires the Phase 0 selected PhysX backend")
        if "cpu" not in str(SimulationManager.get_device()).lower():
            raise RuntimeError("production adapter requires the Phase 0 selected CPU device")
        self._bodies = dict(bodies)
        self._masses = {name: float(value) for name, value in masses_kg.items()}
        self._initial_masses = dict(self._masses)
        self._constraints = dict(constraints)
        self._initial_constraint_active = {
            name: bool(joint.GetJointEnabledAttr().Get())
            for name, joint in self._constraints.items()
        }
        self._constraint_active = dict(self._initial_constraint_active)
        self._initial_collision_pair_active = dict(collision_pair_active)
        self._collision_pair_active = dict(self._initial_collision_pair_active)
        self._dt_s = float(dt_s)
        self._reference_frame = reference_frame
        self._guide_reaction = guide_reaction
        self._contact_readers = dict(contact_readers or {})
        self._resync_callback = resync_callback
        self._reset_callback = reset_callback
        self._time_s = 0.0
        self._step_index = 0
        self._pending = False
        self._peak_solver_offset_m = 0.0
        self._last_reaction: GuideReaction | None = None
        self._capabilities = BackendCapabilities(
            backend="physx",
            device="cpu",
            features={
                "fixed_time_step": True,
                "resync": True,
                "always_present_collision_pair": True,
                "contact_reporting": True,
                "translated_accelerating_frame": True,
                "solver_constraint_reaction": False,
                "system_load_reconstruction": True,
                # The path is held by a commanded reaction, not by a joint.  A component
                # that needs to know whether the backend already enforces normal motion
                # must be able to ask rather than infer it from the backend name.
                "commanded_path_reaction": guide_reaction is not None,
            },
        )

    @property
    def capabilities(self) -> BackendCapabilities:
        return self._capabilities

    @property
    def peak_solver_offset_m(self) -> float:
        """Largest solver-coordinate magnitude seen, over every body and every read.

        The translated frame exists to keep this small; after release the cart decelerates
        away from an inertially coasting frame, so this is the measured form of the
        disclosed post-release precision limitation rather than an assumption about it.
        """
        return self._peak_solver_offset_m

    @property
    def last_guide_reaction(self) -> GuideReaction | None:
        """Most recent commanded guide reaction, retained for telemetry and diagnostics."""
        return self._last_reaction

    def _read_bodies(
        self, frame: TranslatedFrameState
    ) -> tuple[dict[str, BodyState], dict[str, Vec3]]:
        """Reconstruct global SI body state, keeping the raw solver positions alongside.

        The solver positions are returned rather than recovered by subtracting the frame
        again: they are the application points ``apply`` needs, and re-reading the poses for
        them would double the tensor traffic of the busiest call in the step loop.
        """
        bodies: dict[str, BodyState] = {}
        solver_positions: dict[str, Vec3] = {}
        for name, rigid in self._bodies.items():
            raw_position, orientation = rigid.get_world_poses()
            solver_linear, angular = rigid.get_velocities()
            solver_position = _vector(raw_position)
            solver_positions[name] = solver_position
            self._peak_solver_offset_m = max(self._peak_solver_offset_m, norm(solver_position))
            bodies[name] = BodyState(
                name=name,
                position=add(solver_position, frame.position_m),
                orientation=_quaternion(orientation),
                linear_velocity=add(_vector(solver_linear), frame.velocity_mps),
                angular_velocity=_vector(angular),
                mass_kg=self._masses[name],
            )
        return bodies, solver_positions

    def _global_state(
        self,
        frame: TranslatedFrameState,
        *,
        contacts: Mapping[str, ContactReport],
        bodies: Mapping[str, BodyState] | None = None,
    ) -> SimulationState:
        if bodies is None:
            bodies = self._read_bodies(frame)[0]
        return SimulationState(
            time_s=self._time_s,
            step_index=self._step_index,
            dt_s=self._dt_s,
            bodies=MappingProxyType(dict(bodies)),
            contacts=MappingProxyType(dict(contacts)),
            joint_active=MappingProxyType(dict(self._constraint_active)),
            collision_pair_active=MappingProxyType(dict(self._collision_pair_active)),
        )

    def read_state(self) -> SimulationState:
        frame = self._reference_frame(self._time_s)
        contacts = {name: reader(self._dt_s) for name, reader in self._contact_readers.items()}
        return self._global_state(frame, contacts=contacts)

    def apply(self, effects: AggregatedEffects) -> AppliedEffects:
        if self._pending:
            raise RuntimeError("Isaac effects have already been applied for this step")
        frame = self._reference_frame(self._time_s)
        bodies, solver_positions = self._read_bodies(frame)
        reaction = None
        if self._guide_reaction is not None:
            # Contacts are deliberately excluded from this state: the reaction reads none of
            # them, and the fixed-size contact buffers are already read once per step by the
            # orchestrator's own state read.
            reaction = self._guide_reaction.evaluate(
                self._global_state(frame, contacts={}, bodies=bodies),
                {name: effects.load(name).force_n for name in self._bodies},
            )
        self._last_reaction = reaction

        applied_loads: dict[str, BodyLoad] = {}
        for name, rigid in self._bodies.items():
            accepted = effects.load(name)
            extra_force = ZERO3
            extra_torque = ZERO3
            if reaction is not None and reaction.body == name:
                extra_force = reaction.force_n
                extra_torque = reaction.torque_nm
            force_n = add(accepted.force_n, extra_force)
            torque_nm = add(accepted.torque_nm, extra_torque)
            by_slot = dict(accepted.force_by_slot)
            torque_by_slot = dict(accepted.torque_by_slot)
            if extra_force != ZERO3:
                by_slot[SLOT_BACKEND_ADAPTER] = add(
                    by_slot.get(SLOT_BACKEND_ADAPTER, ZERO3), extra_force
                )
            if extra_torque != ZERO3:
                torque_by_slot[SLOT_BACKEND_ADAPTER] = add(
                    torque_by_slot.get(SLOT_BACKEND_ADAPTER, ZERO3), extra_torque
                )
            if force_n != ZERO3 or torque_nm != ZERO3 or by_slot or torque_by_slot:
                applied_loads[name] = BodyLoad(
                    body=name,
                    force_n=force_n,
                    torque_nm=torque_nm,
                    force_by_slot=by_slot,
                    torque_by_slot=torque_by_slot,
                )
            # The fictitious force is a property of the frame, not of any slot: adding
            # -m*a_r here is exactly what makes the solver reproduce the applied global
            # load, so it is not reported as an applied effect.
            solver_force = sub(force_n, scale(frame.acceleration_mps2, self._masses[name]))
            # The application point is the body origin, matching the Phase 0 qualified
            # runner exactly.  ``read_state`` leaves ``com_offset_m`` zero for the same
            # reason, so the core resolves torques about the same point the adapter applies
            # force at.  Authoring an explicit body centre of mass and inertia so that the
            # origin is provably the centre of mass is deferred: it would perturb the
            # qualified fixture's mass properties and therefore its bound evidence.
            rigid.apply_forces_and_torques_at_pos(
                solver_force,
                torque_nm,
                positions=solver_positions[name],
                local_frame=False,
            )

        for update in effects.mass_updates:
            if update.momentum_policy is not MomentumPolicy.CONSERVE:
                raise ValueError("production adapter does not yet support accounted mass flow")
            if update.effective_time_s > self._time_s + 1e-12:
                raise ValueError("production adapter does not queue future mass updates")
            body = bodies[update.body]
            velocity_scale = body.mass_kg / update.mass_kg
            # The policy is expressed in global SI. Scaling raw solver velocity would not
            # conserve momentum in a translated frame because frame velocity is omitted.
            solver_linear_velocity = sub(
                scale(body.linear_velocity, velocity_scale), frame.velocity_mps
            )
            self._bodies[update.body].set_velocities(
                solver_linear_velocity, body.angular_velocity
            )
            self._bodies[update.body].set_masses([update.mass_kg])
            self._masses[update.body] = update.mass_kg
        for command in effects.constraint_commands:
            try:
                joint = self._constraints[command.constraint]
            except KeyError:
                raise KeyError(f"unknown Isaac constraint {command.constraint!r}") from None
            enabled = command.action is ConstraintAction.ENABLE
            joint.GetJointEnabledAttr().Set(enabled)
            self._constraint_active[command.constraint] = enabled
        for command in effects.collision_commands:
            requested = command.action is CollisionAction.ENABLE
            current = self._collision_pair_active.get(command.pair)
            if current is None:
                raise KeyError(f"unknown Isaac collision pair {command.pair!r}")
            if requested != current:
                raise ValueError(
                    "production PhysX uses the qualified always-present collision pair; "
                    "live collision-filter mutation is not permitted"
                )
        self._pending = True
        return AppliedEffects(
            loads=applied_loads,
            mass_updates=effects.mass_updates,
            constraint_commands=effects.constraint_commands,
            collision_commands=effects.collision_commands,
        )

    def step(self) -> None:
        SimulationManager.step()
        self._time_s += self._dt_s
        self._step_index += 1
        self._pending = False

    def resync(self) -> None:
        if self._resync_callback is None:
            raise RuntimeError("production adapter has no configured physics resync callback")
        self._resync_callback()

    def reset(self) -> None:
        if self._reset_callback is None:
            raise RuntimeError("production adapter has no configured stop/rebuild reset callback")
        self._reset_callback()
        self._masses = dict(self._initial_masses)
        # Restoring the mirror is not enough: an applied mass update wrote through to the
        # prim, and the reset callback re-authors pose and velocity but not mass. Writing
        # the initial masses back here keeps ``read_state`` truthful for any reset
        # callback, rather than depending on one that happens to cover mass. The baseline
        # motor is constant-mass, so today this restores values that never changed; the
        # adapter accepts mass updates and ``SLOT_MASS_OWNERSHIP`` exists to anticipate a
        # propellant-depletion model, and a reset that silently missed them would be found
        # only by the resulting energy residual.
        for name, mass_kg in self._masses.items():
            self._bodies[name].set_masses([mass_kg])
        # The rebuild re-authors the joint and the always-present pair, so the adapter's
        # mirror of them has to return to the authored state too.  Leaving the released
        # joint recorded as disabled would make the next run's coupling reset fail its
        # own precondition instead of running.
        self._constraint_active = dict(self._initial_constraint_active)
        self._collision_pair_active = dict(self._initial_collision_pair_active)
        self._time_s = 0.0
        self._step_index = 0
        self._pending = False
        self._peak_solver_offset_m = 0.0
        self._last_reaction = None


__all__ = ["IsaacPhysxBackend", "TranslatedFrameState", "flatten_numbers"]
