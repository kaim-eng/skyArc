# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Wire the common mission orchestrator onto the built production scene.

Import this module only after ``SimulationApp`` exists.

Nothing here re-implements the mission.  The step ordering, release transaction, ignition
interlock, telemetry phases and reset semantics all live in
:mod:`..orchestrator`, which is backend-neutral and already exercised end to end against
the analytic adapter.  This module supplies the four things that orchestrator needs from a
real Isaac scene and cannot obtain for itself: stable rigid-body handles, the translated
accelerating reference frame, a physics resynchronization that does not advance time, and a
stop/rebuild reset that reproduces the authored initial state.

The resync and reset sequences are the ones the Phase 0 curved-guide evidence measured.
Resync brackets a Kit update with pause/play, which the accepted artifact recorded as a
0.0 mutation discontinuity -- the release transaction's 1e-9 continuity tolerance depends
on that being exact rather than merely small.  Reset stops the timeline, re-authors the USD
rigid state while physics is absent, re-enables the joint, and only then rebuilds physics
and re-creates every view; a paused tensor write is not sufficient because stopping can
sync the last simulated transforms back over it.
"""

from __future__ import annotations

import math
from typing import Any, Callable, Mapping, Sequence

import isaacsim.core.experimental.utils.app as app_utils
import omni.kit.app
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, UsdPhysics

from ..configuration.schema import ScenarioConfig
from ..effects.backends.isaac import IsaacPhysxBackend, flatten_numbers
from ..linalg import ZERO3, Vec3, add, scale
from ..names import (
    BODY_CART,
    BODY_ROCKET,
    JOINT_COUPLING,
    PAIR_ROCKET_CRADLE,
    SLOT_CART_BRAKE,
    SLOT_LAUNCH_FORCE,
    SLOT_ROCKET_AERODYNAMICS,
    SLOT_ROCKET_MOTOR,
)
from ..orchestrator import SimulationOrchestrator, build_mission
from ..state import ContactReport, SimulationState
from ..telemetry.recorder import TelemetrySink
from ..visualization.cameras import AuthoredCameraRig, camera_views
from ..visualization.overlays import ForceOverlay, MagneticFieldOverlay
from ..visualization.proxies import GlobalVisualProxies
from .geometry import TubePath, path_pose
from .path_controller import (
    ForceResolvedPathReaction,
    LaunchProfileReferenceFrame,
    PathControllerGains,
)
from .production import (
    InitialRigidState,
    ProductionFixture,
    ProductionScenePlan,
    combined_pitch_inertia_kg_m2,
    resolve_initial_solver_states,
)
from .scene import BuiltLauncherScene, build_launcher_scene


MAX_CONTACT_DATA_COUNT = 16
"""Fixed contact buffer size, matching the Phase 0 qualified rocket/cradle contact view."""


class _RigidBodyHandle:
    """Stable body handle over a ``RigidPrim`` view that a reset rebuilds.

    ``IsaacPhysxBackend`` copies the body mapping it is given, so a reset that creates new
    views would otherwise leave the adapter holding handles into a destroyed physics scene.
    Indirecting through this object keeps the adapter's mapping valid across a rebuild
    without the adapter reaching back into the runtime that owns the scene.
    """

    def __init__(self, prim_path: str) -> None:
        self._prim_path = prim_path
        self._view = RigidPrim(prim_path)

    @property
    def prim_path(self) -> str:
        return self._prim_path

    @property
    def view(self) -> RigidPrim:
        return self._view

    def rebind(self) -> None:
        self._view = RigidPrim(self._prim_path)

    def get_world_poses(self) -> Any:
        return self._view.get_world_poses()

    def get_velocities(self) -> Any:
        return self._view.get_velocities()

    def get_inertias(self) -> Any:
        return self._view.get_inertias()

    def set_masses(self, masses: Sequence[float]) -> Any:
        return self._view.set_masses(masses)

    def set_world_poses(self, **kwargs: Any) -> Any:
        return self._view.set_world_poses(**kwargs)

    def set_velocities(self, *args: Any, **kwargs: Any) -> Any:
        return self._view.set_velocities(*args, **kwargs)

    def apply_forces_and_torques_at_pos(self, *args: Any, **kwargs: Any) -> Any:
        return self._view.apply_forces_and_torques_at_pos(*args, **kwargs)


def _author_stopped_rigid_state(
    stage: Any,
    prim_path: str,
    state: InitialRigidState,
) -> dict[str, str]:
    """Author the USD rigid state that the next physics rebuild will read."""
    prim = stage.GetPrimAtPath(prim_path)
    translate_attr = prim.GetAttribute("xformOp:translate")
    orient_attr = prim.GetAttribute("xformOp:orient")
    translate_type = str(translate_attr.GetTypeName())
    orient_type = str(orient_attr.GetTypeName())
    translate_value = (
        Gf.Vec3f(*state.position_m)
        if translate_type == "float3"
        else Gf.Vec3d(*state.position_m)
    )
    w, x, y, z = state.orientation_wxyz
    orient_value = (
        Gf.Quatf(w, Gf.Vec3f(x, y, z))
        if orient_type == "quatf"
        else Gf.Quatd(w, Gf.Vec3d(x, y, z))
    )
    if not translate_attr.Set(translate_value) or not orient_attr.Set(orient_value):
        raise RuntimeError(f"failed to author stopped rigid transform for {prim_path}")
    rigid_api = UsdPhysics.RigidBodyAPI(prim)
    rigid_api.CreateVelocityAttr().Set(Gf.Vec3f(*state.linear_velocity_mps))
    rigid_api.CreateAngularVelocityAttr().Set(
        Gf.Vec3f(*(math.degrees(value) for value in state.angular_velocity_radps))
    )
    return {"translate_type": translate_type, "orient_type": orient_type}


class ProductionMissionRuntime:
    """Own the Isaac-side lifetime of one production mission."""

    def __init__(
        self,
        stage: Any,
        config: ScenarioConfig,
        layout: TubePath,
        fixture: ProductionFixture,
        plan: ProductionScenePlan,
        *,
        gains: PathControllerGains = PathControllerGains(),
        gravity_mps2: Vec3 = (0.0, 0.0, -9.81),
        telemetry_sink_factory: Callable[[SimulationState], TelemetrySink] | None = None,
        author_visuals: bool = True,
        update: Callable[[], None] | None = None,
    ) -> None:
        """Build the scene, the adapter and the mission exactly once.

        The telemetry sink arrives as a factory rather than as a finished object because
        :class:`~..telemetry.recorder.TelemetryRecorder` needs the backend's initial state,
        which does not exist until the scene has been authored and warmed.  Constructing the
        runtime twice to break that cycle would author the same USD prims twice and
        duplicate their transform ops.
        """
        if plan.candidate != "force_resolved_path_controller_v1":
            raise ValueError("the production runtime implements only the accepted candidate")
        self._stage = stage
        self._layout = layout
        self._gravity_mps2 = gravity_mps2
        self._update = update if update is not None else omni.kit.app.get_app().update
        self._dt_s = float(config.simulation.physics_dt_s)

        # The frame follows the cart past the exit, over the same distance the cart brake
        # is configured to stop in: the exit track less the stop margin it must not spend.
        # An inertially coasting frame aborts a complete mission -- see
        # LaunchProfileReferenceFrame -- because it runs away from the decelerating cart.
        exit_track = config.tube.exit_track
        brake_distance_m = (
            exit_track.length_m if exit_track is not None else config.tube.exit_brake_track_length_m
        ) - config.cart.brake_stop_margin_m
        if brake_distance_m <= 0.0:
            raise ValueError(
                "the exit track is shorter than the cart's configured stop margin, so the "
                "reference frame has no braking distance to follow the cart over"
            )
        self._reference = LaunchProfileReferenceFrame(
            layout,
            target_exit_speed_mps=config.launch_control.target_exit_speed_mps,
            brake_distance_m=brake_distance_m,
        )
        self._initial_states = resolve_initial_solver_states(
            layout, plan, fixture, self._reference.sample(0.0)
        )

        self._built = build_launcher_scene(
            stage,
            config,
            plan,
            cart_position_m=self._initial_states[BODY_CART].position_m,
            rocket_position_m=self._initial_states[BODY_ROCKET].position_m,
            orientation_wxyz=self._initial_states[BODY_CART].orientation_wxyz,
            cart_velocity_mps=self._initial_states[BODY_CART].linear_velocity_mps,
            rocket_velocity_mps=self._initial_states[BODY_ROCKET].linear_velocity_mps,
            angular_velocity_radps=self._initial_states[BODY_CART].angular_velocity_radps,
            author_visuals=author_visuals,
        )
        self._paths = {
            BODY_CART: self._built.cart_path,
            BODY_ROCKET: self._built.rocket_path,
        }
        # Author every visualization prim before physics views exist. Mutating visibility on
        # live rigid prims can force a Fabric/physics resync; in the target build that allowed
        # the attached assembly to settle a few millimetres before its first controlled step.
        # Proxy transforms remain the only transforms written once simulation is running.
        self._visual_proxies = None
        self._camera_rig = None
        self._force_overlay = None
        self._field_overlay = None
        if author_visuals:
            self._visual_proxies = GlobalVisualProxies(
                stage,
                plan,
                simulated_paths=(self._built.cart_path, self._built.rocket_path),
            )
            self._camera_rig = AuthoredCameraRig(stage, camera_views(layout))
            self._force_overlay = ForceOverlay(stage)
            self._field_overlay = MagneticFieldOverlay(stage)
        self._handles = {
            name: _RigidBodyHandle(path) for name, path in self._paths.items()
        }
        self._contact_view: Any = None

        app_utils.play()
        self._update()
        self._create_views()
        # ``play()`` followed by an update advances physics before any guide force exists,
        # so the assembly free-falls for that first update. Snapping pose *and* velocity
        # back is what makes step zero start exactly on the centerline; restoring only the
        # velocity would discard the fall speed but keep the accumulated position error.
        self._restore_authored_state()

        masses_kg = {
            BODY_CART: fixture.cradle.mass_kg,
            BODY_ROCKET: fixture.rocket.mass_kg,
        }
        reaction = ForceResolvedPathReaction(
            layout,
            coupled_pitch_inertia_kg_m2=combined_pitch_inertia_kg_m2(
                flatten_numbers(self._handles[BODY_CART].get_inertias()),
                flatten_numbers(self._handles[BODY_ROCKET].get_inertias()),
                cart_mass_kg=masses_kg[BODY_CART],
                rocket_mass_kg=masses_kg[BODY_ROCKET],
                offset_m=plan.cart_to_rocket_offset_m,
            ),
            cart_pitch_inertia_kg_m2=float(
                flatten_numbers(self._handles[BODY_CART].get_inertias())[4]
            ),
            gains=gains,
            gravity_mps2=gravity_mps2,
        )
        self._reaction = reaction
        self._backend = IsaacPhysxBackend(
            bodies=self._handles,
            masses_kg=masses_kg,
            constraints={JOINT_COUPLING: self._built.coupling_joint},
            collision_pair_active={PAIR_ROCKET_CRADLE: True},
            dt_s=self._dt_s,
            reference_frame=self._reference.sample,
            guide_reaction=reaction,
            contact_readers={PAIR_ROCKET_CRADLE: self._read_contact},
            resync_callback=self._resync,
            reset_callback=self._reset,
        )
        if self._visual_proxies is not None:
            initial_global = self._backend.read_state()
            self._visual_proxies.update(initial_global)
            self._camera_rig.update(initial_global)
        self._telemetry_sink = (
            None if telemetry_sink_factory is None else telemetry_sink_factory(self._backend.read_state())
        )
        self._mission = build_mission(
            config,
            layout,
            self._backend,
            gravity_mps2=gravity_mps2,
            telemetry_sink=self._telemetry_sink,
        )

    # --- accessors ------------------------------------------------------------------

    @property
    def mission(self) -> SimulationOrchestrator:
        return self._mission

    @property
    def backend(self) -> IsaacPhysxBackend:
        return self._backend

    @property
    def built_scene(self) -> BuiltLauncherScene:
        return self._built

    @property
    def reference_frame(self) -> LaunchProfileReferenceFrame:
        return self._reference

    @property
    def guide_reaction(self) -> ForceResolvedPathReaction:
        return self._reaction

    @property
    def initial_solver_states(self) -> Mapping[str, InitialRigidState]:
        return dict(self._initial_states)

    @property
    def telemetry_sink(self) -> TelemetrySink | None:
        return self._telemetry_sink

    def step(self):  # type: ignore[no-untyped-def]
        """Advance the common mission and refresh visualization-only global proxies."""
        mission_state = self._mission.step()
        if self._visual_proxies is None:
            return mission_state
        state = self._backend.read_state()
        self._visual_proxies.update(state)
        if self._camera_rig is not None:
            self._camera_rig.update(state)
        applied = self._mission.last_applied_effects
        if applied is not None and self._force_overlay is not None:
            cart = state.body(BODY_CART)
            rocket = state.body(BODY_ROCKET)

            def slot_force(body: str, slot: str):  # type: ignore[no-untyped-def]
                load = applied.loads.get(body)
                if load is None:
                    return (0.0, 0.0, 0.0)
                return load.force_by_slot.get(slot, (0.0, 0.0, 0.0))

            self._force_overlay.update(
                {
                    "launch": (cart.position, slot_force(BODY_CART, SLOT_LAUNCH_FORCE)),
                    "brake": (cart.position, slot_force(BODY_CART, SLOT_CART_BRAKE)),
                    "drag": (
                        rocket.position,
                        slot_force(BODY_ROCKET, SLOT_ROCKET_AERODYNAMICS),
                    ),
                    "thrust": (rocket.position, slot_force(BODY_ROCKET, SLOT_ROCKET_MOTOR)),
                    "gravity": (
                        rocket.position,
                        tuple(rocket.mass_kg * value for value in self._gravity_mps2),
                    ),
                }
            )
        if self._field_overlay is not None:
            cart = state.body(BODY_CART)
            pose = path_pose(self._layout, self._layout.axial_position(cart.position))
            self._field_overlay.update(cart.position, pose.tangent)
        return mission_state

    # --- Isaac-side lifecycle -------------------------------------------------------

    def _create_views(self) -> None:
        for handle in self._handles.values():
            handle.rebind()
        self._contact_view = SimulationManager.get_physics_simulation_view().create_rigid_contact_view(
            [self._paths[BODY_ROCKET]],
            filter_patterns=[[self._paths[BODY_CART]]],
            max_contact_data_count=MAX_CONTACT_DATA_COUNT,
        )

    def _restore_authored_state(self) -> None:
        for name, handle in self._handles.items():
            state = self._initial_states[name]
            handle.set_world_poses(
                positions=[state.position_m], orientations=[state.orientation_wxyz]
            )
            handle.set_velocities(state.linear_velocity_mps, state.angular_velocity_radps)

    def _read_contact(self, dt_s: float) -> ContactReport:
        """Report the rocket/cradle pair as an impulse with its time scaling recorded.

        ``get_contact_data`` returns per-contact normal forces when its argument is the
        physics timestep, so the impulse is recovered by multiplying back by that timestep.
        Section 10.4 requires the scaling to travel with the value because nothing in the
        returned array distinguishes a force from an impulse.
        """
        if self._contact_view is None:
            return ContactReport(pair=PAIR_ROCKET_CRADLE)
        forces, _points, normals, _distances, counts, *_ = self._contact_view.get_contact_data(dt_s)
        force_values = flatten_numbers(forces)
        normal_values = flatten_numbers(normals)
        pair_count = int(sum(flatten_numbers(counts)))
        if len(normal_values) != 3 * len(force_values):
            raise RuntimeError(
                "contact normals and forces disagree in length; "
                f"{len(normal_values)} normal components for {len(force_values)} forces"
            )
        # The buffer is fixed size and only its first ``pair_count`` entries are written
        # this step. Summing the whole buffer would add whatever the previous step left in
        # the tail, which is how a stale impact re-appears as a live contact impulse.
        reported = min(pair_count, len(force_values))
        impulse = ZERO3
        for index in range(reported):
            normal = (
                normal_values[3 * index],
                normal_values[3 * index + 1],
                normal_values[3 * index + 2],
            )
            impulse = add(impulse, scale(normal, force_values[index] * dt_s))
        return ContactReport(
            pair=PAIR_ROCKET_CRADLE,
            impulse_ns=impulse,
            time_scaling="impulse",
            active=reported > 0,
        )

    def _resync(self) -> None:
        """Make a runtime joint write visible without advancing the simulation."""
        app_utils.pause()
        self._update()
        app_utils.play()

    def _reset(self) -> None:
        """Stop, re-author the initial USD rigid state, rebuild physics, restore views."""
        joint = self._built.coupling_joint
        app_utils.pause()
        joint.GetJointEnabledAttr().Set(False)
        self._update()
        app_utils.stop()
        self._update()
        for name, path in self._paths.items():
            _author_stopped_rigid_state(self._stage, path, self._initial_states[name])
        joint.GetJointEnabledAttr().Set(True)
        self._update()
        app_utils.play()
        self._update()
        self._create_views()
        self._restore_authored_state()


__all__ = ["MAX_CONTACT_DATA_COUNT", "ProductionMissionRuntime"]
