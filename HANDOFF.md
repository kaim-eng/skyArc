# Handoff: skyArc implementation

**Date:** 2026-09-02
**Spec:** `DESIGN_REVIEW.md` v0.32 in this directory. It is the authority; this file only records
where the implementation of it stands.

**Read this first.** The project's objective is to decide whether this launcher can replace a
first stage — not to simulate a launch vehicle. The upper stage is a parameterised constraint
(DESIGN_REVIEW section 10.6), not a body. Work that does not move that question forward, or does
not keep its evidence honest, is not the priority however interesting it looks.

**Headline results as of this revision.** The first complete mission ran end to end on real CPU
PhysX: 120,679 steps to `flight_window_complete`, exit speed 1999.9254 m/s (3.7e-5 relative
error), peak load 9.066G, peak centerline tracking 0.0004619 m against a 0.05 m limit, and the
orchestrator's stop/rebuild reset restoring the authored state within 43 nm. Scored against a
200 km circular target, the launcher supplies **2,030 m/s** of ideal delta-v and hands over at
31.267 km where a ground launch's drag and gravity losses are largely avoided. The binding
constraint is now the upper stage, not the launcher.
**Status:** The backend-neutral Phase 1 core is complete. The contract, geometry, configuration, registry,
and baseline observer slices are implemented. The schema-v3 curved configuration, centerline, stage
mapping, preflight, analytic backend, guided drag, launch controller, and guided trajectory slices
are now implemented too. The ideal analytic guide and jerk-limited cart brake are implemented and
run through the same adapter. The ordered release, passive separation, ignition interlock, rocket
motor/aerodynamics, concurrent mission state machine, and common orchestrator are implemented as
well. The versioned telemetry schema, streaming recorder, energy accounting, run summary, and safe
output-path contract are implemented and connected to that orchestrator. Complete manifests,
dependency-closure hashes, versioned criteria, named random streams, and paired contrasts are now
implemented too. Swept cart/rocket clearance, local tube regularity, and nonlocal overlap/unique-
projection gates complete the curve geometry. The v0.20 review closure additionally makes the
downrange-reversal gate analytic per segment, makes accumulated-float endpoint evaluation safe,
and bounds the spatial-index cell count for differently scaled tubes. Phase 0 evidence execution is
complete. Under v0.29 the panel amended the solver-reaction criterion and accepted the
force-resolved controller as a non-constraint treatment for system-level production simulation;
hardware contact-load validation remains explicitly out of scope:
`standalone/qualify_phase0.py` now produces strict, hashed, uniquely named CPU PhysX/Newton evidence
for runtime selection, force, an articulated inclined prismatic guide, joint reaction capability,
solver-confirmed release, first-step collision/contact reporting, and fully exercised stop/rebuild
reset. Final matched evidence moves the cart/rocket fixed joint outside the guide articulation.
CPU PhysX then proves solver-side live release, rejects live collision activation, and passes the
always-present pair treatment including nonzero-force contact and full reset. Newton cannot step the
required external-joint topology and remains unable to report incoming guide reaction. CPU PhysX is
selected for curved-guide work. The authored cylindrical-rocket/open-front-cradle pair passes its
2.5 km/s 1.0/0.5/0.25 ms matrix without CCD, and its 1 ms CCD control is identical. The
force-resolved curved controller also passes its complete 54.116 km physical profile at 2 km/s,
including inferred load, tracking, attachment geometry, release, and cold reset at 1.0/0.5/0.25 ms
using the exact production bodies. It does not expose a solver constraint reaction. The other three
candidates are formally rejected or limited to visualization fallback. The production Isaac layer is
implemented: an extension manifest/entrypoint, pure resolved-scene plan, PhysX/CPU translated-frame
adapter, exact scene builder, explicit lighting, a standalone scene-construction runner, and — as of
this revision — the common mission orchestrator running on that scene through `IsaacPhysxBackend`.
The suite now contains **175 stdlib unit tests**. At this handoff, 168 pass and 7 assertions
correctly reject artifacts produced before the current source closure — every artifact is stale,
because the extension was renamed out of NVIDIA's `isaacsim.examples.*` namespace into the
project's own `skyarc`, which changes both the closure and the runner hashes. A single
requalification pass clears all seven. **All 10 Kit integration tests pass** as of the last run
before that rename, including the real-PhysX conserved-momentum case; they need one more run
against the renamed package. The
backend-neutral tree compiles without forbidden Isaac Sim, Omni, Warp, or NumPy imports; the four
Kit boundary modules intentionally import Isaac/Omni only after `SimulationApp` startup, and
`tests/unit/test_production_mission.py` now enforces that rule by parsing the imports rather than
by trusting it. Two earlier review passes closed the preflight and contract defects recorded in
sections 2.1–2.2; the execution-slice review corrections are in section 2.3 and the mission-wiring
corrections in section 2.4.

Version 0.20 implements schema version 3 in the backend-neutral core as a parallel, explicit
extension. `configs/baseline.yaml` remains the unchanged schema-version-2 default, including its
resolved SHA-256. `configs/curved_2kms.yaml` is the checked-in high-speed reference and resolves to a
54.116 km planar centerline, a 15-degree exit tangent, and the separate 25 km braking track. Do not
reinterpret schema-v2 fields or treat pure-math acceptance as physical validation. The panel has
accepted the translated-frame treatment for system-level simulation, and the first curved Isaac
scene slice is now implemented; that acceptance does not convert commanded/reconstructed load into
solver constraint reaction.

The implemented curve contract includes the v0.7 through v0.9 review corrections: oriented signed curvature;
an explicit 25 km straight braking track continuing the 15-degree exit tangent; separate 10G
resultant budgets for the attached assembly and post-release cart; a cart brake supervisor that
counts drag, resistance, contact, and guide reaction together; a declared schema-v3 exterior
atmosphere with `constant_v1` restricted to zero-duration launch-only evidence; an explicit
66-second free-flight horizon independent of the 130-second abort timeout; factor-two atmosphere
stage refinement with numerical tolerances; a speed-resolved 40 m density blend; and a named
2,500 m/s anti-tunneling release pair. The v0.9 two-sided guide-normal bound is implemented as well,
so straight portions account for gravity-normal guide support instead of incorrectly reporting zero.

---

## 1. Environment findings (verified this session)

These were checked against the machine, not assumed. They constrain the implementation.

| Fact | Consequence |
|---|---|
| Isaac Sim source + full build at `C:\Dev\Isaacsim\IsaacSim`, `VERSION` = `6.0.1-rc.7` | Matches the doc header exactly. Kit tests and the standalone runner can actually be run here. |
| Bundled interpreter: `C:\Dev\Isaacsim\IsaacSim\_build\windows-x86_64\release\python.bat`, Python 3.12.13 | Use this for the unit suite. |
| `import numpy` **fails** in that interpreter outside a Kit app | Confirms section 12's claim. **The whole `tests/unit` core must be numpy-free.** This is why `linalg.py` exists (pure-Python tuple math) instead of using numpy. Do not "fix" this by adding numpy to the core. |
| `import yaml` works (PyYAML 6.0.3) | Config loading can use YAML in the core. |
| `import pytest` **fails** | Unit tests must use stdlib `unittest`, run via `python.bat -m unittest discover`. This also matches `omni.kit.test`, which is unittest-based. |

### API surface confirmed by reading the target build's source

Every runtime claim the design doc makes in sections 5.1, 10.4, 12 and 14 was spot-checked and
holds on this build:

- `isaacsim.core.experimental.prims.RigidPrim.apply_forces(..., local_frame: bool = False)` and
  `apply_forces_and_torques_at_pos(..., positions=..., local_frame=False)` — the prim API takes a
  **local**-frame flag. The doc's warning about polarity inverting between layers is real; keep
  the translation inside the adapter only.
- `RigidPrim.get_world_poses()` / `set_world_poses()` / `get_coms()` return quaternions **`wxyz`**.
  The tensor view beneath is scalar-last. `linalg.quat_wxyz_from_xyzw` exists for the adapter and
  nothing else should call it.
- `RigidPrim.get_net_contact_forces(dt=1.0)` and `get_contact_force_matrix(dt=1.0)` — **the default
  returns impulses**; `dt` is a time-scaling multiplier and nothing in the return value says which
  you got. `ContactReport.time_scaling` in `state.py` carries this with the value.
- Contact views require `contact_filter_paths` / `max_contact_count` **at `RigidPrim` construction**
  — the fixed-size buffer and the "filtered pairs established when the view is created" constraint
  from section 10.2 are both confirmed. Matched evidence rejects live activation and selects a
  startup-authored pair whose reports are ignored while the fixed joint remains active.
- `isaacsim.core.simulation_manager.SimulationManager`:
  `register_callback(callback, event, *, order=0)` / `deregister_callback(uid)`,
  `SimulationEvent.PHYSICS_PRE_STEP` / `PHYSICS_POST_STEP`,
  `IsaacEvents.PHYSICS_WARMUP` / `SIMULATION_VIEW_CREATED` / `PHYSICS_READY` / `POST_RESET` / `TIMELINE_STOP`,
  `get_active_physics_engine() -> Literal["physx","newton","remotesim"]`, `get_default_engine()`,
  `switch_physics_engine(...)`, `setup_simulation(dt, device)`, `step()`, `enable_ccd(flag, physics_scene)`,
  `is_ccd_enabled(...)`, `set_solver_type`, `set_physics_dt`, `enable_fabric`, `get_physics_simulation_view`.
  The `order=` argument on `register_callback` is how section 9.3's "pinned registration order" is
  actually implemented — record the value used in the manifest.
- `PhysxScene.get_enabled_ccd()/set_enabled_ccd()` exists but `PhysicsScene` (the generic base) has
  no CCD accessor at all. This is the concrete form of "CCD is a PhysX scene property only".
- Useful scene-building helpers: `isaacsim.core.experimental.objects` provides `Cube`, `Cylinder`,
  `Capsule`, `Cone`, `Sphere`, `Plane`, `Mesh`, `Camera`, and `DistantLight`/`DomeLight`/`SphereLight`/
  `CylinderLight`/`DiskLight` (needed for the section 13.2 lighting rig).
  `isaacsim.core.experimental.utils.stage` provides `create_new_stage`, `define_prim`, `save_stage`,
  `set_stage_up_axis`, `get_stage_units`, `add_reference_to_stage`.

### Extension conventions to copy

Mirror `C:\Dev\Isaacsim\IsaacSim\source\extensions\isaacsim.examples.interactive`:
`config/extension.toml` (with `[[python.module]]`, `[[test]]` blocks and `writeTarget.kit = true`),
`premake5.lua` using `get_current_extension_info()` + `repo_build.prebuild_link`, `docs/CHANGELOG.md`,
`docs/Overview.md`, `data/icon.png`, `data/preview.png`. Apache-2.0 SPDX header on every file.

---

## 2. What exists now

All under
`exts/skyarc/skyarc/`.
`isaacsim/` and `isaacsim/examples/` are intentionally **namespace packages** (no `__init__.py`), the
same as the shipped Isaac extensions.

| File | Contents | Doc sections |
|---|---|---|
| `__init__.py` | Package docstring stating the core/Isaac split rule. `__version__`. Imports nothing. | 15 |
| `linalg.py` | Pure-Python `Vec3`/`Quat` math: add/sub/scale/dot/cross/norm/normalize, quaternion multiply/rotate/conjugate/from-axis-angle, `clamp`, `is_finite`, and the two adapter-only reordering helpers `quat_wxyz_from_xyzw` / `quat_xyzw_from_wxyz`. | 5.1, 14 |
| `names.py` | All stable identifiers: bodies, `JOINT_COUPLING`, `JOINT_GUIDE`, `PAIR_ROCKET_CRADLE`, the four markers, the eleven slots. Plus the executable ownership tables `SLOT_BODY_OWNERSHIP`, `SLOT_CONSTRAINT_OWNERSHIP`, `SLOT_COLLISION_OWNERSHIP`, `SLOT_MASS_OWNERSHIP`. | 5.1, 9.3, 10.4 |
| `state.py` | `BodyState`, `ContactReport` (carries `time_scaling`), `SimulationState` (+`.frozen()`), `AxialQuantities`, `Observation`, `MarkerSpec`, and helpers `marker_world_position`, `body_com_world`, `combined_mass`. Keeps latent state and observation as distinct types. | 5.1, 7, 14 |
| `events.py` | `Event` with a **closed set** of event names; recorder assigns the monotonic `sequence` via `with_sequence`. | 5.1, 11, 14 |
| `effects/__init__.py` | Re-exports the public effect API. | 5.1 |
| `effects/types.py` | The four effect kinds — `Wrench`, `MassUpdate`, `ConstraintCommand`, `CollisionPairCommand` — plus `Frame`, `ConstraintAction`, `CollisionAction`, `MomentumPolicy`, and `EffectBatch`. Every wrench carries an explicit frame, application point and `units="SI"`. | 5.1, 9.2 |
| `effects/validation.py` | `validate_wrench` / `validate_mass_update` / `validate_constraint_command` / `validate_collision_command` / `validate_batch` / `validate_capabilities`. Rejects unknown bodies, missing frames, non-finite values, wrong units, ownership violations, `ACCOUNTED` mass updates without an exhaust velocity, and per-effect `source` disagreeing with its batch. | 5.1, 16.1 |
| `effects/aggregator.py` | `resolve_wrench` (the one place body-vs-world frame is interpreted), `aggregate` (ownership check → world resolution about COM → sum, with conflict detection on duplicate mass updates and contradictory constraint/pair commands), `BodyLoad`/`AggregatedEffects` retaining the **per-slot** force decomposition, and `axial_force` / `axial_slot_force` / `scaled_load`. | 9.3, 16.2 |
| `effects/adapter.py` | `BackendAdapter` runtime-checkable protocol, immutable `BackendCapabilities`, and a distinct `AppliedEffects` record so accepted and backend-applied effects remain separable. Includes the mandatory `resync` lifecycle boundary. | 5.1, 10.2, 14 |
| `effects/backends/analytic.py` | Deterministic translation-only `BackendAdapter`: semi-implicit Euler, combined-mass attached motion, exact straight/curved path coordinates for the guided cart, fixed-offset rocket motion while coupled, 3-D ballistic rocket translation after release, reversible constraint/pair state, mass updates, reset, and explicit rejection of unsupported torque/contact claims. | 5.1, 10, 16.2 |
| `components/contract.py` | `ComponentDescriptor`, determinism claim, deeply frozen `ScenarioContext`, `StepOutput`, and the six-method `Component` lifecycle. | 5.1 |
| `components/diagnostics.py` | Registered unit/shape metadata, namespace and reserved-key enforcement, finite JSON-safe scalar values, fixed rectangular arrays, record-size bounds, and immutable diagnostic records. | 9.2, 16.1 |
| `components/registry.py` | Deterministic `(slot, model_id)` registration/resolution with duplicate and descriptor-truthfulness checks. | 5.1, 14.1 |
| `components/observers.py` | `ground_truth_v1`; derives axial quantities and marker-sampled density while retaining a separate frozen latent-state packet. | 5.1, 8, 14 |
| `launcher/geometry.py` | Straight baseline transforms plus schema-v3 planar straight/clothoid/arc centerlines, pose/frame/signed-curvature evaluation, endpoint-tangent-extending nearest-path projection, normal jerk, the v0.9 two-sided guide-normal bound, arc-length atmosphere-stage lookup, deterministic stage refinement, and swept-envelope certificates. The sweep combines a global-curvature rigid-body bound, local tube-regularity gate, and chord-error-padded spatial search for nonlocal tube overlap/ambiguous projection. | 7, 8, 16.1–16.2 |
| `launcher/atmosphere.py` | `density_drag_v1` with signed air-relative quadratic drag, vacuum/tailwind behavior, one equivalent attached-assembly wrench, and model-owned state. | 8, 9.3, 16.1 |
| `launcher/launch_force.py` | `abstract_axial_v1` with constant-force, constant-acceleration, target-exit-speed, and force-versus-position modes; acceleration-before-force clamping; authored-force caps for both force-specified modes; drag/grade/resistance compensation; rest-state ramp bootstrap; and the schema-v3 resultant-load supervisor. | 6.5, 9.1–9.3, 16.1 |
| `launcher/guide.py` | `ideal_prismatic_v1`/analytic `tangent_following_v1` resistance wrench, two-sided normal-load calculation, centerline tracking monitor, and one-shot abort event on excess tracking error. It does not fabricate a normal wrench in parallel with the backend constraint. | 7, 9.3, 10.2, 16.1 |
| `launcher/path_controller.py` | Backend-neutral force-resolved guide reaction and its translated accelerating frame: `PathControllerGains` pinned to the accepted Phase 0 values, `LaunchProfileReferenceFrame` (uniform commanded profile, inertial continuation past the exit), and `ForceResolvedPathReaction`, which sizes the normal/binormal reaction and attitude torque for the attached assembly or the released cart, subtracting the accepted external load first. Pure and unit-tested; `effects/backends/isaac.py` is its only production caller. | 7, 9.3, 10.2, 17 |
| `launcher/production_runtime.py` | **Kit boundary.** Builds the scene, holds rebindable rigid-body handles, reads the qualified rocket/cradle contact view, supplies the reference frame, and hands `IsaacPhysxBackend` to `build_mission`. Owns the two lifecycle operations the orchestrator cannot express: a pause/update/play resync that does not advance time, and the qualified stop/rebuild reset that re-authors USD rigid state while physics is absent. | 10.2, 12, 13 |
| `launcher/brake.py` | `force_limited_v1` remaining-distance controller with force and jerk ceilings, shared vector cart-load budget, drag/grade/resistance accounting, no-reversal clamp, hold state, and stopped/reversal events. | 6.2, 10.5, 16.1–16.2 |
| `launcher/analytic.py` | Contact-free guided and cart-braking runners using the real component/effect/adapter path, marker-defined exit, stage/exterior drag, resistance and brake work, Section 7 normal-jerk tracking from accepted tangential acceleration, and work accounting. At 1 ms the curve reaches 1,999.991 m/s at 54.115 s; its cart stops in 23.000 km and 22.504 s at 9.066G without reversal. | 16.1–16.2 |
| `coupling/` | `fixed_joint_v1`, the ordered six-step reversible release transaction, explicit adapter resync and mutation-time continuity check, named-envelope gap/rate measurement, passive `none_v1` actuator, timeout/impulse/recontact monitor, and fail-closed seven-gate ignition interlock. | 10.1–10.4, 16.1–16.2 |
| `rocket/` | Interlock-commanded `constant_mass_thrust_v1` body-axial thrust and detached `quadratic_point_drag_v1` at the rocket centre of mass. Neither can act on the cart. | 5.1, 10.4–10.5, 16.1–16.2 |
| `state_machine.py` | Finite mission states with one stable `LAUNCH_STAGE` plus stage index/name, strict event ordering, abort dominance, and concurrent post-detach cart/rocket branches whose join alone produces `COMPLETE`. | 11 |
| `orchestrator.py` | Common read/observe/gate/pre-step/aggregate/apply/resync/step/read/boundary/post-step loop, marker-defined exit/release transitions, ignition dispatch, evidence-window completion, reset/replay, and an analytic full-mission result. Release mutation steps intentionally carry no attached-equivalent wrench. | 10–11, 14, 16.2 |
| `telemetry/` | Closed versioned core schema (`core_telemetry_v2`) with type/unit/frame/sample-phase/per-field-validity metadata; collision-safe UUID run-instance paths; streaming CSV, registered diagnostic JSONL, monotonically sequenced event JSONL, backend-applied per-slot energy accounting including resistance, separation and the backend's own `guide_reaction` term, and run summaries. The recorder keeps accepted and applied effects distinct, integrates energy at physics rate even when samples are decimated, and marks rotational closure invalid until inertia is part of the body contract without aborting the run. | 14, 16.1–16.2 |
| `experiments/` | Canonical value hashing; declared source-closure code identities with resolved external versions; complete fail-closed manifests over every component slot; versioned outcome criteria and schema-v2 evidence resolution; access-order-independent named streams; exact nested factor diffs; and paired adjacent/baseline contrasts that reject unpaired seeds or initial states. The built-in component closure is conservatively package-wide until packaging generates narrower declarations. | 14.1, 16.1, 16.4 |
| `configuration/` | Immutable schema-v2/v3 dataclasses, strict version-routed YAML loading, the closed `EXECUTION_PROFILES` table, source/resolved SHA-256 hashes, curve and stage resolution, exit-track continuity, exterior-atmosphere/evidence rules, vector load and jerk gates, launch feasibility, and grade-aware jerk-limited braking/run-time preflight. | 6, 12, 14.1, 16.1 |
| `configs/baseline.yaml` | Complete baseline including grade, guide resistance, release latency/confirmation, markers, anti-tunneling pair, validity policy, criterion policy, and capture profile. It resolves successfully. | 6.6–6.7, 12–14 |
| `configs/curved_2kms.yaml` | Complete schema-v3 reference: 400 kg attached mass, 10G resultant limits, 2 km/s target, 54.116 km centerline, three transition stages, exponential exterior atmosphere, 25 km exit track, and explicit evidence/refinement controls. It resolves successfully. | 6.7–8.1, 16.1 |
| `effects/backends/isaac.py` | **Kit boundary.** Production `BackendAdapter` for PhysX/CPU: Warp-safe tensor reads, translated-frame reconstruction into global SI, the fictitious `-m*a_r` body force, the commanded guide reaction reported under `backend_adapter` in `AppliedEffects`, mass/constraint mutation, the refusal of live collision-filter changes, and `peak_solver_offset_m` as the measured form of the post-release precision limitation. | 10.2, 12, 14 |
| `tests/` (inside the package) | 9 `omni.kit.test` integration cases for the adapter boundary: selected runtime and capabilities, global-SI reconstruction on the centerline, applied-minus-accepted equalling exactly the backend slot, refusal of live collision mutation, a resync that does not advance time, 200 guided steps inside the guide clearance and on the commanded profile, a stop/rebuild reset that restores state and clears mission history, a release transaction resynced with forces already pending, and the reaction resizing from assembly to cart on release. Excluded from every component's source closure. | 16.2–16.3 |
| `tests/unit/` | 157 NumPy/Isaac-free `unittest` cases covering the prior contracts plus schema-v3 routing, curve resolution and exterior projection, analytic backend/release semantics, all launch modes and authored/acceleration/resultant ceilings, drag scaling/sign, guide monitoring, Section 7 normal jerk, jerk/vector/no-reversal braking and hold, all ignition gates, separation/recontact failure, strict concurrent state progression, full baseline mission reset/replay, baseline energy convergence, telemetry contracts, controlled resistance/separation-inclusive energy closure, dependency identities, complete manifests, criteria, named streams, factor lineage, paired contrasts, swept rigid-body clearance, local singularity, global self-overlap, localized downrange reversal, accumulated-float endpoints, differently scaled spatial indexing, the 2 km/s launch/stop limits, Phase 0 runtime and artifact contracts, the production anti-tunneling matrix, curved-guide source closure, full-profile timestep convergence, the force-resolved reaction law and reference frame, the executable import boundary, full-wrench guide-reaction work attribution, frozen telemetry V1 compatibility, and the bound production mission artifacts. | 16.1 |

### 2.1 Defects found and fixed by review

Four preflight holes were found by probing the loader with hostile configurations rather than
by reading it. Each now has a regression test. The baseline resolved SHA-256 is unchanged by
these fixes, because none of them altered a configured value.

1. **Non-finite signed values defeated the whole preflight.** `exit_track_grade_deg: .nan` was
   *accepted*, yielding `required_distance_m = nan` and `remaining_margin_m = nan`: the gate read
   `if remaining_margin_m < 0.0`, and every comparison against NaN is false. `angle_deg` and
   `axial_air_velocity_mps` were equally unguarded. Infinities were worse — they escaped as a bare
   `ValueError: math domain error` from `math.radians`, naming no field, and since
   `ConfigurationError` subclasses `ValueError` a caller catching the latter would not catch it.
   Fixed in three places: `loader._number` now rejects non-finite values, which closes the class
   for every numeric leaf including vector components; `validate_scenario` and `braking_preflight`
   apply an explicit `_finite` check to the signed fields, because both are exported and callable
   on a config built in code that never passes through the loader; and the braking gate is now
   `if not remaining_margin_m >= 0.0` so a NaN margin fails.
2. **CCD was accepted with `device: auto`.** Section 12 records that the target build ignores CCD
   on CUDA with a warning rather than refusing it, so an unpinned device is not a lesser case of
   the CUDA rule — it is the case that produces an archived configuration claiming a setting that
   was never in force. CCD now requires backend `physx` *and* an explicitly pinned non-CUDA device,
   with a distinct message per rule.
3. **Evidence status was a substring match on the profile name.** A profile called
   `headless_rendered` used for an evidence run accepted `backend: auto`. Replaced with the closed
   `EXECUTION_PROFILES` table in `configuration/schema.py`; unknown profile names are now rejected
   outright. Each profile also declares `fixed_time_stepping`, which section 12 requires be pinned
   per profile, and the dataclass enforces that an evidence profile cannot disable it.
4. **A self-disabling check.** `GroundTruthObserver.prepare` skipped its marker validation entirely
   when `context.markers` was empty — precisely the case where every marker is missing.

### 2.2 Defects found and fixed by the clean-context audit

Six additional contract holes were found after walking the whole Phase 1 slice against v0.5.
They are covered by new or expanded hostile-case regressions. The baseline resolved SHA-256 and
the documented preflight figures remain unchanged.

1. **Evidence runs could spell `auto` with different case or surrounding whitespace.** The evidence
   gate used exact string equality even though the CCD gate already treated backend/device
   identifiers case-insensitively. Both checks now use the same stripped, lower-case identity.
2. **Tailwind feasibility always used the densest stage.** Density drag assists rather than opposes
   motion when air speed exceeds target speed; in that case the least-dense stage is limiting. The
   old calculation could credit a vacuum stage with tailwind assistance and accept a launch force
   that could not overcome grade there. Preflight now selects the limiting density by drag sign.
3. **Named marker roles were not tied to bodies.** Required names could be attached to any known
   body, allowing stagnation density or separation geometry to be sampled from the wrong body.
   Baseline marker/body ownership is now explicit, and the observer rejects marker definitions that
   disagree with the immutable scenario context.
4. **Aggregated effect records were only shallowly frozen.** Callers could mutate `loads` or a
   `BodyLoad.force_by_slot` mapping after validation, changing accepted-effect telemetry without a
   new aggregation pass. Both mappings are now immutable copies.
5. **Registry parameters were only shallowly frozen.** Nested lists and mappings supplied to a
   component factory remained mutable, undermining parameter-hash provenance. Registry resolution
   now applies the same recursive freezing as `ScenarioContext`.
6. **Events claimed a bounded JSON-safe contract but checked only their names.** Mutable mappings,
   arbitrary objects, non-finite numbers, and invalid timing/sequence metadata could enter the
   future telemetry path. Event construction now validates finite metadata, recursively freezes
   JSON-safe payloads, and enforces explicit field, node, string, and nesting bounds.

### 2.3 Defects found and fixed by the execution-slice review

Three mismatches were corrected without changing the schema-v2 evidence hash or the resolved
geometry, exit-speed, load, energy-convergence, and braking baselines. Four regressions cover them.

1. **Force-specified modes could exceed their authored command.** The rest-state hold bias could
   raise a low `constant_force` or `force_vs_position` request. Both modes now retain the authored
   request and cap the delivered force at it after the acceleration ceiling has been applied.
2. **Nearest-path projection clamped outside the centerline interval.** `nearest_s` now projects
   onto the entrance and exit tangents before its interior search, matching `path_pose`'s existing
   extrapolation and restoring inverse consistency on both sides of the authored path.
3. **The runner reported a derivative of the guide-normal bound as normal jerk.** It now evaluates
   Section 7's signed normal-jerk expression from speed, accepted tangential acceleration,
   curvature, and curvature rate. The 1 ms reference result is 49.371 m/s³ versus the 49.383 m/s³
   preflight sweep; the remaining difference is timestep sampling at the exit peak.

### 2.4 Defects found and fixed while wiring the mission through the production adapter

All three were invisible to inspection and were found by running the mission, not by reading it.

1. **The adapter could not read its own tensors.** `_vector`/`_quaternion` walked `value.shape`
   and then indexed. Warp arrays expose `shape` but raise `RuntimeError: Item indexing is not
   supported on wp.array objects`, so every production read would have failed on the first step.
   The Phase 0 runners never exercised this code, and nothing else had. Both helpers now go through
   the shared `flatten_numbers`, which converts via `numpy()` first — the same conversion the
   evidence runners use.
2. **The launcher was silently inactive at the start of the mission.** The first fix moved the
   assembly from the Phase 0 qualified COM-at-entrance condition to cart-at-entrance merely because
   `abstract_axial_v1` gated on cart position. v0.31 removes that control-induced physical change:
   launch control now uses mass-weighted assembly COM progress and speed, while
   `resolve_initial_solver_states` again reproduces the qualified COM-at-entrance state.
   `standalone/run_mission.py` and `standalone/run_launcher.py` share the placement helper.
3. **Two runs could not be built to attach a telemetry sink.** The recorder needs the backend's
   initial state, which needs a built scene, so the first attempt constructed the runtime twice.
   Authoring the same USD prims twice duplicates their transform ops. The sink is now supplied as a
   factory that the runtime calls once the backend exists.

A review of this slice found three more, all now fixed:

4. **The artifact's peak solver offset described the reset, not the run.** `IsaacPhysxBackend.reset`
   zeroes `_peak_solver_offset_m`, and `run_mission.py` assembled its summary after the
   `--reset-replay` block, so with that flag the figure came from the re-read of the restored
   initial state. The peak is now captured before any reset. Note what this does *not* show: on a
   healthy run the reported value barely moves, because the assembly tracks the reference frame and
   the true peak is the rocket's own 3.26 m fixed-joint offset from the frame origin either way.
   The bug matters exactly when the run is unhealthy — while defect 2 above was live, the assembly
   drifted away from the frame and the honest peak was 6.72 m. A masked measurement that agrees
   with the truth whenever nothing is wrong is worse than a missing one.
5. **A reset restored the adapter's mass mirror but not the prims'.** `apply` writes mass updates
   through with `RigidPrim.set_masses`; the reset callback re-authors pose and velocity only. The
   adapter would then report a mass the solver did not have. Latent — `constant_mass_thrust_v1`
   emits no mass updates — but the adapter accepts them and `SLOT_MASS_OWNERSHIP` exists to
   anticipate propellant depletion. `reset` now writes the initial masses back itself rather than
   trusting a callback to cover them.
6. **`resolve_initial_solver_states` briefly took an unused `fixture`.** That symptom came from the
   rejected cart-seating workaround. v0.31 restores the fixture parameter because the qualified COM
   placement correctly needs the two masses.

The Kit integration suite gained two cases for the one boundary this slice left unexercised: a
resync performed while forces are already pending on the prims, which the Phase 0 release probe
never did, and the reaction resizing from the assembly to the cart once the joint goes inactive.
Both pass, and the release resync reproduces the qualified 0.0 mutation discontinuity.

Two facts about the boundary are recorded rather than fixed, because both are properties of the
qualified condition rather than defects:

- Starting the timeline turns `/app/runLoops/main/rateLimitEnabled` back on. It does not throttle a
  run that drives `SimulationManager.step()` directly, but it does mean a post-run reading is not
  comparable to the Phase 0 runtime probe, which samples before `play()`. `run_mission.py` reports
  both readings.
- Forces are applied at the body origin and `read_state` leaves `com_offset_m` zero, matching the
  Phase 0 qualified runner exactly. Authoring an explicit centre of mass and inertia so that the
  origin is provably the centre of mass would perturb the qualified fixture's mass properties and
  therefore its bound evidence; it is a separate, requalifying change.

### 2.5 Known nits, deliberately not fixed

Judged not worth the churn now; revisit if the surrounding code is touched anyway.

- `validation.py` derives one policy from label text
  (`allow_zero=label.endswith("distance_m") or label.endswith("coefficient")`). Correct today, but a
  future field named `..._coefficient` would silently inherit permission to be zero.
- `geometry.world_position` and `geometry.gravity_projection` validate the axis by calling
  `axial_position(...)` for its side effect and discarding the result. A named
  `_require_unit_axis(axis)` would say what it means.
- `aggregator.scaled_load` is dead code; nothing imports it and `effects/__init__.py` dropped it.
- Extra unregistered marker names are accepted, while every other section rejects unknown keys.
  This may be deliberate (markers as an open set that a later model extends) — decide and comment.
- The `force_vs_position` closed-form feasibility screen uses the table's maximum point force over
  the full path. That is optimistic when a high force exists only over a short interval. Treat it
  as an early rejection screen; trajectory acceptance still comes from the analytic runner until
  the preflight integrates the piecewise force profile.
- The production mission runs at roughly **40 physics steps per wall second** against the Phase 0
  runner's 262. The difference is the component stack and the orchestrator's three state reads per
  step, not the solver: a complete 1 ms mission is therefore on the order of an hour. That is
  acceptable for an offline evidence run and unacceptable for anything interactive. The obvious
  first move is to let the orchestrator pass its already-read pre-step state into `apply` instead of
  having the adapter read it again, which needs a `BackendAdapter` protocol change and so was left
  out of this slice.

### Design decisions already made (keep or overturn deliberately)

1. **Core is numpy-free and Isaac-free.** Forced by the environment finding above.
2. **Per-slot force contributions are retained through aggregation** (`BodyLoad.force_by_slot`).
   Section 16.2's energy identity needs `W_launch`, `W_drag`, `W_brake`, `W_thrust`, `W_res`
   separately, and they cannot be recovered from a summed force after the fact.
   `AppliedEffects` carries the same decomposition, which is what lets the production adapter
   report a load larger than the accepted one without that difference being unattributable.
3. **The coupling slot owns no wrench at all** — only constraint and collision-pair commands
   (`SLOT_BODY_OWNERSHIP[SLOT_COUPLING] == frozenset()`). This is what mechanically prevents a
   future piston/ejection model from being smuggled into the electromagnetic slot (section 20.1).
4. **Ownership is a table, not a convention.** `names.py` holds it; `effects/validation.py` enforces it.
5. **The analytic adapter is the pure execution oracle.** `AnalyticBackend` uses semi-implicit
   Euler over the two bodies (cart on the resolved straight/curved path, rocket at a fixed path
   offset while coupled and 3-D translation after release). Keep the state machine, interlocks,
   and energy identity runnable through it without a Kit application; it is the natural home for
   section 16.2's controlled analytic cases without contact.
6. **`state_machine.py` and `orchestrator.py` at package root.** Section 15 lists no directory for
   them and they span launcher + coupling. This is a deliberate, minor addition to the §15 layout.
7. **Normal motion is the backend's job, not the guide component's.** `IdealPathGuide` owns only the
   guide's tangential resistance. `AnalyticBackend` discharges the normal constraint by integrating
   along the path; `IsaacPhysxBackend` has no path joint, so it commands the reaction from
   `launcher/path_controller.py` and reports it as an applied effect. Putting that reaction in the
   guide component instead would need two guide models and a capability-driven swap in
   `build_mission`, and would make `applied == accepted` true while the accepted load no longer
   described what the solver saw.
8. **The reference frame follows the commanded profile, not the measured trajectory.** An analytic
   reference has an exact analytic acceleration; a finite-differenced one would feed integration
   noise back through the fictitious force. Past the tube exit the frame stops accelerating and
   coasts, because after release the cart and rocket separate by tens of kilometres and no single
   accelerating frame stays near both. `IsaacPhysxBackend.peak_solver_offset_m` measures the
   resulting solver-coordinate growth instead of assuming it stays small.

---

## 3. What is missing

The remaining work is ordered roughly by dependency. Items listed in section 2 are implemented
and tested; do not recreate them.

### Phase 1 — backend-neutral core (complete)

No backend-neutral implementation item remains. Keep future pure-core tests using the existing
bootstrap, which intentionally inserts the directory containing the pure `skyarc`
package; inserting the extension root instead is shadowed by the bundled runtime's regular
`isaacsim` package outside a Kit application.

### Phases 2–5 — Isaac Sim layer (mission execution implemented; visualization outstanding)

Implemented:

- `effects/backends/isaac.py` — production adapter for PhysX/CPU state mapping, translated-frame
  fictitious load, accepted wrenches, the commanded guide reaction reported under
  `backend_adapter`, mass, coupling, reset, and resync.
- `launcher/path_controller.py` — the backend-neutral reaction law and reference frame.
- `launcher/production.py` and `launcher/scene.py` — strict fixture/scene planning plus the four
  tube bands, exit marker/track, exact open U cradle and X-axis cylinder, always-present fixed
  coupling, explicit lighting rig, and the shared initial-state placement.
- `launcher/production_runtime.py` — the Isaac-side mission lifetime, wiring `build_mission` to the
  production scene.
- `extension.py`, `config/extension.toml`, and `docs/`. The module entrypoint is
  `skyarc.extension`, preserving the Isaac-free package root.
- `standalone/run_launcher.py` — scene construction only, target-build-smoked.
- `standalone/run_mission.py` — target-build mission execution with bounded runs, telemetry, and an
  orchestrator reset replay.
- `exts/.../skyarc/tests/` — seven Kit integration tests for the adapter boundary.

Also implemented since:

- `launcher/feasibility.py` — the §10.6 upper-stage screen, and the `stage2_constraint` config
  block with loader support and validation that delegates to the screen rather than restating it.
- The cart-following reference frame. The v0.30 inertial continuation is **withdrawn**: it ran
  away from the decelerating cart to 34,046.9 m of solver offset and aborted a complete mission on
  `guide_tracking_error` six seconds *after* the cart had stopped, at 0.0495851 m against the
  0.05 m limit. Replaying the recorded trajectory against the new frame gives 247.5 m peak cart
  offset, and the re-run mission completed with 0.0004619 m tracking.
- Static rendering. `run_launcher.py --capture-dir` renders authored views through replicator and
  records mean luminance, variance and `schematic_tube` per view; `artifacts/production/renders/`
  holds the current pair. §13.4 explains the two-scale band sets.

Outstanding:

- `visualization/` — `ui.py` (Kit panel, §13.1 controls), `overlays.py` (force arrows, coil bands,
  field arrows), `cameras.py` (the remaining five of the seven §13.3 views).
- **Animated mission rendering is blocked.** The visuals are authored in global coordinates and
  the bodies live in solver coordinates; they coincide only at t=0. Rendering a mission today
  would show the vehicle parked at the tube entrance for all 54 s. The fix is visual proxy prims
  driven from the reconstructed global state — which does *not* touch the Phase 0
  `transform_writes_during_run: 0` invariant, because proxies are not the simulated bodies.
- **A trajectory-conditioned ignition trigger.** See DESIGN_REVIEW §10.4: the seven gates are all
  safety gates, so the rocket ignites 0.56 s after release rather than coasting to altitude, and
  ignition timing is a declared exploration axis that is not yet supported.
- Delivered-state outputs (apogee altitude/time, downrange, flight-path angle) as first-class
  summary fields, so runs self-score instead of needing a script against raw telemetry; then a
  criterion policy gating on the §10.6 margin; then a sweep runner.
- Replacing the §10.6 loss allowance with a measured quantity. It moves the margin ±500 m/s and is
  currently a declared guess, which makes it the highest-value modelling work remaining.
- Remaining packaging assets (`premake5.lua`, `data/`) where required by the repository build.

### Phase 0 — evidence complete; non-constraint treatment accepted

Section 17's Phase 0 is a real prerequisite, not paperwork. The top-level
`artifacts/phase0/physx_runtime_and_force.json` and `newton_runtime_and_force.json` files are
historical schema-v1 measurements. They establish the early force discrepancy and API availability
but do not select a mechanism. Schema v2 now places only the cart prismatic joint in the guide
articulation and authors the cart/rocket fixed joint with `excludeFromArticulation=true`. The matched
final runs use SHA-256 `9026842cbe7a3ec062e6dcaca2360f51fd27f3d06c1610a3bbe6abe508fcef55`.
CPU PhysX passes runtime, force, inclined guide, incoming reaction, solver release, always-present
nonzero-force contact, and complete stop/rebuild reset. Its live-activation control proves release
but passes the shapes through without force. Newton produces repeated `Adjoint.return_var` step
errors for the external-joint topology, leaves the independent force body motionless, and still
lacks incoming reaction reporting. CPU PhysX/CPU/1 ms/TGS with the always-present pair is the
selection. The production anti-tunneling artifacts use runner SHA-256
`2f900a390ed4be5cd8f43b27e22e74eb85d82838b796f7a56dfb969a8fe54665` and fixture SHA-256
`7004d7df2bca9f91f7cab07a3c303511bb2bd381041ba00784d3d1acae5a64ec`. A 4.0 m by 0.5 m X-axis
cylindrical rocket impacts the rear wall of an open-front U-shaped cradle at 2.5 km/s. Every
discrete 1.0/0.5/0.25 ms run reports seven physical contacts before traversal; impulses are
261.32/262.37/270.90 kN s. The 1 ms CCD result is sample-identical to discrete, so CCD is
unnecessary for this production geometry's no-pass-through outcome. The curved-guide refinement
artifacts use runner SHA-256
`76000ec00846eee33953d35e595f12befac1e683c9a96861002470ab864f4f9f` and project-source closure
SHA-256 `5d32212ac2d979b95f3c81e07e685bf5088bf14bf2e1bf82834d23c6a8765897`. This is historical v0.29
evidence; v0.31 source changes invalidate it until the complete matrix is regenerated. The force-resolved candidate
uses a translated accelerating solver frame because direct 25 km float32 coordinates fail the
tracking invariant; it reconstructs global SI state, applies exact per-body fictitious forces, and
writes no transforms during integration. All 1.0/0.5/0.25 ms full profiles pass. Adjacent
exit-speed changes are 2.15e-9 and 1.13e-8 relative; peak assembly-load changes are 0.0000048G and
0.0000094G; tracking is 246.3/285.3/338.0 micrometres; and backend-force error converges
0.0614%/0.0307%/0.0154%. Historical throughput was 256.78–267.21 physics steps/s; no replacement
range is accepted until an unloaded controlled performance run follows correctness requalification. Schema-v3 evidence validation
now rejects every backend/device condition except the Phase 0 selected PhysX/CPU target, and the
checked-in curved configuration pins that target. The native path constraint is unavailable, the
joint chain is rejected for requiring unqualified repeated topology transfer, and the kinematic
candidate is visualization-only. The controller still lacks solver reaction read-back; v0.29
accepts commanded and backend-reconstructed load for system-level production simulation only.
Hardware contact-load validation remains blocked pending a constraint-capable mechanism.

---

## 4. How to run things

```powershell
cd C:\Dev\Isaacsim\IsaacSim\_build\windows-x86_64\release

# unit suite (pure Python, no Kit)
.\python.bat -m unittest discover -s C:\Dev\Isaacsim\skyArc\tests\unit -v

# production mission on the target build (bounded)
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\run_mission.py --headless --max-steps 5000 --reset-replay

# Kit integration suite
.\kit\kit.exe .\apps\isaacsim.exp.full.kit --no-window `
  --ext-folder C:\Dev\Isaacsim\skyArc\exts `
  --enable skyarc --enable omni.kit.test `
  --/exts/omni.kit.test/runTestsAndQuit=true `
  --/exts/omni.kit.test/includeTests/0='skyarc*'
```

`python` on PATH is the Windows Store stub and does not work. Always use `python.bat`.

The Kit runner prints its `Ran N tests` summary and then does not exit on this app configuration;
read the result from stdout and stop the process. Do not read a hung process as a failing suite.

**Every change to a `.py` file under the package root invalidates the Phase 0 curved-guide
evidence.** `qualify_curved_guide.py` records a conservative project-source closure over the whole
backend-neutral package, and `tests/unit/test_phase0_runner.py` recomputes it and demands equality.
The three refinement artifacts must therefore be regenerated after any core edit, using the commands
in `standalone/README.md`. That is a deliberate property of the conservative closure, not a bug: it
guarantees no bound result can outlive the code that produced it. Budget for it — at the throughput
this machine currently sustains the three runs take about three hours.

---

## 5. Immediate next step

**Requalify first.** The source closure has moved several times since the curved-guide artifacts
were last written, most recently for `feasibility.py`, the `stage2_constraint` schema field, the
cart-following frame and the §13.4 scene changes. The reference configuration's own hash moved too,
when the `stage2_constraint` block was added. Regenerate the 1.0/0.5/0.25 ms matrix, the scene
smoke and both mission artifacts in one pass —
`scratchpad/requalify_all.ps1` does this and fails loudly if the closure moves mid-run, which is
how two earlier passes were silently invalidated. Only after correctness closure matches should an
unloaded run be used to update throughput; the historical 256.78–267.21 steps/s range is not a
measurement of this revision.

Then, in objective order rather than interest order:

1. **A trajectory-conditioned ignition trigger** (§10.4). Ignition timing is a declared exploration
   axis and today the rocket lights 0.56 s after release. Keep the seven safety gates as
   preconditions and add a separate *when*.
2. **Delivered-state outputs and a margin criterion**, so a run self-scores rather than needing a
   script against raw telemetry, followed by a sweep runner. The manifest, criteria, named-stream
   and paired-contrast machinery all exist; nothing drives them.
3. **Replace the §10.6 loss allowance with a measured quantity.** It moves the answer ±500 m/s.
4. **`visualization/`** — but note the animation blocker above: proxy prims first, or an animated
   render will show the vehicle parked at the entrance. The scene-construction runner intentionally
   reports `mission_execution: not_started_scene_construction_slice`; do not turn that smoke test
   into an alternate mission loop, and do not let the panel drive physics directly — it observes the
   orchestrator.
5. **Reduce the per-step cost.** See the throughput nit in section 2.5; the orchestrator reads state
   three times per step and the adapter reads it a fourth.

The 54.116 km tube remains a schema-v3 reference rather than the schema-v2 default.

Latest verified curved clearance certificate: **0.317531 m cart wall clearance** and **0.699967 m
rocket wall clearance** after the 0.05 m guide allowance. The cart is limiting. The nonlocal tube
gate requires 2.0 m centerline separation and uses at most 24.487 m polyline spacing with a 4.997 mm
conservative chord-error bound. These figures certify geometry only, not physical guide behavior.

Latest verified baseline preflight values: **34.1348 m braking distance**, **0.8652 m remaining
track margin**, **62.5 m minimum launch distance**, **83.0 m conservative available launch
distance** after reserving both configured ramp regions. Resolved config SHA-256 at this revision:
`665ab38ec8678d32d75e9fd7db1d76b0f17dad624c402480b26c3db4004ad7c7` (unchanged by the section 2.1
fixes). The braking figure reproduces the section 6.8 narrative exactly, including its warning that
the 35 m example passes with under a metre of headroom.

When adding a preflight rule, probe it with a hostile configuration rather than trusting a reading
of the code. All four defects in section 2.1 were invisible to inspection and to a passing test
suite; three of them were comparisons that a NaN or an unpinned `auto` walks straight through.
