# Phase 0 qualification runner

`qualify_phase0.py` is an evidence-producing probe for the exact Isaac Sim build. It is not the
production launcher or backend adapter. Run each backend in a separate process so extension startup,
engine selection, tensor state, and settings cannot leak between conditions.

The runner writes schema-v2 artifacts to unique, backend-scoped paths under
`artifacts/phase0/<backend>/`. It refuses to replace an explicitly selected output unless
`--overwrite` is supplied. Every artifact records the Isaac Sim source revision plus SHA-256
identities for the runner, experience, version file, and Python/Kit executable; non-finite backend measurements are encoded as invalid
values rather than non-standard JSON constants.

```powershell
cd C:\Dev\Isaacsim\IsaacSim\_build\windows-x86_64\release

# Build and optionally save the approved production scene (mission execution is not started)
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\run_launcher.py --headless --save-usd C:\Dev\Isaacsim\skyArc\artifacts\production\scene_smoke.usda --summary C:\Dev\Isaacsim\skyArc\artifacts\production\scene_smoke.json

# Execute the common mission orchestrator on that scene. --max-steps bounds the run and
# marks the artifact `completed_mission: false`; omit it to run to COMPLETE or ABORT.
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\run_mission.py --headless --max-steps 5000 --reset-replay --summary C:\Dev\Isaacsim\skyArc\artifacts\production\mission_smoke.json
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\run_mission.py --headless --max-steps 1000 --telemetry-directory <scratch> --summary C:\Dev\Isaacsim\skyArc\artifacts\production\mission_telemetry.json

# materialize and preflight the paired ignition-altitude sweep without launching Kit
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\sweep_mission.py --dry-run

# execute the production sweep sequentially; each condition writes a complete manifest
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\sweep_mission.py

# rebuild sweep.json from completed summaries after an aggregation-only interruption
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\sweep_mission.py --aggregate-only

.\python.bat C:\Dev\Isaacsim\skyArc\standalone\qualify_phase0.py --backend physx --collision-treatment always_present
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\qualify_phase0.py --backend physx --collision-treatment live_activation
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\qualify_phase0.py --backend newton --collision-treatment always_present

# Named 100 m/s normal rocket/saddle anti-tunneling matrix on selected CPU PhysX
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\qualify_anti_tunneling.py --physics-dt-s 0.001 --ccd disabled
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\qualify_anti_tunneling.py --physics-dt-s 0.0005 --ccd disabled
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\qualify_anti_tunneling.py --physics-dt-s 0.00025 --ccd disabled
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\qualify_anti_tunneling.py --physics-dt-s 0.001 --ccd enabled

# Full force-resolved curved-guide profile on selected CPU PhysX
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\qualify_curved_guide.py --physics-dt-s 0.001 --normal-kp-per-s2 400 --normal-kd-per-s 40 --telemetry-stride 100 --output C:\Dev\Isaacsim\skyArc\artifacts\phase0\curved_guide\physx_cpu_1ms_full_profile_v2.json --overwrite
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\qualify_curved_guide.py --physics-dt-s 0.0005 --normal-kp-per-s2 400 --normal-kd-per-s 40 --telemetry-stride 100 --output C:\Dev\Isaacsim\skyArc\artifacts\phase0\curved_guide\physx_cpu_0p5ms_full_profile.json --overwrite
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\qualify_curved_guide.py --physics-dt-s 0.00025 --normal-kp-per-s2 400 --normal-kd-per-s 40 --telemetry-stride 100 --output C:\Dev\Isaacsim\skyArc\artifacts\phase0\curved_guide\physx_cpu_0p25ms_full_profile.json --overwrite

# Required negative control for the translated-frame treatment
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\qualify_curved_guide.py --physics-dt-s 0.001 --normal-kp-per-s2 400 --normal-kd-per-s 40 --telemetry-stride 100 --coordinate-frame global --output C:\Dev\Isaacsim\skyArc\artifacts\phase0\curved_guide\physx_cpu_1ms_global_control.json --overwrite

# Analytic brake architecture/G/jerk trade study. This does not run Isaac Sim or alter
# the qualified mission configuration.
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\sweep_brake_design.py
```

The current slice records and gates the resolved fixed-step runtime, checks world-frame tensor force
application against the semi-implicit analytic solution, and drives a fixed cart/rocket assembly
through a 45-degree prismatic guide. The guide is a fixed-base articulation with one internal
prismatic DOF; the axial command is generalized effort. The cart/rocket fixed joint is explicitly
excluded from that articulation so its live disable is a solver-consumed external constraint
mutation. The collision fixture begins with 0.01 m clearance, proves release with one separating
step, then applies a bounded compressive approach. A contact passes only when the view reports a
nonzero finite force; zero-force proximity manifolds inside the contact offset are recorded but do
not count as impact. The runner also probes incoming joint reactions and
stop/rebuild/full-view-recreation reset.

The two top-level `*_runtime_and_force.json` files and intermediate mechanism artifacts are
historical diagnostics. The matched final runs use runner SHA-256
`9026842cbe7a3ec062e6dcaca2360f51fd27f3d06c1610a3bbe6abe508fcef55`. CPU PhysX passes every probe
with `always_present`; its matched `live_activation` control proves release but lets the collision
shapes pass through without force. Newton cannot step the external-joint topology and still lacks
incoming joint reactions. CPU PhysX plus the always-present pair is therefore the selected condition
used by the curved-guide characterization below.

`qualify_anti_tunneling.py` writes separately hashed artifacts under
`artifacts/phase0/anti_tunneling/`. Its fixture is
`configs/phase0_anti_tunneling_slab_cradle.json`: the reference 4.0 m by 1.0 m X-axis cylindrical
rocket approaches all three saddle stations vertically at 100 m/s,
starting with 0.01 m clearance. This 100 m/s round-number gate exceeds the 1.25-margin
braking-relative minimum for both the baseline (44.88 m/s) and 30 G candidate (77.90 m/s); the rocket and
cart are co-moving near the Mach-7 exit, so 2,500 m/s is not their collision-pair speed. A pass
requires a contact manifold plus nonzero finite reported force or finite momentum change, and
no full pad traversal. The matched matrix uses runner SHA-256
`bc69c18fab9fe76853bee157cc75ca5e9dc69924bc296650c99e4b401f5fc313` and fixture SHA-256
`39079a26ea82c6da9a1d49314b016655c0736d3ca8a18720f935cc633e7e5f5b`. All four v0.37 conditions
pass without traversal: discrete 1.0/0.5/0.25 ms plus the 1 ms CCD control. CCD is not required
for this named production geometry's no-pass-through outcome. This vertical saddle-system case is a
mechanism stress test, not a prediction of the fixed-joint attachment load.

`qualify_curved_guide.py` records the force-resolved curved mechanism under
`artifacts/phase0/curved_guide/`. Direct global coordinates are a rejected control: float32 pose
quantization at 25 km creates normal drift inconsistent with the velocity tensor. The accepted run
uses a translated accelerating solver frame, reconstructs global SI position/velocity/acceleration,
applies the exact uniform fictitious force to each body, and performs no transform writes during
integration. The physical guide/launch resultant remains cart-owned so fixed-joint attachment load
is measured rather than bypassed.

The historical v0.35 exact-production-geometry baseline refinement series is bound to project-source closure
SHA-256 `991ef45fd3bec0a16382707e8314bd572121e6c5aafeff6141648448920747ee`.
All 1.0/0.5/0.25 ms co-moving profiles pass at 2,000.001139/2,000.001139/2,000.001140 m/s;
peak loads are 6.826258/6.826379/6.826402 G and peak centerline errors are
368.361/364.109/356.691 micrometres. Their unloaded physics loops sustain
276.738/269.117/268.850 steps/s. The matched 1 ms global-coordinate control fails as required with
1.8094 m peak tracking error and 19.9186 G peak load. The series uses the same production fixture
and reads the combined pitch inertia from PhysX; release and cold reset pass in every co-moving run.
The v0.37 cart and source changes invalidate those records; regenerate the co-moving series,
matched global control, and 30 G candidate before treating their pass or throughput values as current.

The feedback correction is not solver constraint-reaction read-back. Under v0.29 the panel accepts
the force-resolved controller, translated accelerating frame, and measured throughput for
system-level production simulation only. Hardware contact-load validation still requires a new
constraint mechanism.

`run_launcher.py` proves scene construction — extension startup, PhysX/CPU selection, exact scene
construction, view warm-up, and optional USD export — and reports
`mission_execution: not_started_scene_construction_slice`. It must stay that way: an alternate
mission loop in that runner is exactly the drift the shared `build_mission` factory exists to
prevent.

The development scene and live global proxy reference
`assets/vehicles/jupiter_c/Explorer_JupiterC_NoStage1.usdc` as a visual-only child. Its native Z-up
bounds are rotated onto +X and fitted to the conservative 4.0 m by 1.0 m cylinder; the cylinder
alone owns collision, mass, and inertia, and scene construction scans the composed reference for
physics APIs/properties. `scene_smoke.json` records SHA-256 identities for both the USD and manifest
in addition to the resolved visual prim path. The manifest records the official NASA source,
NASA/Michael D. Carbajal attribution, and NASA Images and Media Usage Guidelines. Source
redistribution is cleared under those terms; extension packaging remains withheld until an installed
package smoke is run, and development exports retain a nonportable absolute source reference.

`run_mission.py` is the runner that executes a mission, and it does so by handing the production
adapter to that same factory. It writes `artifacts/production/mission_smoke.json` and
`mission_telemetry.json`, both bound by `tests/unit/test_production_mission.py` to the complete
production source closure, runner/configuration/fixture hashes, and Isaac build identity. The
current bounded 5,000-step record preserves `launch_stage` and three run events across reset replay,
reports the true 184.788 m/s vector speed, holds 3.00 micrometre peak centerline tracking error while
following the commanded 36.9577 m/s² profile, and restores the authored state within its 10 micrometre
tolerance. Its measured throughput is not a controlled figure because another user-owned Isaac
session was active; use the unloaded qualification rates above for performance evidence.

The executed 35/40/43 km sweep is durable partial evidence. Every 10 G reference condition aborts at
step 54,389 with `separation_contact_impulse_exceeded`, before the altitude trigger can act. The
aggregate preserves null handoff/stage-2 metrics, declares `contrast_status:
partial_missing_metrics`, and reports an exactly zero contrast for the common 31,117.456 m apogee.
The 30 G candidate clears the same release with zero contact impulse and stops the cart at
8,000.000006 m, but its 65-second bound reaches only 35,898.976 m altitude, so the 40 km trigger and
stage-2 handoff remain outside that artifact's scope.

The 45-degree entrance makes the launcher's engagement self-checking: with no launcher force the
assembly does not merely accelerate slowly, it rolls backwards at `-g sin 45`. Both the bounded
artifact and the Kit integration suite assert the measured speed against the commanded profile
rather than merely asserting that it is positive.
