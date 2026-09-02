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

.\python.bat C:\Dev\Isaacsim\skyArc\standalone\qualify_phase0.py --backend physx --collision-treatment always_present
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\qualify_phase0.py --backend physx --collision-treatment live_activation
.\python.bat C:\Dev\Isaacsim\skyArc\standalone\qualify_phase0.py --backend newton --collision-treatment always_present

# Named 2,500 m/s rocket/cradle anti-tunneling matrix on selected CPU PhysX
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
`configs/phase0_anti_tunneling_open_cradle.json`: the reference 4.0 m by 0.5 m X-axis cylindrical
rocket starts with 0.01 m clearance inside a compound open-front U-shaped cradle and impacts the
rear wall at 2,500 m/s. A pass requires nonzero finite contact force and no full wall traversal. The
matched matrix uses runner SHA-256
`2f900a390ed4be5cd8f43b27e22e74eb85d82838b796f7a56dfb969a8fe54665` and fixture SHA-256
`7004d7df2bca9f91f7cab07a3c303511bb2bd381041ba00784d3d1acae5a64ec`. All four conditions pass
with seven contacts and no traversal; discrete 1.0/0.5/0.25 ms reported impulses are
261.32/262.37/270.90 kN s, and the 1 ms CCD result is sample-for-sample identical to discrete. CCD
is not required for this named production geometry's no-pass-through outcome. This conservative
rear-wall collision remains a mechanism stress test, not an attachment-load prediction.

`qualify_curved_guide.py` records the force-resolved curved mechanism under
`artifacts/phase0/curved_guide/`. Direct global coordinates are a rejected control: float32 pose
quantization at 25 km creates normal drift inconsistent with the velocity tensor. The accepted run
uses a translated accelerating solver frame, reconstructs global SI position/velocity/acceleration,
applies the exact uniform fictitious force to each body, and performs no transform writes during
integration. The physical guide/launch resultant remains cart-owned so fixed-joint attachment load
is measured rather than bypassed.

The following exact-production-geometry refinement figures are historical v0.29 evidence. The
v0.31 source closure has changed, so they are not current qualification until all three artifacts
are regenerated. The historical series uses runner SHA-256
`76000ec00846eee33953d35e595f12befac1e683c9a96861002470ab864f4f9f` and project-source closure
SHA-256 `5d32212ac2d979b95f3c81e07e685bf5088bf14bf2e1bf82834d23c6a8765897`. It uses the same fixture
hash as the anti-tunneling matrix and reads 1,356.4024 kg m² combined pitch inertia from PhysX. The
1 ms run passes the four gates it evaluates: 2,000.001708 m/s exit speed, 246.3 micrometre peak
centerline error, 2.424 micrometre peak attachment-spacing error, 6.826264G inferred assembly load,
0.04394% gated controller-feedback correction, 0.0614% maximum backend-force error,
solver-confirmed release, and cold reset. The 0.5 and 0.25 ms profiles also pass. Their adjacent
exit-speed changes are 2.15e-9 and 1.13e-8 relative, while peak assembly-load changes are
0.0000048G and 0.0000094G. Tracking remains bounded at 246.3/285.3/338.0 micrometres, and
backend-force error converges from 0.0614% to 0.0307% to 0.0154%. Those runs sustained 256.78–267.21
physics steps/s; do not replace that range from a loaded/background requalification. The matched
global-frame control fails the tracking, attachment, attitude, and
load gates while using the same runner and source closure.

The feedback correction is not solver constraint-reaction read-back. Under v0.29 the panel accepts
the force-resolved controller, translated accelerating frame, and measured throughput for
system-level production simulation only. Hardware contact-load validation still requires a new
constraint mechanism.

`run_launcher.py` proves scene construction — extension startup, PhysX/CPU selection, exact scene
construction, view warm-up, and optional USD export — and reports
`mission_execution: not_started_scene_construction_slice`. It must stay that way: an alternate
mission loop in that runner is exactly the drift the shared `build_mission` factory exists to
prevent.

`run_mission.py` is the runner that executes a mission, and it does so by handing the production
adapter to that same factory. It writes `artifacts/production/mission_smoke.json` and
`mission_telemetry.json`, both bound by `tests/unit/test_production_mission.py` to the complete
production source closure, runner/configuration/fixture hashes, and Isaac build identity. The
current v0.31 bounded 5,000-step record preserves `launch_stage` and three run events across reset
replay, reports the true 184.788 m/s vector speed, holds 1.02 micrometre peak centerline tracking
error and 0.00035 degree peak attitude error while following the commanded 36.9577 m/s² profile,
and restores the authored state within 43 nanometres. Guided throughput is 37–38 physics steps per
wall second, an order of magnitude below the historical qualification runner's, because
each step also runs the full component stack and the orchestrator's state reads.

The 45-degree entrance makes the launcher's engagement self-checking: with no launcher force the
assembly does not merely accelerate slowly, it rolls backwards at `-g sin 45`. Both the bounded
artifact and the Kit integration suite assert the measured speed against the commanded profile
rather than merely asserting that it is positive.
