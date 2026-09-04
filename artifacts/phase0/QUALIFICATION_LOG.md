# Qualification run log

## 2026-09-04 — slab-and-saddle cart architecture

- Replaced the production cart's full-height side and rear walls with a 0.10 m tapered slab and
  three discrete two-pad saddle stations at axial positions -1.5, 0.0, and 1.5 m.
- The current target-build scene smoke passes on CPU PhysX/TGS and its USD contains the slab and
  six saddle pads, with no `LeftRail`, `RightRail`, or `RearWall` prims. The complete Kit boundary
  audit passes 11/11, including 200 guided steps, reset, release, and momentum conservation.
- The new 100 m/s vertical rocket/saddle-system fixture passes all four named cases: discrete contact at
  1.0, 0.5, and 0.25 ms, plus the 1 ms CCD control. Every case reports an 18-contact peak and no
  complete pad traversal; solver momentum impulse ranges from 14.7610 to 14.9591 kN s.
- The 100 m/s gate exceeds the 1.25-margin braking-relative minimum for the baseline
  (44.88 m/s) and 30 G candidate (77.90 m/s); it is not based on the 2 km/s common-mode
  launch velocity. At release the rocket and cart initially co-move.
- Evidence identity: runner SHA-256
  `bc69c18fab9fe76853bee157cc75ca5e9dc69924bc296650c99e4b401f5fc313`; fixture SHA-256
  `39079a26ea82c6da9a1d49314b016655c0736d3ca8a18720f935cc633e7e5f5b`.
- The older curved-guide, complete-mission, mission-sweep, and 30 G candidate artifacts remain
  useful historical results but are source-stale after this production geometry/configuration
  change. They were not represented as requalified by this short architecture pass.

## 2026-09-03 — current-source curved-guide closure

- Venue: local laptop; no cloud, Compute Lab, Kubernetes, or remote resources used.
- Host: Intel Core Ultra 9 285H, 16 cores, 64 GB Windows memory.
- GPU discovered: NVIDIA RTX PRO 3000 Blackwell Generation Laptop GPU, 12 GB, driver 596.53.
- Physics condition: accepted PhysX/CPU/TGS condition. The GPU was not used for physics; live
  telemetry showed 0% GPU utilization while the headless apps reserved renderer/runtime memory.
- Geometry correction: the 4.0 m rocket is centered inside a 4.2 m long, 1.2 m wide,
  1.4 m deep open U-cradle. It has 0.01 m rear clearance, 0.09 m nose margin, and
  0.35 m floor clearance. Both caps are inside the cart envelope at release.
- Execution: the 1.0, 0.5, and 0.25 ms curved-guide refinements ran as staggered
  baseline/candidate pairs in isolated Kit processes from `2026-09-03T18:34:01Z` to
  `2026-09-03T19:08:07Z` while the production mission ran concurrently.
- Artifacts: `curved_guide/physx_cpu_1ms_full_profile_v2.json`,
  `curved_guide/physx_cpu_0p5ms_full_profile.json`, and
  `curved_guide/physx_cpu_0p25ms_full_profile.json`.
- Result: all six baseline/candidate timestep artifacts report `passed: true` and bind
  source closure `3ca6a97000dd3d195c6d2ff26f8c40ea3ec2aa86eaf5154ab4eac2d98a0fab8b`.
  The matched global-coordinate control reports `passed: false` as required.
- Performance note: per-artifact throughput is contended and is not an authoritative performance
  measurement. Use an unloaded sequential run to update the accepted throughput range.
- Follow-on closure: baseline and candidate anti-tunneling matrices, production mission
  smoke/reset, and telemetry artifacts were regenerated against the corrected geometry;
  the complete unit suite passes 200/200.
- Cleanup: all local Isaac/Kit processes exited; no allocations, jobs, pods, labels, ports,
  credentials, or remote resources were created or left behind.

## 2026-09-03 — 30 G induction-plate brake candidate

- Venue and hardware: same local laptop and accepted PhysX/CPU/TGS runtime above; GPU compute
  remained at 0% during the CPU qualification work.
- Candidate: 43.986 kg induction-plate cart, 12.941 kN brake ceiling, 300 m/s³ jerk limit,
  30.01 G resultant-load bound, and 10 km exit track. The launch-force ceiling was rebased to
  11.8 kN so the lighter attached assembly remains inside the unchanged 10 G launch envelope.
- Anti-tunneling: 4/4 candidate-fixture cases passed (1/0.5/0.25 ms discrete and 1 ms CCD).
- Curved guide: 3/3 candidate timestep cases passed at 1/0.5/0.25 ms.
- Production brake probe: 65,000 fixed steps passed at 33.45 steps/s while refinement
  jobs ran concurrently. Release was confirmed at 54.118 s; separation and ignition
  occurred at 54.416 s with 0.0 N s contact impulse. `cart_stopped` occurred at mission
  time 61.996 s and exit-track progress 8,000.000 m, leaving 2,000.000 m of track. Final
  cart speed was 3.533e-6 m/s and peak resultant load was 26.044 G.
- Evidence boundary: the probe deliberately ended after cart rest. It does not claim completion
  of the rocket's remaining free-flight tail. Telemetry energy closure is formally invalid due
  to unmodeled inertia for a residual 3.299e-5 rad/s cart angular rate; this remains disclosed in
  `artifacts/design/brake_30g_qualification/README.md`.
- Resources: all Kit processes exited; no cloud or remote resources were created.
