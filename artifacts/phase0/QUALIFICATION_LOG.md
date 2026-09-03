# Qualification run log

## 2026-09-03 — current-source curved-guide closure

- Venue: local laptop; no cloud, Compute Lab, Kubernetes, or remote resources used.
- Host: Intel Core Ultra 9 285H, 16 cores, 64 GB Windows memory.
- GPU discovered: NVIDIA RTX PRO 3000 Blackwell Generation Laptop GPU, 12 GB, driver 596.53.
- Physics condition: accepted PhysX/CPU/TGS condition. The GPU was not used for physics; live
  telemetry showed 0% GPU utilization while the headless apps reserved renderer/runtime memory.
- Execution: the 1.0, 0.5, and 0.25 ms curved-guide refinements ran concurrently in isolated Kit
  processes. All three started at `2026-09-03T13:47:13Z`; the final process finished at
  `2026-09-03T14:04:55Z`, for approximately 17.7 minutes elapsed.
- Artifacts: `curved_guide/physx_cpu_1ms_full_profile_v2.json`,
  `curved_guide/physx_cpu_0p5ms_full_profile.json`, and
  `curved_guide/physx_cpu_0p25ms_full_profile.json`.
- Result: all artifacts report `passed: true`, bind source closure
  `069a07c621030475100b3fbcdf9cd3bd14260c0f8d1f73741817ce296a73f357`, and pass the 18-test
  Phase 0 contract/convergence suite.
- Performance note: per-artifact throughput is contended and is not an authoritative performance
  measurement. Use an unloaded sequential run to update the accepted throughput range.
- Follow-on closure: production mission smoke and telemetry artifacts were regenerated
  sequentially; after adding the brake-candidate contracts, the complete unit suite passes
  200/200.
- Cleanup: all local Isaac/Kit processes exited; no allocations, jobs, pods, labels, ports,
  credentials, or remote resources were created or left behind.

## 2026-09-03 — 30 G induction-plate brake candidate

- Venue and hardware: same local laptop and accepted PhysX/CPU/TGS runtime above; GPU compute
  remained at 0% during the CPU qualification work.
- Candidate: 43.986 kg induction-plate cart, 12.941 kN brake ceiling, 300 m/s³ jerk limit,
  30.01 G resultant-load bound, and 10 km exit track. The launch-force ceiling was rebased to
  11.8 kN so the lighter attached assembly remains inside the unchanged 10 G launch envelope.
- Anti-tunneling: 4/4 candidate-fixture cases passed (1/0.5/0.25 ms discrete and 1 ms CCD).
- Curved guide: 3/3 candidate timestep cases passed. The parallel matrix ran from
  `2026-09-03T14:30:06Z` to `2026-09-03T14:45:58Z`, approximately 15.9 minutes elapsed.
- Production brake probe: 65,000 fixed steps passed at 39.82 steps/s. `cart_stopped` occurred at
  mission time 61.999 s and exit-track progress 8,000.000 m, leaving 2,000.000 m of track. Final
  cart speed was 1.822e-6 m/s and peak resultant load was 26.034 G.
- Evidence boundary: the probe deliberately ended after cart rest. It does not claim completion
  of the rocket's remaining free-flight tail. Telemetry energy closure is formally invalid due
  to unmodeled inertia for a residual 5.238e-6 rad/s cart angular rate; this remains disclosed in
  `artifacts/design/brake_30g_qualification/README.md`.
- Resources: all Kit processes exited; no cloud or remote resources were created.
