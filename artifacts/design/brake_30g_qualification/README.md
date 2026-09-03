# 30 G induction-plate candidate qualification

The brake-focused candidate passed the available CPU/PhysX qualification checks on
2026-09-03. The committed 10 G reference configuration remains unchanged; this evidence
belongs to `configs/curved_2kms_brake_30g_candidate.yaml` and
`configs/brake_30g_induction_candidate_fixture.json`.

## Result

| Check | Result |
|---|---:|
| Static configuration and fixture consistency | pass |
| Anti-tunneling fixture matrix (1/0.5/0.25 ms discrete, 1 ms CCD) | 4/4 pass |
| Curved-guide refinement matrix (1/0.5/0.25 ms) | 3/3 pass |
| 65 s production brake-stop probe | pass |
| Cart stop time | 61.999 s mission time |
| Cart exit-track progress at stop | 8,000.000 m |
| Remaining physical track | 2,000.000 m |
| Final cart speed at 65 s | 1.822e-6 m/s |
| Peak resultant load | 26.034 G (30.01 G limit) |
| Peak commanded brake force | 11.222 kN (12.941 kN limit) |
| Exit speed | 1,999.949 m/s |

The conservative preflight predicts 7,702.105 m of stopping travel and requires
9,702.105 m including the 2 km declared margin. The live controller deliberately uses
the available 8 km active region and stops at its boundary, leaving the exact 2 km
physical margin.

The three curved-guide jobs ran concurrently from 14:30:06Z to 14:45:58Z, or 15.9
minutes wall time. The 65,000-step production probe ran at 39.82 physics steps per wall
second and used 1,632.2 seconds in the physics loop. GPU compute utilization remained
0%; the qualified runtime is explicitly PhysX/CPU.

## Evidence boundary

`mission_brake_stop_65s.json` is intentionally bounded after the cart stopped. It has
`completed_mission: false` and `termination_reason: step_budget_exhausted`; it does not
claim completion of the rocket's remaining free-flight tail. Its `passed: true` means
the production runtime, release, separation, braking, and non-abort gates passed through
65 s.

The telemetry energy closure is formally invalid because the recorder cannot close the
cart's residual rotational kinetic energy without modeled body inertia. The reported
angular rate is 5.238e-6 rad/s. This is an evidence-accounting limitation, not a brake
stop failure, and must remain disclosed before the candidate replaces the reference.

The retained event stream is `events.jsonl`. Its SHA-256 is
`dae58dea74bdcc8a0cfd28d60d404f8fdaf040472431e4aed299c35bba9ca318`.
The complete scratch telemetry CSV is not part of this artifact directory; its SHA-256
at run completion was
`fa7e69f484f1e2dede8eb6cf0d838fad28e874c9ab90c87c107b950f88cdb81a`.
