# 30 G induction-plate candidate qualification

The brake-focused candidate passed the available CPU/PhysX qualification checks on
2026-09-03. The 10 G reference brake settings remain available as a control; both scenarios
now use the same full-length 4.2 m by 1.2 m by 1.4 m open U-cradle. This evidence belongs to
`configs/curved_2kms_brake_30g_candidate.yaml` and
`configs/brake_30g_induction_candidate_fixture.json`.

## Result

| Check | Result |
|---|---:|
| Static configuration and fixture consistency | pass |
| Anti-tunneling fixture matrix (1/0.5/0.25 ms discrete, 1 ms CCD) | 4/4 pass |
| Curved-guide refinement matrix (1/0.5/0.25 ms) | 3/3 pass |
| 65 s production brake-stop probe | pass |
| Cart stop time | 61.996 s mission time |
| Cart exit-track progress at stop | 8,000.000 m |
| Remaining physical track | 2,000.000 m |
| Final cart speed at 65 s | 3.533e-6 m/s |
| Peak resultant load | 26.044 G (30.01 G limit) |
| Peak commanded brake force | 11.226 kN (12.941 kN limit) |
| Exit speed | 1,999.963 m/s |
| Separation contact impulse | 0.0 N s |

The conservative preflight predicts 7,702.105 m of stopping travel and requires
9,702.105 m including the 2 km declared margin. The live controller deliberately uses
the available 8 km active region and stops at its boundary, leaving the exact 2 km
physical margin.

The curved-guide jobs were run in matched baseline/candidate pairs at 1/0.5/0.25 ms.
The 65,000-step production probe ran at 33.45 physics steps per wall second and used
1,943.1 seconds in the physics loop while timestep refinements ran concurrently. GPU
compute is not used by this qualified runtime; it is explicitly PhysX/CPU. RTX was used
for the independently replayed GIF renders.

## Evidence boundary

`mission_brake_stop_65s.json` is intentionally bounded after the cart stopped. It has
`completed_mission: false` and `termination_reason: step_budget_exhausted`; it does not
claim completion of the rocket's remaining free-flight tail. Its `passed: true` means
the production runtime, release, separation, braking, and non-abort gates passed through
65 s.

The telemetry energy closure is formally invalid because the recorder cannot close the
cart's residual rotational kinetic energy without modeled body inertia. The reported
angular rate is 3.299e-5 rad/s. This is an evidence-accounting limitation, not a brake
stop failure, and must remain disclosed before the candidate replaces the reference.

The retained event stream is `events.jsonl`. Its SHA-256 is
`40df890e08d560e0780a812f6c93336241bb2fcf81e569c7014b23b34e559e1d`.
The complete scratch telemetry CSV is not part of this artifact directory; its SHA-256
at run completion was
`d971dae2cc6d72878e25068b381d8a502b873129390368af43cfb630bcfe9781`.
