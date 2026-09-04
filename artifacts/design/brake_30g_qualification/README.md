# 30 G induction-plate candidate qualification

The brake-focused candidate passed the available CPU/PhysX qualification checks on
2026-09-03 using the former full-length open U-cradle. The 10 G reference brake settings remain
available as a control, but both current scenarios now use the tapered slab and three-saddle
cart introduced in v0.37. This directory is therefore historical evidence and must be regenerated
before it qualifies the current configuration. It was produced from the earlier revisions of
`configs/curved_2kms_brake_30g_candidate.yaml` and its now-retired candidate fixture.

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

The historical 1.0/0.5/0.25 ms curved-guide artifacts are bound to project-source closure
`991ef45fd3bec0a16382707e8314bd572121e6c5aafeff6141648448920747ee` and all pass. The
65,000-step production probe also uses that closure and runner SHA-256
`03558bece7fe666fee02433a95271b5fafc198080461d21de2561b180143744e`.
Its recorded 26.12 physics steps/s is not performance evidence because a separate user-owned
Isaac session was active. GPU compute is not used by the qualified physics runtime; it is
explicitly PhysX/CPU. RTX was used for the independently replayed GIF renders.

## Evidence boundary

`mission_brake_stop_65s.json` is intentionally bounded after the cart stopped. It has
`completed_mission: false` and `termination_reason: step_budget_exhausted`; it does not
claim completion of the rocket's remaining free-flight tail. Its execution `passed: true`
means the production runtime, release, separation, braking, and non-abort gates passed
through 65 s. Its separate `curved_reference_v1` criterion is false: apogee at the bound is
35,898.976 m, below the 40 km trajectory trigger, so ignition, handoff, and stage-2 margin
are correctly absent. A trajectory-triggered run has no ignition/burnout deadline before a
measured ignition event; schema-v2 safety-only triggering retains the original post-exit deadline.

The telemetry energy closure is formally invalid because the recorder cannot close the
cart's residual rotational kinetic energy without modeled body inertia. The reported
angular rate is 3.299e-5 rad/s. This is an evidence-accounting limitation, not a brake
stop failure, and must remain disclosed before the candidate replaces the reference.

The retained event stream is `events.jsonl`. Its SHA-256 is
`1ff29446d5dd0a61871fb227f9108e2010e9276f35e92b7f064b60c4fec16f89`.
The complete telemetry run is retained beneath this directory; its CSV SHA-256 is
`342c19787c46e7b03b765e591376509fccd0cf48361b19d800f87ed9e8b6350c` and its manifest
SHA-256 is `9fa2a613ee3615abdc4878f037f4aef0c323b3228847d83b5e9803cbe340427b`.
