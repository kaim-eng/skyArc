# skyArc brake design trade study

This is an analytic design screen, not qualification evidence. The qualified mission
configuration is unchanged.

## Decision

Select **induction plate**, **30 G**, and **300 m/s³** as the next design point to validate.

| Quantity | Current configured cart | Selected design point |
|---|---:|---:|
| Cart mass | 250.00 kg | 43.99 kg |
| Active stopping track | 22.42 km | 7.77 km |
| Total track with margin | 24.42 km | 9.77 km |
| Peak brake force | 24.41 kN | 12.94 kN |
| Peak brake power | 48.82 MW | 25.88 MW |
| Recoverable cart energy | 500.00 MJ | 87.97 MJ |
| Grid return at 90% | 450.00 MJ | 79.17 MJ |
| Payload energy fraction | n/a | 77.3% |

The selected point is the lowest-G, lowest-jerk conventional induction-plate case
that meets the **10 km** total-track target.
It keeps all active equipment track-side and avoids both a moving cryostat and the
less-certain skin-depth-thinned plate assumption.

## Architecture frontier

| Architecture | Meets target | G | Jerk (m/s³) | Cart (kg) | Total track (km) | Peak power (MW) | Recoverable energy (MJ) |
|---|:---:|---:|---:|---:|---:|---:|---:|
| permanent magnet | no | 15 | 1000 | 106.49 | 15.74 | 31.33 | 212.98 |
| induction plate | yes | 30 | 300 | 43.99 | 9.77 | 25.88 | 87.97 |
| superconducting | yes | 30 | 300 | 102.17 | 9.77 | 60.12 | 204.35 |
| thin plate | yes | 30 | 300 | 35.89 | 9.77 | 21.11 | 71.77 |

## Limits before changing the mission

- The force-density, guide-load and structural coefficients are order-of-magnitude estimates.
- The stopping calculation gives no credit for the actual 15-degree uphill grade or aerodynamic drag.
- Peak power is the instantaneous mechanical power at brake entry; power electronics, sectioning, thermal paths and grid acceptance are not modeled.
- Thirty-G operation is applied only to the empty cart after release. It still needs a structural and guide-load validation before becoming a mission requirement.
- Regeneration is mandatory. The reported return assumes the declared efficiency; it is not a measured electrical result.

Full machine-readable sweep: `240` cases in `artifacts/design/brake_design_sweep.json`.

## Candidate qualification status

The selected point passed the available candidate checks on the former full-length cradle.
Those long-duration candidate records are now source-stale because both configurations use
the tapered slab and three-saddle cart introduced in v0.37:

- 4/4 anti-tunneling fixture cases passed;
- 3/3 curved-guide timestep cases passed;
- the production brake-stop probe stopped at 8.000 km with 2.000 km of track remaining;
- peak measured resultant load was 26.044 G against the 30.01 G candidate bound;
- the fully seated rocket separated with 0.0 N s measured contact impulse before ignition.

The production probe was deliberately stopped after cart rest, so this is brake-focused
qualification rather than a claim that the rocket's full free-flight tail completed.
The candidate fixture and brake-stop probe must be regenerated before these results can be
treated as qualification of the slab-and-saddle production geometry.
See `artifacts/design/brake_30g_qualification/README.md` for the evidence boundary and
the disclosed telemetry energy-closure limitation.
