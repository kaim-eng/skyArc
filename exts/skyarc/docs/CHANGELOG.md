# Changelog

## Unreleased — 2026-09-04

- Replaced the continuous side rails and rear wall with a tapered low slab and three
  discrete two-pad saddle stations, eliminating full-height cart walls from the exposed
  Mach-7 vehicle geometry.
- Kept the rocket and cart origins coincident and transferred guided axial load through the
  always-present fixed coupling instead of a rear-wall seat.
- Rebased the rocket/cart anti-tunneling gate on braking-relative clearance speed and added
  a four-case normal rocket/saddle qualification matrix.
- Reduced the provisional cart frontal area from 0.50 to 0.30 m2. This is still a system
  coefficient assumption pending compressible-flow CFD or wind-tunnel calibration.

## 0.2.0 — 2026-09-01

- Ran the common backend-neutral mission orchestrator on the production scene through
  `IsaacPhysxBackend`. The step ordering, release transaction, ignition interlock and
  telemetry phases are the shared ones; nothing about the mission is re-implemented here.
- Added `launcher/path_controller.py`, the backend-neutral force-resolved guide reaction and
  its translated accelerating frame, so the accepted mechanism is unit-testable outside Kit.
- Added `launcher/production_runtime.py`, which owns the Isaac-side lifetime: rebindable body
  handles, the qualified rocket/cradle contact view, a resync that does not advance time, and
  the qualified stop/rebuild reset.
- The adapter now reports its commanded reaction through `AppliedEffects` under the
  `backend_adapter` slot, and Section 16.2 gained a matching `guide_reaction` work term.
- Fixed the adapter's tensor reads, which walked `shape` and then indexed; Warp arrays refuse
  item indexing, so every production read would have failed on the first step.
- Added Kit integration tests for the adapter boundary.

## 0.1.0 — 2026-09-01

- Added the selected PhysX/CPU production adapter.
- Added resolved-centerline tube bands, exit track, marker, lighting, open-front cradle,
  cylindrical rocket, and always-present fixed coupling scene authoring.
- Preserved the translated accelerating-frame and non-constraint reaction-evidence labels.
