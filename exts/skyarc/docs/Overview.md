# Vacuum Tube Launcher

This extension contains the selected production Isaac Sim layer for the schema-version-3
vacuum-tube launcher. It uses CPU PhysX, an always-present rocket/cradle collision pair,
and the approved force-resolved path controller in a translated accelerating solver frame.

The controller's normal load is commanded and independently reconstructed from backend
motion. It is not a solver-reported constraint reaction and must not be presented as
hardware contact-load validation. Because it is commanded rather than supplied by a
workless constraint, it does a small amount of work every step; that work is reported
under its own `guide_reaction` energy channel instead of being folded into guide
resistance or left in the residual.

There are two standalone entry points, and the difference between them is deliberate:

- `standalone/run_launcher.py` builds and optionally exports the scene and stops. It
  reports `mission_execution: not_started_scene_construction_slice` and must never grow an
  alternate mission loop.
- `standalone/run_mission.py` executes a mission. It does so by handing the production
  adapter to the same `build_mission` factory the analytic suite uses, so the step
  ordering, release transaction, interlock and telemetry phases are the shared ones.

Both create `SimulationApp` before enabling or importing any extension-dependent module.

The Isaac-facing modules are `extension.py`, `effects/backends/isaac.py`,
`launcher/scene.py` and `launcher/production_runtime.py`. Everything else in the package
imports without Isaac Sim, Omni, USD, Warp or NumPy, and
`tests/unit/test_production_mission.py` enforces that by parsing the imports.
