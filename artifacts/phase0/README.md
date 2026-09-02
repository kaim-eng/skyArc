# Phase 0 qualification artifacts

The two JSON files in this directory root use
`vacuum_tube_phase0_qualification_v1`. They are retained as historical measurements and must not be
treated as current release/contact or reset qualification evidence: the v1 harness checked the
authored joint flag without a solver-side motion discriminator and recreated only rigid-body views
during reset.

The corrected runner emits `vacuum_tube_phase0_qualification_v2` to unique paths under
`physx/` and `newton/`. A v2 artifact is admissible for review only when its recorded runner hash
matches the reviewed `standalone/qualify_phase0.py`, its provenance hashes identify the tested Kit
build, and the required probe has `passed: true`. Do not copy v1 measurements into v2 artifacts;
rerun the exact backend process instead.

The current admissible reruns are:

- `physx/20260831T175442.203024Z_f628bf4b445a4fadbf72fcd23c993a8d.json`
- `newton/20260831T175819.981686Z_214aa7b79add4659b563fe86776b4765.json`

Both record runner SHA-256
`ad851fafce907a594f6d8a06bf2f3d17ec82795e7b121afd4eda64ee01245b36`. Both pass runtime
selection, force, inclined-guide, and full stop/rebuild reset probes. PhysX passes joint-reaction
reporting; Newton reports it unsupported. Both fail solver-confirmed release because the first
separating-effort step produces zero relative motion after the authored joint flag changes. Contact
therefore remains unqualified rather than being attributed solely to live collision activation.
