# Crawler-derived cart slab

`Crawler_Slab.usdc` is a visual-only deck extracted from NASA's
[Crawler](https://science.nasa.gov/3d-resources/crawler/) model, credited to
NASA/Michael D. Carbajal. The source file has SHA-256
`655f9adff798c18d8f6bdd467f30e055094e1fc817b2abea52310b2dc6df4ff2`.
NASA's 3D Resources hub identifies its assets as free to download and use under the
[NASA Images and Media Usage Guidelines](https://www.nasa.gov/nasa-brand-center/images-and-media/).
The guidelines expressly cover polygon and texture data used to render 3D models.
Redistribution is therefore marked `cleared`, subject to acknowledging NASA as the
source, avoiding any implication of NASA endorsement, and respecting NASA's protected
identifiers and brand rules.

The extraction selects the largest coordinate-connected component of the
`initialShadingGroup2` material subset (668 faces and 472 source points), converts
the source from Y-up to Z-up, centres it, and tapers the forward seventh. At runtime
the result is fitted to the configured slab envelope. It is only a render skin: the
hidden analytic slab and the authored saddle pads remain authoritative for collision,
mass, inertia, and contact behavior.

To regenerate the asset, first decompress and weld the source without modifying the
original provenance file:

```powershell
npx @gltf-transform/cli copy Crawler.glb Crawler_Decompressed.glb --vertex-layout separate
npx @gltf-transform/cli weld Crawler_Decompressed.glb Crawler_Welded.glb
```

Then send `standalone/prepare_crawler_slab_remote.py` to a running Isaac Sim instance,
injecting absolute paths for `source_path`, `conversion_source_path` (the welded GLB),
and this `output_dir`. The script validates the selected face count, writes the USD and
manifest, and rejects any generated physics API.
