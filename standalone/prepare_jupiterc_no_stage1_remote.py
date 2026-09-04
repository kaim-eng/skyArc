# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Run the Jupiter-C asset preparation inside an existing Isaac Sim session."""

import json
from pathlib import Path

import omni.kit.asset_converter
from isaacsim.core.experimental.utils.app import enable_extension, update_app_async


for required_name in ("source_path", "helper_path", "output_dir"):
    if required_name not in dir():
        raise ValueError(
            f"{required_name} must be injected by isaacsim_send.py --args-json"
        )

HELPER = Path(helper_path).resolve()
OUTPUT_DIR = Path(output_dir).resolve()
SOURCE = Path(source_path).resolve()
CONVERSION_SOURCE = OUTPUT_DIR / "Explorer_JupiterC_ImportReady.glb"

if not SOURCE.is_file():
    raise FileNotFoundError(SOURCE)
if not CONVERSION_SOURCE.is_file():
    raise FileNotFoundError(CONVERSION_SOURCE)
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

helper_namespace = {
    "__file__": str(HELPER),
    "__name__": "prepare_jupiterc_no_stage1",
}
exec(compile(HELPER.read_text(encoding="utf-8"), str(HELPER), "exec"), helper_namespace)

enable_extension("omni.kit.asset_converter")
await update_app_async(steps=5)

context = omni.kit.asset_converter.AssetConverterContext()
context.ignore_materials = False
context.ignore_animations = True
context.ignore_camera = True
context.export_preview_surface = True
context.use_meter_as_world_unit = True
context.create_world_as_default_root_prim = True
context.disabling_instancing = True
context.convert_stage_up_z = True

full_usd = OUTPUT_DIR / "Explorer_JupiterC_Full.usdc"
if full_usd.exists():
    full_usd.unlink()
converter = omni.kit.asset_converter.get_instance()
task = converter.create_converter_task(
    CONVERSION_SOURCE.as_posix(),
    full_usd.as_posix(),
    None,
    context,
)
if not await task.wait_until_finished():
    raise RuntimeError(f"Isaac Sim asset conversion failed: {SOURCE}")

manifest = helper_namespace["_edit_converted_asset"](
    SOURCE, OUTPUT_DIR, omni.kit.app.get_app()
)
print(json.dumps(manifest, indent=2))
