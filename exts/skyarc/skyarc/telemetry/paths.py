# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Collision-safe output path allocation for one execution instance."""

from __future__ import annotations

import os
import uuid
from dataclasses import dataclass
from pathlib import Path


def _segment(value: str | int, label: str) -> str:
    text = str(value)
    if (
        not text
        or text in {".", ".."}
        or os.path.isabs(text)
        or "/" in text
        or "\\" in text
        or "\x00" in text
    ):
        raise ValueError(f"{label} must be one non-empty path segment")
    return text


@dataclass(frozen=True)
class RunPaths:
    root: Path
    resolved_config: Path
    experiment_manifest: Path
    scene: Path
    telemetry_schema: Path
    telemetry_csv: Path
    diagnostics_jsonl: Path
    events_jsonl: Path
    summary_json: Path
    frames: Path
    video: Path
    run_instance_id: str

    @classmethod
    def create(
        cls,
        base_directory: str | Path,
        *,
        experiment_id: str,
        condition_id: str,
        replicate_id: str | int,
        run_instance_id: str | None = None,
    ) -> "RunPaths":
        base = Path(base_directory).resolve()
        experiment = _segment(experiment_id, "experiment_id")
        condition = _segment(condition_id, "condition_id")
        replicate = _segment(replicate_id, "replicate_id")
        instance = _segment(run_instance_id or str(uuid.uuid4()), "run_instance_id")
        root = (base / experiment / condition / replicate / instance).resolve()
        try:
            root.relative_to(base)
        except ValueError:
            raise ValueError("resolved run path escapes the configured output directory") from None
        root.mkdir(parents=True, exist_ok=False)
        frames = root / "frames"
        frames.mkdir()
        return cls(
            root=root,
            resolved_config=root / "resolved_config.yaml",
            experiment_manifest=root / "experiment_manifest.json",
            scene=root / "scene.usd",
            telemetry_schema=root / "telemetry_schema.json",
            telemetry_csv=root / "telemetry.csv",
            diagnostics_jsonl=root / "diagnostics.jsonl",
            events_jsonl=root / "events.jsonl",
            summary_json=root / "summary.json",
            frames=frames,
            video=root / "launch.mp4",
            run_instance_id=instance,
        )
