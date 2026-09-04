# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Run a declared, paired production-mission sweep and aggregate its evidence.

The sweep process does not contain mission physics.  It materializes strict scenario
configurations, launches ``run_mission.py`` once per condition, and consumes the complete
run manifests that runner writes.  Criterion results, paired stream seeds, initial-state
identity, and contrast arithmetic therefore come from the same experiment machinery as a
single evidence run.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
EXTENSION_ROOT = PROJECT_ROOT / "exts" / "skyarc"
if str(EXTENSION_ROOT) not in sys.path:
    sys.path.insert(0, str(EXTENSION_ROOT))

from skyarc.configuration import load_yaml  # noqa: E402
from skyarc.experiments import (  # noqa: E402
    ConditionResult,
    NamedRandomStreams,
    paired_contrasts,
)
from skyarc.names import ALL_SLOTS  # noqa: E402


SCHEMA_VERSION = "production_mission_sweep_v1"
DEFAULT_METRICS = (
    "stage2_margin_mps",
    "handoff_altitude_m",
    "handoff_speed_mps",
    "handoff_flight_path_angle_deg",
    "apogee_altitude_m",
)


@dataclass(frozen=True)
class MaterializedCondition:
    condition_id: str
    parent_condition_id: str | None
    changes: Mapping[str, Any]
    configuration: Path
    controlled_factors: Path


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{label} must be a string-keyed mapping")
    return value


def _set_path(root: dict[str, Any], dotted_path: str, value: Any) -> None:
    parts = dotted_path.split(".")
    if not dotted_path or any(not part for part in parts):
        raise ValueError(f"invalid factor path {dotted_path!r}")
    target: dict[str, Any] = root
    for part in parts[:-1]:
        child = target.get(part)
        if not isinstance(child, dict):
            raise ValueError(f"factor path {dotted_path!r} does not resolve through {part!r}")
        target = child
    if parts[-1] not in target:
        raise ValueError(f"factor path {dotted_path!r} targets an unknown field")
    target[parts[-1]] = value


def materialize_conditions(plan_path: Path, output_directory: Path) -> tuple[MaterializedCondition, ...]:
    plan = _mapping(yaml.safe_load(plan_path.read_text(encoding="utf-8")), "sweep plan")
    allowed = {
        "schema_version",
        "base_configuration",
        "experiment_id",
        "replicate_id",
        "seed",
        "metrics",
        "conditions",
    }
    unknown = sorted(set(plan) - allowed)
    if unknown:
        raise ValueError(f"unknown sweep-plan fields: {unknown}")
    if plan.get("schema_version") != 1:
        raise ValueError("sweep plan schema_version must be 1")
    base_path = (plan_path.parent / str(plan["base_configuration"])).resolve()
    base = _mapping(yaml.safe_load(base_path.read_text(encoding="utf-8")), "base configuration")
    conditions = plan.get("conditions")
    if not isinstance(conditions, list) or not conditions:
        raise ValueError("sweep plan requires a nonempty conditions list")

    output_directory.mkdir(parents=True, exist_ok=True)
    seen: set[str] = set()
    roots = 0
    result = []
    for raw in conditions:
        condition = _mapping(raw, "sweep condition")
        unknown = sorted(set(condition) - {"condition_id", "parent_condition_id", "changes"})
        if unknown:
            raise ValueError(f"unknown sweep-condition fields: {unknown}")
        condition_id = str(condition.get("condition_id", ""))
        parent = condition.get("parent_condition_id")
        if parent is not None:
            parent = str(parent)
        if not condition_id or condition_id in seen:
            raise ValueError("condition IDs must be nonempty and unique")
        if parent is None:
            roots += 1
        elif parent not in seen:
            raise ValueError(f"condition {condition_id!r} must follow its parent {parent!r}")
        seen.add(condition_id)
        changes = dict(_mapping(condition.get("changes", {}), "condition changes"))
        resolved = json.loads(json.dumps(base))
        for path, value in sorted(changes.items()):
            _set_path(resolved, path, value)
        experiment = _mapping(resolved.get("experiment"), "base experiment")
        experiment = dict(experiment)
        experiment.update(
            {
                "experiment_id": str(plan["experiment_id"]),
                "condition_id": condition_id,
                "parent_condition_id": parent,
                "replicate_id": int(plan["replicate_id"]),
                "seed": int(plan["seed"]),
            }
        )
        resolved["experiment"] = experiment
        config_path = output_directory / f"{condition_id}.yaml"
        config_path.write_text(
            yaml.safe_dump(resolved, sort_keys=False), encoding="utf-8"
        )
        # Load now so a bad factor fails before any expensive Kit process starts.
        load_yaml(config_path)
        factor_path = output_directory / f"{condition_id}.factors.json"
        factor_path.write_text(
            json.dumps(changes, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        result.append(
            MaterializedCondition(condition_id, parent, changes, config_path, factor_path)
        )
    if roots != 1:
        raise ValueError(f"sweep plan must contain exactly one root condition, found {roots}")
    return tuple(result)


def aggregate_manifests(
    manifest_paths: Sequence[Path],
    *,
    metrics: Sequence[str] = DEFAULT_METRICS,
) -> dict[str, Any]:
    records = []
    expected_streams: Mapping[str, int] | None = None
    for manifest_path in manifest_paths:
        manifest = _mapping(
            json.loads(manifest_path.read_text(encoding="utf-8")), "experiment manifest"
        )
        identity = _mapping(manifest["identity"], "manifest identity")
        lineage = _mapping(manifest["lineage"], "manifest lineage")
        hashes = _mapping(manifest["hashes"], "manifest hashes")
        random = _mapping(manifest["random"], "manifest random")
        streams = _mapping(random["streams"], "manifest random streams")
        outcome = _mapping(manifest["outcome"], "manifest outcome")
        outcome_metrics = _mapping(outcome["metrics"], "manifest outcome metrics")
        selected_metrics: dict[str, float | None] = {}
        for name in metrics:
            value = outcome_metrics.get(name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                selected_metrics[name] = None
            else:
                selected_metrics[name] = float(value)
        declared = _mapping(lineage["controlled_factors"], "manifest controlled factors").get(
            "declared", {}
        )
        stream_values = {str(name): int(seed) for name, seed in streams.items()}
        if expected_streams is None:
            expected_streams = stream_values
        records.append(
            {
                "identity": identity,
                "lineage": lineage,
                "hashes": hashes,
                "outcome": outcome,
                "manifest_path": manifest_path,
                "declared": declared,
                "stream_values": stream_values,
                "metrics": selected_metrics,
            }
        )

    contrast_metrics = tuple(
        name
        for name in metrics
        if all(record["metrics"][name] is not None for record in records)
    )
    results = []
    condition_rows = []
    for record in records:
        identity = record["identity"]
        lineage = record["lineage"]
        hashes = record["hashes"]
        selected_metrics = {
            name: record["metrics"][name]
            for name in contrast_metrics
        }
        results.append(
            ConditionResult(
                experiment_id=str(identity["experiment_id"]),
                condition_id=str(identity["condition_id"]),
                parent_condition_id=(
                    None
                    if lineage.get("parent_condition_id") is None
                    else str(lineage["parent_condition_id"])
                ),
                replicate_id=identity["replicate_id"],
                factors=_mapping(record["declared"], "declared controlled factors"),
                initial_state_sha256=str(hashes["initial_state_sha256"]),
                stream_seeds=record["stream_values"],
                metrics=selected_metrics,
            )
        )
        condition_rows.append(
            {
                "condition_id": identity["condition_id"],
                "manifest": str(record["manifest_path"].resolve()),
                "criterion_result": record["outcome"]["criterion_result"],
                "metrics": record["metrics"],
                "unavailable_metrics": [
                    name for name in metrics if record["metrics"][name] is None
                ],
            }
        )
    return {
        "schema_version": SCHEMA_VERSION,
        "metrics": list(metrics),
        "contrast_metrics": list(contrast_metrics),
        "contrast_status": (
            "complete" if len(contrast_metrics) == len(metrics) else "partial_missing_metrics"
        ),
        "conditions": condition_rows,
        "paired_stream_seeds": dict(expected_streams or {}),
        "contrasts": [item.to_dict() for item in paired_contrasts(results)],
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--plan",
        type=Path,
        default=PROJECT_ROOT / "configs" / "mission_sweep.yaml",
    )
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=PROJECT_ROOT / "artifacts" / "sweeps" / "ignition_altitude",
    )
    parser.add_argument("--python", type=Path, default=Path(sys.executable))
    parser.add_argument("--max-steps", type=int)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--aggregate-only",
        action="store_true",
        help="Rebuild sweep.json from existing per-condition summaries without rerunning missions.",
    )
    args = parser.parse_args(argv)

    plan = _mapping(yaml.safe_load(args.plan.read_text(encoding="utf-8")), "sweep plan")
    metrics = tuple(str(item) for item in plan.get("metrics", DEFAULT_METRICS))
    generated = args.output_directory / "configurations"
    conditions = materialize_conditions(args.plan, generated)
    if args.dry_run:
        print(
            json.dumps(
                {
                    "schema_version": SCHEMA_VERSION,
                    "dry_run": True,
                    "configurations": [str(item.configuration.resolve()) for item in conditions],
                },
                indent=2,
                sort_keys=True,
            )
        )
        return 0

    runner = PROJECT_ROOT / "standalone" / "run_mission.py"
    summaries = args.output_directory / "summaries"
    telemetry = args.output_directory / "runs"
    summaries.mkdir(parents=True, exist_ok=True)
    manifest_by_condition: dict[str, Path] = {}
    manifest_paths = []
    for condition in conditions:
        summary_path = summaries / f"{condition.condition_id}.json"
        if not args.aggregate_only:
            command = [
                str(args.python),
                str(runner),
                "--headless",
                "--configuration",
                str(condition.configuration),
                "--telemetry-directory",
                str(telemetry),
                "--summary",
                str(summary_path),
                "--controlled-factors",
                str(condition.controlled_factors),
            ]
            if condition.parent_condition_id is not None:
                command.extend(
                    ["--parent-manifest", str(manifest_by_condition[condition.parent_condition_id])]
                )
            if args.max_steps is not None:
                command.extend(["--max-steps", str(args.max_steps)])
            completed = subprocess.run(command, check=False)
            if completed.returncode not in (0, 2) or not summary_path.is_file():
                raise RuntimeError(
                    f"condition {condition.condition_id!r} failed before producing a summary "
                    f"(exit {completed.returncode})"
                )
        elif not summary_path.is_file():
            raise RuntimeError(
                f"condition {condition.condition_id!r} has no existing summary to aggregate"
            )
        summary = _mapping(json.loads(summary_path.read_text(encoding="utf-8")), "run summary")
        manifest_value = summary.get("experiment_manifest")
        if not isinstance(manifest_value, str):
            raise RuntimeError(f"condition {condition.condition_id!r} produced no manifest")
        manifest_path = Path(manifest_value)
        manifest_by_condition[condition.condition_id] = manifest_path
        manifest_paths.append(manifest_path)

    aggregate = aggregate_manifests(manifest_paths, metrics=metrics)
    streams = NamedRandomStreams(int(plan["seed"]), ALL_SLOTS)
    if aggregate["paired_stream_seeds"] != dict(streams.seeds):
        raise RuntimeError("run manifests do not contain the sweep plan's paired stream seeds")
    output = args.output_directory / "sweep.json"
    output.write_text(
        json.dumps(aggregate, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({**aggregate, "output": str(output.resolve())}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
