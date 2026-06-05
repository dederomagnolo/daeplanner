#!/usr/bin/env python3

"""Helpers to resolve and read canonical ground-truth CSV files."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, List, Optional


def workspace_root() -> Path:
    return Path(__file__).resolve().parent.parent


def biomass_ground_truth_root() -> Path:
    return workspace_root() / "external" / "biomass-simulation-resources" / "ground_truth"


def _candidate_ground_truth_paths(world_name: str) -> List[Path]:
    root = biomass_ground_truth_root()
    names = [
        f"{world_name}.csv",
        f"{world_name}_ground_truth.csv",
    ]
    paths: List[Path] = []
    for name in names:
        paths.append(root / world_name / name)
        paths.append(root / name)
    return paths


def resolve_ground_truth_csv(world_name: str, explicit_csv: str = "") -> Path:
    if explicit_csv.strip():
        return Path(explicit_csv).expanduser().resolve()

    for candidate in _candidate_ground_truth_paths(world_name):
        if candidate.exists():
            return candidate.resolve()

    return _candidate_ground_truth_paths(world_name)[0].resolve()


def _as_float(value, default: float = 0.0) -> float:
    try:
        return float(str(value).strip())
    except Exception:
        return default


def _as_optional_int(value) -> Optional[int]:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        return int(float(text))
    except Exception:
        return None


def load_ground_truth_rows(csv_path: Path, trees_only: bool = True) -> List[Dict[str, object]]:
    if not csv_path.exists():
        raise FileNotFoundError("ground-truth CSV not found: {}".format(csv_path))

    rows: List[Dict[str, object]] = []
    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fieldnames = set(reader.fieldnames or [])

        required = {"id", "type", "model_name", "source_uri", "x", "y", "z", "roll", "pitch", "yaw"}
        if not required.issubset(fieldnames):
            raise ValueError(
                "unsupported ground-truth CSV schema: {} (expected fields: {})".format(
                    csv_path,
                    ", ".join(sorted(required)),
                )
            )

        for idx, row in enumerate(reader, start=1):
            obj_type = str(row.get("type", "")).strip().lower()
            if trees_only and obj_type != "tree":
                continue
            rows.append(
                {
                    "tree_id": _as_optional_int(row.get("id")),
                    "name": str(row.get("model_name", "")).strip(),
                    "x": _as_float(row.get("x")),
                    "y": _as_float(row.get("y")),
                    "z": _as_float(row.get("z")),
                    "roll": _as_float(row.get("roll")),
                    "pitch": _as_float(row.get("pitch")),
                    "yaw": _as_float(row.get("yaw")),
                    "uri": str(row.get("source_uri", "")).strip(),
                    "type": obj_type,
                    "index": idx,
                }
            )

    rows.sort(
        key=lambda r: (
            r["tree_id"] is None,
            r["tree_id"] if r["tree_id"] is not None else 999999,
            str(r.get("name", "")),
        )
    )
    return rows
