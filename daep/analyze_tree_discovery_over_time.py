#!/usr/bin/env python3

"""Compare tree discovery counts over time across experiment runs.

The script reads the lightweight runtime products already saved by
experiment_run.sh:

  <run>/snapshots/<tag>/tree_map_final.csv
  <run>/data/tree_map_final.csv

It does not need ROS, numpy, matplotlib, octomaps, point clouds, or PKLs.
"""

import argparse
import csv
import fnmatch
import importlib.util
import json
import math
import pickle
import re
import sys
from datetime import datetime
from pathlib import Path
from xml.sax.saxutils import escape

from ground_truth_utils import load_ground_truth_rows, resolve_ground_truth_csv


PALETTE = [
    "#1f77b4",
    "#d62728",
    "#2ca02c",
    "#9467bd",
    "#ff7f0e",
    "#17becf",
    "#8c564b",
    "#bcbd22",
    "#e377c2",
    "#7f7f7f",
]

EPOCH_SECONDS_CUTOFF = 1000000.0


def repo_root():
    return Path(__file__).resolve().parents[1]


def default_base_dir():
    return repo_root() / "daep" / "experimentos" / "runs"


def default_output_dir():
    return repo_root() / "daep" / "experimentos" / "tree_discovery_comparison"


def pickle_helper_path():
    return repo_root() / "daep" / "meus-resultados" / "plot_pickle_svg.py"


def load_pickle_helper():
    path = pickle_helper_path()
    if not path.exists():
        return None
    spec = importlib.util.spec_from_file_location("plot_pickle_svg", str(path))
    if spec is None or spec.loader is None:
        return None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def relpath(path, start=None):
    path = Path(path)
    start = Path(start or Path.cwd())
    try:
        return str(path.resolve().relative_to(start.resolve()))
    except Exception:
        return str(path)


def float_or_none(value):
    if value is None:
        return None
    try:
        text = str(value).strip()
        if not text:
            return None
        return float(text)
    except Exception:
        return None


def boolish(value):
    if value is None:
        return False
    return str(value).strip().lower() in ("1", "true", "yes", "y", "sim")


def read_key_value_env(path):
    values = {}
    path = Path(path)
    if not path.exists():
        return values
    for line in path.read_text(errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip().strip("'").strip('"')
    return values


def load_run_meta(run_dir):
    run_dir = Path(run_dir)
    meta = {
        "run_id": run_dir.name,
        "experiment_name": "",
        "seed": "",
    }

    meta_path = run_dir / "run_meta.json"
    if meta_path.exists():
        try:
            data = json.loads(meta_path.read_text(errors="replace"))
            meta["run_id"] = str(data.get("run_id") or meta["run_id"])
            meta["experiment_name"] = str(data.get("experiment_name") or "")
            if data.get("seed") is not None:
                meta["seed"] = str(data.get("seed"))
            if data.get("created_at") is not None:
                meta["created_at"] = str(data.get("created_at"))
        except Exception:
            pass

    env = read_key_value_env(run_dir / "run_context.env")
    meta["run_id"] = env.get("EXPERIMENT_RUN_ID", meta["run_id"])
    meta["experiment_name"] = env.get("EXPERIMENT_NAME", meta["experiment_name"])
    meta["seed"] = env.get("EXPERIMENT_SEED", meta["seed"])
    meta["world_name"] = env.get("EXPERIMENT_WORLD_NAME", "")
    meta["created_wall_epoch"] = parse_created_wall_epoch(meta.get("created_at"))
    return meta


def parse_created_wall_epoch(value):
    if not value:
        return None
    text = str(value).strip()
    match = re.match(r"(\d{4}-\d{2}-\d{2})[T ](\d{2}:\d{2}:\d{2})", text)
    if not match:
        return None
    try:
        # Treat the run metadata and snapshot tags as the same local wall-clock
        # domain. This avoids timezone-offset noise when only elapsed time is
        # needed.
        return datetime.strptime(" ".join(match.groups()), "%Y-%m-%d %H:%M:%S").timestamp()
    except Exception:
        return None


def load_truth_count(run_dir):
    run_dir = Path(run_dir)
    metrics_path = run_dir / "result" / "metrics.json"
    if metrics_path.exists():
        try:
            value = json.loads(metrics_path.read_text(errors="replace")).get("truth_count")
            if value is not None:
                return int(value)
        except Exception:
            pass

    meta = load_run_meta(run_dir)
    world_name = str(meta.get("world_name") or "").strip()
    if not world_name:
        return None
    try:
        truth_csv = resolve_ground_truth_csv(world_name)
        return len(load_ground_truth_rows(truth_csv, trees_only=True))
    except Exception:
        return None
    return None


def read_tree_map_csv(path):
    rows = []
    path = Path(path)
    if path.exists() and path.stat().st_size > 0:
        with path.open("r", newline="") as f:
            rows = [dict(r) for r in csv.DictReader(f)]

    confirmed_count = 0
    suspect_merge_count = 0
    time_candidates = []
    max_last_seen = None

    for row in rows:
        if boolish(row.get("confirmed")):
            confirmed_count += 1
        if boolish(row.get("suspect_merge")):
            suspect_merge_count += 1

        last_seen = float_or_none(row.get("last_seen_sec"))
        age = float_or_none(row.get("age_sec"))
        if last_seen is not None:
            max_last_seen = last_seen if max_last_seen is None else max(max_last_seen, last_seen)
            if age is not None:
                time_candidates.append(last_seen + max(age, 0.0))

    csv_time = max(time_candidates) if time_candidates else None
    return {
        "total_count": len(rows),
        "confirmed_count": confirmed_count,
        "candidate_count": len(rows) - confirmed_count,
        "suspect_merge_count": suspect_merge_count,
        "csv_time_sec": csv_time,
        "max_last_seen_sec": max_last_seen,
    }


def parse_snapshot_tag(tag):
    tag = str(tag)
    match = re.fullmatch(r"t(\d+(?:\.\d+)?)", tag)
    if match:
        return float(match.group(1)), None

    match = re.search(r"(\d{8})_(\d{6})", tag)
    if match:
        try:
            dt = datetime.strptime("".join(match.groups()), "%Y%m%d%H%M%S")
            return None, dt.timestamp()
        except Exception:
            return None, None

    return None, None


def read_tree_snapshot_pkl(path, helper=None):
    path = Path(path)
    data = None
    try:
        with path.open("rb") as f:
            data = pickle.load(f, encoding="latin1")
    except Exception:
        if helper is not None:
            try:
                data = helper.load_data(path)
            except Exception:
                data = None

    if not isinstance(data, dict):
        return None

    meta = data.get("meta") if isinstance(data.get("meta"), dict) else {}
    confirmed = data.get("map_confirmed") or []
    candidates = data.get("map_candidates") or []
    saved_at_sec = float_or_none(meta.get("saved_at_sec"))
    return {
        "total_count": len(confirmed) + len(candidates),
        "confirmed_count": len(confirmed),
        "candidate_count": len(candidates),
        "suspect_merge_count": 0,
        "csv_time_sec": saved_at_sec,
        "max_last_seen_sec": None,
    }


def gather_raw_points(run_dir, include_final=True, include_pkl=True):
    run_dir = Path(run_dir)
    raw = []
    helper = load_pickle_helper() if include_pkl else None

    snapshots_dir = run_dir / "snapshots"
    if snapshots_dir.exists():
        for child in snapshots_dir.iterdir():
            csv_path = child / "tree_map_final.csv"
            if child.is_dir() and csv_path.exists():
                tag_seconds, wall_epoch = parse_snapshot_tag(child.name)
                row = read_tree_map_csv(csv_path)
                row.update(
                    {
                        "source_kind": "snapshot",
                        "snapshot_tag": child.name,
                        "tag_seconds": tag_seconds,
                        "wall_epoch": wall_epoch,
                        "source_path": csv_path,
                    }
                )
                raw.append(row)

    if include_pkl:
        tree_snapshot_dir = run_dir / "tree_snapshots"
        if tree_snapshot_dir.exists():
            for pkl_path in sorted(tree_snapshot_dir.glob("*.pkl")):
                row = read_tree_snapshot_pkl(pkl_path, helper=helper)
                if row is None:
                    continue
                tag_seconds, wall_epoch = parse_snapshot_tag(pkl_path.stem)
                row.update(
                    {
                        "source_kind": "tree_snapshot",
                        "snapshot_tag": pkl_path.stem,
                        "tag_seconds": tag_seconds,
                        "wall_epoch": wall_epoch,
                        "source_path": pkl_path,
                    }
                )
                raw.append(row)

    final_csv = run_dir / "data" / "tree_map_final.csv"
    if include_final and final_csv.exists():
        row = read_tree_map_csv(final_csv)
        row.update(
            {
                "source_kind": "final",
                "snapshot_tag": "final",
                "tag_seconds": None,
                "wall_epoch": None,
                "source_path": final_csv,
            }
        )
        raw.append(row)

    return raw


def assign_times(raw_points, created_wall_epoch=None):
    anchors = [
        p
        for p in raw_points
        if p.get("csv_time_sec") is not None
        and p.get("csv_time_sec") < EPOCH_SECONDS_CUTOFF
        and p.get("wall_epoch") is not None
    ]
    anchor = sorted(anchors, key=lambda p: p["wall_epoch"])[0] if anchors else None
    wall_min = min(
        [p["wall_epoch"] for p in raw_points if p.get("wall_epoch") is not None],
        default=None,
    )
    csv_epoch_min = min(
        [
            p["csv_time_sec"]
            for p in raw_points
            if p.get("csv_time_sec") is not None and p.get("csv_time_sec") >= EPOCH_SECONDS_CUTOFF
        ],
        default=None,
    )

    for point in raw_points:
        time_sec = None
        time_source = ""
        csv_time = point.get("csv_time_sec")
        if csv_time is not None and csv_time < EPOCH_SECONDS_CUTOFF:
            time_sec = point["csv_time_sec"]
            time_source = "csv_last_seen_plus_age"
        elif csv_time is not None and point.get("wall_epoch") is not None and created_wall_epoch is not None:
            time_sec = point["wall_epoch"] - created_wall_epoch
            time_source = "snapshot_wall_time_relative_to_run_start"
        elif csv_time is not None and point.get("wall_epoch") is not None and wall_min is not None:
            time_sec = point["wall_epoch"] - wall_min
            time_source = "snapshot_wall_time_relative_to_first_snapshot"
        elif csv_time is not None and csv_epoch_min is not None:
            time_sec = csv_time - csv_epoch_min
            time_source = "csv_wall_time_relative_to_first_csv"
        elif point.get("tag_seconds") is not None:
            time_sec = point["tag_seconds"]
            time_source = "snapshot_tag"
        elif point.get("wall_epoch") is not None and created_wall_epoch is not None:
            time_sec = point["wall_epoch"] - created_wall_epoch
            time_source = "snapshot_wall_time_relative_to_run_start"
        elif point.get("wall_epoch") is not None and anchor is not None:
            time_sec = anchor["csv_time_sec"] + (point["wall_epoch"] - anchor["wall_epoch"])
            time_source = "wall_clock_aligned_to_csv_time"
        elif point.get("wall_epoch") is not None and wall_min is not None:
            time_sec = point["wall_epoch"] - wall_min
            time_source = "wall_clock_relative"

        point["time_sec"] = time_sec
        point["time_source"] = time_source


def compress_points(points):
    usable = [p for p in points if p.get("time_sec") is not None and p["time_sec"] >= 0.0]
    usable.sort(key=lambda p: (p["time_sec"], p.get("source_kind") != "snapshot", str(p.get("source_path"))))

    by_time = {}
    for point in usable:
        key = round(float(point["time_sec"]), 3)
        prev = by_time.get(key)
        if prev is None:
            by_time[key] = point
            continue

        for count_key in ("total_count", "confirmed_count", "candidate_count", "suspect_merge_count"):
            prev[count_key] = max(int(prev.get(count_key, 0)), int(point.get(count_key, 0)))
        if point.get("source_kind") == "final":
            prev["source_kind"] = "final"
            prev["source_path"] = point["source_path"]

    out = list(by_time.values())
    out.sort(key=lambda p: p["time_sec"])
    add_cumulative_counts(out)
    return out


def add_cumulative_counts(points):
    running_total = 0
    running_confirmed = 0
    for point in points:
        running_total = max(running_total, int(point.get("total_count", 0)))
        running_confirmed = max(running_confirmed, int(point.get("confirmed_count", 0)))
        point["cumulative_total_count"] = running_total
        point["cumulative_confirmed_count"] = running_confirmed


def gather_run(run_dir, include_final=True, include_pkl=True):
    meta = load_run_meta(run_dir)
    raw_points = gather_raw_points(run_dir, include_final=include_final, include_pkl=include_pkl)
    assign_times(raw_points, created_wall_epoch=meta.get("created_wall_epoch"))
    points = compress_points(raw_points)
    return {
        "run_dir": Path(run_dir),
        "meta": meta,
        "truth_count": load_truth_count(run_dir),
        "points": points,
        "raw_point_count": len(raw_points),
    }


def metric_value(point, metric, cumulative=True):
    if metric == "total":
        if cumulative:
            return int(point.get("cumulative_total_count", point.get("total_count", 0)))
        return int(point.get("total_count", 0))
    if cumulative:
        return int(point.get("cumulative_confirmed_count", point.get("confirmed_count", 0)))
    return int(point.get("confirmed_count", 0))


def first_time_at(points, metric, threshold):
    for point in sorted(points, key=lambda p: p["time_sec"]):
        if metric_value(point, metric) >= threshold:
            return float(point["time_sec"])
    return None


def fmt_float(value, digits=3):
    if value is None:
        return ""
    return ("{:.%df}" % digits).format(float(value))


def fmt_minutes(value):
    if value is None:
        return ""
    return fmt_float(float(value) / 60.0, 2)


def make_summary(run, metric, target_count):
    points = run["points"]
    final = points[-1] if points else None
    final_metric = metric_value(final, metric, cumulative=False) if final else 0
    peak_metric = max([metric_value(point, metric, cumulative=True) for point in points], default=0)
    target = target_count or run.get("truth_count") or peak_metric or final_metric or None

    def pct_time(pct):
        if not target:
            return None
        threshold = int(math.ceil(float(target) * pct))
        return first_time_at(points, metric, threshold)

    summary = {
        "run_id": run["meta"]["run_id"],
        "experiment_name": run["meta"].get("experiment_name", ""),
        "seed": run["meta"].get("seed", ""),
        "run_dir": str(run["run_dir"]),
        "snapshot_points": len([p for p in points if p.get("source_kind") == "snapshot"]),
        "raw_points": run["raw_point_count"],
        "truth_count": run.get("truth_count"),
        "target_count": target,
        "metric": metric,
        "final_time_sec": final["time_sec"] if final else None,
        "final_total_count": int(final.get("total_count", 0)) if final else 0,
        "final_confirmed_count": int(final.get("confirmed_count", 0)) if final else 0,
        "final_candidate_count": int(final.get("candidate_count", 0)) if final else 0,
        "final_suspect_merge_count": int(final.get("suspect_merge_count", 0)) if final else 0,
        "final_metric_count": final_metric,
        "peak_total_count": max([int(p.get("cumulative_total_count", 0)) for p in points], default=0),
        "peak_confirmed_count": max([int(p.get("cumulative_confirmed_count", 0)) for p in points], default=0),
        "peak_metric_count": peak_metric,
        "time_to_first_sec": first_time_at(points, metric, 1),
        "time_to_25pct_sec": pct_time(0.25),
        "time_to_50pct_sec": pct_time(0.50),
        "time_to_80pct_sec": pct_time(0.80),
        "time_to_100pct_sec": pct_time(1.00),
        "time_to_final_metric_sec": first_time_at(points, metric, final_metric) if final_metric > 0 else None,
        "time_to_peak_metric_sec": first_time_at(points, metric, peak_metric) if peak_metric > 0 else None,
    }
    return summary


def select_run_dirs(base_dir, positional, includes, excludes):
    base_dir = Path(base_dir)
    selected = []
    if positional:
        for item in positional:
            path = Path(item)
            if not path.exists():
                path = base_dir / item
            if path.is_dir():
                selected.append(path)
            else:
                print("warning: run not found: {}".format(item), file=sys.stderr)
        return selected

    if not base_dir.exists():
        return []

    for child in sorted(base_dir.iterdir()):
        if not child.is_dir():
            continue
        run_id = child.name
        if includes and not any(fnmatch.fnmatch(run_id, pat) for pat in includes):
            continue
        if excludes and any(fnmatch.fnmatch(run_id, pat) for pat in excludes):
            continue
        selected.append(child)
    return selected


def write_timeseries_csv(path, runs, metric):
    fields = [
        "run_id",
        "experiment_name",
        "seed",
        "time_sec",
        "time_min",
        "total_count",
        "confirmed_count",
        "candidate_count",
        "suspect_merge_count",
        "cumulative_total_count",
        "cumulative_confirmed_count",
        "metric",
        "metric_count",
        "cumulative_metric_count",
        "source_kind",
        "snapshot_tag",
        "time_source",
        "source_path",
    ]
    with Path(path).open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for run in runs:
            meta = run["meta"]
            for point in run["points"]:
                writer.writerow(
                    {
                        "run_id": meta["run_id"],
                        "experiment_name": meta.get("experiment_name", ""),
                        "seed": meta.get("seed", ""),
                        "time_sec": fmt_float(point["time_sec"], 3),
                        "time_min": fmt_minutes(point["time_sec"]),
                        "total_count": point.get("total_count", 0),
                        "confirmed_count": point.get("confirmed_count", 0),
                        "candidate_count": point.get("candidate_count", 0),
                        "suspect_merge_count": point.get("suspect_merge_count", 0),
                        "cumulative_total_count": point.get("cumulative_total_count", point.get("total_count", 0)),
                        "cumulative_confirmed_count": point.get("cumulative_confirmed_count", point.get("confirmed_count", 0)),
                        "metric": metric,
                        "metric_count": metric_value(point, metric, cumulative=False),
                        "cumulative_metric_count": metric_value(point, metric, cumulative=True),
                        "source_kind": point.get("source_kind", ""),
                        "snapshot_tag": point.get("snapshot_tag", ""),
                        "time_source": point.get("time_source", ""),
                        "source_path": relpath(point.get("source_path", "")),
                    }
                )


def write_summary_csv(path, summaries):
    fields = [
        "run_id",
        "experiment_name",
        "seed",
        "snapshot_points",
        "raw_points",
        "truth_count",
        "target_count",
        "metric",
        "final_time_sec",
        "final_time_min",
        "final_total_count",
        "final_confirmed_count",
        "final_candidate_count",
        "final_suspect_merge_count",
        "final_metric_count",
        "peak_total_count",
        "peak_confirmed_count",
        "peak_metric_count",
        "time_to_first_sec",
        "time_to_first_min",
        "time_to_25pct_sec",
        "time_to_25pct_min",
        "time_to_50pct_sec",
        "time_to_50pct_min",
        "time_to_80pct_sec",
        "time_to_80pct_min",
        "time_to_100pct_sec",
        "time_to_100pct_min",
        "time_to_final_metric_sec",
        "time_to_final_metric_min",
        "time_to_peak_metric_sec",
        "time_to_peak_metric_min",
        "run_dir",
    ]
    with Path(path).open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for summary in summaries:
            row = dict(summary)
            for key in list(row.keys()):
                if key.endswith("_sec"):
                    row[key] = fmt_float(row[key], 3)
            row["final_time_min"] = fmt_minutes(summary.get("final_time_sec"))
            row["time_to_first_min"] = fmt_minutes(summary.get("time_to_first_sec"))
            row["time_to_25pct_min"] = fmt_minutes(summary.get("time_to_25pct_sec"))
            row["time_to_50pct_min"] = fmt_minutes(summary.get("time_to_50pct_sec"))
            row["time_to_80pct_min"] = fmt_minutes(summary.get("time_to_80pct_sec"))
            row["time_to_100pct_min"] = fmt_minutes(summary.get("time_to_100pct_sec"))
            row["time_to_final_metric_min"] = fmt_minutes(summary.get("time_to_final_metric_sec"))
            row["time_to_peak_metric_min"] = fmt_minutes(summary.get("time_to_peak_metric_sec"))
            row["run_dir"] = relpath(row["run_dir"])
            writer.writerow({field: row.get(field, "") for field in fields})


def chart_path(points, x_scale, y_scale, left, bottom, metric, max_time_sec, max_count):
    if not points:
        return ""
    ordered = sorted(points, key=lambda p: p["time_sec"])
    x0 = left
    y0 = bottom
    path = ["M {:.2f} {:.2f}".format(x0, y0)]
    last_x = x0
    last_y = y0

    for point in ordered:
        x = left + min(point["time_sec"], max_time_sec) * x_scale
        y = bottom - min(metric_value(point, metric, cumulative=True), max_count) * y_scale
        path.append("H {:.2f}".format(x))
        path.append("V {:.2f}".format(y))
        last_x = x
        last_y = y

    path.append("H {:.2f}".format(left + max_time_sec * x_scale))
    return " ".join(path)


def write_svg(path, runs, metric, target_count):
    width = 1160
    height = 720
    left = 86
    right = 280
    top = 48
    bottom_margin = 82
    chart_w = width - left - right
    chart_h = height - top - bottom_margin
    bottom = top + chart_h

    max_time_sec = max(
        [p["time_sec"] for run in runs for p in run["points"] if p.get("time_sec") is not None],
        default=1.0,
    )
    max_count = max(
        [metric_value(p, metric, cumulative=True) for run in runs for p in run["points"]],
        default=1,
    )
    if target_count:
        max_count = max(max_count, int(target_count))
    max_time_sec = max(max_time_sec, 1.0)
    max_count = max(max_count, 1)

    x_scale = chart_w / max_time_sec
    y_scale = chart_h / max_count

    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="{0}" height="{1}" viewBox="0 0 {0} {1}">'.format(
            width, height
        ),
        "<style>",
        "text{font-family:Arial,Helvetica,sans-serif;fill:#202020}",
        ".grid{stroke:#d7d7d7;stroke-width:1}",
        ".axis{stroke:#333;stroke-width:1.4}",
        ".series{fill:none;stroke-width:2.6;stroke-linejoin:round;stroke-linecap:round}",
        ".muted{fill:#666;font-size:13px}",
        ".label{font-size:14px}",
        ".title{font-size:21px;font-weight:700}",
        "</style>",
        '<rect x="0" y="0" width="{0}" height="{1}" fill="#fff"/>'.format(width, height),
        '<text x="{0}" y="28" class="title">Tree discovery over time ({1}, cumulative)</text>'.format(
            left, escape(metric)
        ),
    ]

    x_ticks = 6
    for i in range(x_ticks + 1):
        x = left + chart_w * i / x_ticks
        t_min = (max_time_sec * i / x_ticks) / 60.0
        lines.append('<line x1="{0:.2f}" y1="{1}" x2="{0:.2f}" y2="{2}" class="grid"/>'.format(x, top, bottom))
        lines.append(
            '<text x="{0:.2f}" y="{1}" text-anchor="middle" class="muted">{2:.1f}</text>'.format(
                x, bottom + 26, t_min
            )
        )

    y_ticks = min(max_count, 10)
    for i in range(y_ticks + 1):
        count = max_count * i / y_ticks if y_ticks else 0
        y = bottom - chart_h * i / y_ticks if y_ticks else bottom
        lines.append('<line x1="{0}" y1="{1:.2f}" x2="{2}" y2="{1:.2f}" class="grid"/>'.format(left, y, left + chart_w))
        lines.append(
            '<text x="{0}" y="{1:.2f}" text-anchor="end" dominant-baseline="middle" class="muted">{2:g}</text>'.format(
                left - 10, y, count
            )
        )

    lines.append('<line x1="{0}" y1="{1}" x2="{0}" y2="{2}" class="axis"/>'.format(left, top, bottom))
    lines.append('<line x1="{0}" y1="{1}" x2="{2}" y2="{1}" class="axis"/>'.format(left, bottom, left + chart_w))
    lines.append('<text x="{0}" y="{1}" text-anchor="middle" class="label">tempo (min)</text>'.format(left + chart_w / 2, height - 22))
    lines.append(
        '<text x="24" y="{0}" transform="rotate(-90 24 {0})" text-anchor="middle" class="label">arvores detectadas</text>'.format(
            top + chart_h / 2
        )
    )

    if target_count:
        y = bottom - int(target_count) * y_scale
        lines.append(
            '<line x1="{0}" y1="{1:.2f}" x2="{2}" y2="{1:.2f}" stroke="#555" stroke-width="1.3" stroke-dasharray="6 5"/>'.format(
                left, y, left + chart_w
            )
        )
        lines.append(
            '<text x="{0}" y="{1:.2f}" class="muted">target={2}</text>'.format(
                left + chart_w + 12, y + 4, int(target_count)
            )
        )

    legend_x = left + chart_w + 28
    legend_y = top + 18
    for idx, run in enumerate(runs):
        color = PALETTE[idx % len(PALETTE)]
        path_d = chart_path(run["points"], x_scale, y_scale, left, bottom, metric, max_time_sec, max_count)
        if path_d:
            lines.append('<path d="{0}" class="series" stroke="{1}"/>'.format(path_d, color))

        label = run["meta"]["run_id"]
        final = run["points"][-1] if run["points"] else None
        final_count = metric_value(final, metric, cumulative=True) if final else 0
        y = legend_y + idx * 44
        lines.append('<line x1="{0}" y1="{1}" x2="{2}" y2="{1}" stroke="{3}" stroke-width="3"/>'.format(legend_x, y, legend_x + 28, color))
        lines.append('<text x="{0}" y="{1}" class="label">{2}</text>'.format(legend_x + 38, y + 4, escape(label[:36])))
        lines.append(
            '<text x="{0}" y="{1}" class="muted">peak={2}, pontos={3}</text>'.format(
                legend_x + 38, y + 22, final_count, len(run["points"])
            )
        )

    lines.append("</svg>")
    Path(path).write_text("\n".join(lines))


def write_markdown(path, summaries, metric):
    lines = [
        "# Tree discovery over time",
        "",
        "Metric used for threshold times: cumulative `{}`.".format(metric),
        "",
        "| run | seed | peak | target | first min | 50% min | 80% min | 100% min |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summaries:
        lines.append(
            "| {run_id} | {seed} | {peak_metric_count} | {target} | {first} | {p50} | {p80} | {p100} |".format(
                run_id=row["run_id"],
                seed=row.get("seed", ""),
                peak_metric_count=row["peak_metric_count"],
                target=row.get("target_count") or "",
                first=fmt_minutes(row.get("time_to_first_sec")) or "-",
                p50=fmt_minutes(row.get("time_to_50pct_sec")) or "-",
                p80=fmt_minutes(row.get("time_to_80pct_sec")) or "-",
                p100=fmt_minutes(row.get("time_to_100pct_sec")) or "-",
            )
        )
    lines.extend(
        [
            "",
            "Generated files:",
            "",
            "- `tree_discovery_timeseries.csv`",
            "- `tree_discovery_summary.csv`",
            "- `tree_discovery_curves.svg`",
            "",
            "Times are derived from `last_seen_sec + age_sec` when available. For empty early snapshots, the script falls back to snapshot tags or wall-clock tags aligned to the first CSV timestamp.",
        ]
    )
    Path(path).write_text("\n".join(lines) + "\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="*", help="Run ids or run directories. If omitted, --include filters base dir.")
    parser.add_argument("--base-dir", default=str(default_base_dir()), help="Directory containing experiment runs.")
    parser.add_argument("--include", action="append", default=[], help="fnmatch pattern for run ids, e.g. 'baseline*'.")
    parser.add_argument("--exclude", action="append", default=[], help="fnmatch pattern to skip run ids.")
    parser.add_argument("--output-dir", default=str(default_output_dir()), help="Directory for CSV/SVG outputs.")
    parser.add_argument("--metric", choices=("confirmed", "total"), default="confirmed", help="Count used for curves and threshold times.")
    parser.add_argument("--target-count", type=int, default=None, help="Ground-truth target count for percentage timing.")
    parser.add_argument("--no-final", action="store_true", help="Do not include data/tree_map_final.csv as the final point.")
    parser.add_argument("--no-pkl", action="store_true", help="Do not include tree_snapshots/*.pkl points.")
    args = parser.parse_args(argv)

    run_dirs = select_run_dirs(args.base_dir, args.runs, args.include, args.exclude)
    if not run_dirs:
        print("No runs selected.", file=sys.stderr)
        return 2

    runs = [
        gather_run(path, include_final=not args.no_final, include_pkl=not args.no_pkl)
        for path in run_dirs
    ]
    runs = [run for run in runs if run["points"]]
    if not runs:
        print("Selected runs have no timed tree_map_final.csv points.", file=sys.stderr)
        return 2

    inferred_target = args.target_count
    if inferred_target is None:
        truth_counts = [run["truth_count"] for run in runs if run.get("truth_count")]
        if truth_counts:
            inferred_target = max(truth_counts)

    summaries = [make_summary(run, args.metric, inferred_target) for run in runs]
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    write_timeseries_csv(output_dir / "tree_discovery_timeseries.csv", runs, args.metric)
    write_summary_csv(output_dir / "tree_discovery_summary.csv", summaries)
    write_svg(output_dir / "tree_discovery_curves.svg", runs, args.metric, inferred_target)
    write_markdown(output_dir / "summary.md", summaries, args.metric)

    print("Wrote {}".format(output_dir / "tree_discovery_timeseries.csv"))
    print("Wrote {}".format(output_dir / "tree_discovery_summary.csv"))
    print("Wrote {}".format(output_dir / "tree_discovery_curves.svg"))
    print("Wrote {}".format(output_dir / "summary.md"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
