#!/usr/bin/env python3

"""Bundle latest experiment artifacts and generate evaluation report.

This script creates:
  <base_dir>/<experiment_name>/
with copied raw files, generated plots, and a Markdown summary.
"""

import argparse
import csv
import datetime as dt
import importlib.util
import json
import math
import os
import pickle
import shutil
import statistics as st
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from ground_truth_utils import load_ground_truth_rows, resolve_ground_truth_csv as resolve_canonical_ground_truth_csv


def run_cmd(cmd: Sequence[str], cwd: Optional[Path] = None) -> str:
    proc = subprocess.run(
        list(cmd),
        cwd=str(cwd) if cwd else None,
        check=True,
        universal_newlines=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    return proc.stdout


def latest_file(directory: Path, pattern: str) -> Optional[Path]:
    if not directory.exists():
        return None
    candidates = [p for p in directory.glob(pattern) if p.is_file()]
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime)


def copy_file(src: Path, dst_dir: Path, manifest: Dict[str, dict], key: str) -> Optional[Path]:
    if not src.exists() or (not src.is_file()):
        return None
    dst = dst_dir / src.name
    shutil.copy2(str(src), str(dst))
    stat = src.stat()
    manifest[key] = {
        "source": str(src.resolve()),
        "copied_to": str(dst.resolve()),
        "size_bytes": int(stat.st_size),
        "mtime_iso": dt.datetime.fromtimestamp(stat.st_mtime).isoformat(),
    }
    return dst


def register_input_file(src: Path, manifest: Dict[str, dict], key: str) -> Optional[Path]:
    if not src.exists() or (not src.is_file()):
        return None
    stat = src.stat()
    manifest[key] = {
        "source": str(src.resolve()),
        "size_bytes": int(stat.st_size),
        "mtime_iso": dt.datetime.fromtimestamp(stat.st_mtime).isoformat(),
    }
    return src


def load_truth(csv_path: Path) -> List[dict]:
    return [
        {
            "tree_id": row.get("tree_id"),
            "x": float(row["x"]),
            "y": float(row["y"]),
        }
        for row in load_ground_truth_rows(csv_path, trees_only=True)
    ]


def write_ground_truth_svg(path: Path, truth: List[dict], title: str, x_min: float, x_max: float, y_min: float, y_max: float) -> None:
    width = 900
    height = 680
    left = 70
    right = 30
    top = 55
    bottom = 75
    plot_w = width - left - right
    plot_h = height - top - bottom

    def sx(x: float) -> float:
        return left + ((float(x) - x_min) / max(x_max - x_min, 1e-6)) * plot_w

    def sy(y: float) -> float:
        return top + (1.0 - ((float(y) - y_min) / max(y_max - y_min, 1e-6))) * plot_h

    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" viewBox="0 0 {} {}">'.format(width, height, width, height),
        '<rect width="100%" height="100%" fill="white" />',
        '<text x="{}" y="32" font-family="Arial" font-size="18" font-weight="bold">{}</text>'.format(left, title),
        '<rect x="{0}" y="{1}" width="{2}" height="{3}" fill="#fafafa" stroke="#333" />'.format(left, top, plot_w, plot_h),
    ]

    tick_count = 5
    for i in range(tick_count + 1):
        frac = float(i) / float(tick_count)
        x_val = x_min + frac * (x_max - x_min)
        y_val = y_min + frac * (y_max - y_min)
        x = left + frac * plot_w
        y = top + (1.0 - frac) * plot_h
        lines.append('<line x1="{0:.2f}" y1="{1}" x2="{0:.2f}" y2="{2}" stroke="#e5e5e5" />'.format(x, top, top + plot_h))
        lines.append('<line x1="{0}" y1="{1:.2f}" x2="{2}" y2="{1:.2f}" stroke="#e5e5e5" />'.format(left, y, left + plot_w))
        lines.append('<text x="{:.2f}" y="{}" font-family="Arial" font-size="11" text-anchor="middle">{:.1f}</text>'.format(x, top + plot_h + 18, x_val))
        lines.append('<text x="{}" y="{:.2f}" font-family="Arial" font-size="11" text-anchor="end">{:.1f}</text>'.format(left - 8, y + 4, y_val))

    map_w = max(0.0, float(x_max) - float(x_min))
    map_h = max(0.0, float(y_max) - float(y_min))
    map_info = "mapa XY: x[{:.1f},{:.1f}] y[{:.1f},{:.1f}] | tamanho {:.1f} x {:.1f} m | GT arvores {}".format(
        x_min, x_max, y_min, y_max, map_w, map_h, len(truth)
    )
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12" fill="#333">{}</text>'.format(left + 8, top + 16, map_info))

    for tree in truth:
        lines.append('<circle cx="{:.2f}" cy="{:.2f}" r="7" fill="none" stroke="#1f77b4" stroke-width="2" />'.format(sx(tree["x"]), sy(tree["y"])))
        if tree.get("tree_id") is not None:
            lines.append('<text x="{:.2f}" y="{:.2f}" font-family="Arial" font-size="10" fill="#1f77b4">{}</text>'.format(sx(tree["x"]) + 8, sy(tree["y"]) - 8, tree["tree_id"]))

    legend_y = height - 42
    lines.append('<circle cx="{}" cy="{}" r="6" fill="none" stroke="#1f77b4" stroke-width="2" />'.format(left, legend_y))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">GT</text>'.format(left + 14, legend_y + 4))
    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n")


def _get_float(row: dict, key: str, default: float = 0.0) -> float:
    try:
        return float(str(row.get(key, "")).strip())
    except Exception:
        return float(default)


def _get_int(row: dict, key: str, default: int = 0) -> int:
    try:
        return int(float(str(row.get(key, "")).strip()))
    except Exception:
        return int(default)


def _get_optional_float(row: dict, key: str) -> Optional[float]:
    value = str(row.get(key, "")).strip()
    if not value:
        return None
    try:
        return float(value)
    except Exception:
        return None


def load_map(csv_path: Path) -> List[dict]:
    rows = []
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(
                {
                    "map_id": _get_int(row, "map_id", -1),
                    "x": _get_float(row, "x"),
                    "y": _get_float(row, "y"),
                    "z": _get_float(row, "z"),
                    "diameter_m": _get_float(row, "diameter_m"),
                    "hits": _get_int(row, "hits"),
                    "std_xy": _get_float(row, "std_xy"),
                    "std_diameter": _get_float(row, "std_diameter"),
                    "confidence": _get_float(row, "confidence"),
                    "confirmed": _get_int(row, "confirmed"),
                    "suspect_merge": _get_int(row, "suspect_merge"),
                }
            )
    rows.sort(key=lambda d: d["map_id"])
    return rows


def load_path_goals(csv_path: Path) -> List[dict]:
    if not csv_path.exists():
        return []
    goals = []
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for idx, row in enumerate(reader, start=1):
            clean = {str(k).strip(): v for k, v in row.items() if k is not None}
            try:
                goals.append(
                    {
                        "seq": idx,
                        "x": float(str(clean.get("Goal x", "")).strip()),
                        "y": float(str(clean.get("Goal y", "")).strip()),
                        "z": float(str(clean.get("Goal z", "0")).strip()),
                        "planner": str(clean.get("Planner", "")).strip(),
                    }
                )
            except Exception:
                continue
    return goals


def make_xy_box(x_min: float, x_max: float, y_min: float, y_max: float) -> dict:
    width = max(0.0, float(x_max) - float(x_min))
    height = max(0.0, float(y_max) - float(y_min))
    return {
        "x_min": float(x_min),
        "x_max": float(x_max),
        "y_min": float(y_min),
        "y_max": float(y_max),
        "width_m": width,
        "height_m": height,
        "area_m2": width * height,
    }


def _parse_xy_vector_from_yaml_line(line: str) -> Optional[Tuple[float, float]]:
    if ":" not in line:
        return None
    raw = line.split(":", 1)[1].strip()
    if not raw:
        return None
    if raw.startswith("[") and raw.endswith("]"):
        raw = raw[1:-1]
    parts = [p.strip() for p in raw.split(",") if p.strip()]
    if len(parts) < 2:
        return None
    try:
        return float(parts[0]), float(parts[1])
    except Exception:
        return None


def load_config_xy_bbox(planner_config_path: Path) -> Optional[dict]:
    if not planner_config_path.exists() or (not planner_config_path.is_file()):
        return None
    min_xy = None
    max_xy = None
    for line in planner_config_path.read_text().splitlines():
        clean = line.strip()
        if clean.startswith("boundary/min:"):
            min_xy = _parse_xy_vector_from_yaml_line(clean)
        elif clean.startswith("boundary/max:"):
            max_xy = _parse_xy_vector_from_yaml_line(clean)
    if not min_xy or not max_xy:
        return None
    return make_xy_box(min_xy[0], max_xy[0], min_xy[1], max_xy[1])


def resolve_ground_truth_csv(script_dir: Path, world_name: str, explicit_csv: str) -> Path:
    del script_dir
    return resolve_canonical_ground_truth_csv(world_name, explicit_csv)


def auto_xy_limits(
    truth_rows: List[dict],
    map_rows: List[dict],
    goals: List[dict],
    config_bbox: Optional[dict],
    x_min_arg: Optional[float],
    x_max_arg: Optional[float],
    y_min_arg: Optional[float],
    y_max_arg: Optional[float],
) -> Tuple[float, float, float, float]:
    if None not in (x_min_arg, x_max_arg, y_min_arg, y_max_arg):
        x0 = float(x_min_arg)
        x1 = float(x_max_arg)
        y0 = float(y_min_arg)
        y1 = float(y_max_arg)
        if x1 > x0 and y1 > y0:
            return x0, x1, y0, y1

    if not config_bbox:
        raise RuntimeError(
            "Missing planner boundary for automatic limits. "
            "Provide --x-min/--x-max/--y-min/--y-max or valid --planner-config."
        )

    margin_xy_m = 5.0
    return (
        float(config_bbox["x_min"]) - margin_xy_m,
        float(config_bbox["x_max"]) + margin_xy_m,
        float(config_bbox["y_min"]) - margin_xy_m,
        float(config_bbox["y_max"]) + margin_xy_m,
    )


def path_stats(
    goals: List[dict],
    x_min: Optional[float] = None,
    x_max: Optional[float] = None,
    y_min: Optional[float] = None,
    y_max: Optional[float] = None,
    config_bbox: Optional[dict] = None,
) -> dict:
    stats = {
        "goal_count": len(goals),
        "path_length_xy_m": 0.0,
        "start": None,
        "end": None,
        "map_xy": None,
        "config_xy": None,
        "config_area_coverage_pct": None,
    }
    if not goals:
        return stats
    stats["start"] = {"x": goals[0]["x"], "y": goals[0]["y"], "z": goals[0]["z"]}
    stats["end"] = {"x": goals[-1]["x"], "y": goals[-1]["y"], "z": goals[-1]["z"]}
    if config_bbox:
        stats["config_xy"] = dict(config_bbox)
    if None not in (x_min, x_max, y_min, y_max):
        stats["map_xy"] = make_xy_box(float(x_min), float(x_max), float(y_min), float(y_max))
    map_xy = stats.get("map_xy") or {}
    cfg_xy = stats.get("config_xy") or {}
    map_area = float(map_xy.get("area_m2", 0.0))
    cfg_area = float(cfg_xy.get("area_m2", 0.0))
    if map_area > 1e-9 and cfg_area > 0.0:
        stats["config_area_coverage_pct"] = 100.0 * (cfg_area / map_area)
    total = 0.0
    for a, b in zip(goals[:-1], goals[1:]):
        total += math.hypot(b["x"] - a["x"], b["y"] - a["y"])
    stats["path_length_xy_m"] = total
    return stats


def load_rrt_log_stats(tree_log_path: Path, goal_log_path: Path) -> Optional[dict]:
    if not tree_log_path.exists() and not goal_log_path.exists():
        return None

    stats = {
        "tree_log_source": str(tree_log_path),
        "goal_log_source": str(goal_log_path),
        "tree_log_rows": 0,
        "goal_log_rows": 0,
        "planning_iteration_count": 0,
        "nodes_per_iteration_mean": None,
        "nodes_per_iteration_max": None,
        "max_dynamic_score": None,
        "tree_mode_counts": {},
        "goal_mode_counts": {},
        "selected_goal_source_counts": {},
        "selected_goal_nodes_in_tree": 0,
        "clear_goal_count": 0,
        "first_stamp_sec": None,
        "last_stamp_sec": None,
    }

    stamps = []
    nodes_per_iteration = {}
    max_dynamic_score = None

    if tree_log_path.exists():
        with tree_log_path.open("r", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                stats["tree_log_rows"] += 1
                iteration = str(row.get("planning_iteration", "")).strip()
                if iteration:
                    nodes_per_iteration[iteration] = nodes_per_iteration.get(iteration, 0) + 1

                mode = str(row.get("planner_mode", "")).strip() or "unknown"
                stats["tree_mode_counts"][mode] = stats["tree_mode_counts"].get(mode, 0) + 1

                if _get_int(row, "is_selected_goal", 0) == 1:
                    stats["selected_goal_nodes_in_tree"] += 1

                dynamic_score = _get_optional_float(row, "dynamic_score")
                if dynamic_score is not None:
                    max_dynamic_score = dynamic_score if max_dynamic_score is None else max(max_dynamic_score, dynamic_score)

                stamp = _get_optional_float(row, "stamp_sec")
                if stamp is not None:
                    stamps.append(stamp)

    if nodes_per_iteration:
        per_iter_values = list(nodes_per_iteration.values())
        stats["planning_iteration_count"] = len(per_iter_values)
        stats["nodes_per_iteration_mean"] = float(st.mean(per_iter_values))
        stats["nodes_per_iteration_max"] = int(max(per_iter_values))

    stats["max_dynamic_score"] = max_dynamic_score

    if goal_log_path.exists():
        with goal_log_path.open("r", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                stats["goal_log_rows"] += 1

                mode = str(row.get("planner_mode", "")).strip() or "unknown"
                stats["goal_mode_counts"][mode] = stats["goal_mode_counts"].get(mode, 0) + 1

                source = str(row.get("selected_goal_source", "")).strip() or "unknown"
                stats["selected_goal_source_counts"][source] = stats["selected_goal_source_counts"].get(source, 0) + 1

                if _get_int(row, "is_clear", 0) == 1:
                    stats["clear_goal_count"] += 1

                stamp = _get_optional_float(row, "stamp_sec")
                if stamp is not None:
                    stamps.append(stamp)

    if stamps:
        stats["first_stamp_sec"] = float(min(stamps))
        stats["last_stamp_sec"] = float(max(stamps))

    return stats


def load_rrt_goal_rows(goal_log_path: Path) -> List[dict]:
    if not goal_log_path.exists():
        return []

    rows = []
    with goal_log_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            iteration = _get_int(row, "planning_iteration", -1)
            if iteration < 0:
                continue
            rows.append(
                {
                    "planning_iteration": iteration,
                    "stamp_sec": _get_optional_float(row, "stamp_sec"),
                    "planner_mode": str(row.get("planner_mode", "")).strip() or "unknown",
                    "selected_goal_source": str(row.get("selected_goal_source", "")).strip() or "unknown",
                    "tree_node_count": _get_int(row, "tree_node_count", 0),
                    "best_dynamic_score": _get_optional_float(row, "best_dynamic_score"),
                    "best_x": _get_optional_float(row, "best_x"),
                    "best_y": _get_optional_float(row, "best_y"),
                    "selected_x": _get_optional_float(row, "selected_x"),
                    "selected_y": _get_optional_float(row, "selected_y"),
                }
            )
    rows.sort(key=lambda r: r["planning_iteration"])
    return rows


def load_rrt_tree_rows(tree_log_path: Path) -> Dict[int, List[dict]]:
    if not tree_log_path.exists():
        return {}

    by_iteration = {}
    with tree_log_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            iteration = _get_int(row, "planning_iteration", -1)
            if iteration < 0:
                continue
            by_iteration.setdefault(iteration, []).append(
                {
                    "node_id": _get_int(row, "node_id", -1),
                    "parent_id": _get_int(row, "parent_id", -1),
                    "depth": _get_int(row, "depth", 0),
                    "x": _get_float(row, "x"),
                    "y": _get_float(row, "y"),
                    "dynamic_score": _get_optional_float(row, "dynamic_score"),
                    "is_root": _get_int(row, "is_root", 0),
                    "is_best_node": _get_int(row, "is_best_node", 0),
                    "is_selected_goal": _get_int(row, "is_selected_goal", 0),
                    "is_best_branch": _get_int(row, "is_best_branch", 0),
                }
            )
    return by_iteration


def _rrt_source_color(source: str) -> str:
    if source == "rrt_tree":
        return "#1f77b4"
    if source == "cached_branch":
        return "#ff7f0e"
    if source == "frontier":
        return "#d62728"
    return "#777777"


def write_rrt_goal_timeline_svg(path: Path, goal_rows: List[dict], title: str) -> None:
    if not goal_rows:
        return

    width, height = 980, 420
    left, right, top, bottom = 70, 24, 42, 58
    plot_w = width - left - right
    plot_h = height - top - bottom

    iterations = [r["planning_iteration"] for r in goal_rows]
    scores = [r.get("best_dynamic_score") for r in goal_rows if r.get("best_dynamic_score") is not None]
    min_iter, max_iter = min(iterations), max(iterations)
    max_score = max(scores) if scores else 1.0
    max_score = max(max_score, 1.0)

    def sx(iteration: int) -> float:
        if max_iter == min_iter:
            return left + plot_w * 0.5
        return left + ((float(iteration) - min_iter) / float(max_iter - min_iter)) * plot_w

    def sy(score: Optional[float]) -> float:
        value = 0.0 if score is None else float(score)
        return top + (1.0 - (value / max_score)) * plot_h

    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" viewBox="0 0 {} {}">'.format(width, height, width, height),
        '<rect width="100%" height="100%" fill="white" />',
        '<text x="{}" y="26" font-family="Arial" font-size="18" font-weight="bold">{}</text>'.format(left, title),
        '<rect x="{}" y="{}" width="{}" height="{}" fill="#fafafa" stroke="#333" />'.format(left, top, plot_w, plot_h),
    ]

    for frac in [0.0, 0.25, 0.5, 0.75, 1.0]:
        y = top + (1.0 - frac) * plot_h
        score = frac * max_score
        lines.append('<line x1="{}" y1="{:.2f}" x2="{}" y2="{:.2f}" stroke="#e4e4e4" />'.format(left, y, left + plot_w, y))
        lines.append('<text x="{}" y="{:.2f}" font-family="Arial" font-size="11" text-anchor="end">{:.0f}</text>'.format(left - 8, y + 4, score))

    for frac in [0.0, 0.25, 0.5, 0.75, 1.0]:
        x = left + frac * plot_w
        iteration = min_iter + frac * (max_iter - min_iter)
        lines.append('<line x1="{:.2f}" y1="{}" x2="{:.2f}" y2="{}" stroke="#e4e4e4" />'.format(x, top, x, top + plot_h))
        lines.append('<text x="{:.2f}" y="{}" font-family="Arial" font-size="11" text-anchor="middle">{:.0f}</text>'.format(x, top + plot_h + 18, iteration))

    points = ["{:.2f},{:.2f}".format(sx(r["planning_iteration"]), sy(r.get("best_dynamic_score"))) for r in goal_rows]
    if len(points) >= 2:
        lines.append('<polyline fill="none" stroke="#4a4a4a" stroke-width="1.6" points="{}" />'.format(" ".join(points)))

    for row in goal_rows:
        source = row.get("selected_goal_source", "unknown")
        x = sx(row["planning_iteration"])
        y = sy(row.get("best_dynamic_score"))
        radius = 4.2 if source == "rrt_tree" else 3.4
        lines.append(
            '<circle cx="{:.2f}" cy="{:.2f}" r="{:.1f}" fill="{}" stroke="white" stroke-width="0.9" />'.format(
                x, y, radius, _rrt_source_color(source)
            )
        )

    legend_y = height - 24
    legend_x = left
    for label, color in [("goal da RRT", "#1f77b4"), ("ramo reaproveitado", "#ff7f0e"), ("frontier", "#d62728")]:
        lines.append('<circle cx="{}" cy="{}" r="5" fill="{}" />'.format(legend_x, legend_y, color))
        lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">{}</text>'.format(legend_x + 10, legend_y + 4, label))
        legend_x += 150

    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">iteracao do planner</text>'.format(left + plot_w * 0.43, height - 6))
    lines.append('<text x="18" y="{}" font-family="Arial" font-size="12" transform="rotate(-90 18,{})">best_dynamic_score</text>'.format(top + plot_h * 0.62, top + plot_h * 0.62))
    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n")


def _select_rrt_sample_iterations(iterations: List[int], max_samples: int = 6) -> List[int]:
    if len(iterations) <= max_samples:
        return list(iterations)
    selected = []
    for i in range(max_samples):
        idx = int(round(i * (len(iterations) - 1) / float(max_samples - 1)))
        value = iterations[idx]
        if value not in selected:
            selected.append(value)
    return selected


def write_rrt_tree_samples_svg(
    path: Path,
    tree_rows_by_iteration: Dict[int, List[dict]],
    goal_rows: List[dict],
    title: str,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
) -> None:
    iterations = sorted(tree_rows_by_iteration.keys())
    if not iterations:
        return

    selected_iterations = _select_rrt_sample_iterations(iterations, max_samples=6)
    goal_by_iteration = {r["planning_iteration"]: r for r in goal_rows}

    width, height = 1080, 720
    cols, rows = 3, 2
    margin_x, margin_y = 34, 56
    gutter_x, gutter_y = 24, 42
    cell_w = (width - 2 * margin_x - (cols - 1) * gutter_x) / cols
    cell_h = (height - margin_y - 28 - (rows - 1) * gutter_y) / rows
    pad = 26

    def sx(x: float, cell_left: float) -> float:
        return cell_left + pad + ((float(x) - x_min) / max(x_max - x_min, 1e-6)) * (cell_w - 2 * pad)

    def sy(y: float, cell_top: float) -> float:
        return cell_top + pad + (1.0 - ((float(y) - y_min) / max(y_max - y_min, 1e-6))) * (cell_h - 2 * pad)

    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" viewBox="0 0 {} {}">'.format(width, height, width, height),
        '<rect width="100%" height="100%" fill="white" />',
        '<text x="{}" y="30" font-family="Arial" font-size="18" font-weight="bold">{}</text>'.format(margin_x, title),
    ]

    for sample_idx, iteration in enumerate(selected_iterations):
        col = sample_idx % cols
        row = sample_idx // cols
        cell_left = margin_x + col * (cell_w + gutter_x)
        cell_top = margin_y + row * (cell_h + gutter_y)
        inner_left = cell_left + pad
        inner_top = cell_top + pad
        inner_w = cell_w - 2 * pad
        inner_h = cell_h - 2 * pad

        nodes = tree_rows_by_iteration.get(iteration, [])
        by_id = {n["node_id"]: n for n in nodes}
        goal = goal_by_iteration.get(iteration, {})

        lines.append('<rect x="{:.2f}" y="{:.2f}" width="{:.2f}" height="{:.2f}" fill="#fbfbfb" stroke="#333" />'.format(cell_left, cell_top, cell_w, cell_h))
        lines.append('<rect x="{:.2f}" y="{:.2f}" width="{:.2f}" height="{:.2f}" fill="white" stroke="#dddddd" />'.format(inner_left, inner_top, inner_w, inner_h))

        for frac in [0.25, 0.5, 0.75]:
            gx = inner_left + frac * inner_w
            gy = inner_top + frac * inner_h
            lines.append('<line x1="{:.2f}" y1="{:.2f}" x2="{:.2f}" y2="{:.2f}" stroke="#eeeeee" />'.format(gx, inner_top, gx, inner_top + inner_h))
            lines.append('<line x1="{:.2f}" y1="{:.2f}" x2="{:.2f}" y2="{:.2f}" stroke="#eeeeee" />'.format(inner_left, gy, inner_left + inner_w, gy))

        for node in nodes:
            parent = by_id.get(node["parent_id"])
            if not parent:
                continue
            color = "#d62728" if node.get("is_best_branch", 0) and parent.get("is_best_branch", 0) else "#b8b8b8"
            width_px = 2.0 if color == "#d62728" else 0.9
            lines.append(
                '<line x1="{:.2f}" y1="{:.2f}" x2="{:.2f}" y2="{:.2f}" stroke="{}" stroke-width="{:.1f}" opacity="0.85" />'.format(
                    sx(parent["x"], cell_left), sy(parent["y"], cell_top),
                    sx(node["x"], cell_left), sy(node["y"], cell_top),
                    color, width_px,
                )
            )

        for node in nodes:
            fill = "#666666"
            radius = 2.2
            if node.get("is_root", 0):
                fill = "#111111"
                radius = 3.4
            if node.get("is_best_branch", 0):
                fill = "#d62728"
                radius = 3.0
            if node.get("is_best_node", 0):
                fill = "#9467bd"
                radius = 4.4
            if node.get("is_selected_goal", 0):
                fill = "#2ca02c"
                radius = 4.8
            lines.append('<circle cx="{:.2f}" cy="{:.2f}" r="{:.1f}" fill="{}" opacity="0.95" />'.format(sx(node["x"], cell_left), sy(node["y"], cell_top), radius, fill))

        if goal.get("selected_x") is not None and goal.get("selected_y") is not None and goal.get("selected_goal_source") == "cached_branch":
            x = sx(goal["selected_x"], cell_left)
            y = sy(goal["selected_y"], cell_top)
            lines.append('<circle cx="{:.2f}" cy="{:.2f}" r="5.2" fill="#ff7f0e" stroke="white" stroke-width="1.2" />'.format(x, y))

        score = goal.get("best_dynamic_score")
        score_text = "score=N/A" if score is None else "score={:.1f}".format(score)
        label = "iter {} | {} | {}".format(iteration, goal.get("selected_goal_source", "unknown"), score_text)
        lines.append('<text x="{:.2f}" y="{:.2f}" font-family="Arial" font-size="12" font-weight="bold">{}</text>'.format(cell_left + 8, cell_top + 18, label))
        lines.append('<text x="{:.2f}" y="{:.2f}" font-family="Arial" font-size="11" fill="#555">nos={}</text>'.format(cell_left + 8, cell_top + cell_h - 8, len(nodes)))

    legend_y = height - 16
    legend_x = margin_x
    for label, color in [("raiz", "#111111"), ("ramo melhor", "#d62728"), ("best node", "#9467bd"), ("goal RRT", "#2ca02c"), ("goal cache", "#ff7f0e")]:
        lines.append('<circle cx="{}" cy="{}" r="5" fill="{}" />'.format(legend_x, legend_y, color))
        lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">{}</text>'.format(legend_x + 10, legend_y + 4, label))
        legend_x += 118

    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n")


def _pair_dist(a: dict, b: dict) -> float:
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"])


def _sorted_pairs(truth: List[dict], maps: List[dict]) -> List[Tuple[float, int, int]]:
    out = []
    for ti, t in enumerate(truth):
        for mi, m in enumerate(maps):
            out.append((_pair_dist(t, m), ti, mi))
    out.sort(key=lambda x: x[0])
    return out


def _one_to_one(
    sorted_pairs: List[Tuple[float, int, int]],
    n_truth: int,
    n_map: int,
    threshold_m: float,
) -> Tuple[List[Tuple[float, int, int]], List[int], List[int]]:
    used_truth = set()
    used_map = set()
    kept = []
    for dist, ti, mi in sorted_pairs:
        if dist > threshold_m:
            break
        if ti in used_truth or mi in used_map:
            continue
        used_truth.add(ti)
        used_map.add(mi)
        kept.append((dist, ti, mi))

    fn = [i for i in range(n_truth) if i not in used_truth]
    fp = [i for i in range(n_map) if i not in used_map]
    return kept, fn, fp


def _prf(tp: int, fn: int, fp: int) -> Tuple[float, float, float]:
    precision = float(tp) / float(tp + fp) if (tp + fp) > 0 else 0.0
    recall = float(tp) / float(tp + fn) if (tp + fn) > 0 else 0.0
    f1 = (2.0 * precision * recall / (precision + recall)) if (precision + recall) > 0 else 0.0
    return precision, recall, f1


def _diameter_stats(rows: List[dict], gt_diameter_m: float) -> dict:
    stats = {
        "population_count": len(rows),
        "confirmed_count": sum(int(r.get("confirmed", 0)) for r in rows),
        "suspect_merge_count": sum(int(r.get("suspect_merge", 0)) for r in rows),
        "ground_truth_diameter_m": gt_diameter_m,
        "diameter_mean_m": None,
        "diameter_min_m": None,
        "diameter_max_m": None,
        "diameter_bias_m": None,
        "diameter_mae_m": None,
        "diameter_rmse_m": None,
        "diameter_rmse_cm": None,
        "confidence_mean": None,
        "std_xy_mean_m": None,
        "hits_median": None,
        "hits_max": None,
    }

    if not rows:
        return stats

    diameters = [r["diameter_m"] for r in rows]
    diameter_errors = [d - gt_diameter_m for d in diameters]
    diameter_abs_errors = [abs(e) for e in diameter_errors]
    diameter_sq_errors = [e * e for e in diameter_errors]

    stats.update(
        {
            "diameter_mean_m": sum(diameters) / float(len(diameters)),
            "diameter_min_m": min(diameters),
            "diameter_max_m": max(diameters),
            "diameter_bias_m": sum(diameter_errors) / float(len(diameter_errors)),
            "diameter_mae_m": sum(diameter_abs_errors) / float(len(diameter_abs_errors)),
            "diameter_rmse_m": math.sqrt(sum(diameter_sq_errors) / float(len(diameter_sq_errors))),
            "confidence_mean": sum(r["confidence"] for r in rows) / float(len(rows)),
            "std_xy_mean_m": sum(r["std_xy"] for r in rows) / float(len(rows)),
            "hits_median": st.median([r["hits"] for r in rows]),
            "hits_max": max(r["hits"] for r in rows),
        }
    )
    stats["diameter_rmse_cm"] = stats["diameter_rmse_m"] * 100.0
    return stats


def compute_metrics(
    truth: List[dict],
    maps: List[dict],
    selected_threshold_m: float,
    ground_truth_diameter_m: float,
) -> Tuple[dict, List[dict]]:
    if not truth:
        raise RuntimeError("Ground-truth CSV has no rows.")
    if not maps:
        raise RuntimeError("tree_map_final.csv has no rows.")

    nearest = []
    for t in truth:
        m = min(maps, key=lambda mm: _pair_dist(t, mm))
        dist = _pair_dist(t, m)
        nearest.append(
            {
                "tree_id": t["tree_id"],
                "nearest_map_id": m["map_id"],
                "distance_m": dist,
                "gt_x": t["x"],
                "gt_y": t["y"],
                "map_x": m["x"],
                "map_y": m["y"],
            }
        )

    dists = sorted([n["distance_m"] for n in nearest])
    p90_idx = int(round((len(dists) - 1) * 0.9))
    nearest_stats = {
        "count": len(dists),
        "mean_m": sum(dists) / float(len(dists)),
        "median_m": st.median(dists),
        "p90_m": dists[p90_idx],
        "max_m": max(dists),
        "lte_0_3m": sum(d <= 0.3 for d in dists),
        "lte_0_5m": sum(d <= 0.5 for d in dists),
        "lte_0_8m": sum(d <= 0.8 for d in dists),
    }

    sorted_pairs = _sorted_pairs(truth, maps)
    threshold_eval = {}
    for th in (0.3, 0.5, 0.6, 0.8):
        keep, fn, fp = _one_to_one(sorted_pairs, len(truth), len(maps), th)
        prec, rec, f1 = _prf(len(keep), len(fn), len(fp))
        threshold_eval[str(th)] = {
            "tp": len(keep),
            "fn": len(fn),
            "fp": len(fp),
            "precision": prec,
            "recall": rec,
            "f1": f1,
        }

    keep, fn, fp = _one_to_one(sorted_pairs, len(truth), len(maps), selected_threshold_m)
    precision, recall, f1 = _prf(len(keep), len(fn), len(fp))

    matching_rows = []
    for dist, ti, mi in sorted(keep, key=lambda x: x[0]):
        t = truth[ti]
        m = maps[mi]
        matching_rows.append(
            {
                "tree_id": t["tree_id"],
                "map_id": m["map_id"],
                "distance_m": dist,
                "gt_x": t["x"],
                "gt_y": t["y"],
                "map_x": m["x"],
                "map_y": m["y"],
                "map_diameter_m": m["diameter_m"],
                "map_confidence": m["confidence"],
                "map_hits": m["hits"],
                "map_std_xy": m["std_xy"],
            }
        )

    worst_matches = sorted(matching_rows, key=lambda r: r["distance_m"], reverse=True)[:5]
    unmatched_truth_ids = [truth[i]["tree_id"] for i in fn]
    unmatched_map_ids = [maps[i]["map_id"] for i in fp]

    gt_d = max(float(ground_truth_diameter_m), 0.0)
    confirmed_maps = [m for m in maps if int(m.get("confirmed", 0)) == 1]
    matched_tp_maps = [maps[mi] for _, _, mi in keep]

    map_stats_all = _diameter_stats(maps, gt_d)
    map_stats_confirmed = _diameter_stats(confirmed_maps, gt_d)
    map_stats_matched_tp = _diameter_stats(matched_tp_maps, gt_d)
    map_stats = map_stats_all
    map_stats_views = {
        "all_map": map_stats_all,
        "confirmed_only": map_stats_confirmed,
        "matched_tp_only": map_stats_matched_tp,
    }

    metrics = {
        "truth_count": len(truth),
        "map_count": len(maps),
        "nearest_stats": nearest_stats,
        "threshold_eval": threshold_eval,
        "selected_threshold_m": selected_threshold_m,
        "selected_eval": {
            "tp": len(keep),
            "fn": len(fn),
            "fp": len(fp),
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "unmatched_truth_ids": unmatched_truth_ids,
            "unmatched_map_ids": unmatched_map_ids,
            "worst_matches": [
                {
                    "tree_id": r["tree_id"],
                    "map_id": r["map_id"],
                    "distance_m": r["distance_m"],
                }
                for r in worst_matches
            ],
        },
        "map_stats": map_stats,
        "map_stats_views": map_stats_views,
    }
    return metrics, matching_rows


def write_matching_csv(path: Path, rows: List[dict]) -> None:
    fields = [
        "tree_id",
        "map_id",
        "distance_m",
        "gt_x",
        "gt_y",
        "map_x",
        "map_y",
        "map_diameter_m",
        "map_confidence",
        "map_hits",
        "map_std_xy",
    ]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in rows:
            out = dict(row)
            out["distance_m"] = "{:.6f}".format(row["distance_m"])
            out["gt_x"] = "{:.6f}".format(row["gt_x"])
            out["gt_y"] = "{:.6f}".format(row["gt_y"])
            out["map_x"] = "{:.6f}".format(row["map_x"])
            out["map_y"] = "{:.6f}".format(row["map_y"])
            out["map_diameter_m"] = "{:.6f}".format(row["map_diameter_m"])
            out["map_confidence"] = "{:.6f}".format(row["map_confidence"])
            out["map_std_xy"] = "{:.6f}".format(row["map_std_xy"])
            w.writerow(out)


def load_map_history(csv_path: Path) -> List[dict]:
    snapshots = {}
    order = []
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            exported_at_sec = _get_float(row, "exported_at_sec")
            raw_seq = str(row.get("export_seq", "")).strip()
            key = "{}:{:.3f}".format(raw_seq, exported_at_sec) if raw_seq else "{:.3f}".format(exported_at_sec)
            if key not in snapshots:
                snapshots[key] = {
                    "export_seq": _get_int(row, "export_seq", len(order) + 1),
                    "exported_at_sec": exported_at_sec,
                    "reason": str(row.get("reason", "")).strip(),
                    "frame_id": str(row.get("frame_id", "")).strip(),
                    "count_total": _get_int(row, "count_total"),
                    "count_confirmed": _get_int(row, "count_confirmed"),
                    "maps": [],
                }
                order.append(key)

            map_id = str(row.get("map_id", "")).strip()
            if not map_id:
                continue

            snapshots[key]["maps"].append(
                {
                    "map_id": _get_int(row, "map_id", -1),
                    "x": _get_float(row, "x"),
                    "y": _get_float(row, "y"),
                    "z": _get_float(row, "z"),
                    "diameter_m": _get_float(row, "diameter_m"),
                    "hits": _get_int(row, "hits"),
                    "std_xy": _get_float(row, "std_xy"),
                    "std_diameter": _get_float(row, "std_diameter"),
                    "confidence": _get_float(row, "confidence"),
                    "confirmed": _get_int(row, "confirmed"),
                    "suspect_merge": _get_int(row, "suspect_merge"),
                }
            )

    out = [snapshots[key] for key in order]
    out.sort(key=lambda s: (s["exported_at_sec"], s["export_seq"]))
    for snap in out:
        if snap["count_total"] <= 0 and snap["maps"]:
            snap["count_total"] = len(snap["maps"])
        if snap["count_confirmed"] <= 0 and snap["maps"]:
            snap["count_confirmed"] = sum(int(m.get("confirmed", 0)) for m in snap["maps"])
    return out


def _seconds_to_minutes(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    return float(value) / 60.0


def compute_temporal_discovery(
    truth: List[dict],
    snapshots: List[dict],
    selected_threshold_m: float,
) -> Tuple[Optional[dict], List[dict]]:
    if not truth or not snapshots:
        return None, []

    t0 = snapshots[0]["exported_at_sec"]
    cumulative_truth_ids = set()
    rows = []

    for snap in snapshots:
        rel_sec = max(0.0, snap["exported_at_sec"] - t0)
        maps_all = list(snap.get("maps", []))
        maps_confirmed = [m for m in maps_all if int(m.get("confirmed", 0)) == 1]

        sorted_pairs = _sorted_pairs(truth, maps_confirmed)
        keep, fn, fp = _one_to_one(sorted_pairs, len(truth), len(maps_confirmed), selected_threshold_m)
        precision, recall, f1 = _prf(len(keep), len(fn), len(fp))

        matched_truth_ids = []
        match_errors = []
        for dist, ti, _ in keep:
            matched_truth_ids.append(truth[ti]["tree_id"])
            match_errors.append(dist)
        cumulative_truth_ids.update(matched_truth_ids)

        cumulative_tp = len(cumulative_truth_ids)
        cumulative_recall = float(cumulative_tp) / float(len(truth))
        mean_match_error = sum(match_errors) / float(len(match_errors)) if match_errors else None

        rows.append(
            {
                "export_seq": snap["export_seq"],
                "time_sec": rel_sec,
                "time_min": _seconds_to_minutes(rel_sec),
                "total_count": snap["count_total"],
                "confirmed_count": snap["count_confirmed"],
                "candidate_count": max(int(snap["count_total"]) - int(snap["count_confirmed"]), 0),
                "tp": len(keep),
                "fn": len(fn),
                "fp": len(fp),
                "precision": precision,
                "recall": recall,
                "f1": f1,
                "cumulative_tp": cumulative_tp,
                "cumulative_recall": cumulative_recall,
                "mean_match_error_m": mean_match_error,
            }
        )

    duration_sec = rows[-1]["time_sec"] if rows else 0.0
    auc = 0.0
    if len(rows) == 1:
        auc = rows[0]["cumulative_recall"]
    elif duration_sec > 0.0:
        for prev, cur in zip(rows[:-1], rows[1:]):
            dt_sec = max(cur["time_sec"] - prev["time_sec"], 0.0)
            auc += 0.5 * (prev["cumulative_recall"] + cur["cumulative_recall"]) * dt_sec
        auc = auc / duration_sec

    def first_time_for_recall(target: float) -> Optional[float]:
        for row in rows:
            if row["cumulative_recall"] >= target:
                return row["time_sec"]
        return None

    final = rows[-1]
    summary = {
        "source": "tree_map_history.csv",
        "scope": "confirmed_only",
        "match_threshold_m": selected_threshold_m,
        "snapshot_count": len(rows),
        "target_count": len(truth),
        "duration_sec": duration_sec,
        "duration_min": _seconds_to_minutes(duration_sec),
        "final_tp": final["tp"],
        "final_fn": final["fn"],
        "final_fp": final["fp"],
        "final_precision": final["precision"],
        "final_recall": final["recall"],
        "final_f1": final["f1"],
        "final_cumulative_tp": final["cumulative_tp"],
        "final_cumulative_recall": final["cumulative_recall"],
        "peak_cumulative_tp": max(r["cumulative_tp"] for r in rows),
        "peak_cumulative_recall": max(r["cumulative_recall"] for r in rows),
        "recall_auc_normalized": auc,
        "time_to_first_tp_sec": first_time_for_recall(1.0 / float(len(truth))),
        "time_to_25pct_sec": first_time_for_recall(0.25),
        "time_to_50pct_sec": first_time_for_recall(0.50),
        "time_to_80pct_sec": first_time_for_recall(0.80),
        "time_to_100pct_sec": first_time_for_recall(1.00),
    }
    for key in ("time_to_first_tp_sec", "time_to_25pct_sec", "time_to_50pct_sec", "time_to_80pct_sec", "time_to_100pct_sec"):
        summary[key.replace("_sec", "_min")] = _seconds_to_minutes(summary[key])

    return summary, rows


def write_temporal_timeseries_csv(path: Path, rows: List[dict]) -> None:
    fields = [
        "export_seq",
        "time_sec",
        "time_min",
        "total_count",
        "confirmed_count",
        "candidate_count",
        "tp",
        "fn",
        "fp",
        "precision",
        "recall",
        "f1",
        "cumulative_tp",
        "cumulative_recall",
        "mean_match_error_m",
    ]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in rows:
            out = dict(row)
            for key in ("time_sec", "time_min", "precision", "recall", "f1", "cumulative_recall"):
                out[key] = "{:.6f}".format(row[key])
            out["mean_match_error_m"] = "" if row["mean_match_error_m"] is None else "{:.6f}".format(row["mean_match_error_m"])
            w.writerow(out)


def write_temporal_summary_csv(path: Path, summary: dict) -> None:
    fields = [
        "source",
        "scope",
        "match_threshold_m",
        "snapshot_count",
        "target_count",
        "duration_sec",
        "duration_min",
        "final_tp",
        "final_fn",
        "final_fp",
        "final_precision",
        "final_recall",
        "final_f1",
        "final_cumulative_tp",
        "final_cumulative_recall",
        "peak_cumulative_tp",
        "peak_cumulative_recall",
        "recall_auc_normalized",
        "time_to_first_tp_sec",
        "time_to_first_tp_min",
        "time_to_25pct_sec",
        "time_to_25pct_min",
        "time_to_50pct_sec",
        "time_to_50pct_min",
        "time_to_80pct_sec",
        "time_to_80pct_min",
        "time_to_100pct_sec",
        "time_to_100pct_min",
    ]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerow(summary)


def compute_snapshot_metrics(
    snapshot_root: Path,
    truth: List[dict],
    selected_threshold_m: float,
) -> Tuple[Optional[dict], List[dict]]:
    if not snapshot_root.exists() or not snapshot_root.is_dir():
        return None, []

    rows = []
    for snap_dir in sorted([p for p in snapshot_root.iterdir() if p.is_dir()], key=lambda p: p.name):
        map_csv = snap_dir / "tree_map_final.csv"
        if not map_csv.exists():
            continue
        try:
            maps = load_map(map_csv)
        except Exception:
            continue

        meta_json = snap_dir / "tree_map_final.json"
        exported_at_sec = None
        if meta_json.exists():
            try:
                payload = json.loads(meta_json.read_text())
                exported_at_sec = float(payload.get("exported_at_sec"))
            except Exception:
                exported_at_sec = None
        if exported_at_sec is None:
            exported_at_sec = float(snap_dir.stat().st_mtime)

        confirmed_maps = [m for m in maps if int(m.get("confirmed", 0)) == 1]
        sorted_pairs = _sorted_pairs(truth, confirmed_maps)
        keep, fn, fp = _one_to_one(sorted_pairs, len(truth), len(confirmed_maps), selected_threshold_m)
        precision, recall, f1 = _prf(len(keep), len(fn), len(fp))
        goals = load_path_goals(snap_dir / "path.csv")
        rows.append(
            {
                "snapshot": snap_dir.name,
                "exported_at_sec": exported_at_sec,
                "total_count": len(maps),
                "confirmed_count": len(confirmed_maps),
                "candidate_count": max(len(maps) - len(confirmed_maps), 0),
                "tp": len(keep),
                "fn": len(fn),
                "fp": len(fp),
                "precision": precision,
                "recall": recall,
                "f1": f1,
                "goal_count": len(goals),
                "path_length_xy_m": path_stats(goals)["path_length_xy_m"],
            }
        )

    if not rows:
        return None, []
    rows.sort(key=lambda r: (r["exported_at_sec"], r["snapshot"]))
    t0 = rows[0]["exported_at_sec"]
    for row in rows:
        row["time_sec"] = max(row["exported_at_sec"] - t0, 0.0)
        row["time_min"] = _seconds_to_minutes(row["time_sec"])

    best_recall = max(r["recall"] for r in rows)
    best_f1 = max(r["f1"] for r in rows)
    summary = {
        "source": str(snapshot_root),
        "snapshot_count": len(rows),
        "duration_sec": rows[-1]["time_sec"],
        "duration_min": rows[-1]["time_min"],
        "final_snapshot": rows[-1]["snapshot"],
        "final_tp": rows[-1]["tp"],
        "final_fn": rows[-1]["fn"],
        "final_fp": rows[-1]["fp"],
        "final_precision": rows[-1]["precision"],
        "final_recall": rows[-1]["recall"],
        "final_f1": rows[-1]["f1"],
        "best_recall": best_recall,
        "best_f1": best_f1,
    }
    return summary, rows


def write_snapshot_summary_csv(path: Path, rows: List[dict]) -> None:
    fields = [
        "snapshot",
        "time_sec",
        "time_min",
        "total_count",
        "confirmed_count",
        "candidate_count",
        "tp",
        "fn",
        "fp",
        "precision",
        "recall",
        "f1",
        "goal_count",
        "path_length_xy_m",
    ]
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in rows:
            out = dict(row)
            for key in ("time_sec", "time_min", "precision", "recall", "f1", "path_length_xy_m"):
                out[key] = "{:.6f}".format(row[key])
            w.writerow({k: out.get(k, "") for k in fields})


def write_temporal_svg(path: Path, rows: List[dict], title: str) -> None:
    width = 900
    height = 460
    left = 70
    right = 30
    top = 45
    bottom = 65
    plot_w = width - left - right
    plot_h = height - top - bottom
    max_time = max([r["time_min"] for r in rows] + [1.0])

    def sx(t_min: float) -> float:
        return left + (float(t_min) / max_time) * plot_w

    def sy(v: float) -> float:
        return top + (1.0 - max(0.0, min(1.0, float(v)))) * plot_h

    def poly(metric: str, color: str) -> str:
        points = " ".join("{:.2f},{:.2f}".format(sx(r["time_min"]), sy(r[metric])) for r in rows)
        return '<polyline fill="none" stroke="{}" stroke-width="2.5" points="{}" />'.format(color, points)

    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" viewBox="0 0 {} {}">'.format(width, height, width, height),
        '<rect width="100%" height="100%" fill="white" />',
        '<text x="{}" y="28" font-family="Arial" font-size="18" font-weight="bold">{}</text>'.format(left, title),
        '<line x1="{0}" y1="{1}" x2="{2}" y2="{1}" stroke="#333" />'.format(left, top + plot_h, left + plot_w),
        '<line x1="{0}" y1="{1}" x2="{0}" y2="{2}" stroke="#333" />'.format(left, top, top + plot_h),
    ]
    for value in (0.0, 0.25, 0.5, 0.75, 1.0):
        y = sy(value)
        lines.append('<line x1="{0}" y1="{1:.2f}" x2="{2}" y2="{1:.2f}" stroke="#ddd" />'.format(left, y, left + plot_w))
        lines.append('<text x="18" y="{:.2f}" font-family="Arial" font-size="12">{:.0f}%</text>'.format(y + 4, value * 100.0))
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        x = left + frac * plot_w
        t = frac * max_time
        lines.append('<line x1="{0:.2f}" y1="{1}" x2="{0:.2f}" y2="{2}" stroke="#ddd" />'.format(x, top, top + plot_h))
        lines.append('<text x="{:.2f}" y="{}" font-family="Arial" font-size="12" text-anchor="middle">{:.1f}</text>'.format(x, height - 35, t))
    lines.append(poly("cumulative_recall", "#1f77b4"))
    lines.append(poly("precision", "#ff7f0e"))
    lines.append(poly("f1", "#2ca02c"))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">tempo (min)</text>'.format(left + plot_w * 0.45, height - 12))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12" fill="#1f77b4">recall acumulado</text>'.format(left + 15, top + 18))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12" fill="#ff7f0e">precision</text>'.format(left + 150, top + 18))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12" fill="#2ca02c">F1</text>'.format(left + 235, top + 18))
    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n")


def write_route_tree_svg(
    path: Path,
    truth: List[dict],
    maps: List[dict],
    goals: List[dict],
    title: str,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    config_bbox: Optional[dict] = None,
) -> None:
    width = 900
    height = 680
    left = 70
    right = 30
    top = 55
    bottom = 75
    plot_w = width - left - right
    plot_h = height - top - bottom

    def sx(x: float) -> float:
        return left + ((float(x) - x_min) / max(x_max - x_min, 1e-6)) * plot_w

    def sy(y: float) -> float:
        return top + (1.0 - ((float(y) - y_min) / max(y_max - y_min, 1e-6))) * plot_h

    map_w = max(0.0, float(x_max) - float(x_min))
    map_h = max(0.0, float(y_max) - float(y_min))
    map_area = map_w * map_h

    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" viewBox="0 0 {} {}">'.format(width, height, width, height),
        '<rect width="100%" height="100%" fill="white" />',
        '<text x="{}" y="32" font-family="Arial" font-size="18" font-weight="bold">{}</text>'.format(left, title),
        '<rect x="{0}" y="{1}" width="{2}" height="{3}" fill="#fafafa" stroke="#333" />'.format(left, top, plot_w, plot_h),
    ]

    tick_count = 5
    for i in range(tick_count + 1):
        fx = float(i) / float(tick_count)
        x_val = x_min + fx * (x_max - x_min)
        y_val = y_min + fx * (y_max - y_min)
        x = left + fx * plot_w
        y = top + (1.0 - fx) * plot_h
        lines.append('<line x1="{0:.2f}" y1="{1}" x2="{0:.2f}" y2="{2}" stroke="#e5e5e5" />'.format(x, top, top + plot_h))
        lines.append('<line x1="{0}" y1="{1:.2f}" x2="{2}" y2="{1:.2f}" stroke="#e5e5e5" />'.format(left, y, left + plot_w))
        lines.append('<text x="{:.2f}" y="{}" font-family="Arial" font-size="11" text-anchor="middle">{:.1f}</text>'.format(x, top + plot_h + 18, x_val))
        lines.append('<text x="{}" y="{:.2f}" font-family="Arial" font-size="11" text-anchor="end">{:.1f}</text>'.format(left - 8, y + 4, y_val))

    map_info = "mapa XY: x[{:.1f},{:.1f}] y[{:.1f},{:.1f}] | tamanho {:.1f} x {:.1f} m | area {:.1f} m2".format(
        x_min, x_max, y_min, y_max, map_w, map_h, map_area
    )
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12" fill="#333">{}</text>'.format(left + 8, top + 16, map_info))
    if config_bbox:
        cfg_info = "bbox configuracao XY: x[{:.1f},{:.1f}] y[{:.1f},{:.1f}] | area {:.1f} m2".format(
            float(config_bbox["x_min"]),
            float(config_bbox["x_max"]),
            float(config_bbox["y_min"]),
            float(config_bbox["y_max"]),
            float(config_bbox["area_m2"]),
        )
        lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12" fill="#6b46a1">{}</text>'.format(left + 8, top + 32, cfg_info))
        cb_left = sx(float(config_bbox["x_min"]))
        cb_right = sx(float(config_bbox["x_max"]))
        cb_top = sy(float(config_bbox["y_max"]))
        cb_bottom = sy(float(config_bbox["y_min"]))
        lines.append(
            '<rect x="{:.2f}" y="{:.2f}" width="{:.2f}" height="{:.2f}" fill="none" stroke="#9467bd" stroke-width="2" stroke-dasharray="7,5" />'.format(
                min(cb_left, cb_right),
                min(cb_top, cb_bottom),
                abs(cb_right - cb_left),
                abs(cb_bottom - cb_top),
            )
        )

    if goals:
        points = " ".join("{:.2f},{:.2f}".format(sx(g["x"]), sy(g["y"])) for g in goals)
        lines.append('<polyline fill="none" stroke="#444" stroke-width="2" stroke-linejoin="round" stroke-linecap="round" points="{}" />'.format(points))
        for g in goals:
            r = 2.0 if g["seq"] not in (1, len(goals)) else 4.0
            lines.append('<circle cx="{:.2f}" cy="{:.2f}" r="{:.1f}" fill="#444" opacity="0.45" />'.format(sx(g["x"]), sy(g["y"]), r))
        lines.append('<circle cx="{:.2f}" cy="{:.2f}" r="6" fill="#2ca02c" stroke="white" stroke-width="1.5" />'.format(sx(goals[0]["x"]), sy(goals[0]["y"])))
        lines.append('<circle cx="{:.2f}" cy="{:.2f}" r="6" fill="#d62728" stroke="white" stroke-width="1.5" />'.format(sx(goals[-1]["x"]), sy(goals[-1]["y"])))

    for t in truth:
        lines.append('<circle cx="{:.2f}" cy="{:.2f}" r="7" fill="none" stroke="#1f77b4" stroke-width="2" />'.format(sx(t["x"]), sy(t["y"])))
        if t.get("tree_id") is not None:
            lines.append('<text x="{:.2f}" y="{:.2f}" font-family="Arial" font-size="10" fill="#1f77b4">{}</text>'.format(sx(t["x"]) + 8, sy(t["y"]) - 8, t["tree_id"]))

    for m in maps:
        color = "#ff7f0e" if int(m.get("confirmed", 0)) == 1 else "#999999"
        x = sx(m["x"])
        y = sy(m["y"])
        lines.append('<line x1="{:.2f}" y1="{:.2f}" x2="{:.2f}" y2="{:.2f}" stroke="{}" stroke-width="2" />'.format(x - 5, y - 5, x + 5, y + 5, color))
        lines.append('<line x1="{:.2f}" y1="{:.2f}" x2="{:.2f}" y2="{:.2f}" stroke="{}" stroke-width="2" />'.format(x - 5, y + 5, x + 5, y - 5, color))

    legend_y = height - 42
    lines.append('<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="#444" stroke-width="2" />'.format(left, legend_y, left + 38, legend_y))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">rota de goals</text>'.format(left + 45, legend_y + 4))
    lines.append('<circle cx="{}" cy="{}" r="6" fill="none" stroke="#1f77b4" stroke-width="2" />'.format(left + 170, legend_y))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">GT</text>'.format(left + 184, legend_y + 4))
    lines.append('<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="#ff7f0e" stroke-width="2" />'.format(left + 230, legend_y - 5, left + 240, legend_y + 5))
    lines.append('<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="#ff7f0e" stroke-width="2" />'.format(left + 230, legend_y + 5, left + 240, legend_y - 5))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">arvore detectada</text>'.format(left + 250, legend_y + 4))
    lines.append('<circle cx="{}" cy="{}" r="5" fill="#2ca02c" />'.format(left + 390, legend_y))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">inicio</text>'.format(left + 402, legend_y + 4))
    lines.append('<circle cx="{}" cy="{}" r="5" fill="#d62728" />'.format(left + 460, legend_y))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">fim</text>'.format(left + 472, legend_y + 4))
    lines.append('<rect x="{:.1f}" y="{:.1f}" width="12" height="12" fill="none" stroke="#333" stroke-width="1.5" />'.format(left + 520, legend_y - 6))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">bbox mapa (analise)</text>'.format(left + 538, legend_y + 4))
    lines.append('<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="#9467bd" stroke-width="2" stroke-dasharray="7,5" />'.format(left + 700, legend_y, left + 736, legend_y))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">bbox configuracao</text>'.format(left + 742, legend_y + 4))
    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n")


def write_summary_md(path: Path, exp_name: str, metrics: dict) -> None:
    ns = metrics["nearest_stats"]
    se = metrics["selected_eval"]
    ms = metrics["map_stats"]
    ms_views = metrics.get("map_stats_views", {})

    def _fmt_m(v: Optional[float], decimals: int = 3) -> str:
        if v is None:
            return "N/A"
        return ("{:.%df}" % decimals).format(v)

    def _fmt_min(v: Optional[float]) -> str:
        if v is None:
            return "-"
        return "{:.2f} min".format(v)

    def _fmt_num(v: Optional[float], decimals: int = 2) -> str:
        if v is None:
            return "N/A"
        return ("{:.%df}" % decimals).format(v)

    def _fmt_counts(d: dict) -> str:
        if not d:
            return "N/A"
        return ", ".join("{}={}".format(k, d[k]) for k in sorted(d.keys()))

    def _append_map_stats(lines_out: List[str], title: str, s: dict) -> None:
        lines_out.append("### {}".format(title))
        lines_out.append("- Populacao avaliada: {}".format(s.get("population_count", 0)))
        lines_out.append("- Confirmadas: {}".format(s.get("confirmed_count", 0)))
        lines_out.append("- Suspeitas de merge: {}".format(s.get("suspect_merge_count", 0)))
        lines_out.append("- Diametro medio: {} m".format(_fmt_m(s.get("diameter_mean_m"))))
        lines_out.append(
            "- Diametro min/max: {} / {} m".format(
                _fmt_m(s.get("diameter_min_m")),
                _fmt_m(s.get("diameter_max_m")),
            )
        )
        lines_out.append("- GT diametro de referencia: {} m".format(_fmt_m(s.get("ground_truth_diameter_m"))))
        lines_out.append("- Erro medio absoluto (MAE): {} m".format(_fmt_m(s.get("diameter_mae_m"))))
        lines_out.append(
            "- RMSE de diametro: {} m ({} cm)".format(
                _fmt_m(s.get("diameter_rmse_m")),
                _fmt_m(s.get("diameter_rmse_cm"), decimals=2),
            )
        )
        lines_out.append("- Bias de diametro: {} m".format(_fmt_m(s.get("diameter_bias_m"))))
        lines_out.append("- Confianca media: {}".format(_fmt_m(s.get("confidence_mean"))))
        lines_out.append("- std_xy medio: {} m".format(_fmt_m(s.get("std_xy_mean_m"))))
        if s.get("hits_median") is None:
            lines_out.append("- hits mediana/max: N/A / N/A")
        else:
            lines_out.append(
                "- hits mediana/max: {} / {}".format(
                    _fmt_m(float(s.get("hits_median")), decimals=1),
                    int(s.get("hits_max", 0)),
                )
            )

    lines = []
    lines.append("# {} - Avaliacao vs Ground Truth".format(exp_name))
    lines.append("")
    lines.append("## Contagens")
    lines.append("- Ground truth: {}".format(metrics["truth_count"]))
    lines.append("- Mapa detectado (CSV): {}".format(metrics["map_count"]))
    lines.append("")
    lines.append("## Erro de Posicao (GT -> mapa mais proximo)")
    lines.append("- Media: {:.3f} m".format(ns["mean_m"]))
    lines.append("- Mediana: {:.3f} m".format(ns["median_m"]))
    lines.append("- P90: {:.3f} m".format(ns["p90_m"]))
    lines.append("- Maximo: {:.3f} m".format(ns["max_m"]))
    lines.append("- <= 0.3 m: {}/{}".format(ns["lte_0_3m"], metrics["truth_count"]))
    lines.append("- <= 0.5 m: {}/{}".format(ns["lte_0_5m"], metrics["truth_count"]))
    lines.append("")
    lines.append("## Matching 1:1 (limiar {:.2f} m)".format(metrics["selected_threshold_m"]))
    lines.append("- TP: {}".format(se["tp"]))
    lines.append("- FN: {}".format(se["fn"]))
    lines.append("- FP: {}".format(se["fp"]))
    lines.append("- Precision: {:.3f}".format(se["precision"]))
    lines.append("- Recall: {:.3f}".format(se["recall"]))
    lines.append("- F1: {:.3f}".format(se["f1"]))
    lines.append("- GT sem match: {}".format(se["unmatched_truth_ids"]))
    lines.append("- Map sem match: {}".format(se["unmatched_map_ids"]))
    lines.append("")
    te = metrics.get("temporal_eval")
    if te:
        lines.append("## Descoberta ao Longo do Tempo")
        lines.append("- Fonte: {}".format(te.get("source", "tree_map_history.csv")))
        lines.append("- Escopo: {}".format(te.get("scope", "confirmed_only")))
        lines.append("- Snapshots avaliados: {}".format(te.get("snapshot_count", 0)))
        lines.append("- Duracao observada: {}".format(_fmt_min(te.get("duration_min"))))
        lines.append(
            "- Final TP/FN/FP: {}/{}/{}".format(
                te.get("final_tp", 0),
                te.get("final_fn", 0),
                te.get("final_fp", 0),
            )
        )
        lines.append(
            "- Final precision/recall/F1: {:.3f}/{:.3f}/{:.3f}".format(
                te.get("final_precision", 0.0),
                te.get("final_recall", 0.0),
                te.get("final_f1", 0.0),
            )
        )
        lines.append(
            "- Recall acumulado pico/final: {:.3f}/{:.3f}".format(
                te.get("peak_cumulative_recall", 0.0),
                te.get("final_cumulative_recall", 0.0),
            )
        )
        lines.append("- AUC recall acumulado normalizado: {:.3f}".format(te.get("recall_auc_normalized", 0.0)))
        lines.append("- Tempo ate primeira arvore: {}".format(_fmt_min(te.get("time_to_first_tp_min"))))
        lines.append("- Tempo ate 50%: {}".format(_fmt_min(te.get("time_to_50pct_min"))))
        lines.append("- Tempo ate 80%: {}".format(_fmt_min(te.get("time_to_80pct_min"))))
        lines.append("- Tempo ate 100%: {}".format(_fmt_min(te.get("time_to_100pct_min"))))
        lines.append("")
    else:
        lines.append("## Descoberta ao Longo do Tempo")
        lines.append("- Sem tree_map_history.csv nesta run; gere uma nova run com o stack atualizado para obter curvas temporais.")
        lines.append("")
    ss = metrics.get("snapshot_eval")
    if ss:
        lines.append("## Snapshots Salvos")
        lines.append("- Snapshots avaliados: {}".format(ss.get("snapshot_count", 0)))
        lines.append("- Duracao coberta: {}".format(_fmt_min(ss.get("duration_min"))))
        lines.append("- Ultimo snapshot: {}".format(ss.get("final_snapshot", "")))
        lines.append(
            "- Ultimo TP/FN/FP: {}/{}/{}".format(
                ss.get("final_tp", 0),
                ss.get("final_fn", 0),
                ss.get("final_fp", 0),
            )
        )
        lines.append(
            "- Ultimo precision/recall/F1: {:.3f}/{:.3f}/{:.3f}".format(
                ss.get("final_precision", 0.0),
                ss.get("final_recall", 0.0),
                ss.get("final_f1", 0.0),
            )
        )
        lines.append("- Melhor recall nos snapshots: {:.3f}".format(ss.get("best_recall", 0.0)))
        lines.append("")
    ps = metrics.get("path_stats")
    if ps:
        lines.append("## Rota Gerada")
        lines.append("- Goals no path.csv: {}".format(ps.get("goal_count", 0)))
        lines.append("- Comprimento XY aproximado entre goals: {:.3f} m".format(ps.get("path_length_xy_m", 0.0)))
        map_xy = ps.get("map_xy") or {}
        if map_xy:
            lines.append(
                "- Limites XY do mapa (analise): x[{:.3f}, {:.3f}] y[{:.3f}, {:.3f}]".format(
                    float(map_xy.get("x_min", 0.0)),
                    float(map_xy.get("x_max", 0.0)),
                    float(map_xy.get("y_min", 0.0)),
                    float(map_xy.get("y_max", 0.0)),
                )
            )
            lines.append(
                "- Area XY do mapa (analise): {:.3f} m2".format(float(map_xy.get("area_m2", 0.0)))
            )
        cfg_xy = ps.get("config_xy") or {}
        if cfg_xy:
            lines.append(
                "- Limites XY configurados (planner): x[{:.3f}, {:.3f}] y[{:.3f}, {:.3f}]".format(
                    float(cfg_xy.get("x_min", 0.0)),
                    float(cfg_xy.get("x_max", 0.0)),
                    float(cfg_xy.get("y_min", 0.0)),
                    float(cfg_xy.get("y_max", 0.0)),
                )
            )
            lines.append(
                "- Area XY configurada (planner): {:.3f} m2".format(float(cfg_xy.get("area_m2", 0.0)))
            )
        coverage_pct = ps.get("config_area_coverage_pct")
        if coverage_pct is not None:
            lines.append("- Cobertura bbox configurado vs mapa: {:.2f}%".format(float(coverage_pct)))
        lines.append("- Grafico: route_trees_ground_truth.svg")
        lines.append("")
    rs = metrics.get("rrt_stats")
    if rs:
        lines.append("## RRT do Planner")
        lines.append("- Iteracoes com arvore: {}".format(rs.get("planning_iteration_count", 0)))
        lines.append("- Linhas de nos: {}".format(rs.get("tree_log_rows", 0)))
        lines.append("- Nos por iteracao media/max: {} / {}".format(_fmt_num(rs.get("nodes_per_iteration_mean")), rs.get("nodes_per_iteration_max", "N/A")))
        lines.append("- Max dynamic_score observado: {}".format(_fmt_num(rs.get("max_dynamic_score"), decimals=3)))
        lines.append("- Decisoes por modo: {}".format(_fmt_counts(rs.get("goal_mode_counts", {}))))
        lines.append("- Origem do goal escolhido: {}".format(_fmt_counts(rs.get("selected_goal_source_counts", {}))))
        lines.append("- Fonte: data/rrt_tree_log.csv e data/rrt_goal_log.csv")
        if metrics.get("rrt_visualizations"):
            lines.append("- Graficos: {}".format(", ".join(metrics.get("rrt_visualizations", []))))
        lines.append("")
    lines.append("## Qualidade do Mapa (diametro)")
    if ms_views:
        _append_map_stats(lines, "all_map", ms_views.get("all_map", {}))
        lines.append("")
        _append_map_stats(lines, "confirmed_only", ms_views.get("confirmed_only", {}))
        lines.append("")
        _append_map_stats(lines, "matched_tp_only", ms_views.get("matched_tp_only", {}))
    else:
        _append_map_stats(lines, "all_map", ms)
    lines.append("")
    lines.append("## Arquivos")
    lines.append("- matching.csv")
    lines.append("- metrics.json")
    if te:
        lines.append("- tree_discovery_timeseries.csv")
        lines.append("- tree_discovery_summary.csv")
        lines.append("- tree_discovery_curves.svg")
    if ss:
        lines.append("- snapshot_discovery_summary.csv")
    if ps:
        lines.append("- route_trees_ground_truth.svg")
    if metrics.get("rrt_stats"):
        lines.append("- rrt_tree_log.csv e rrt_goal_log.csv (insumos em data/)")
        for rrt_viz in metrics.get("rrt_visualizations", []):
            lines.append("- {}".format(rrt_viz))
    path.write_text("\n".join(lines) + "\n")


def try_extract_pkl_meta(pkl_path: Path, out_json: Path) -> Optional[str]:
    if not pkl_path.exists():
        return None
    try:
        with pkl_path.open("rb") as f:
            try:
                data = pickle.load(f)
            except UnicodeDecodeError:
                f.seek(0)
                data = pickle.load(f, encoding="latin1")
        meta = {}
        if isinstance(data, dict):
            obj = data.get("meta", {})
            if isinstance(obj, dict):
                meta = obj
        out_json.write_text(json.dumps(meta, indent=2, sort_keys=True))
        return None
    except Exception as exc:
        return str(exc)


def resolve_default_path(primary: Path, fallback: Path) -> Path:
    return primary if primary.exists() else fallback


def matched_png_for_pkl(pkl_path: Path) -> Optional[Path]:
    candidate = pkl_path.with_suffix(".png")
    if candidate.exists() and candidate.is_file():
        return candidate
    return None


def clean_stale_input_copies(exp_dir: Path, input_names: Sequence[str]) -> None:
    for name in input_names:
        candidate = exp_dir / name
        if candidate.exists() and candidate.is_file():
            candidate.unlink()
    for pattern in ("octomap_*.bt", "tree_cluster_state_*.pkl", "tree_cluster_state_*.png"):
        for candidate in exp_dir.glob(pattern):
            if candidate.is_file():
                candidate.unlink()


def main() -> int:
    raw_argv = sys.argv[1:]

    def _has_cli_option(option: str) -> bool:
        return any(arg == option or arg.startswith(option + "=") for arg in raw_argv)

    script_dir = Path(__file__).resolve().parent
    env_run_dir = os.environ.get("EXPERIMENT_RUN_DIR", "").strip()
    env_data_dir = os.environ.get("EXPERIMENT_DATA_DIR", "").strip()
    env_snapshots_dir = os.environ.get("EXPERIMENT_SNAPSHOT_DIR", "").strip()
    env_octomaps_dir = os.environ.get("EXPERIMENT_OCTOMAP_DIR", "").strip()
    host_data_dir = Path(env_data_dir) if env_data_dir else Path("/home/daep/data")
    host_snapshots_dir = Path(env_snapshots_dir) if env_snapshots_dir else Path("/home/daep/tree_snapshots")
    host_octomaps_dir = Path(env_octomaps_dir) if env_octomaps_dir else Path("/home/daep/octomaps")
    host_base_out = Path(env_run_dir).parent if env_run_dir else Path("/home/daep/experimentos")
    host_planner_config = Path("/home/daep/catkin_ws/src/aeplanner/rpl_exploration/config/world_jean_exploration.yaml")
    host_ti_scripts = Path("/home/daep/catkin_ws/src/tree_identifier/scripts")
    host_svg_pkl_plotter = Path("/home/daep/meus-resultados/plot_pickle_svg.py")

    local_data_dir = script_dir / "data"
    local_snapshots_dir = script_dir / "tree_snapshots"
    local_octomaps_dir = script_dir / "octomaps"
    local_base_out = script_dir / "experimentos"
    local_planner_config = script_dir / "catkin_ws/src/aeplanner/rpl_exploration/config/world_jean_exploration.yaml"
    local_ti_scripts = script_dir / "catkin_ws/src/tree_identifier/scripts"
    local_svg_pkl_plotter = script_dir / "meus-resultados/plot_pickle_svg.py"

    default_data_dir = resolve_default_path(host_data_dir, local_data_dir)
    default_snapshots_dir = resolve_default_path(host_snapshots_dir, local_snapshots_dir)
    default_octomaps_dir = resolve_default_path(host_octomaps_dir, local_octomaps_dir)
    default_base_out = resolve_default_path(host_base_out, local_base_out)
    default_planner_config = resolve_default_path(host_planner_config, local_planner_config)
    default_ti_scripts = resolve_default_path(host_ti_scripts, local_ti_scripts)
    default_svg_pkl_plotter = resolve_default_path(host_svg_pkl_plotter, local_svg_pkl_plotter)

    parser = argparse.ArgumentParser(
        description="Generate full experiment report and bundle artifacts into /experimentos/<subfolder>."
    )
    parser.add_argument("--name", default="", help="Experiment subfolder name. Default: exp_YYYYmmdd_HHMMSS")
    parser.add_argument("--base-dir", default=str(default_base_out), help="Base output directory for experiment folders.")
    parser.add_argument("--data-dir", default=str(default_data_dir), help="Directory containing tree_map_final and logs.")
    parser.add_argument("--snapshots-dir", default=str(default_snapshots_dir), help="Directory containing PKL snapshots.")
    parser.add_argument("--runtime-snapshots-dir", default="", help="Directory containing manual/autosnapshot folders.")
    parser.add_argument("--octomaps-dir", default=str(default_octomaps_dir), help="Directory containing saved .bt files.")
    parser.add_argument("--planner-config", default=str(default_planner_config), help="Planner YAML with boundary/min and boundary/max.")
    parser.add_argument("--world-name", required=True, help="World name used by this run (required, no inference).")
    parser.add_argument(
        "--ground-truth-csv",
        default="",
        help="Ground-truth CSV path. If empty, uses catkin_ws/src/biomass-simulation-resources/ground_truth for the selected world.",
    )
    parser.add_argument("--x-min", type=float, default=None, help="Optional fixed X min. If omitted, auto-fit.")
    parser.add_argument("--x-max", type=float, default=None, help="Optional fixed X max. If omitted, auto-fit.")
    parser.add_argument("--y-min", type=float, default=None, help="Optional fixed Y min. If omitted, auto-fit.")
    parser.add_argument("--y-max", type=float, default=None, help="Optional fixed Y max. If omitted, auto-fit.")
    parser.add_argument("--match-threshold", type=float, default=0.5, help="1:1 matching distance threshold (meters).")
    parser.add_argument(
        "--ground-truth-diameter-m",
        type=float,
        default=0.30,
        help="Ground-truth trunk diameter used for diameter error metrics (meters).",
    )
    parser.add_argument(
        "--tree-identifier-scripts-dir",
        default=str(default_ti_scripts),
        help="Directory with world_tree_compare_plotter.py and tree_map_csv_plotter.py.",
    )
    parser.add_argument(
        "--pkl-svg-plotter",
        default=str(default_svg_pkl_plotter),
        help="Optional plot_pickle_svg.py path for PKL SVG plots.",
    )
    parser.add_argument("--overwrite", action="store_true", help="Overwrite folder if it already exists.")
    parser.add_argument("--copy-inputs", action="store_true", help="Also copy raw input files into the result folder.")
    parser.add_argument("--clean-output", action="store_true", help="Remove stale copied inputs from the result folder before writing.")
    args = parser.parse_args()

    exp_name = args.name.strip() if args.name.strip() else "exp_{}".format(dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    base_dir = Path(args.base_dir).expanduser().resolve()
    data_dir = Path(args.data_dir).expanduser().resolve()
    snapshots_dir = Path(args.snapshots_dir).expanduser().resolve()
    runtime_snapshots_dir = Path(args.runtime_snapshots_dir).expanduser().resolve() if args.runtime_snapshots_dir.strip() else data_dir.parent / "snapshots"
    octomaps_dir = Path(args.octomaps_dir).expanduser().resolve()

    detected_run_dir = None
    run_candidate = base_dir / exp_name
    legacy_run_candidate = base_dir / "runs" / exp_name
    explicit_input_dirs = any(
        _has_cli_option(opt)
        for opt in ("--data-dir", "--snapshots-dir", "--runtime-snapshots-dir", "--octomaps-dir")
    )
    if args.name.strip() and (not explicit_input_dirs):
        if (run_candidate / "data" / "tree_map_final.csv").exists():
            detected_run_dir = run_candidate.resolve()
        elif (legacy_run_candidate / "data" / "tree_map_final.csv").exists():
            detected_run_dir = legacy_run_candidate.resolve()

    if detected_run_dir:
        base_dir = detected_run_dir
        exp_name = "result"
        data_dir = detected_run_dir / "data"
        snapshots_dir = detected_run_dir / "tree_snapshots"
        runtime_snapshots_dir = detected_run_dir / "snapshots"
        octomaps_dir = detected_run_dir / "octomaps"
        if not args.copy_inputs:
            args.clean_output = True

    exp_dir = base_dir / exp_name

    if exp_dir.exists() and (not args.overwrite):
        print("Output folder already exists: {} (use --overwrite)".format(exp_dir), file=sys.stderr)
        return 2
    exp_dir.mkdir(parents=True, exist_ok=True)

    planner_config_path = Path(args.planner_config).expanduser().resolve()
    ti_scripts_dir = Path(args.tree_identifier_scripts_dir).expanduser().resolve()
    pkl_svg_plotter = Path(args.pkl_svg_plotter).expanduser().resolve()

    manifest = {
        "experiment_name": exp_name,
        "generated_at_iso": dt.datetime.now().isoformat(),
        "copy_inputs": bool(args.copy_inputs),
        "detected_run_dir": str(detected_run_dir) if detected_run_dir else "",
        "world_name": "",
        "inputs": {},
        "input_dirs": {
            "data_dir": str(data_dir),
            "tree_snapshots_dir": str(snapshots_dir),
            "runtime_snapshots_dir": str(runtime_snapshots_dir),
            "octomaps_dir": str(octomaps_dir),
        },
        "generated_files": {},
        "warnings": [],
    }

    required_data_files = [
        "tree_map_final.csv",
        "tree_map_final.json",
        "tree_map_history.csv",
        "tree_detection_history.csv",
        "tree_guidance_waypoints.csv",
        "coverage.csv",
        "path.csv",
        "logfile.csv",
        "intervals.csv",
        "collision.csv",
        "rrt_tree_log.csv",
        "rrt_goal_log.csv",
    ]
    if args.clean_output and (not args.copy_inputs):
        clean_stale_input_copies(exp_dir, required_data_files)

    for name in required_data_files:
        src = data_dir / name
        registered = copy_file(src, exp_dir, manifest["inputs"], key=name) if args.copy_inputs else register_input_file(src, manifest["inputs"], key=name)
        if registered is None and name in ("tree_map_final.csv", "tree_map_final.json"):
            print("Missing required file: {}".format(src), file=sys.stderr)
            return 1

    latest_pkl = latest_file(snapshots_dir, "*.pkl")
    latest_png = latest_file(snapshots_dir, "*.png")
    latest_bt = latest_file(octomaps_dir, "*.bt")

    if latest_pkl:
        if args.copy_inputs:
            copy_file(latest_pkl, exp_dir, manifest["inputs"], key="latest_snapshot_pkl")
        else:
            register_input_file(latest_pkl, manifest["inputs"], key="latest_snapshot_pkl")
    else:
        manifest["warnings"].append("No PKL found in snapshots dir.")
    if latest_pkl:
        matched_png = matched_png_for_pkl(latest_pkl)
        if matched_png:
            if args.copy_inputs:
                copy_file(matched_png, exp_dir, manifest["inputs"], key="latest_snapshot_png")
            else:
                register_input_file(matched_png, manifest["inputs"], key="latest_snapshot_png")
        else:
            manifest["warnings"].append("No PNG found matching latest PKL snapshot name.")
    elif latest_png:
        if args.copy_inputs:
            copy_file(latest_png, exp_dir, manifest["inputs"], key="latest_snapshot_png")
        else:
            register_input_file(latest_png, manifest["inputs"], key="latest_snapshot_png")
    if latest_bt:
        if args.copy_inputs:
            copy_file(latest_bt, exp_dir, manifest["inputs"], key="latest_octomap_bt")
        else:
            register_input_file(latest_bt, manifest["inputs"], key="latest_octomap_bt")
    else:
        manifest["warnings"].append("No .bt found in octomaps dir.")

    map_csv_path = (exp_dir / "tree_map_final.csv") if args.copy_inputs else (data_dir / "tree_map_final.csv")
    history_csv_path = (exp_dir / "tree_map_history.csv") if args.copy_inputs else (data_dir / "tree_map_history.csv")
    path_csv_path = (exp_dir / "path.csv") if args.copy_inputs else (data_dir / "path.csv")
    rrt_tree_log_path = (exp_dir / "rrt_tree_log.csv") if args.copy_inputs else (data_dir / "rrt_tree_log.csv")
    rrt_goal_log_path = (exp_dir / "rrt_goal_log.csv") if args.copy_inputs else (data_dir / "rrt_goal_log.csv")
    pkl_input_path = (exp_dir / latest_pkl.name) if (args.copy_inputs and latest_pkl) else latest_pkl

    world_name = args.world_name.strip()
    if not world_name:
        print("Missing --world-name (no inference mode).", file=sys.stderr)
        return 1
    ground_truth_csv_path = resolve_ground_truth_csv(script_dir, world_name, args.ground_truth_csv)
    manifest["world_name"] = world_name

    if not ground_truth_csv_path.exists():
        print("Missing ground-truth CSV: {}".format(ground_truth_csv_path), file=sys.stderr)
        return 1

    register_input_file(ground_truth_csv_path, manifest["inputs"], key="ground_truth_csv")
    manifest["inputs"]["ground_truth_csv"]["world_name"] = world_name

    has_numpy = importlib.util.find_spec("numpy") is not None

    if pkl_svg_plotter.exists() and latest_pkl and has_numpy:
        try:
            cluster_svg = exp_dir / "tree_cluster_state_clusters.svg"
            run_cmd(
                [
                    "python3",
                    str(pkl_svg_plotter),
                    str(pkl_input_path),
                    "--output-prefix",
                    str(exp_dir / "tree_cluster_state"),
                ]
            )
            if cluster_svg.exists() and cluster_svg.stat().st_size < 100:
                manifest["warnings"].append(
                    "PKL cluster SVG appears empty (very small file). Install numpy/matplotlib in runtime env for full PKL plotting."
                )
        except Exception as exc:
            manifest["warnings"].append("PKL SVG plotter failed: {}".format(exc))
    elif latest_pkl and (not has_numpy):
        manifest["warnings"].append("Numpy not available: skipped PKL SVG plotting and PKL meta extraction.")

    pkl_error = None
    if latest_pkl and has_numpy:
        pkl_error = try_extract_pkl_meta(pkl_input_path, exp_dir / "pkl_meta.json")
        if pkl_error:
            manifest["warnings"].append("PKL meta extraction failed: {}".format(pkl_error))

    truth_rows = load_truth(ground_truth_csv_path)
    map_rows = load_map(map_csv_path)
    metrics, matching_rows = compute_metrics(
        truth_rows,
        map_rows,
        selected_threshold_m=args.match_threshold,
        ground_truth_diameter_m=args.ground_truth_diameter_m,
    )
    metrics["experiment_name"] = exp_name

    temporal_paths = []
    if history_csv_path.exists():
        try:
            snapshots = load_map_history(history_csv_path)
            temporal_summary, temporal_rows = compute_temporal_discovery(
                truth_rows,
                snapshots,
                selected_threshold_m=args.match_threshold,
            )
            if temporal_summary and temporal_rows:
                metrics["temporal_eval"] = temporal_summary
                temporal_timeseries_path = exp_dir / "tree_discovery_timeseries.csv"
                temporal_summary_path = exp_dir / "tree_discovery_summary.csv"
                temporal_svg_path = exp_dir / "tree_discovery_curves.svg"
                write_temporal_timeseries_csv(temporal_timeseries_path, temporal_rows)
                write_temporal_summary_csv(temporal_summary_path, temporal_summary)
                write_temporal_svg(temporal_svg_path, temporal_rows, "{}: Tree discovery over time".format(exp_name))
                temporal_paths = [temporal_timeseries_path, temporal_summary_path, temporal_svg_path]
        except Exception as exc:
            manifest["warnings"].append("Temporal discovery analysis failed: {}".format(exc))
    else:
        manifest["warnings"].append("No tree_map_history.csv found; skipped temporal discovery analysis.")

    snapshot_paths = []
    snapshot_summary, snapshot_rows = compute_snapshot_metrics(
        runtime_snapshots_dir,
        truth_rows,
        selected_threshold_m=args.match_threshold,
    )
    if snapshot_summary and snapshot_rows:
        metrics["snapshot_eval"] = snapshot_summary
        snapshot_summary_path = exp_dir / "snapshot_discovery_summary.csv"
        write_snapshot_summary_csv(snapshot_summary_path, snapshot_rows)
        snapshot_paths = [snapshot_summary_path]
    else:
        manifest["warnings"].append("No runtime snapshots with tree_map_final.csv found; skipped snapshot summary.")

    route_paths = []
    config_bbox = load_config_xy_bbox(planner_config_path)
    if not config_bbox:
        manifest["warnings"].append(
            "Planner config boundary (boundary/min, boundary/max) not found; route plot will show only map limits."
        )
    goals = load_path_goals(path_csv_path)
    plot_x_min, plot_x_max, plot_y_min, plot_y_max = auto_xy_limits(
        truth_rows,
        map_rows,
        goals,
        config_bbox,
        args.x_min,
        args.x_max,
        args.y_min,
        args.y_max,
    )
    route_stats = path_stats(goals, plot_x_min, plot_x_max, plot_y_min, plot_y_max, config_bbox=config_bbox)
    metrics["path_stats"] = route_stats
    if goals:
        route_svg = exp_dir / "route_trees_ground_truth.svg"
        write_route_tree_svg(
            route_svg,
            truth_rows,
            map_rows,
            goals,
            "{}: route, detections and ground truth".format(exp_name),
            plot_x_min,
            plot_x_max,
            plot_y_min,
            plot_y_max,
            config_bbox=config_bbox,
        )
        route_paths = [route_svg]
    else:
        manifest["warnings"].append("No path.csv goals found; skipped route plot.")

    rrt_stats = load_rrt_log_stats(rrt_tree_log_path, rrt_goal_log_path)
    rrt_paths = []
    if rrt_stats:
        metrics["rrt_stats"] = rrt_stats
        rrt_goal_rows = load_rrt_goal_rows(rrt_goal_log_path)
        rrt_tree_rows = load_rrt_tree_rows(rrt_tree_log_path)

        if rrt_tree_rows:
            rrt_samples_svg = exp_dir / "rrt_tree_samples.svg"
            write_rrt_tree_samples_svg(
                rrt_samples_svg,
                rrt_tree_rows,
                rrt_goal_rows,
                "{}: sampled RRT trees".format(exp_name),
                plot_x_min,
                plot_x_max,
                plot_y_min,
                plot_y_max,
            )
            rrt_paths.append(rrt_samples_svg)

        if rrt_paths:
            metrics["rrt_visualizations"] = [p.name for p in rrt_paths]
    else:
        manifest["warnings"].append("No RRT logs found; skipped RRT planning analysis.")

    metrics_path = exp_dir / "metrics.json"
    matching_path = exp_dir / "matching.csv"
    summary_path = exp_dir / "summary.md"

    metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True))
    write_matching_csv(matching_path, matching_rows)
    write_summary_md(summary_path, exp_name, metrics)

    for generated in (
        metrics_path,
        matching_path,
        summary_path,
    ) + tuple(temporal_paths) + tuple(snapshot_paths) + tuple(route_paths) + tuple(rrt_paths):
        if generated.exists():
            manifest["generated_files"][generated.name] = str(generated.resolve())

    manifest_path = exp_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True))

    sel = metrics["selected_eval"]
    ns = metrics["nearest_stats"]
    print("Experiment folder: {}".format(exp_dir))
    print("TP/FN/FP @ {:.2f}m: {}/{}/{}".format(args.match_threshold, sel["tp"], sel["fn"], sel["fp"]))
    print("Precision/Recall/F1: {:.3f}/{:.3f}/{:.3f}".format(sel["precision"], sel["recall"], sel["f1"]))
    print("Nearest error mean/median/p90: {:.3f}/{:.3f}/{:.3f} m".format(ns["mean_m"], ns["median_m"], ns["p90_m"]))
    if manifest["warnings"]:
        print("Warnings:")
        for w in manifest["warnings"]:
            print(" - {}".format(w))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
