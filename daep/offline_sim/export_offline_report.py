#!/usr/bin/env python3

"""Generate a compact report for offline exploration simulator runs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from offline_explorer import load_point_cloud, load_simple_yaml  # noqa: E402


Point2 = Tuple[float, float]


def default_ground_truth_csv() -> Path:
    return SCRIPT_DIR.parent / "ground_truth" / "world_tree_ground_truth.csv"


def read_csv(path: Path) -> List[dict]:
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8") as f:
        return [{str(k).strip(): v for k, v in row.items()} for row in csv.DictReader(f)]


def as_float(value, default: float = 0.0) -> float:
    try:
        return float(str(value).strip())
    except Exception:
        return default


def as_optional_float(value) -> Optional[float]:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        return float(text)
    except Exception:
        return None


def as_int(value, default: int = 0) -> int:
    try:
        return int(float(str(value).strip()))
    except Exception:
        return default


def load_summary(run_dir: Path) -> Dict[str, object]:
    path = run_dir / "summary.json"
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def load_path_points(path_csv: Path) -> List[dict]:
    points = []
    for idx, row in enumerate(read_csv(path_csv), start=1):
        x = as_float(row.get("Goal x"))
        y = as_float(row.get("Goal y"))
        z = as_float(row.get("Goal z"))
        planner = str(row.get("Planner", "")).strip()
        points.append({"seq": idx, "x": x, "y": y, "z": z, "planner": planner})
    return points


def load_rrt_goal_rows(goal_log_path: Path) -> List[dict]:
    rows = []
    for row in read_csv(goal_log_path):
        iteration = as_int(row.get("planning_iteration"), -1)
        if iteration < 0:
            continue
        rows.append(
            {
                "planning_iteration": iteration,
                "stamp_sec": as_optional_float(row.get("stamp_sec")),
                "planner_mode": str(row.get("planner_mode", "")).strip() or "unknown",
                "selected_goal_source": str(row.get("selected_goal_source", "")).strip() or "unknown",
                "tree_node_count": as_int(row.get("tree_node_count"), 0),
                "best_dynamic_score": as_optional_float(row.get("best_dynamic_score")),
                "best_x": as_optional_float(row.get("best_x")),
                "best_y": as_optional_float(row.get("best_y")),
                "selected_x": as_optional_float(row.get("selected_x")),
                "selected_y": as_optional_float(row.get("selected_y")),
            }
        )
    rows.sort(key=lambda r: r["planning_iteration"])
    return rows


def load_rrt_tree_rows(tree_log_path: Path) -> Dict[int, List[dict]]:
    by_iteration: Dict[int, List[dict]] = {}
    for row in read_csv(tree_log_path):
        iteration = as_int(row.get("planning_iteration"), -1)
        if iteration < 0:
            continue
        by_iteration.setdefault(iteration, []).append(
            {
                "node_id": as_int(row.get("node_id"), -1),
                "parent_id": as_int(row.get("parent_id"), -1),
                "depth": as_int(row.get("depth"), 0),
                "x": as_float(row.get("x")),
                "y": as_float(row.get("y")),
                "dynamic_score": as_optional_float(row.get("dynamic_score")),
                "is_root": as_int(row.get("is_root"), 0),
                "is_best_node": as_int(row.get("is_best_node"), 0),
                "is_selected_goal": as_int(row.get("is_selected_goal"), 0),
                "is_best_branch": as_int(row.get("is_best_branch"), 0),
            }
        )
    return by_iteration


def load_tree_positions_from_csv(csv_path: Path) -> List[dict]:
    if not csv_path.exists():
        raise FileNotFoundError("ground-truth CSV not found: {}".format(csv_path))

    trees = []
    for row in read_csv(csv_path):
        tree_id_text = str(row.get("tree_id", "")).strip()
        trees.append(
            {
                "tree_id": as_int(tree_id_text) if tree_id_text else None,
                "name": str(row.get("name", "")).strip(),
                "x": as_float(row.get("x")),
                "y": as_float(row.get("y")),
                "z": as_float(row.get("z")),
                "roll": as_float(row.get("roll")),
                "pitch": as_float(row.get("pitch")),
                "yaw": as_float(row.get("yaw")),
                "uri": str(row.get("uri", "")).strip(),
                "raw_x": as_float(row.get("raw_x"), as_float(row.get("x"))),
                "raw_y": as_float(row.get("raw_y"), as_float(row.get("y"))),
                "offset_x_local": as_float(row.get("offset_x_local")),
                "offset_y_local": as_float(row.get("offset_y_local")),
                "offset_status": str(row.get("offset_status", "")).strip(),
            }
        )
    trees.sort(key=lambda t: (t["tree_id"] is None, t["tree_id"] if t["tree_id"] is not None else 999999, t["name"]))
    return trees


def path_length(points: Sequence[dict], start: Optional[Sequence[float]] = None) -> float:
    total = 0.0
    prev = {"x": start[0], "y": start[1], "z": start[2]} if start else None
    for point in points:
        if prev:
            total += math.sqrt((point["x"] - prev["x"]) ** 2 + (point["y"] - prev["y"]) ** 2 + (point["z"] - prev["z"]) ** 2)
        prev = point
    return total


def parse_bounds_from_summary(summary: Dict[str, object]) -> Optional[Tuple[Point2, Point2]]:
    params = summary.get("params")
    if not isinstance(params, dict):
        return None
    bmin = params.get("boundary_min")
    bmax = params.get("boundary_max")
    if not isinstance(bmin, list) or not isinstance(bmax, list) or len(bmin) < 2 or len(bmax) < 2:
        return None
    return (float(bmin[0]), float(bmin[1])), (float(bmax[0]), float(bmax[1]))


def parse_bounds_from_config(config: Optional[Path]) -> Optional[Tuple[Point2, Point2]]:
    if not config:
        return None
    cfg = load_simple_yaml(config)
    bmin = cfg.get("boundary/min")
    bmax = cfg.get("boundary/max")
    if not isinstance(bmin, list) or not isinstance(bmax, list) or len(bmin) < 2 or len(bmax) < 2:
        return None
    return (float(bmin[0]), float(bmin[1])), (float(bmax[0]), float(bmax[1]))


def infer_bounds(points: Sequence[dict], cloud_points: Sequence[Tuple[float, float, float]]) -> Tuple[Point2, Point2]:
    xs = [p["x"] for p in points]
    ys = [p["y"] for p in points]
    xs.extend(p[0] for p in cloud_points)
    ys.extend(p[1] for p in cloud_points)
    if not xs or not ys:
        return (-1.0, -1.0), (1.0, 1.0)
    pad_x = max(1.0, (max(xs) - min(xs)) * 0.08)
    pad_y = max(1.0, (max(ys) - min(ys)) * 0.08)
    return (min(xs) - pad_x, min(ys) - pad_y), (max(xs) + pad_x, max(ys) + pad_y)


def svg_escape(text: object) -> str:
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


class SvgCanvas:
    def __init__(self, width: int = 1000, height: int = 720):
        self.width = width
        self.height = height
        self.items: List[str] = []

    def add(self, item: str) -> None:
        self.items.append(item)

    def text(self, x: float, y: float, text: object, size: int = 14, fill: str = "#27313a", weight: str = "400") -> None:
        self.add(
            '<text x="{:.2f}" y="{:.2f}" font-family="Arial, sans-serif" font-size="{}" font-weight="{}" fill="{}">{}</text>'.format(
                x, y, size, weight, fill, svg_escape(text)
            )
        )

    def write(self, path: Path) -> None:
        body = "\n".join(self.items)
        path.write_text(
            '<svg xmlns="http://www.w3.org/2000/svg" width="{0}" height="{1}" viewBox="0 0 {0} {1}">\n'
            '<rect width="100%" height="100%" fill="#fbfcfd"/>\n{2}\n</svg>\n'.format(self.width, self.height, body),
            encoding="utf-8",
        )


def write_route_svg(
    path: Path,
    goals: Sequence[dict],
    cloud_points: Sequence[Tuple[float, float, float]],
    tree_positions: Sequence[dict],
    seen_tree_ids: set,
    detections: Sequence[dict],
    bounds: Tuple[Point2, Point2],
    start: Optional[Sequence[float]],
    title: str = "Offline: route and ground truth",
) -> None:
    width = 900
    height = 680
    left = 70
    right = 30
    top = 55
    bottom = 75
    plot_w = width - left - right
    plot_h = height - top - bottom
    (min_x, min_y), (max_x, max_y) = bounds
    if max_x <= min_x:
        max_x = min_x + 1.0
    if max_y <= min_y:
        max_y = min_y + 1.0

    def sx(x: float) -> float:
        return left + ((float(x) - min_x) / max(max_x - min_x, 1e-6)) * plot_w

    def sy(y: float) -> float:
        return top + (1.0 - ((float(y) - min_y) / max(max_y - min_y, 1e-6))) * plot_h

    map_w = max(0.0, max_x - min_x)
    map_h = max(0.0, max_y - min_y)
    map_area = map_w * map_h

    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" viewBox="0 0 {} {}">'.format(width, height, width, height),
        '<rect width="100%" height="100%" fill="white" />',
        '<text x="{}" y="32" font-family="Arial" font-size="18" font-weight="bold">{}</text>'.format(left, svg_escape(title)),
        '<rect x="{0}" y="{1}" width="{2}" height="{3}" fill="#fafafa" stroke="#333" />'.format(left, top, plot_w, plot_h),
    ]

    cell_m = max(0.12, min(0.25, max(max_x - min_x, max_y - min_y) / 90.0))
    cloud_bins: Dict[Tuple[int, int], int] = {}
    for x, y, _z in cloud_points:
        if min_x <= x <= max_x and min_y <= y <= max_y:
            ix = int(math.floor((x - min_x) / cell_m))
            iy = int(math.floor((y - min_y) / cell_m))
            cloud_bins[(ix, iy)] = cloud_bins.get((ix, iy), 0) + 1

    if cloud_bins:
        max_count = max(cloud_bins.values())
        for (ix, iy), count in sorted(cloud_bins.items(), key=lambda item: item[1]):
            x0 = min_x + ix * cell_m
            x1 = min(x0 + cell_m, max_x)
            y0 = min_y + iy * cell_m
            y1 = min(y0 + cell_m, max_y)
            px0 = sx(x0)
            px1 = sx(x1)
            py0 = sy(y1)
            py1 = sy(y0)
            density = math.log1p(count) / max(math.log1p(max_count), 1e-6)
            opacity = 0.08 + 0.24 * density
            lines.append(
                '<rect x="{:.2f}" y="{:.2f}" width="{:.2f}" height="{:.2f}" fill="#555555" opacity="{:.3f}"/>'.format(
                    px0,
                    py0,
                    max(px1 - px0, 0.7),
                    max(py1 - py0, 0.7),
                    opacity,
                )
            )

    tick_count = 5
    for i in range(tick_count + 1):
        frac = float(i) / float(tick_count)
        x_val = min_x + frac * (max_x - min_x)
        y_val = min_y + frac * (max_y - min_y)
        x = left + frac * plot_w
        y = top + (1.0 - frac) * plot_h
        lines.append('<line x1="{0:.2f}" y1="{1}" x2="{0:.2f}" y2="{2}" stroke="#e5e5e5" />'.format(x, top, top + plot_h))
        lines.append('<line x1="{0}" y1="{1:.2f}" x2="{2}" y2="{1:.2f}" stroke="#e5e5e5" />'.format(left, y, left + plot_w))
        lines.append('<text x="{:.2f}" y="{}" font-family="Arial" font-size="11" text-anchor="middle">{:.1f}</text>'.format(x, top + plot_h + 18, x_val))
        lines.append('<text x="{}" y="{:.2f}" font-family="Arial" font-size="11" text-anchor="end">{:.1f}</text>'.format(left - 8, y + 4, y_val))

    map_info = "mapa XY: x[{:.1f},{:.1f}] y[{:.1f},{:.1f}] | tamanho {:.1f} x {:.1f} m | area {:.1f} m2".format(
        min_x, max_x, min_y, max_y, map_w, map_h, map_area
    )
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12" fill="#333">{}</text>'.format(left + 8, top + 16, svg_escape(map_info)))

    route = []
    if start:
        route.append({"seq": 0, "x": float(start[0]), "y": float(start[1])})
    route.extend({"seq": int(p.get("seq", idx)), "x": float(p["x"]), "y": float(p["y"])} for idx, p in enumerate(goals, start=1))
    route_points = [(sx(p["x"]), sy(p["y"])) for p in route]
    if len(route_points) >= 2:
        poly = " ".join("{:.2f},{:.2f}".format(x, y) for x, y in route_points)
        lines.append('<polyline fill="none" stroke="#444" stroke-width="2" stroke-linejoin="round" stroke-linecap="round" points="{}" />'.format(poly))
        for idx, point in enumerate(route):
            radius = 4.0 if idx in (0, len(route) - 1) else 2.0
            lines.append('<circle cx="{:.2f}" cy="{:.2f}" r="{:.1f}" fill="#444" opacity="0.45" />'.format(sx(point["x"]), sy(point["y"]), radius))

    for tree in tree_positions:
        x = float(tree.get("x", 0.0))
        y = float(tree.get("y", 0.0))
        if not (min_x <= x <= max_x and min_y <= y <= max_y):
            continue
        px = sx(x)
        py = sy(y)
        label = tree.get("tree_id")
        label_text = "" if label is None else str(label)
        seen = label_text in seen_tree_ids
        if seen:
            lines.append('<circle cx="{:.2f}" cy="{:.2f}" r="10" fill="#2ca02c" opacity="0.20" />'.format(px, py))
            lines.append('<circle cx="{:.2f}" cy="{:.2f}" r="7" fill="#eaffea" stroke="#2ca02c" stroke-width="2.5" />'.format(px, py))
        else:
            lines.append('<circle cx="{:.2f}" cy="{:.2f}" r="7" fill="none" stroke="#1f77b4" stroke-width="2" />'.format(px, py))
        if label is not None:
            fill = "#1d7d32" if seen else "#1f77b4"
            weight = "bold" if seen else "normal"
            lines.append(
                '<text x="{:.2f}" y="{:.2f}" font-family="Arial" font-size="10" font-weight="{}" fill="{}">{}</text>'.format(
                    px + 8, py - 8, weight, fill, svg_escape(label)
                )
            )

    for detection in detections:
        x = as_optional_float(detection.get("x"))
        y = as_optional_float(detection.get("y"))
        if x is None or y is None or not (min_x <= x <= max_x and min_y <= y <= max_y):
            continue
        px = sx(x)
        py = sy(y)
        matched = str(detection.get("matched_tree_id", "")).strip() != ""
        fill = "#ff8c00" if matched else "#9b59b6"
        lines.append('<circle cx="{:.2f}" cy="{:.2f}" r="3.2" fill="{}" stroke="white" stroke-width="0.8" opacity="0.9" />'.format(px, py, fill))

    if route:
        lines.append('<circle cx="{:.2f}" cy="{:.2f}" r="6" fill="#2ca02c" stroke="white" stroke-width="1.5" />'.format(sx(route[0]["x"]), sy(route[0]["y"])))
        lines.append('<circle cx="{:.2f}" cy="{:.2f}" r="6" fill="#d62728" stroke="white" stroke-width="1.5" />'.format(sx(route[-1]["x"]), sy(route[-1]["y"])))

    if cloud_bins:
        cloud_info = "PCD: {} celulas XY ocupadas a partir de {} pontos | GT arvores: {}".format(len(cloud_bins), len(cloud_points), len(tree_positions))
        lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12" fill="#555">{}</text>'.format(left + 8, top + 32, svg_escape(cloud_info)))

    legend_y = height - 42
    lines.append('<line x1="{}" y1="{}" x2="{}" y2="{}" stroke="#444" stroke-width="2" />'.format(left, legend_y, left + 38, legend_y))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">rota de goals</text>'.format(left + 45, legend_y + 4))
    lines.append('<circle cx="{}" cy="{}" r="6" fill="none" stroke="#1f77b4" stroke-width="2" />'.format(left + 170, legend_y))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">GT</text>'.format(left + 184, legend_y + 4))
    lines.append('<circle cx="{}" cy="{}" r="6" fill="#eaffea" stroke="#2ca02c" stroke-width="2" />'.format(left + 220, legend_y))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">GT visto</text>'.format(left + 234, legend_y + 4))
    lines.append('<circle cx="{}" cy="{}" r="3.2" fill="#ff8c00" stroke="white" stroke-width="0.8" />'.format(left + 306, legend_y))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">detector</text>'.format(left + 318, legend_y + 4))
    lines.append('<rect x="{:.1f}" y="{:.1f}" width="12" height="12" fill="#555555" opacity="0.22" />'.format(left + 400, legend_y - 6))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">PCD</text>'.format(left + 418, legend_y + 4))
    lines.append('<circle cx="{}" cy="{}" r="5" fill="#2ca02c" />'.format(left + 470, legend_y))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">inicio</text>'.format(left + 482, legend_y + 4))
    lines.append('<circle cx="{}" cy="{}" r="5" fill="#d62728" />'.format(left + 540, legend_y))
    lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">fim</text>'.format(left + 552, legend_y + 4))
    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def nice_ticks(vmin: float, vmax: float, count: int = 6) -> List[float]:
    if vmax <= vmin:
        return [vmin]
    span = vmax - vmin
    raw = span / max(1, count - 1)
    magnitude = 10 ** math.floor(math.log10(raw))
    residual = raw / magnitude
    if residual >= 5:
        step = 5 * magnitude
    elif residual >= 2:
        step = 2 * magnitude
    else:
        step = magnitude
    start = math.floor(vmin / step) * step
    ticks = []
    value = start
    while value <= vmax + step * 0.5:
        if value >= vmin - step * 0.5:
            ticks.append(value)
        value += step
    return ticks


def write_series_svg(path: Path, rows: Sequence[dict], x_key: str, y_key: str, title: str, y_label: str, color: str) -> None:
    canvas = SvgCanvas(1000, 560)
    left, top, plot_w, plot_h = 80, 70, 820, 380
    xs = [as_float(r.get(x_key)) for r in rows]
    ys = [as_float(r.get(y_key)) for r in rows]
    if not xs or not ys:
        canvas.text(80, 35, title, size=22, weight="700")
        canvas.text(80, 100, "No data available.", size=15, fill="#6b747d")
        canvas.write(path)
        return
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    if max_x <= min_x:
        max_x = min_x + 1
    if max_y <= min_y:
        max_y = min_y + 1
    y_pad = (max_y - min_y) * 0.08
    min_y = max(0.0, min_y - y_pad)
    max_y += y_pad

    def sx(x: float) -> float:
        return left + (x - min_x) / (max_x - min_x) * plot_w

    def sy(y: float) -> float:
        return top + plot_h - (y - min_y) / (max_y - min_y) * plot_h

    canvas.text(80, 35, title, size=22, weight="700")
    canvas.add('<rect x="{0}" y="{1}" width="{2}" height="{3}" fill="#ffffff" stroke="#bac4cf"/>'.format(left, top, plot_w, plot_h))
    for tick in nice_ticks(min_y, max_y):
        y = sy(tick)
        canvas.add('<line x1="{0}" y1="{1:.2f}" x2="{2}" y2="{1:.2f}" stroke="#e4e9ee"/>'.format(left, y, left + plot_w))
        canvas.text(left - 62, y + 4, "{:.1f}".format(tick), size=11, fill="#66717d")
    poly = " ".join("{:.2f},{:.2f}".format(sx(x), sy(y)) for x, y in zip(xs, ys))
    canvas.add('<polyline points="{}" fill="none" stroke="{}" stroke-width="3" stroke-linejoin="round" stroke-linecap="round"/>'.format(poly, color))
    for x, y in zip(xs, ys):
        canvas.add('<circle cx="{:.2f}" cy="{:.2f}" r="3" fill="{}"/>'.format(sx(x), sy(y), color))
    canvas.text(left, top + plot_h + 35, "time / iteration", size=13, fill="#5a6470")
    canvas.text(left + plot_w - 120, top + plot_h + 35, y_label, size=13, fill="#5a6470")
    canvas.write(path)


def select_rrt_sample_iterations(iterations: Sequence[int], max_samples: int = 6) -> List[int]:
    sorted_iterations = sorted(iterations)
    if len(sorted_iterations) <= max_samples:
        return list(sorted_iterations)
    selected = []
    for i in range(max_samples):
        idx = int(round(i * (len(sorted_iterations) - 1) / float(max_samples - 1)))
        value = sorted_iterations[idx]
        if value not in selected:
            selected.append(value)
    return selected


def write_rrt_tree_samples_svg(
    path: Path,
    tree_rows_by_iteration: Dict[int, List[dict]],
    goal_rows: Sequence[dict],
    title: str,
    bounds: Tuple[Point2, Point2],
) -> None:
    iterations = sorted(tree_rows_by_iteration.keys())
    if not iterations:
        return

    selected_iterations = select_rrt_sample_iterations(iterations, max_samples=6)
    goal_by_iteration = {as_int(r.get("planning_iteration"), -1): r for r in goal_rows}
    (x_min, y_min), (x_max, y_max) = bounds
    if x_max <= x_min:
        x_max = x_min + 1.0
    if y_max <= y_min:
        y_max = y_min + 1.0

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
        '<text x="{}" y="30" font-family="Arial" font-size="18" font-weight="bold">{}</text>'.format(margin_x, svg_escape(title)),
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
                    sx(parent["x"], cell_left),
                    sy(parent["y"], cell_top),
                    sx(node["x"], cell_left),
                    sy(node["y"], cell_top),
                    color,
                    width_px,
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
            lines.append(
                '<circle cx="{:.2f}" cy="{:.2f}" r="{:.1f}" fill="{}" opacity="0.95" />'.format(
                    sx(node["x"], cell_left), sy(node["y"], cell_top), radius, fill
                )
            )

        if goal.get("selected_x") is not None and goal.get("selected_y") is not None and goal.get("selected_goal_source") == "cached_branch":
            x = sx(float(goal["selected_x"]), cell_left)
            y = sy(float(goal["selected_y"]), cell_top)
            lines.append('<circle cx="{:.2f}" cy="{:.2f}" r="5.2" fill="#ff7f0e" stroke="white" stroke-width="1.2" />'.format(x, y))

        score = goal.get("best_dynamic_score")
        score_text = "score=N/A" if score is None else "score={:.1f}".format(float(score))
        label = "iter {} | {} | {}".format(iteration, goal.get("selected_goal_source", "unknown"), score_text)
        lines.append('<text x="{:.2f}" y="{:.2f}" font-family="Arial" font-size="12" font-weight="bold">{}</text>'.format(cell_left + 8, cell_top + 18, svg_escape(label)))
        lines.append('<text x="{:.2f}" y="{:.2f}" font-family="Arial" font-size="11" fill="#555">nodes={}</text>'.format(cell_left + 8, cell_top + cell_h - 8, len(nodes)))

    legend_y = height - 16
    legend_x = margin_x
    for label, color in [("root", "#111111"), ("best branch", "#d62728"), ("best node", "#9467bd"), ("RRT goal", "#2ca02c"), ("cache goal", "#ff7f0e")]:
        lines.append('<circle cx="{}" cy="{}" r="5" fill="{}" />'.format(legend_x, legend_y, color))
        lines.append('<text x="{}" y="{}" font-family="Arial" font-size="12">{}</text>'.format(legend_x + 10, legend_y + 4, svg_escape(label)))
        legend_x += 126

    lines.append("</svg>")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def copy_inputs(run_dir: Path, output_dir: Path) -> None:
    data_dir = output_dir / "data"
    data_dir.mkdir(exist_ok=True)
    for name in (
        "path.csv",
        "coverage.csv",
        "tree_coverage.csv",
        "tree_detections.csv",
        "tree_observations.csv",
        "logfile.csv",
        "rrt_tree_log.csv",
        "rrt_goal_log.csv",
        "summary.json",
    ):
        src = run_dir / name
        if src.exists():
            shutil.copy2(str(src), str(data_dir / name))


def remove_copied_inputs(output_dir: Path) -> None:
    data_dir = output_dir / "data"
    if data_dir.exists() and data_dir.is_dir():
        shutil.rmtree(str(data_dir))


def remove_stale_report_files(output_dir: Path) -> None:
    # Names from earlier versions of the offline report. The current route
    # figure intentionally uses the same name as the normal report.
    for name in ("route_xy.svg",):
        path = output_dir / name
        if path.exists() and path.is_file():
            path.unlink()


def write_report(
    output_dir: Path,
    run_dir: Path,
    summary: Dict[str, object],
    goals: Sequence[dict],
    coverage_rows: Sequence[dict],
    tree_coverage_rows: Sequence[dict],
    tree_observation_rows: Sequence[dict],
    rrt_goal_rows: Sequence[dict],
    cloud_path: Optional[Path],
    ground_truth_csv: Path,
    tree_positions: Sequence[dict],
    tree_truth_csv: Optional[Path],
    route_gt_svg: Path,
    coverage_svg: Path,
    tree_coverage_svg: Optional[Path],
    rrt_svg: Path,
    rrt_tree_samples_svg: Optional[Path],
) -> None:
    coverage = summary.get("coverage") if isinstance(summary.get("coverage"), dict) else {}
    tree_coverage = summary.get("tree_coverage") if isinstance(summary.get("tree_coverage"), dict) else {}
    params = summary.get("params") if isinstance(summary.get("params"), dict) else {}
    start = None
    if goals:
        start = "see command/summary input"
    final_tree_row = tree_coverage_rows[-1] if tree_coverage_rows else {}
    trees_total = int(float(tree_coverage.get("trees_total", final_tree_row.get("Trees total", 0)) or 0))
    trees_seen = int(float(tree_coverage.get("trees_seen", final_tree_row.get("Trees seen", 0)) or 0))
    trees_seen_pct = float(tree_coverage.get("trees_seen_pct", final_tree_row.get("Trees seen (%)", 0.0)) or 0.0)
    path_length_m = float(summary.get("path_length_m", path_length(goals)) or 0.0)
    tree_seen_per_meter = trees_seen / path_length_m if path_length_m > 0.0 else 0.0

    lines = [
        "# Offline Exploration Report",
        "",
        "## Run",
        "",
        "- Run dir: `{}`".format(run_dir),
        "- Cloud: `{}`".format(cloud_path if cloud_path else ""),
        "- Tree GT source: `{}`".format(ground_truth_csv),
        "- Seed: `{}`".format(summary.get("seed", "")),
        "- Stop reason: `{}`".format(summary.get("stop_reason", "")),
        "- Goals in path.csv: `{}`".format(len(goals)),
        "- Path length: `{:.3f} m`".format(path_length_m),
        "- Elapsed simulated time: `{:.3f} s`".format(float(summary.get("elapsed_time_sec", 0.0) or 0.0)),
        "- Final coverage: `{:.3f}%`".format(float(coverage.get("coverage_pct", 0.0) or 0.0)),
        "- Final trees seen: `{}/{}` (`{:.3f}%`)".format(trees_seen, trees_total, trees_seen_pct),
        "- Trees seen per meter: `{:.3f}`".format(tree_seen_per_meter),
        "",
        "## Map And Sensor",
        "",
        "- Boundary min: `{}`".format(params.get("boundary_min", "")),
        "- Boundary max: `{}`".format(params.get("boundary_max", "")),
        "- Resolution: `{}`".format(params.get("resolution", "")),
        "- FoV: horizontal `{}` deg, vertical `{}` deg".format(params.get("hfov", ""), params.get("vfov", "")),
        "- Range: `r_max={}` m".format(params.get("r_max", "")),
        "- Tree ground truth positions: `{}`".format(len(tree_positions)),
        "- Tree detector: z slice `{}..{}` m, RANSAC threshold `{}` m, GT match threshold `{}` m".format(
            params.get("tree_detector_slice_z_min", ""),
            params.get("tree_detector_slice_z_max", ""),
            params.get("tree_detector_ransac_distance_threshold", ""),
            tree_coverage.get("tree_match_threshold_m", params.get("tree_match_threshold", "")),
        ),
        "",
        "## Figures",
        "",
        "- [Ground truth trees + cloud + route]({})".format(route_gt_svg.name),
        "- [Coverage]({})".format(coverage_svg.name),
        "- [RRT goal score]({})".format(rrt_svg.name),
    ]
    if tree_coverage_svg is not None:
        lines.append("- [Tree coverage]({})".format(tree_coverage_svg.name))
    if rrt_tree_samples_svg is not None:
        lines.append("- [RRT tree samples]({})".format(rrt_tree_samples_svg.name))
    if tree_truth_csv is not None:
        lines.append("- [Tree ground truth CSV]({})".format(tree_truth_csv.name))
    if tree_observation_rows:
        lines.append("- [Tree observations](tree_observations.csv)")
    if (run_dir / "tree_detections.csv").exists():
        lines.append("- [Tree detector candidates](tree_detections.csv)")
    lines.extend(
        [
            "",
            "## Notes",
            "",
            "The ground-truth route figure uses the hidden point cloud projected in XY and marks tree positions read from the shared ground-truth CSV. Offline tree coverage counts GT trees matched by candidates produced by the offline detector.",
            "",
            "This report is generated from the offline simulator outputs. It does not include PKL snapshots, Gazebo collisions, or OctoMap artifacts from the full ROS simulation.",
        ]
    )
    (output_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate a report from daep/offline_sim/offline_explorer.py outputs.")
    parser.add_argument("--run-dir", type=Path, required=True, help="Offline run directory containing path.csv, coverage.csv and summary.json.")
    parser.add_argument("--output-dir", type=Path, help="Report output directory. Default: <run-dir>/report")
    parser.add_argument("--cloud", type=Path, help="Optional point cloud used for route background.")
    parser.add_argument("--config", type=Path, help="Optional planner config used for bounds if summary.json is missing.")
    parser.add_argument("--ground-truth-csv", type=Path, default=default_ground_truth_csv(), help="Shared tree ground-truth CSV.")
    parser.add_argument("--copy-inputs", action="store_true", help="Copy run CSV/JSON inputs into report/data. Omit this to avoid duplicate files.")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    run_dir = args.run_dir.expanduser().resolve()
    output_dir = (args.output_dir or (run_dir / "report")).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    remove_stale_report_files(output_dir)

    summary = load_summary(run_dir)
    goals = load_path_points(run_dir / "path.csv")
    coverage_rows = read_csv(run_dir / "coverage.csv")
    tree_coverage_rows = read_csv(run_dir / "tree_coverage.csv")
    tree_observation_rows = read_csv(run_dir / "tree_observations.csv")
    tree_detection_rows = read_csv(run_dir / "tree_detections.csv")
    rrt_goal_rows = load_rrt_goal_rows(run_dir / "rrt_goal_log.csv")
    rrt_tree_rows = load_rrt_tree_rows(run_dir / "rrt_tree_log.csv")

    cloud_points: List[Tuple[float, float, float]] = []
    if args.cloud:
        cloud_points = load_point_cloud(args.cloud.expanduser().resolve())
    ground_truth_csv = args.ground_truth_csv.expanduser().resolve()
    try:
        tree_positions = load_tree_positions_from_csv(ground_truth_csv)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    bounds = parse_bounds_from_summary(summary) or parse_bounds_from_config(args.config) or infer_bounds(goals, cloud_points)
    start = summary.get("start_state")
    if not isinstance(start, list):
        # The original command commonly uses the simulation-aligned start.
        start = None

    route_gt_svg = output_dir / "route_trees_ground_truth.svg"
    coverage_svg = output_dir / "coverage.svg"
    tree_coverage_svg = output_dir / "tree_coverage.svg"
    rrt_svg = output_dir / "rrt_goal_score.svg"
    rrt_tree_samples_svg = output_dir / "rrt_tree_samples.svg"
    tree_truth_csv = output_dir / "world_tree_ground_truth.csv"
    tree_observations_csv = output_dir / "tree_observations.csv"
    tree_detections_csv = output_dir / "tree_detections.csv"
    if tree_positions:
        shutil.copy2(str(ground_truth_csv), str(tree_truth_csv))
    elif tree_truth_csv.exists():
        tree_truth_csv.unlink()
    if (run_dir / "tree_observations.csv").exists():
        shutil.copy2(str(run_dir / "tree_observations.csv"), str(tree_observations_csv))
    elif tree_observations_csv.exists():
        tree_observations_csv.unlink()
    if (run_dir / "tree_detections.csv").exists():
        shutil.copy2(str(run_dir / "tree_detections.csv"), str(tree_detections_csv))
    elif tree_detections_csv.exists():
        tree_detections_csv.unlink()

    seen_tree_ids = {str(r.get("tree_id", "")).strip() for r in tree_observation_rows if as_int(r.get("seen"), 0) == 1}
    write_route_svg(route_gt_svg, goals, cloud_points, tree_positions, seen_tree_ids, tree_detection_rows, bounds, start)
    write_series_svg(coverage_svg, coverage_rows, "Time", "Coverage (%)", "Coverage Over Time", "coverage (%)", "#1b8a5a")
    if tree_coverage_rows:
        write_series_svg(tree_coverage_svg, tree_coverage_rows, "Time", "Trees seen (%)", "Tree Coverage Over Time", "trees seen (%)", "#a46412")
    elif tree_coverage_svg.exists():
        tree_coverage_svg.unlink()
    write_series_svg(rrt_svg, rrt_goal_rows, "planning_iteration", "best_dynamic_score", "RRT Best Dynamic Score", "dynamic score", "#1764aa")
    if rrt_tree_rows:
        write_rrt_tree_samples_svg(rrt_tree_samples_svg, rrt_tree_rows, rrt_goal_rows, "Sampled RRT Trees", bounds)
    elif rrt_tree_samples_svg.exists():
        rrt_tree_samples_svg.unlink()

    if args.copy_inputs:
        copy_inputs(run_dir, output_dir)
    else:
        remove_copied_inputs(output_dir)

    generated_files = {
        "report_md": str(output_dir / "report.md"),
        "route_trees_ground_truth_svg": str(route_gt_svg),
        "coverage_svg": str(coverage_svg),
        "rrt_goal_score_svg": str(rrt_svg),
    }
    if tree_coverage_svg.exists():
        generated_files["tree_coverage_svg"] = str(tree_coverage_svg)
    if rrt_tree_samples_svg.exists():
        generated_files["rrt_tree_samples_svg"] = str(rrt_tree_samples_svg)
    if tree_truth_csv.exists():
        generated_files["world_tree_ground_truth_csv"] = str(tree_truth_csv)
    if tree_observations_csv.exists():
        generated_files["tree_observations_csv"] = str(tree_observations_csv)
    if tree_detections_csv.exists():
        generated_files["tree_detections_csv"] = str(tree_detections_csv)

    manifest = {
        "run_dir": str(run_dir),
        "cloud": str(args.cloud.expanduser().resolve()) if args.cloud else "",
        "ground_truth_csv": str(ground_truth_csv),
        "tree_ground_truth_count": len(tree_positions),
        "generated_files": generated_files,
    }
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    write_report(
        output_dir,
        run_dir,
        summary,
        goals,
        coverage_rows,
        tree_coverage_rows,
        tree_observation_rows,
        rrt_goal_rows,
        args.cloud,
        ground_truth_csv,
        tree_positions,
        tree_truth_csv if tree_truth_csv.exists() else None,
        route_gt_svg,
        coverage_svg,
        tree_coverage_svg if tree_coverage_svg.exists() else None,
        rrt_svg,
        rrt_tree_samples_svg if rrt_tree_samples_svg.exists() else None,
    )
    print("Offline report written to: {}".format(output_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
