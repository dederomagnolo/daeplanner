#!/usr/bin/env python3

"""Generate lightweight tree-guidance diagnostics for one experiment run.

The script intentionally reads only compact runtime products (CSV/JSON/world).
It does not load octomaps, PKL cluster states, point clouds, or ROS bags.
"""

import argparse
import csv
import importlib.util
import json
import math
import os
import re
import sys
from pathlib import Path
from xml.sax.saxutils import escape


DEFAULT_MATCH_THRESHOLD_M = 0.60


def _repo_root():
    return Path(__file__).resolve().parents[1]


def _script_path():
    return (
        _repo_root()
        / "daep"
        / "catkin_ws"
        / "src"
        / "tree_identifier"
        / "scripts"
        / "world_tree_ground_truth_plotter.py"
    )


def _load_ground_truth_parser():
    path = _script_path()
    spec = importlib.util.spec_from_file_location("world_tree_ground_truth_plotter", str(path))
    if spec is None or spec.loader is None:
        raise RuntimeError("failed to import {}".format(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _float(value, default=None):
    if value is None:
        return default
    try:
        text = str(value).strip()
        if not text:
            return default
        return float(text)
    except Exception:
        return default


def _int_bool(value):
    if value is None:
        return False
    text = str(value).strip().lower()
    return text in ("1", "true", "yes", "y", "sim")


def _first_key(row, candidates):
    normalized = {k.strip().lower(): k for k in row.keys()}
    for candidate in candidates:
        key = normalized.get(candidate.lower())
        if key is not None:
            return key
    return None


def _read_csv_dicts(path):
    path = Path(path)
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open("r", newline="") as f:
        return [dict(r) for r in csv.DictReader(f)]


def load_map(path):
    rows = _read_csv_dicts(path)
    detections = []
    for i, row in enumerate(rows, 1):
        x_key = _first_key(row, ("x", "center_x", "map_x"))
        y_key = _first_key(row, ("y", "center_y", "map_y"))
        if x_key is None or y_key is None:
            continue
        x = _float(row.get(x_key))
        y = _float(row.get(y_key))
        if x is None or y is None:
            continue
        detections.append(
            {
                "map_id": str(row.get(_first_key(row, ("map_id", "id", "tree_id")) or i)).strip() or str(i),
                "x": x,
                "y": y,
                "z": _float(row.get(_first_key(row, ("z", "center_z", "map_z"))), 0.0),
                "diameter_m": _float(row.get(_first_key(row, ("diameter_m", "diameter", "dbh_m")))),
                "hits": _float(row.get(_first_key(row, ("hits", "n_hits", "observations"))), 0.0),
                "std_xy": _float(row.get(_first_key(row, ("std_xy", "std_position", "sigma_xy")))),
                "confidence": _float(row.get(_first_key(row, ("confidence", "score", "probability")))),
                "confirmed": _int_bool(row.get(_first_key(row, ("confirmed", "is_confirmed")))),
                "suspect_merge": _int_bool(row.get(_first_key(row, ("suspect_merge", "merge", "is_merge")))),
                "raw": row,
            }
        )
    return detections


def load_path(path):
    rows = _read_csv_dicts(path)
    points = []
    for i, row in enumerate(rows):
        x_key = _first_key(row, ("goal x", "x", "pose_x", "position_x"))
        y_key = _first_key(row, ("goal y", "y", "pose_y", "position_y"))
        z_key = _first_key(row, ("goal z", "z", "pose_z", "position_z"))
        if x_key is None or y_key is None:
            continue
        x = _float(row.get(x_key))
        y = _float(row.get(y_key))
        if x is None or y is None:
            continue
        points.append(
            {
                "idx": i,
                "x": x,
                "y": y,
                "z": _float(row.get(z_key), 0.0) if z_key else 0.0,
                "planner": str(row.get(_first_key(row, ("planner", "source"))) or "").strip(),
            }
        )
    return points


def load_guidance(path):
    rows = _read_csv_dicts(path)
    guidance = []
    for row in rows:
        x = _float(row.get(_first_key(row, ("goal_x", "x"))))
        y = _float(row.get(_first_key(row, ("goal_y", "y"))))
        z = _float(row.get(_first_key(row, ("goal_z", "z"))), 0.0)
        if x is None or y is None:
            continue
        guidance.append(
            {
                "iteration": _float(row.get(_first_key(row, ("iteration",))), 0.0),
                "planner": str(row.get(_first_key(row, ("planner",))) or "").strip(),
                "x": x,
                "y": y,
                "z": z,
                "yaw": _float(row.get(_first_key(row, ("goal_yaw", "yaw"))), 0.0),
                "goal_static_gain": _float(row.get(_first_key(row, ("goal_static_gain",))), 0.0),
                "goal_base_dynamic_gain": _float(row.get(_first_key(row, ("goal_base_dynamic_gain",))), 0.0),
                "goal_tree_gain_raw": _float(row.get(_first_key(row, ("goal_tree_gain_raw",))), 0.0),
                "goal_tree_gain_weighted": _float(row.get(_first_key(row, ("goal_tree_gain_weighted",))), 0.0),
                "goal_dynamic_gain_final": _float(row.get(_first_key(row, ("goal_dynamic_gain_final",))), 0.0),
                "goal_dynamic_score": _float(row.get(_first_key(row, ("goal_dynamic_score",))), 0.0),
                "goal_best_tree_id": str(row.get(_first_key(row, ("goal_best_tree_id",))) or "-1").strip(),
                "goal_best_tree_distance_m": _float(row.get(_first_key(row, ("goal_best_tree_distance_m",))), 0.0),
                "goal_best_tree_confidence": _float(row.get(_first_key(row, ("goal_best_tree_confidence",))), 0.0),
                "goal_best_tree_confirmed": _int_bool(row.get(_first_key(row, ("goal_best_tree_confirmed",)))),
                "goal_tree_considered_count": _float(row.get(_first_key(row, ("goal_tree_considered_count",))), 0.0),
                "goal_tree_confirmed_considered": _float(row.get(_first_key(row, ("goal_tree_confirmed_considered",))), 0.0),
                "goal_tree_candidate_considered": _float(row.get(_first_key(row, ("goal_tree_candidate_considered",))), 0.0),
                "leaf_x": _float(row.get(_first_key(row, ("leaf_x",))), x),
                "leaf_y": _float(row.get(_first_key(row, ("leaf_y",))), y),
                "leaf_z": _float(row.get(_first_key(row, ("leaf_z",))), z),
                "leaf_tree_gain_weighted": _float(row.get(_first_key(row, ("leaf_tree_gain_weighted",))), 0.0),
                "leaf_dynamic_score": _float(row.get(_first_key(row, ("leaf_dynamic_score",))), 0.0),
                "leaf_best_tree_id": str(row.get(_first_key(row, ("leaf_best_tree_id",))) or "-1").strip(),
                "raw": row,
            }
        )
    return guidance


def _point_key(x, y, z=0.0):
    return (round(float(x), 3), round(float(y), 3), round(float(z), 3))


def load_truth(world_path, uri_filter, apply_mesh_offset, model_root, ground_band_z):
    parser = _load_ground_truth_parser()
    trees = parser.parse_world_trees(
        str(world_path),
        uri_filter=uri_filter,
        apply_mesh_offset=apply_mesh_offset,
        model_root=str(model_root),
        ground_band_z=ground_band_z,
    )
    return [
        {
            "tree_id": t.get("tree_id"),
            "name": t.get("name") or "tree_{}".format(i + 1),
            "x": float(t["x"]),
            "y": float(t["y"]),
            "z": float(t.get("z") or 0.0),
            "raw": t,
        }
        for i, t in enumerate(trees)
    ]


def distance(a, b):
    return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))


def one_to_one_matches(truth, detections, threshold):
    pairs = []
    for ti, t in enumerate(truth):
        for mi, m in enumerate(detections):
            d = distance(t, m)
            if d <= threshold:
                pairs.append((d, ti, mi))
    pairs.sort()
    used_t = set()
    used_m = set()
    matches = []
    for d, ti, mi in pairs:
        if ti in used_t or mi in used_m:
            continue
        used_t.add(ti)
        used_m.add(mi)
        matches.append({"distance_m": d, "truth_index": ti, "map_index": mi})
    return matches, used_t, used_m


def write_truth_csv(path, truth):
    with Path(path).open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["tree_id", "name", "x", "y", "z"])
        for t in truth:
            writer.writerow([t["tree_id"], t["name"], "{:.6f}".format(t["x"]), "{:.6f}".format(t["y"]), t["z"]])


def write_matches_csv(path, truth, detections, matches, used_t, used_m):
    by_t = {m["truth_index"]: m for m in matches}
    by_m = {m["map_index"]: m for m in matches}
    with Path(path).open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "status",
                "tree_id",
                "map_id",
                "distance_m",
                "confirmed",
                "confidence",
                "hits",
                "std_xy",
                "diameter_m",
                "gt_x",
                "gt_y",
                "map_x",
                "map_y",
            ]
        )
        for ti, t in enumerate(truth):
            match = by_t.get(ti)
            if match:
                m = detections[match["map_index"]]
                writer.writerow(
                    [
                        "matched",
                        t["tree_id"],
                        m["map_id"],
                        "{:.4f}".format(match["distance_m"]),
                        int(bool(m["confirmed"])),
                        _fmt_float(m.get("confidence")),
                        _fmt_float(m.get("hits"), digits=0),
                        _fmt_float(m.get("std_xy")),
                        _fmt_float(m.get("diameter_m")),
                        "{:.4f}".format(t["x"]),
                        "{:.4f}".format(t["y"]),
                        "{:.4f}".format(m["x"]),
                        "{:.4f}".format(m["y"]),
                    ]
                )
            else:
                writer.writerow(
                    [
                        "missed_gt",
                        t["tree_id"],
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        "{:.4f}".format(t["x"]),
                        "{:.4f}".format(t["y"]),
                        "",
                        "",
                    ]
                )
        for mi, m in enumerate(detections):
            if mi in used_m:
                continue
            writer.writerow(
                [
                    "unmatched_map",
                    "",
                    m["map_id"],
                    "",
                    int(bool(m["confirmed"])),
                    _fmt_float(m.get("confidence")),
                    _fmt_float(m.get("hits"), digits=0),
                    _fmt_float(m.get("std_xy")),
                    _fmt_float(m.get("diameter_m")),
                    "",
                    "",
                    "{:.4f}".format(m["x"]),
                    "{:.4f}".format(m["y"]),
                ]
            )


def _fmt_float(value, digits=4):
    if value is None:
        return ""
    try:
        return "{:.{}f}".format(float(value), digits)
    except Exception:
        return ""


def _confidence_label(value):
    if value is None:
        return "s/n"
    value = max(0.0, min(1.0, float(value)))
    return "{:.0f}%".format(100.0 * value)


def _tag_sort_key(path):
    name = Path(path).parent.name
    m = re.search(r"t(\d+)", name)
    if m:
        return (int(m.group(1)), name)
    m = re.search(r"(\d{8}_\d{6})", name)
    if m:
        return (10**9, m.group(1))
    try:
        return (int(Path(path).stat().st_mtime), name)
    except Exception:
        return (10**9, name)


def collect_timeline(run_dir, truth, threshold):
    files = sorted(Path(run_dir).glob("snapshots/*/tree_map_final.csv"), key=_tag_sort_key)
    final_candidates = [Path(run_dir) / "data" / "tree_map_final.csv", Path(run_dir) / "result" / "tree_map_final.csv"]
    for candidate in final_candidates:
        if candidate.exists():
            files.append(candidate)
            break

    timeline = []
    seen_labels = set()
    for path in files:
        label = "final" if path.parent.name in ("data", "result") else path.parent.name
        if label in seen_labels:
            continue
        seen_labels.add(label)
        detections = load_map(path)
        matches, used_t, used_m = one_to_one_matches(truth, detections, threshold)
        timeline.append(
            {
                "label": label,
                "path": str(path),
                "map_total": len(detections),
                "confirmed": sum(1 for d in detections if d["confirmed"]),
                "candidate": sum(1 for d in detections if not d["confirmed"]),
                "matched_gt": len(used_t),
                "false_positive": len(detections) - len(used_m),
            }
        )
    return timeline


class SvgCanvas:
    def __init__(self, width, height):
        self.width = int(width)
        self.height = int(height)
        self.items = []

    def add(self, text):
        self.items.append(text)

    def line(self, x1, y1, x2, y2, stroke="#000", width=1, dash=None, opacity=1.0):
        dash_attr = ' stroke-dasharray="{}"'.format(dash) if dash else ""
        self.add(
            '<line x1="{:.2f}" y1="{:.2f}" x2="{:.2f}" y2="{:.2f}" stroke="{}" stroke-width="{:.2f}"{} opacity="{:.3f}" />'.format(
                x1, y1, x2, y2, stroke, width, dash_attr, opacity
            )
        )

    def circle(self, x, y, r, fill="none", stroke="#000", width=1, opacity=1.0, title=None):
        title_xml = "<title>{}</title>".format(escape(title)) if title else ""
        self.add(
            '<circle cx="{:.2f}" cy="{:.2f}" r="{:.2f}" fill="{}" stroke="{}" stroke-width="{:.2f}" opacity="{:.3f}">{}</circle>'.format(
                x, y, r, fill, stroke, width, opacity, title_xml
            )
        )

    def rect(self, x, y, w, h, fill="none", stroke="#000", width=1, opacity=1.0):
        self.add(
            '<rect x="{:.2f}" y="{:.2f}" width="{:.2f}" height="{:.2f}" fill="{}" stroke="{}" stroke-width="{:.2f}" opacity="{:.3f}" />'.format(
                x, y, w, h, fill, stroke, width, opacity
            )
        )

    def polyline(self, points, stroke="#000", width=1, fill="none", opacity=1.0):
        if not points:
            return
        pts = " ".join("{:.2f},{:.2f}".format(x, y) for x, y in points)
        self.add(
            '<polyline points="{}" fill="{}" stroke="{}" stroke-width="{:.2f}" opacity="{:.3f}" stroke-linejoin="round" stroke-linecap="round" />'.format(
                pts, fill, stroke, width, opacity
            )
        )

    def text(self, x, y, text, size=12, fill="#111", anchor="start", weight="normal", rotate=None):
        rot = ' transform="rotate({:.1f} {:.2f} {:.2f})"'.format(rotate, x, y) if rotate is not None else ""
        self.add(
            '<text x="{:.2f}" y="{:.2f}" font-family="Verdana,DejaVu Sans,sans-serif" font-size="{}" fill="{}" text-anchor="{}" font-weight="{}"{}>{}</text>'.format(
                x, y, size, fill, anchor, weight, rot, escape(str(text))
            )
        )

    def save(self, path):
        with Path(path).open("w", encoding="utf-8") as f:
            f.write(
                '<?xml version="1.0" encoding="UTF-8"?>\n'
                '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" viewBox="0 0 {} {}">\n'.format(
                    self.width, self.height, self.width, self.height
                )
            )
            f.write('<rect width="100%" height="100%" fill="#fbfaf7" />\n')
            f.write("\n".join(self.items))
            f.write("\n</svg>\n")


def _nice_ticks(vmin, vmax, target=8):
    span = max(vmax - vmin, 1e-6)
    rough = span / float(target)
    base = 10 ** math.floor(math.log10(rough))
    for mul in (1, 2, 5, 10):
        step = base * mul
        if rough <= step:
            break
    start = math.floor(vmin / step) * step
    ticks = []
    value = start
    guard = 0
    while value <= vmax + 0.5 * step and guard < 100:
        if value >= vmin - 0.5 * step:
            ticks.append(value)
        value += step
        guard += 1
    return ticks


def _bounds(points, fixed_xlim=None, fixed_ylim=None):
    xs = [p["x"] for p in points if p.get("x") is not None]
    ys = [p["y"] for p in points if p.get("y") is not None]
    if not xs:
        xs = [-1.0, 1.0]
    if not ys:
        ys = [-1.0, 1.0]
    xmin, xmax = (fixed_xlim if fixed_xlim else (min(xs), max(xs)))
    ymin, ymax = (fixed_ylim if fixed_ylim else (min(ys), max(ys)))
    if xmin == xmax:
        xmin -= 1.0
        xmax += 1.0
    if ymin == ymax:
        ymin -= 1.0
        ymax += 1.0
    padx = max(0.5, 0.08 * (xmax - xmin))
    pady = max(0.5, 0.08 * (ymax - ymin))
    if not fixed_xlim:
        xmin -= padx
        xmax += padx
    if not fixed_ylim:
        ymin -= pady
        ymax += pady
    return xmin, xmax, ymin, ymax


def _preserve_aspect(xmin, xmax, ymin, ymax, plot_w, plot_h):
    dx = xmax - xmin
    dy = ymax - ymin
    target = plot_w / float(plot_h)
    current = dx / float(dy)
    if current > target:
        new_dy = dx / target
        mid = 0.5 * (ymin + ymax)
        ymin = mid - 0.5 * new_dy
        ymax = mid + 0.5 * new_dy
    else:
        new_dx = dy * target
        mid = 0.5 * (xmin + xmax)
        xmin = mid - 0.5 * new_dx
        xmax = mid + 0.5 * new_dx
    return xmin, xmax, ymin, ymax


def write_overview_svg(path, truth, detections, path_points, guidance_points, matches, used_t, used_m, threshold, fixed_xlim=None, fixed_ylim=None):
    canvas = SvgCanvas(1280, 920)
    left, top, plot_w, plot_h = 75, 70, 880, 760
    right_x = left + plot_w + 35
    all_points = list(truth) + list(detections) + list(path_points) + list(guidance_points)
    xmin, xmax, ymin, ymax = _bounds(all_points, fixed_xlim, fixed_ylim)
    xmin, xmax, ymin, ymax = _preserve_aspect(xmin, xmax, ymin, ymax, plot_w, plot_h)

    def sx(x):
        return left + (float(x) - xmin) / (xmax - xmin) * plot_w

    def sy(y):
        return top + (ymax - float(y)) / (ymax - ymin) * plot_h

    canvas.text(75, 35, "Tree Guidance Overview", size=24, weight="bold", fill="#1f2a33")
    canvas.text(75, 56, "Ground truth, planner goals, final detections and confidence score", size=13, fill="#54606a")
    canvas.rect(left, top, plot_w, plot_h, fill="#fffdf8", stroke="#d8d2c8", width=1.2)

    for tick in _nice_ticks(xmin, xmax):
        x = sx(tick)
        canvas.line(x, top, x, top + plot_h, stroke="#e8e1d6", width=0.8)
        canvas.text(x, top + plot_h + 22, "{:.1f}".format(tick), size=11, fill="#6a6259", anchor="middle")
    for tick in _nice_ticks(ymin, ymax):
        y = sy(tick)
        canvas.line(left, y, left + plot_w, y, stroke="#e8e1d6", width=0.8)
        canvas.text(left - 12, y + 4, "{:.1f}".format(tick), size=11, fill="#6a6259", anchor="end")

    canvas.text(left + plot_w / 2, top + plot_h + 55, "x [m]", size=13, fill="#334", anchor="middle")
    canvas.text(25, top + plot_h / 2, "y [m]", size=13, fill="#334", anchor="middle", rotate=-90)

    if len(path_points) > 1:
        max_points = 2500
        stride = max(1, int(math.ceil(len(path_points) / float(max_points))))
        reduced = path_points[::stride]
        canvas.polyline([(sx(p["x"]), sy(p["y"])) for p in reduced], stroke="#3979b7", width=2.2, opacity=0.74)
    elif len(path_points) == 1:
        canvas.circle(sx(path_points[0]["x"]), sy(path_points[0]["y"]), 7, fill="#3979b7", stroke="#174d7e", width=1.5, title="single path point")

    guidance_by_key = {}
    for guidance in guidance_points:
        guidance_by_key[_point_key(guidance["x"], guidance["y"], guidance["z"])] = guidance
    max_tree_gain_weighted = max([g.get("goal_tree_gain_weighted", 0.0) for g in guidance_points] + [1e-9])
    detections_by_id = {str(d["map_id"]): d for d in detections}

    for guidance in guidance_points:
        if guidance.get("goal_tree_gain_weighted", 0.0) <= 0.0:
            continue
        best_tree_id = guidance.get("goal_best_tree_id", "-1")
        det = detections_by_id.get(str(best_tree_id))
        if det is None:
            continue
        canvas.line(
            sx(guidance["x"]),
            sy(guidance["y"]),
            sx(det["x"]),
            sy(det["y"]),
            stroke="#d18400",
            width=1.1,
            dash="3 4",
            opacity=0.38,
        )

    if path_points:
        max_labeled_waypoints = 180
        label_stride = max(1, int(math.ceil(len(path_points) / float(max_labeled_waypoints))))
        for i, point in enumerate(path_points):
            if i % label_stride != 0 and i not in (0, len(path_points) - 1):
                continue
            guidance = guidance_by_key.get(_point_key(point["x"], point["y"], point["z"]))
            tree_gain_weighted = guidance.get("goal_tree_gain_weighted", 0.0) if guidance else 0.0
            if tree_gain_weighted > 0.0:
                intensity = max(0.08, min(1.0, tree_gain_weighted / max_tree_gain_weighted))
                canvas.circle(
                    sx(point["x"]),
                    sy(point["y"]),
                    10.0 + 12.0 * intensity,
                    fill="#ffc64d",
                    stroke="#d18400",
                    width=1.4,
                    opacity=0.18 + 0.22 * intensity,
                    title="tree weighted gain {:.3f}".format(tree_gain_weighted),
                )
            is_start = i == 0
            is_end = i == len(path_points) - 1
            r = 8.0 if (is_start or is_end) else 4.2
            fill = "#1c6aa6" if is_start else ("#fff7d7" if is_end else "#ffffff")
            stroke = "#174d7e"
            canvas.circle(
                sx(point["x"]),
                sy(point["y"]),
                r,
                fill=fill,
                stroke=stroke,
                width=1.6,
                opacity=0.96,
                title="waypoint {} ({:.2f}, {:.2f}, {:.2f})".format(i + 1, point["x"], point["y"], point["z"]),
            )
            if len(path_points) <= 90 or is_start or is_end:
                label = "S" if is_start else ("E" if is_end else str(i + 1))
            else:
                label = str(i + 1)
            canvas.text(sx(point["x"]) + r + 2, sy(point["y"]) - r - 1, label, size=9, fill="#174d7e", weight="bold")
            if guidance and tree_gain_weighted > 0.0:
                best_id = guidance.get("goal_best_tree_id", "-1")
                gain_label = "TG {:.1f}".format(tree_gain_weighted)
                if best_id and best_id != "-1":
                    gain_label += " T{}".format(best_id)
                canvas.text(sx(point["x"]) + r + 2, sy(point["y"]) + r + 10, gain_label, size=8, fill="#9a6411", weight="bold")

    for m in matches:
        gt = truth[m["truth_index"]]
        det = detections[m["map_index"]]
        canvas.line(sx(gt["x"]), sy(gt["y"]), sx(det["x"]), sy(det["y"]), stroke="#9c9890", width=1.0, dash="5 5", opacity=0.65)

    for i, t in enumerate(truth):
        missed = i not in used_t
        color = "#d14b45" if missed else "#424850"
        fill = "#fff7f5" if missed else "#ffffff"
        radius = 9 if missed else 7
        canvas.circle(sx(t["x"]), sy(t["y"]), radius, fill=fill, stroke=color, width=2.0, title="GT {} ({:.2f}, {:.2f})".format(t["tree_id"], t["x"], t["y"]))
        label = "T{}".format(t["tree_id"] if t["tree_id"] is not None else i + 1)
        canvas.text(sx(t["x"]) + 8, sy(t["y"]) - 8, label, size=10, fill=color, weight="bold")

    for i, det in enumerate(detections):
        matched = i in used_m
        conf = det.get("confidence")
        conf_value = max(0.0, min(1.0, conf)) if conf is not None else 0.55
        radius = 6.0 + 7.0 * conf_value
        if not matched:
            fill, stroke = "#b04c94", "#7d2c66"
            opacity = 0.80
        elif det["confirmed"]:
            fill, stroke = "#2ca25f", "#146b3a"
            opacity = 0.84
        else:
            fill, stroke = "#e69f2a", "#9a6411"
            opacity = 0.84
        if det["suspect_merge"]:
            stroke = "#d44"
        title = "map {} conf={} hits={} confirmed={} ({:.2f}, {:.2f})".format(
            det["map_id"], _confidence_label(conf), _fmt_float(det.get("hits"), digits=0), int(det["confirmed"]), det["x"], det["y"]
        )
        canvas.circle(sx(det["x"]), sy(det["y"]), radius, fill=fill, stroke=stroke, width=2.2, opacity=opacity, title=title)
        canvas.text(
            sx(det["x"]) + radius + 3,
            sy(det["y"]) + 4,
            "{} {}".format(det["map_id"], _confidence_label(conf)),
            size=10,
            fill="#28313a",
            weight="bold" if det["confirmed"] else "normal",
        )

    total = len(detections)
    confirmed = sum(1 for d in detections if d["confirmed"])
    candidates = total - confirmed
    missed = len(truth) - len(used_t)
    false_positive = total - len(used_m)
    precision = (len(used_m) / float(total)) if total else 0.0
    recall = (len(used_t) / float(len(truth))) if truth else 0.0

    canvas.text(right_x, 95, "Resumo", size=18, fill="#1f2a33", weight="bold")
    summary_lines = [
        "GT: {}".format(len(truth)),
        "Mapa final: {}".format(total),
        "Confirmadas: {}".format(confirmed),
        "Candidatas: {}".format(candidates),
        "Waypoints c/ log: {}".format(len(guidance_points)),
        "GT casadas: {}".format(len(used_t)),
        "GT faltantes: {}".format(missed),
        "Deteccoes sem GT: {}".format(false_positive),
        "Precisao aprox.: {:.1f}%".format(100 * precision),
        "Recall aprox.: {:.1f}%".format(100 * recall),
        "Limiar match: {:.2f} m".format(threshold),
    ]
    y = 122
    for line in summary_lines:
        canvas.text(right_x, y, line, size=13, fill="#38424a")
        y += 22

    y += 20
    canvas.text(right_x, y, "Legenda", size=16, fill="#1f2a33", weight="bold")
    y += 26
    legend = [
        ("#ffffff", "#424850", "GT casada"),
        ("#fff7f5", "#d14b45", "GT nao detectada"),
        ("#2ca25f", "#146b3a", "Arvore confirmada"),
        ("#e69f2a", "#9a6411", "Candidata"),
        ("#b04c94", "#7d2c66", "Deteccao sem GT"),
        ("#3979b7", "#174d7e", "Goals do planner"),
        ("#ffc64d", "#d18400", "Halo = tree gain"),
    ]
    for fill, stroke, label in legend:
        canvas.circle(right_x + 8, y - 4, 7, fill=fill, stroke=stroke, width=2)
        canvas.text(right_x + 26, y, label, size=13, fill="#38424a")
        y += 25

    canvas.save(path)


def write_timeline_svg(path, timeline):
    canvas = SvgCanvas(1280, 520)
    left, top, plot_w, plot_h = 80, 70, 1060, 330
    canvas.text(75, 35, "Tree Map Timeline", size=24, weight="bold", fill="#1f2a33")
    canvas.text(75, 56, "Counts per snapshot/final result", size=13, fill="#54606a")
    canvas.rect(left, top, plot_w, plot_h, fill="#fffdf8", stroke="#d8d2c8", width=1.2)
    if not timeline:
        canvas.text(left + 30, top + 80, "Nenhum snapshot/final tree_map_final.csv encontrado.", size=16, fill="#9a4a36")
        canvas.save(path)
        return

    max_y = max(
        1,
        max(
            max(row["map_total"], row["confirmed"], row["candidate"], row["matched_gt"], row["false_positive"])
            for row in timeline
        ),
    )
    max_y = int(math.ceil(max_y / 5.0) * 5) if max_y > 5 else max_y
    n = len(timeline)

    def sx(i):
        if n == 1:
            return left + plot_w / 2.0
        return left + 30 + (plot_w - 60) * i / float(n - 1)

    def sy(v):
        return top + plot_h - (float(v) / max_y) * plot_h

    for tick in _nice_ticks(0, max_y, target=6):
        y = sy(tick)
        canvas.line(left, y, left + plot_w, y, stroke="#e8e1d6", width=0.8)
        canvas.text(left - 12, y + 4, "{:.0f}".format(tick), size=11, fill="#6a6259", anchor="end")

    series = [
        ("map_total", "#38424a", "total"),
        ("confirmed", "#2ca25f", "confirmadas"),
        ("candidate", "#e69f2a", "candidatas"),
        ("matched_gt", "#3979b7", "GT casadas"),
        ("false_positive", "#b04c94", "sem GT"),
    ]
    for key, color, label in series:
        pts = [(sx(i), sy(row[key])) for i, row in enumerate(timeline)]
        canvas.polyline(pts, stroke=color, width=2.4, opacity=0.9)
        for x, y in pts:
            canvas.circle(x, y, 4.6, fill="#fffdf8", stroke=color, width=1.8)

    for i, row in enumerate(timeline):
        label = row["label"]
        if len(label) > 18:
            label = label[:15] + "..."
        canvas.text(sx(i), top + plot_h + 28, label, size=10, fill="#564f47", anchor="middle", rotate=-25)

    x = left + plot_w - 220
    y = 90
    canvas.text(x, y - 18, "Series", size=15, fill="#1f2a33", weight="bold")
    for key, color, label in series:
        canvas.line(x, y, x + 22, y, stroke=color, width=3)
        canvas.text(x + 30, y + 4, label, size=12, fill="#38424a")
        y += 22

    canvas.text(left + plot_w / 2, 490, "snapshot / final", size=13, fill="#334", anchor="middle")
    canvas.text(25, top + plot_h / 2, "contagem", size=13, fill="#334", anchor="middle", rotate=-90)
    canvas.save(path)


def parse_limits(text):
    if not text:
        return None
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("expected min,max")
    return float(parts[0]), float(parts[1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, help="Experiment run directory under daep/experimentos/runs")
    parser.add_argument("--world", required=True, help="Gazebo world used for ground truth (required, no inference).")
    parser.add_argument("--output-dir", default=None, help="Default: <run-dir>/result")
    parser.add_argument("--match-threshold", type=float, default=DEFAULT_MATCH_THRESHOLD_M)
    parser.add_argument("--uri-filter", default="tree")
    parser.add_argument("--no-mesh-offset", action="store_true", help="Disable mesh-origin correction for ground truth")
    parser.add_argument("--model-root", default=None)
    parser.add_argument("--ground-band-z", type=float, default=0.05)
    parser.add_argument("--xlim", type=parse_limits, default=None, help="Optional fixed x axis: min,max")
    parser.add_argument("--ylim", type=parse_limits, default=None, help="Optional fixed y axis: min,max")
    args = parser.parse_args()

    run_dir = Path(args.run_dir).resolve()
    if not run_dir.exists():
        raise SystemExit("run dir does not exist: {}".format(run_dir))
    world_path = Path(args.world).resolve()
    if not world_path.exists():
        raise SystemExit("world does not exist: {}".format(world_path))
    output_dir = Path(args.output_dir).resolve() if args.output_dir else run_dir / "result"
    output_dir.mkdir(parents=True, exist_ok=True)

    model_root = Path(args.model_root).resolve() if args.model_root else world_path.parent.parent / "models"
    truth = load_truth(world_path, args.uri_filter, not args.no_mesh_offset, model_root, args.ground_band_z)

    final_map = run_dir / "data" / "tree_map_final.csv"
    if not final_map.exists():
        final_map = run_dir / "result" / "tree_map_final.csv"
    final_path = run_dir / "data" / "path.csv"
    if not final_path.exists():
        final_path = run_dir / "result" / "path.csv"
    final_guidance = run_dir / "data" / "tree_guidance_waypoints.csv"
    if not final_guidance.exists():
        final_guidance = run_dir / "result" / "tree_guidance_waypoints.csv"
    detections = load_map(final_map)
    path_points = load_path(final_path)
    guidance_points = load_guidance(final_guidance)
    matches, used_t, used_m = one_to_one_matches(truth, detections, args.match_threshold)
    timeline = collect_timeline(run_dir, truth, args.match_threshold)

    write_truth_csv(output_dir / "world_tree_ground_truth.csv", truth)
    write_matches_csv(output_dir / "tree_guidance_matches.csv", truth, detections, matches, used_t, used_m)
    write_overview_svg(
        output_dir / "tree_guidance_overview.svg",
        truth,
        detections,
        path_points,
        guidance_points,
        matches,
        used_t,
        used_m,
        args.match_threshold,
        fixed_xlim=args.xlim,
        fixed_ylim=args.ylim,
    )
    write_timeline_svg(output_dir / "tree_guidance_timeline.svg", timeline)

    metrics = {
        "run_dir": str(run_dir),
        "world": str(world_path),
        "final_map": str(final_map),
        "final_path": str(final_path),
        "final_guidance": str(final_guidance),
        "match_threshold_m": args.match_threshold,
        "ground_truth_count": len(truth),
        "path_points": len(path_points),
        "guidance_points": len(guidance_points),
        "guidance_tree_gain_positive": sum(1 for g in guidance_points if g.get("goal_tree_gain_weighted", 0.0) > 0.0),
        "detections_total": len(detections),
        "detections_confirmed": sum(1 for d in detections if d["confirmed"]),
        "detections_candidate": sum(1 for d in detections if not d["confirmed"]),
        "matched_ground_truth": len(used_t),
        "missed_ground_truth": len(truth) - len(used_t),
        "unmatched_detections": len(detections) - len(used_m),
        "timeline_points": len(timeline),
        "outputs": {
            "overview_svg": str(output_dir / "tree_guidance_overview.svg"),
            "timeline_svg": str(output_dir / "tree_guidance_timeline.svg"),
            "matches_csv": str(output_dir / "tree_guidance_matches.csv"),
            "ground_truth_csv": str(output_dir / "world_tree_ground_truth.csv"),
        },
    }
    with (output_dir / "tree_guidance_metrics.json").open("w") as f:
        json.dump(metrics, f, indent=2, sort_keys=True)

    print("Wrote:")
    for value in metrics["outputs"].values():
        print("  {}".format(value))
    print(
        "Summary: GT={ground_truth_count}, detections={detections_total}, confirmed={detections_confirmed}, matched_gt={matched_ground_truth}, missed_gt={missed_ground_truth}".format(
            **metrics
        )
    )


if __name__ == "__main__":
    main()
