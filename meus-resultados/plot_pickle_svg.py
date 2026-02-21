#!/usr/bin/env python3
"""
Plot tree-detection pickle files without third-party dependencies.

This script loads pickle files that depend on numpy and geometry_msgs by
injecting lightweight compatibility stubs, then writes two SVG plots:
1) XY clusters + detected tree circles
2) Diameter histogram with mean and RMSE
"""

import argparse
import math
import pickle
import struct
import sys
import types
from collections import defaultdict
from pathlib import Path


def install_pickle_stubs():
    """Install minimal numpy/geometry_msgs stubs required for unpickling."""

    class FakeDType:
        def __init__(self, spec="f8"):
            self.spec = spec.decode() if isinstance(spec, (bytes, bytearray)) else str(spec)
            self.byteorder = "<"

        def __setstate__(self, state):
            # Expected pattern: (3, '<', None, None, None, -1, -1, 0)
            if isinstance(state, (tuple, list)) and len(state) >= 2 and isinstance(state[1], str):
                if state[1] in ("<", ">", "|", "="):
                    self.byteorder = state[1]

        def newbyteorder(self, order="="):
            if order in ("<", ">", "|", "="):
                self.byteorder = order
            return self

    def decode_scalar(dtype, raw):
        if not isinstance(raw, (bytes, bytearray)):
            return raw
        spec = getattr(dtype, "spec", "f8")
        byteorder = getattr(dtype, "byteorder", "<")
        if byteorder in ("=", "|"):
            byteorder = "<"

        if spec.startswith("f"):
            fmt = "d" if "8" in spec else "f"
        elif spec.startswith("i"):
            fmt = "q" if "8" in spec else "i"
        elif spec.startswith("u"):
            fmt = "Q" if "8" in spec else "I"
        else:
            fmt = "d"

        try:
            return struct.unpack(byteorder + fmt, raw)[0]
        except Exception:
            return 0.0

    class FakeNDArray:
        def __init__(self, shape=(), dtype=None):
            self.shape = tuple(shape) if isinstance(shape, (list, tuple)) else (shape,)
            self.dtype = dtype if dtype is not None else FakeDType("f8")
            self.data = []

        def __setstate__(self, state):
            # ndarray state: (version, shape, dtype, is_fortran, raw_data)
            if not (isinstance(state, (tuple, list)) and len(state) == 5):
                self.data = []
                return

            _, shape, dtype, _, raw = state
            self.shape = tuple(shape)
            self.dtype = dtype

            if isinstance(raw, (bytes, bytearray)):
                self.data = self._decode_raw(raw)
            else:
                self.data = list(raw)

        def _decode_raw(self, raw):
            total = 1
            for dim in self.shape:
                total *= int(dim)

            spec = getattr(self.dtype, "spec", "f8")
            byteorder = getattr(self.dtype, "byteorder", "<")
            if byteorder in ("=", "|"):
                byteorder = "<"

            if spec.startswith("f"):
                fmt = "d" if "8" in spec else "f"
                size = 8 if "8" in spec else 4
            elif spec.startswith("i"):
                fmt = "q" if "8" in spec else "i"
                size = 8 if "8" in spec else 4
            elif spec.startswith("u"):
                fmt = "Q" if "8" in spec else "I"
                size = 8 if "8" in spec else 4
            else:
                fmt = "d"
                size = 8

            vals = []
            for i in range(total):
                chunk = raw[i * size : (i + 1) * size]
                if len(chunk) < size:
                    break
                vals.append(struct.unpack(byteorder + fmt, chunk)[0])

            if len(self.shape) == 2:
                rows, cols = self.shape
                out = []
                idx = 0
                for _ in range(rows):
                    out.append(vals[idx : idx + cols])
                    idx += cols
                return out

            return vals

        def __len__(self):
            return len(self.data)

        def __iter__(self):
            return iter(self.data)

        def __getitem__(self, idx):
            return self.data[idx]

    def fake_dtype(spec="f8", *args, **kwargs):
        return FakeDType(spec)

    def fake_reconstruct(subtype, shape, dtype):
        return FakeNDArray(shape, dtype)

    def fake_scalar(dtype, raw):
        return decode_scalar(dtype, raw)

    numpy_mod = types.ModuleType("numpy")
    numpy_mod.ndarray = FakeNDArray
    numpy_mod.dtype = fake_dtype

    core_mod = types.ModuleType("numpy.core")
    multi_mod = types.ModuleType("numpy.core.multiarray")
    multi_mod._reconstruct = fake_reconstruct
    multi_mod.scalar = fake_scalar

    core_mod.multiarray = multi_mod
    numpy_mod.core = core_mod

    sys.modules["numpy"] = numpy_mod
    sys.modules["numpy.core"] = core_mod
    sys.modules["numpy.core.multiarray"] = multi_mod

    geometry_mod = types.ModuleType("geometry_msgs")
    msg_mod = types.ModuleType("geometry_msgs.msg")
    point_mod = types.ModuleType("geometry_msgs.msg._Point")

    class Point:
        __slots__ = ("x", "y", "z")

        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

        def __setstate__(self, state):
            # Stored as list [x, y, z]
            if isinstance(state, (tuple, list)) and len(state) >= 3:
                self.x = float(state[0])
                self.y = float(state[1])
                self.z = float(state[2])

    point_mod.Point = Point
    msg_mod._Point = point_mod
    geometry_mod.msg = msg_mod

    sys.modules["geometry_msgs"] = geometry_mod
    sys.modules["geometry_msgs.msg"] = msg_mod
    sys.modules["geometry_msgs.msg._Point"] = point_mod


def load_data(pickle_path):
    install_pickle_stubs()
    with pickle_path.open("rb") as f:
        return pickle.load(f)


def esc(text):
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def cluster_color(label):
    if label == -1:
        return "#111111"
    palette = [
        "#d62828",
        "#2a9d8f",
        "#264653",
        "#e76f51",
        "#1d3557",
        "#f4a261",
        "#6a4c93",
        "#43aa8b",
        "#f94144",
        "#277da1",
        "#577590",
        "#bc6c25",
    ]
    return palette[label % len(palette)]


def as_points(points_xy):
    out = []
    for row in points_xy:
        if isinstance(row, (list, tuple)) and len(row) >= 2:
            out.append((float(row[0]), float(row[1])))
    return out


def as_labels(labels):
    out = []
    for value in labels:
        try:
            out.append(int(value))
        except Exception:
            out.append(-1)
    return out


def world_bounds(points, trees):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    for tree in trees:
        pos = tree.get("position")
        diameter = float(tree.get("diameter", 0.0))
        radius = diameter / 2.0
        try:
            tx, ty = float(pos.x), float(pos.y)
        except Exception:
            tx, ty = float(pos[0]), float(pos[1])
        xs.extend([tx - radius, tx + radius])
        ys.extend([ty - radius, ty + radius])

    if not xs or not ys:
        return -1.0, 1.0, -1.0, 1.0

    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    xpad = max((xmax - xmin) * 0.06, 0.2)
    ypad = max((ymax - ymin) * 0.06, 0.2)
    return xmin - xpad, xmax + xpad, ymin - ypad, ymax + ypad


def write_cluster_svg(data, output_path):
    points = as_points(data.get("points_xy", []))
    labels = as_labels(data.get("labels", []))
    trees = data.get("detected_trees", [])

    if not points:
        output_path.write_text("<svg xmlns='http://www.w3.org/2000/svg'></svg>", encoding="utf-8")
        return

    width, height = 1200, 1200
    margin = 90
    plot_w = width - 2 * margin
    plot_h = height - 2 * margin

    xmin, xmax, ymin, ymax = world_bounds(points, trees)
    xspan = max(xmax - xmin, 1e-9)
    yspan = max(ymax - ymin, 1e-9)

    def sx(x):
        return margin + (x - xmin) / xspan * plot_w

    def sy(y):
        return height - margin - (y - ymin) / yspan * plot_h

    grouped = defaultdict(list)
    for idx, (x, y) in enumerate(points):
        label = labels[idx] if idx < len(labels) else -1
        grouped[label].append((x, y))

    parts = []
    parts.append(f"<svg xmlns='http://www.w3.org/2000/svg' width='{width}' height='{height}' viewBox='0 0 {width} {height}'>")
    parts.append("<rect x='0' y='0' width='100%' height='100%' fill='#ffffff'/>")
    parts.append(
        f"<text x='{width/2:.1f}' y='42' text-anchor='middle' font-size='28' font-family='sans-serif' fill='#111'>Detected Trees (Clusters and Circles)</text>"
    )

    # Grid
    for i in range(11):
        gx = margin + i * (plot_w / 10.0)
        gy = margin + i * (plot_h / 10.0)
        parts.append(f"<line x1='{gx:.2f}' y1='{margin}' x2='{gx:.2f}' y2='{height-margin}' stroke='#ececec' stroke-width='1'/>")
        parts.append(f"<line x1='{margin}' y1='{gy:.2f}' x2='{width-margin}' y2='{gy:.2f}' stroke='#ececec' stroke-width='1'/>")

    parts.append(
        f"<rect x='{margin}' y='{margin}' width='{plot_w}' height='{plot_h}' fill='none' stroke='#888' stroke-width='1.5'/>"
    )

    # Axis labels
    parts.append(
        f"<text x='{width/2:.1f}' y='{height-24}' text-anchor='middle' font-size='18' font-family='sans-serif' fill='#222'>X Coordinate (m)</text>"
    )
    parts.append(
        f"<text x='28' y='{height/2:.1f}' transform='rotate(-90 28 {height/2:.1f})' text-anchor='middle' font-size='18' font-family='sans-serif' fill='#222'>Y Coordinate (m)</text>"
    )

    # Points by label
    for label, cluster_points in grouped.items():
        color = cluster_color(label)
        radius = 2.2 if label == -1 else 3.6
        opacity = 0.7 if label == -1 else 0.85
        for x, y in cluster_points:
            parts.append(
                f"<circle cx='{sx(x):.2f}' cy='{sy(y):.2f}' r='{radius}' fill='{color}' fill-opacity='{opacity}'/>"
            )

    # Tree circles and labels
    for tree in trees:
        pos = tree.get("position")
        tree_id = tree.get("id", "?")
        diameter = float(tree.get("diameter", 0.0))
        radius_m = diameter / 2.0
        try:
            tx, ty = float(pos.x), float(pos.y)
        except Exception:
            tx, ty = float(pos[0]), float(pos[1])

        cx, cy = sx(tx), sy(ty)
        r_px = radius_m / xspan * plot_w
        parts.append(
            f"<circle cx='{cx:.2f}' cy='{cy:.2f}' r='{r_px:.2f}' fill='none' stroke='#d90429' stroke-width='2.5' stroke-dasharray='8 5'/>"
        )
        parts.append(f"<line x1='{cx-6:.2f}' y1='{cy:.2f}' x2='{cx+6:.2f}' y2='{cy:.2f}' stroke='#d90429' stroke-width='2'/>")
        parts.append(f"<line x1='{cx:.2f}' y1='{cy-6:.2f}' x2='{cx:.2f}' y2='{cy+6:.2f}' stroke='#d90429' stroke-width='2'/>")
        parts.append(
            f"<text x='{cx:.2f}' y='{cy-r_px-10:.2f}' text-anchor='middle' font-size='14' font-family='sans-serif' fill='#a4133c'>ID:{esc(tree_id)} | {diameter:.2f}m</text>"
        )

    # Min/max notes
    parts.append(
        f"<text x='{margin}' y='{margin-20}' font-size='13' font-family='monospace' fill='#444'>x:[{xmin:.2f}, {xmax:.2f}]  y:[{ymin:.2f}, {ymax:.2f}]</text>"
    )

    parts.append("</svg>")
    output_path.write_text("\n".join(parts), encoding="utf-8")


def write_histogram_svg(data, output_path):
    trees = data.get("detected_trees", [])
    diameters = [float(tree.get("diameter", 0.0)) for tree in trees]

    width, height = 1100, 700
    margin_l, margin_r, margin_t, margin_b = 90, 40, 80, 80
    plot_w = width - margin_l - margin_r
    plot_h = height - margin_t - margin_b

    parts = []
    parts.append(f"<svg xmlns='http://www.w3.org/2000/svg' width='{width}' height='{height}' viewBox='0 0 {width} {height}'>")
    parts.append("<rect x='0' y='0' width='100%' height='100%' fill='#ffffff'/>")
    parts.append(
        f"<text x='{width/2:.1f}' y='42' text-anchor='middle' font-size='28' font-family='sans-serif' fill='#111'>Tree Diameter (DBH) Distribution</text>"
    )

    parts.append(
        f"<rect x='{margin_l}' y='{margin_t}' width='{plot_w}' height='{plot_h}' fill='none' stroke='#888' stroke-width='1.5'/>"
    )

    if not diameters:
        parts.append(
            f"<text x='{width/2:.1f}' y='{height/2:.1f}' text-anchor='middle' font-size='20' font-family='sans-serif' fill='#333'>No detected trees</text>"
        )
        parts.append("</svg>")
        output_path.write_text("\n".join(parts), encoding="utf-8")
        return

    ground_truth = 0.30
    mean = sum(diameters) / len(diameters)
    rmse = math.sqrt(sum((d - ground_truth) ** 2 for d in diameters) / len(diameters))

    bin_w = 0.05
    min_d = min(diameters)
    max_d = max(diameters)
    start = math.floor((min_d - bin_w) / bin_w) * bin_w
    end = math.ceil((max_d + bin_w) / bin_w) * bin_w
    n_bins = max(1, int(round((end - start) / bin_w)))
    counts = [0] * n_bins

    for d in diameters:
        idx = int((d - start) / bin_w)
        if idx < 0:
            idx = 0
        if idx >= n_bins:
            idx = n_bins - 1
        counts[idx] += 1

    max_count = max(counts) if counts else 1

    def sx(x):
        return margin_l + (x - start) / (end - start) * plot_w

    def sy(y):
        return margin_t + plot_h - (y / max_count) * plot_h

    # Horizontal grid + y labels
    for i in range(max_count + 1):
        y = sy(i)
        parts.append(f"<line x1='{margin_l}' y1='{y:.2f}' x2='{width-margin_r}' y2='{y:.2f}' stroke='#efefef' stroke-width='1'/>")
        parts.append(
            f"<text x='{margin_l-12}' y='{y+4:.2f}' text-anchor='end' font-size='12' font-family='monospace' fill='#444'>{i}</text>"
        )

    # Bars + x labels
    for i, count in enumerate(counts):
        left = start + i * bin_w
        right = left + bin_w
        x = sx(left)
        w = sx(right) - sx(left)
        y = sy(count)
        h = margin_t + plot_h - y
        parts.append(
            f"<rect x='{x+1:.2f}' y='{y:.2f}' width='{max(0.0,w-2):.2f}' height='{h:.2f}' fill='#8ecae6' stroke='#219ebc' stroke-width='1'/>"
        )
        parts.append(
            f"<text x='{x + w/2:.2f}' y='{margin_t+plot_h+18}' text-anchor='middle' font-size='10' font-family='monospace' fill='#555'>{left:.2f}</text>"
        )

    # Mean + GT lines
    mean_x = sx(mean)
    gt_x = sx(ground_truth)
    parts.append(f"<line x1='{mean_x:.2f}' y1='{margin_t}' x2='{mean_x:.2f}' y2='{margin_t+plot_h}' stroke='#e63946' stroke-width='2.5' stroke-dasharray='9 6'/>")
    parts.append(f"<line x1='{gt_x:.2f}' y1='{margin_t}' x2='{gt_x:.2f}' y2='{margin_t+plot_h}' stroke='#2a9d8f' stroke-width='2.5' stroke-dasharray='4 5'/>")

    # Legend text
    legend_y = 62
    parts.append(f"<text x='{margin_l}' y='{legend_y}' font-size='14' font-family='sans-serif' fill='#e63946'>Mean: {mean:.2f}m</text>")
    parts.append(f"<text x='{margin_l+180}' y='{legend_y}' font-size='14' font-family='sans-serif' fill='#2a9d8f'>Ground Truth: {ground_truth:.2f}m</text>")
    parts.append(f"<text x='{margin_l+430}' y='{legend_y}' font-size='14' font-family='sans-serif' fill='#333'>RMSE: {rmse:.2f}m</text>")

    # Axis labels
    parts.append(
        f"<text x='{width/2:.1f}' y='{height-20}' text-anchor='middle' font-size='18' font-family='sans-serif' fill='#222'>Diameter (m)</text>"
    )
    parts.append(
        f"<text x='26' y='{height/2:.1f}' transform='rotate(-90 26 {height/2:.1f})' text-anchor='middle' font-size='18' font-family='sans-serif' fill='#222'>Frequency</text>"
    )

    parts.append("</svg>")
    output_path.write_text("\n".join(parts), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Generate SVG plots from pickle result files.")
    parser.add_argument("pickle_file", help="Path to .pkl file")
    parser.add_argument(
        "--output-prefix",
        help="Output path prefix (without extension). Defaults to input file path without .pkl",
    )
    args = parser.parse_args()

    pickle_path = Path(args.pickle_file).resolve()
    if not pickle_path.exists():
        raise FileNotFoundError(f"File not found: {pickle_path}")

    prefix = Path(args.output_prefix).resolve() if args.output_prefix else pickle_path.with_suffix("")
    cluster_svg = Path(f"{prefix}_clusters.svg")
    hist_svg = Path(f"{prefix}_diameter_hist.svg")

    data = load_data(pickle_path)
    write_cluster_svg(data, cluster_svg)
    write_histogram_svg(data, hist_svg)

    print(f"Generated: {cluster_svg}")
    print(f"Generated: {hist_svg}")


if __name__ == "__main__":
    main()
