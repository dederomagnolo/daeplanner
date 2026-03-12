#!/usr/bin/env python

import argparse
import os
import pickle
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle


def _align_labels(n_points, labels):
    out = [-1] * n_points
    if labels is None:
        return out
    m = min(n_points, len(labels))
    for i in range(m):
        out[i] = int(labels[i])
    return out


def _cluster_colors(labels, alpha=0.62):
    palette = plt.cm.get_cmap("tab20", 20)
    colors = []
    for lb in labels:
        if lb < 0:
            colors.append((0.25, 0.25, 0.25, alpha * 0.65))
        else:
            rgba = list(palette(int(lb) % 20))
            rgba[3] = alpha
            colors.append(tuple(rgba))
    return colors


def _pos_xyz(pos):
    try:
        return float(pos.x), float(pos.y), float(getattr(pos, "z", 0.0))
    except Exception:
        if pos is None:
            return 0.0, 0.0, 0.0
        if len(pos) >= 3:
            return float(pos[0]), float(pos[1]), float(pos[2])
        if len(pos) >= 2:
            return float(pos[0]), float(pos[1]), 0.0
        return 0.0, 0.0, 0.0


def load_payload(path):
    with open(path, "rb") as f:
        try:
            data = pickle.load(f)
        except UnicodeDecodeError:
            # ROS Melodic often writes PKL with Python2 ("str" bytes). When loaded
            # with Python3, force latin1 decoding for backward compatibility.
            if sys.version_info[0] < 3:
                raise
            f.seek(0)
            data = pickle.load(f, encoding="latin1")

    points_xy = np.asarray(data.get("points_xy", []), dtype=float)
    if points_xy.ndim == 1:
        if points_xy.size == 2:
            points_xy = points_xy.reshape((1, 2))
        else:
            points_xy = np.empty((0, 2), dtype=float)
    if points_xy.ndim == 2 and points_xy.shape[1] > 2:
        points_xy = points_xy[:, :2]

    labels = _align_labels(points_xy.shape[0], data.get("labels", []))
    detected_trees = data.get("detected_trees", [])
    map_confirmed = data.get("map_confirmed", [])
    map_candidates = data.get("map_candidates", [])
    meta = data.get("meta", {})

    return points_xy, labels, detected_trees, map_confirmed, map_candidates, meta


def plot_payload(points_xy, labels, detected_trees, map_confirmed, map_candidates, meta, show_labels=True):
    fig, ax = plt.subplots(figsize=(10, 10))

    if points_xy.shape[0] > 0:
        colors = _cluster_colors(labels)
        ax.scatter(points_xy[:, 0], points_xy[:, 1], s=11, c=colors, linewidths=0, label="cluster_points")

    for i, tree in enumerate(detected_trees):
        pos = tree.get("position")
        x, y, _ = _pos_xyz(pos)
        diameter = float(tree.get("diameter", 0.0) or 0.0)
        confidence = float(tree.get("confidence", 0.0) or 0.0)
        radius = max(diameter * 0.5, 0.0)

        if radius > 0.0:
            ax.add_patch(Circle((x, y), radius, fill=False, linewidth=1.5, linestyle="--", color="#b1173b", alpha=0.9))

        ax.plot([x], [y], marker="x", markersize=8, color="#c11f1f")
        if show_labels:
            label_id = tree.get("id", i + 1)
            ax.text(
                x + 0.10,
                y + 0.10,
                "det_%s d=%.2fm c=%.2f" % (str(label_id), diameter, confidence),
                color="#8a1313",
                fontsize=8,
            )

    if map_confirmed:
        map_x = []
        map_y = []
        for item in map_confirmed:
            x, y, _ = _pos_xyz(item.get("position"))
            map_x.append(x)
            map_y.append(y)
        ax.scatter(map_x, map_y, c="#1f8f3a", s=82, marker="x", linewidths=1.8, label="map_confirmed")

        if show_labels:
            for i, item in enumerate(map_confirmed):
                x, y, _ = _pos_xyz(item.get("position"))
                map_id = item.get("map_id", i + 1)
                ax.text(x + 0.10, y + 0.10, "map_%s" % str(map_id), color="#145a32", fontsize=8)

    if map_candidates:
        cand_x = []
        cand_y = []
        for item in map_candidates:
            x, y, _ = _pos_xyz(item.get("position"))
            cand_x.append(x)
            cand_y.append(y)
        ax.scatter(cand_x, cand_y, c="#e67e22", s=60, marker="^", alpha=0.85, label="map_candidate")

        if show_labels:
            for i, item in enumerate(map_candidates):
                x, y, _ = _pos_xyz(item.get("position"))
                map_id = item.get("map_id", i + 1)
                ax.text(x + 0.10, y + 0.10, "cand_%s" % str(map_id), color="#9a4d00", fontsize=8)

    cluster_count = len(set([lb for lb in labels if lb >= 0]))
    title = "Tree Cluster Snapshot"
    if meta:
        reason = meta.get("reason", "")
        saved_at = meta.get("saved_at_sec", "")
        title = "Tree Cluster Snapshot | reason=%s | t=%s" % (str(reason), str(saved_at))

    ax.set_title(title)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal", "box")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right")

    info = "cluster_points=%d | clusters=%d | targets=%d | map=%d | cand=%d" % (
        points_xy.shape[0],
        cluster_count,
        len(detected_trees),
        len(map_confirmed),
        len(map_candidates),
    )
    ax.text(
        0.01,
        0.99,
        info,
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=9,
        bbox=dict(boxstyle="round,pad=0.25", facecolor="white", alpha=0.68),
    )

    return fig


def main():
    parser = argparse.ArgumentParser(description="Plot snapshot PKL from tree_cluster_xy_plotter.")
    parser.add_argument("--pkl", required=True, help="Path to snapshot .pkl")
    parser.add_argument("--out", default="", help="If set, save plot image to this path")
    parser.add_argument("--hide-labels", action="store_true", help="Disable text labels")
    args = parser.parse_args()

    points_xy, labels, detected_trees, map_confirmed, map_candidates, meta = load_payload(args.pkl)
    fig = plot_payload(
        points_xy,
        labels,
        detected_trees,
        map_confirmed,
        map_candidates,
        meta,
        show_labels=(not args.hide_labels),
    )

    if args.out:
        out_path = os.path.abspath(os.path.expanduser(args.out))
        out_dir = os.path.dirname(out_path)
        if out_dir and (not os.path.exists(out_dir)):
            os.makedirs(out_dir)
        fig.savefig(out_path, dpi=180, bbox_inches="tight")
        print("Saved: %s" % out_path)
    else:
        plt.show()


if __name__ == "__main__":
    main()
