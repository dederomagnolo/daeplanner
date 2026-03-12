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


def load_truth(csv_path: Path) -> List[dict]:
    rows = []
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            tree_id = row.get("tree_id", "").strip()
            rows.append(
                {
                    "tree_id": int(tree_id) if tree_id else None,
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                }
            )
    rows.sort(key=lambda d: (d["tree_id"] is None, d["tree_id"] or 999999))
    return rows


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


def write_summary_md(path: Path, exp_name: str, metrics: dict) -> None:
    ns = metrics["nearest_stats"]
    se = metrics["selected_eval"]
    ms = metrics["map_stats"]
    ms_views = metrics.get("map_stats_views", {})

    def _fmt_m(v: Optional[float], decimals: int = 3) -> str:
        if v is None:
            return "N/A"
        return ("{:.%df}" % decimals).format(v)

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
    lines.append("- world_jean_ground_truth.csv/.svg")
    lines.append("- world_jean_compare_vs_csv.svg")
    lines.append("- tree_map_final_plot.svg (ID/hits)")
    lines.append("- tree_map_diameter_plot.svg (ID/diameter)")
    lines.append("- matching.csv")
    lines.append("- metrics.json")
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


def main() -> int:
    script_dir = Path(__file__).resolve().parent
    host_data_dir = Path("/home/daep/data")
    host_snapshots_dir = Path("/home/daep/tree_snapshots")
    host_octomaps_dir = Path("/home/daep/octomaps")
    host_base_out = Path("/home/daep/experimentos")
    host_world = Path("/home/daep/catkin_ws/src/drone_gazebo/worlds/world_jean.world")
    host_ti_scripts = Path("/home/daep/catkin_ws/src/tree_identifier/scripts")
    host_svg_pkl_plotter = Path("/home/daep/meus-resultados/plot_pickle_svg.py")

    local_data_dir = script_dir / "data"
    local_snapshots_dir = script_dir / "tree_snapshots"
    local_octomaps_dir = script_dir / "octomaps"
    local_base_out = script_dir / "experimentos"
    local_world = script_dir / "catkin_ws/src/drone_gazebo/worlds/world_jean.world"
    local_ti_scripts = script_dir / "catkin_ws/src/tree_identifier/scripts"
    local_svg_pkl_plotter = script_dir / "meus-resultados/plot_pickle_svg.py"

    default_data_dir = resolve_default_path(host_data_dir, local_data_dir)
    default_snapshots_dir = resolve_default_path(host_snapshots_dir, local_snapshots_dir)
    default_octomaps_dir = resolve_default_path(host_octomaps_dir, local_octomaps_dir)
    default_base_out = resolve_default_path(host_base_out, local_base_out)
    default_world = resolve_default_path(host_world, local_world)
    default_ti_scripts = resolve_default_path(host_ti_scripts, local_ti_scripts)
    default_svg_pkl_plotter = resolve_default_path(host_svg_pkl_plotter, local_svg_pkl_plotter)

    parser = argparse.ArgumentParser(
        description="Generate full experiment report and bundle artifacts into /experimentos/<subfolder>."
    )
    parser.add_argument("--name", default="", help="Experiment subfolder name. Default: exp_YYYYmmdd_HHMMSS")
    parser.add_argument("--base-dir", default=str(default_base_out), help="Base output directory for experiment folders.")
    parser.add_argument("--data-dir", default=str(default_data_dir), help="Directory containing tree_map_final and logs.")
    parser.add_argument("--snapshots-dir", default=str(default_snapshots_dir), help="Directory containing PKL snapshots.")
    parser.add_argument("--octomaps-dir", default=str(default_octomaps_dir), help="Directory containing saved .bt files.")
    parser.add_argument("--world", default=str(default_world), help="Path to Gazebo .world file.")
    parser.add_argument("--uri-filter", default="world_jean_tree", help="URI filter used for GT extraction.")
    parser.add_argument("--x-min", type=float, default=-10.0)
    parser.add_argument("--x-max", type=float, default=10.0)
    parser.add_argument("--y-min", type=float, default=-8.0)
    parser.add_argument("--y-max", type=float, default=7.0)
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
        help="Directory with world_tree_* and tree_map_csv_plotter scripts.",
    )
    parser.add_argument(
        "--pkl-svg-plotter",
        default=str(default_svg_pkl_plotter),
        help="Optional plot_pickle_svg.py path for PKL SVG plots.",
    )
    parser.add_argument("--overwrite", action="store_true", help="Overwrite folder if it already exists.")
    args = parser.parse_args()

    exp_name = args.name.strip() if args.name.strip() else "exp_{}".format(dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    base_dir = Path(args.base_dir).expanduser().resolve()
    exp_dir = base_dir / exp_name

    if exp_dir.exists() and (not args.overwrite):
        print("Output folder already exists: {} (use --overwrite)".format(exp_dir), file=sys.stderr)
        return 2
    exp_dir.mkdir(parents=True, exist_ok=True)

    data_dir = Path(args.data_dir).expanduser().resolve()
    snapshots_dir = Path(args.snapshots_dir).expanduser().resolve()
    octomaps_dir = Path(args.octomaps_dir).expanduser().resolve()
    world_path = Path(args.world).expanduser().resolve()
    ti_scripts_dir = Path(args.tree_identifier_scripts_dir).expanduser().resolve()
    pkl_svg_plotter = Path(args.pkl_svg_plotter).expanduser().resolve()

    manifest = {
        "experiment_name": exp_name,
        "generated_at_iso": dt.datetime.now().isoformat(),
        "inputs": {},
        "generated_files": {},
        "warnings": [],
    }

    required_data_files = [
        "tree_map_final.csv",
        "tree_map_final.json",
        "coverage.csv",
        "path.csv",
        "logfile.csv",
        "intervals.csv",
        "collision.csv",
    ]
    for name in required_data_files:
        src = data_dir / name
        copied = copy_file(src, exp_dir, manifest["inputs"], key=name)
        if copied is None and name in ("tree_map_final.csv", "tree_map_final.json"):
            print("Missing required file: {}".format(src), file=sys.stderr)
            return 1

    latest_pkl = latest_file(snapshots_dir, "*.pkl")
    latest_png = latest_file(snapshots_dir, "*.png")
    latest_bt = latest_file(octomaps_dir, "*.bt")

    if latest_pkl:
        copy_file(latest_pkl, exp_dir, manifest["inputs"], key="latest_snapshot_pkl")
    else:
        manifest["warnings"].append("No PKL found in snapshots dir.")
    if latest_pkl:
        matched_png = matched_png_for_pkl(latest_pkl)
        if matched_png:
            copy_file(matched_png, exp_dir, manifest["inputs"], key="latest_snapshot_png")
        else:
            manifest["warnings"].append("No PNG found matching latest PKL snapshot name.")
    elif latest_png:
        copy_file(latest_png, exp_dir, manifest["inputs"], key="latest_snapshot_png")
    if latest_bt:
        copy_file(latest_bt, exp_dir, manifest["inputs"], key="latest_octomap_bt")
    else:
        manifest["warnings"].append("No .bt found in octomaps dir.")

    gt_csv = exp_dir / "world_jean_ground_truth.csv"
    gt_svg = exp_dir / "world_jean_ground_truth.svg"
    cmp_svg = exp_dir / "world_jean_compare_vs_csv.svg"
    map_svg = exp_dir / "tree_map_final_plot.svg"
    map_diameter_svg = exp_dir / "tree_map_diameter_plot.svg"

    gt_script = ti_scripts_dir / "world_tree_ground_truth_plotter.py"
    cmp_script = ti_scripts_dir / "world_tree_compare_plotter.py"
    map_script = ti_scripts_dir / "tree_map_csv_plotter.py"

    if not gt_script.exists() or not cmp_script.exists() or not map_script.exists():
        print("Missing one or more required scripts in {}".format(ti_scripts_dir), file=sys.stderr)
        return 1

    run_cmd(
        [
            "python3",
            str(gt_script),
            "--world",
            str(world_path),
            "--uri-filter",
            args.uri_filter,
            "--csv-out",
            str(gt_csv),
            "--svg-out",
            str(gt_svg),
            "--title",
            "World Ground Truth",
            "--x-min",
            str(args.x_min),
            "--x-max",
            str(args.x_max),
            "--y-min",
            str(args.y_min),
            "--y-max",
            str(args.y_max),
        ]
    )

    run_cmd(
        [
            "python3",
            str(cmp_script),
            "--truth-csv",
            str(gt_csv),
            "--map-csv",
            str(exp_dir / "tree_map_final.csv"),
            "--svg-out",
            str(cmp_svg),
            "--title",
            "{}: Ground Truth vs tree_map_final.csv".format(exp_name),
            "--x-min",
            str(args.x_min),
            "--x-max",
            str(args.x_max),
            "--y-min",
            str(args.y_min),
            "--y-max",
            str(args.y_max),
        ]
    )

    run_cmd(
        [
            "python3",
            str(map_script),
            "--csv",
            str(exp_dir / "tree_map_final.csv"),
            "--out",
            str(map_svg),
            "--label",
            "id_hits",
        ]
    )
    run_cmd(
        [
            "python3",
            str(map_script),
            "--csv",
            str(exp_dir / "tree_map_final.csv"),
            "--out",
            str(map_diameter_svg),
            "--label",
            "id_diameter",
        ]
    )

    has_numpy = importlib.util.find_spec("numpy") is not None

    if pkl_svg_plotter.exists() and latest_pkl and has_numpy:
        try:
            cluster_svg = exp_dir / "tree_cluster_state_clusters.svg"
            run_cmd(
                [
                    "python3",
                    str(pkl_svg_plotter),
                    str(exp_dir / latest_pkl.name),
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
        pkl_error = try_extract_pkl_meta(exp_dir / latest_pkl.name, exp_dir / "pkl_meta.json")
        if pkl_error:
            manifest["warnings"].append("PKL meta extraction failed: {}".format(pkl_error))

    truth_rows = load_truth(gt_csv)
    map_rows = load_map(exp_dir / "tree_map_final.csv")
    metrics, matching_rows = compute_metrics(
        truth_rows,
        map_rows,
        selected_threshold_m=args.match_threshold,
        ground_truth_diameter_m=args.ground_truth_diameter_m,
    )
    metrics["experiment_name"] = exp_name

    metrics_path = exp_dir / "metrics.json"
    matching_path = exp_dir / "matching.csv"
    summary_path = exp_dir / "summary.md"

    metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True))
    write_matching_csv(matching_path, matching_rows)
    write_summary_md(summary_path, exp_name, metrics)

    for generated in (
        gt_csv,
        gt_svg,
        cmp_svg,
        map_svg,
        map_diameter_svg,
        metrics_path,
        matching_path,
        summary_path,
    ):
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
