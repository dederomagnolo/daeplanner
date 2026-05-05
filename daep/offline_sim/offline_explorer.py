#!/usr/bin/env python3

"""Offline DAEP/AEP-style exploration simulator.

This script keeps the full point cloud as hidden ground truth, reveals an
observed voxel map through a simulated camera FoV, and runs a lightweight
offline version of the AEP local RRT + gain computation.

It intentionally avoids ROS/Gazebo dependencies so it can be used for quick
algorithmic experiments.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import re
import struct
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Optional, Sequence, Tuple


Point3 = Tuple[float, float, float]
State4 = Tuple[float, float, float, float]
VoxelKey = Tuple[int, int, int]
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_SCENE_START: State4 = (0.0, 0.0, 1.5, 0.0)
DEFAULT_MAX_ITERATIONS = 91
DEFAULT_RUN_NAME = "offline"
DEFAULT_CLOUD_FILENAME = "3d_scene_prepared_10x10.pcd"
DEFAULT_OFFLINE_CONFIG_FILENAME = "world_jean_offline_config.yaml"
DEFAULT_CONFIG_RELATIVE = Path("catkin_ws/src/aeplanner/rpl_exploration/config/world_jean_exploration.yaml")


def strip_comment(line: str) -> str:
    """Strip YAML-style comments while keeping the parser intentionally small."""
    in_quote = False
    quote_char = ""
    out = []
    for ch in line:
        if ch in ("'", '"'):
            if not in_quote:
                in_quote = True
                quote_char = ch
            elif quote_char == ch:
                in_quote = False
        if ch == "#" and not in_quote:
            break
        out.append(ch)
    return "".join(out).strip()


def parse_config_value(raw: str):
    raw = strip_comment(raw).strip()
    if not raw:
        return ""
    if raw.startswith("[") and raw.endswith("]"):
        inner = raw[1:-1].strip()
        if not inner:
            return []
        return [parse_config_value(part.strip()) for part in inner.split(",")]
    lower = raw.lower()
    if lower in ("true", "false"):
        return lower == "true"
    if (raw.startswith('"') and raw.endswith('"')) or (raw.startswith("'") and raw.endswith("'")):
        return raw[1:-1]
    try:
        if any(ch in raw for ch in (".", "e", "E")):
            return float(raw)
        return int(raw)
    except ValueError:
        return raw


def load_simple_yaml(path: Optional[Path]) -> Dict[str, object]:
    """Read the flat ROS-style YAML files used by the planner configs."""
    if not path:
        return {}
    values: Dict[str, object] = {}
    with path.open("r", encoding="utf-8") as f:
        for raw_line in f:
            line = strip_comment(raw_line)
            if not line or ":" not in line:
                continue
            key, raw_value = line.split(":", 1)
            values[key.strip()] = parse_config_value(raw_value)
    return values


def parse_floats(text: str, expected: Optional[int] = None) -> Tuple[float, ...]:
    parts = [p for p in re.split(r"[\s,]+", text.strip()) if p]
    values = tuple(float(p) for p in parts)
    if expected is not None and len(values) != expected:
        raise argparse.ArgumentTypeError(f"expected {expected} numbers, got {len(values)}")
    return values


def frange(start: float, stop: float, step: float) -> Iterator[float]:
    value = start
    eps = abs(step) * 1e-9
    while value < stop - eps:
        yield value
        value += step


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def distance3(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def first_existing_path(candidates: Sequence[Path]) -> Path:
    for path in candidates:
        if path.exists():
            return path
    return candidates[0]


def default_cloud_path() -> Path:
    return first_existing_path(
        [
            SCRIPT_DIR / DEFAULT_CLOUD_FILENAME,
            Path.cwd() / "daep" / "offline_sim" / DEFAULT_CLOUD_FILENAME,
            Path.cwd() / "offline_sim" / DEFAULT_CLOUD_FILENAME,
            SCRIPT_DIR.parent / "daep" / "offline_sim" / DEFAULT_CLOUD_FILENAME,
        ]
    )


def default_config_path() -> Path:
    return first_existing_path(
        [
            SCRIPT_DIR / DEFAULT_OFFLINE_CONFIG_FILENAME,
            SCRIPT_DIR.parent / DEFAULT_CONFIG_RELATIVE,
            Path.cwd() / "daep" / DEFAULT_CONFIG_RELATIVE,
            Path.cwd() / DEFAULT_CONFIG_RELATIVE,
            SCRIPT_DIR.parent / "daep" / DEFAULT_CONFIG_RELATIVE,
        ]
    )


def default_tree_ground_truth_csv() -> Path:
    return first_existing_path(
        [
            SCRIPT_DIR.parent / "ground_truth" / "world_tree_ground_truth.csv",
            Path.cwd() / "daep" / "ground_truth" / "world_tree_ground_truth.csv",
            Path.cwd() / "ground_truth" / "world_tree_ground_truth.csv",
            SCRIPT_DIR.parent / "daep" / "ground_truth" / "world_tree_ground_truth.csv",
        ]
    )


def sanitize_run_name(name: str) -> str:
    clean = re.sub(r"[^A-Za-z0-9_.-]+", "-", name.strip())
    clean = clean.strip("-._")
    return clean or DEFAULT_RUN_NAME


def default_output_dir(run_name: str, seed: int) -> Path:
    raw = run_name.strip() or DEFAULT_RUN_NAME
    if "{seed}" in raw:
        folder = sanitize_run_name(raw.format(seed=seed))
        return SCRIPT_DIR / "runs" / folder
    clean = sanitize_run_name(raw)
    if clean.endswith("seed{}".format(seed)) or clean.endswith(str(seed)):
        folder = clean
    else:
        folder = "{}-seed{}".format(clean, seed)
    return SCRIPT_DIR / "runs" / folder


def as_float(value, default: float = 0.0) -> float:
    try:
        return float(str(value).strip())
    except Exception:
        return default


def as_optional_int(value) -> Optional[int]:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        return int(float(text))
    except Exception:
        return None


@dataclass(frozen=True)
class TreeTruth:
    index: int
    tree_id: Optional[int]
    name: str
    x: float
    y: float
    z: float


def load_tree_truth_csv(path: Path) -> List[TreeTruth]:
    if not path.exists():
        raise FileNotFoundError("tree ground-truth CSV not found: {}".format(path))
    trees: List[TreeTruth] = []
    with path.open("r", newline="", encoding="utf-8") as f:
        for idx, row in enumerate(csv.DictReader(f), start=1):
            trees.append(
                TreeTruth(
                    index=idx,
                    tree_id=as_optional_int(row.get("tree_id")),
                    name=str(row.get("name", "")).strip(),
                    x=as_float(row.get("x")),
                    y=as_float(row.get("y")),
                    z=as_float(row.get("z")),
                )
            )
    trees.sort(key=lambda t: (t.tree_id is None, t.tree_id if t.tree_id is not None else 999999, t.name))
    return trees


@dataclass
class OfflineParams:
    # Camera and ray-casting model. Defaults mirror granso_exploration.yaml.
    hfov: float = 103.2
    vfov: float = 77.4
    dr: float = 0.2
    dphi: float = 10.0
    dtheta: float = 10.0
    resolution: float = 0.2

    # AEP gain/RRT parameters.
    r_min: float = 0.0
    r_max: float = 5.0
    zero_gain: float = 15.0
    lambda_gain: float = 0.75
    extension_range: float = 1.0
    max_sampling_radius: float = 10.0
    init_iterations: int = 15
    max_sampled_initial_nodes: int = 30
    max_sampled_nodes: int = 70
    bounding_radius: float = 0.2
    boosted_boundary_length: float = 1.0
    boost_magnitude: float = 6.0

    # DAEP terms are kept, but dynamic obstacles are disabled in this first
    # offline version. With no DFM, dynamic_gain == static_gain and dfm_score=0.
    zeta: float = 0.5
    drone_linear_velocity: float = 0.5
    drone_angular_velocity: float = 1.0
    fixed_z_from_start: bool = False

    # Environment bounds.
    boundary_min: Point3 = (-5.0, -20.0, 0.0)
    boundary_max: Point3 = (70.0, 40.0, 5.0)

    # Offline simulator controls.
    sample_attempts_per_node: int = 250
    max_iterations: int = DEFAULT_MAX_ITERATIONS
    stop_on_coverage: bool = False
    target_coverage_pct: float = 95.0
    frontier_sample_limit: int = 6000

    # Offline tree detector. Defaults mirror tree_detector_node.py where
    # possible, with a denser synthetic camera ray grid for offline frames.
    tree_detector_enabled: bool = True
    tree_detector_slice_z_min: float = 1.15
    tree_detector_slice_z_max: float = 1.45
    tree_detector_dtheta: float = 2.0
    tree_detector_dphi: float = 2.0
    tree_detector_clustering_mode: str = "dbscan_gmm"
    tree_detector_dbscan_eps: float = 0.30
    tree_detector_dbscan_min_samples: int = 10
    tree_detector_min_cluster_points: int = 3
    tree_detector_ransac_iterations: int = 50
    tree_detector_ransac_distance_threshold: float = 0.01
    tree_detector_ransac_min_inlier_ratio: float = 0.0
    tree_detector_min_diameter: float = 0.15
    tree_detector_max_diameter: float = 1.00
    tree_detector_cell_size: float = 0.20
    tree_detector_min_points_per_cell: int = 3
    tree_detector_min_cells_per_cluster: int = 3
    tree_detector_max_cells_per_cluster: int = 200
    tree_match_threshold: float = 0.60

    @classmethod
    def from_ros_yaml(cls, path: Optional[Path]) -> "OfflineParams":
        cfg = load_simple_yaml(path)
        params = cls()
        mapping = {
            "camera/horizontal_fov": "hfov",
            "camera/vertical_fov": "vfov",
            "raycast/dr": "dr",
            "raycast/dphi": "dphi",
            "raycast/dtheta": "dtheta",
            "res": "resolution",
            "aep/gain/r_min": "r_min",
            "aep/gain/r_max": "r_max",
            "aep/gain/zero": "zero_gain",
            "aep/gain/lambda": "lambda_gain",
            "aep/tree/extension_range": "extension_range",
            "aep/tree/max_sampling_radius": "max_sampling_radius",
            "aep/tree/initial_iterations": "init_iterations",
            "daep/rrt/max_sampled_initial_nodes": "max_sampled_initial_nodes",
            "daep/rrt/max_sampled_nodes": "max_sampled_nodes",
            "system/bbx/r": "bounding_radius",
            "boosted_boundary_length": "boosted_boundary_length",
            "boost_magnitude": "boost_magnitude",
            "daep/dfm/zeta": "zeta",
            "drone_linear_velocity": "drone_linear_velocity",
            "drone_angular_velocity": "drone_angular_velocity",
            "daep/fixed_z_from_start": "fixed_z_from_start",
            "offline/max_iterations": "max_iterations",
            "offline/stop_on_coverage": "stop_on_coverage",
            "offline/tree_detector_enabled": "tree_detector_enabled",
            "offline/tree_detector_slice_z_min": "tree_detector_slice_z_min",
            "offline/tree_detector_slice_z_max": "tree_detector_slice_z_max",
            "offline/tree_detector_dtheta": "tree_detector_dtheta",
            "offline/tree_detector_dphi": "tree_detector_dphi",
            "offline/tree_detector_dbscan_eps": "tree_detector_dbscan_eps",
            "offline/tree_detector_dbscan_min_samples": "tree_detector_dbscan_min_samples",
            "offline/tree_detector_min_cluster_points": "tree_detector_min_cluster_points",
            "offline/tree_detector_ransac_iterations": "tree_detector_ransac_iterations",
            "offline/tree_detector_ransac_distance_threshold": "tree_detector_ransac_distance_threshold",
            "offline/tree_detector_ransac_min_inlier_ratio": "tree_detector_ransac_min_inlier_ratio",
            "offline/tree_detector_min_diameter": "tree_detector_min_diameter",
            "offline/tree_detector_max_diameter": "tree_detector_max_diameter",
            "offline/tree_detector_cell_size": "tree_detector_cell_size",
            "offline/tree_detector_min_points_per_cell": "tree_detector_min_points_per_cell",
            "offline/tree_detector_min_cells_per_cluster": "tree_detector_min_cells_per_cluster",
            "offline/tree_detector_max_cells_per_cluster": "tree_detector_max_cells_per_cluster",
            "offline/tree_match_threshold": "tree_match_threshold",
        }
        for key, attr in mapping.items():
            if key in cfg:
                current = getattr(params, attr)
                value = cfg[key]
                if isinstance(current, bool):
                    if isinstance(value, str):
                        setattr(params, attr, value.strip().lower() in {"1", "true", "yes", "on"})
                    else:
                        setattr(params, attr, bool(value))
                elif isinstance(current, int):
                    setattr(params, attr, int(value))
                else:
                    setattr(params, attr, float(value))
        if "boundary/min" in cfg:
            params.boundary_min = tuple(float(v) for v in cfg["boundary/min"])[:3]  # type: ignore[assignment]
        if "boundary/max" in cfg:
            params.boundary_max = tuple(float(v) for v in cfg["boundary/max"])[:3]  # type: ignore[assignment]
        return params

    def apply_cli(self, args: argparse.Namespace) -> None:
        if args.resolution is not None:
            self.resolution = args.resolution
            self.dr = min(self.dr, args.resolution)
        if args.max_iterations is not None:
            self.max_iterations = args.max_iterations
        if args.target_coverage_pct is not None:
            self.target_coverage_pct = args.target_coverage_pct
            self.stop_on_coverage = True
        if args.boundary_min is not None:
            self.boundary_min = tuple(args.boundary_min)  # type: ignore[assignment]
        if args.boundary_max is not None:
            self.boundary_max = tuple(args.boundary_max)  # type: ignore[assignment]
        if args.fixed_z_from_start is not None:
            self.fixed_z_from_start = args.fixed_z_from_start
        if args.tree_match_threshold is not None:
            self.tree_match_threshold = args.tree_match_threshold
        if args.tree_detector_ransac_distance_threshold is not None:
            self.tree_detector_ransac_distance_threshold = args.tree_detector_ransac_distance_threshold


def load_point_cloud(path: Path, max_points: Optional[int] = None) -> List[Point3]:
    suffix = path.suffix.lower()
    if suffix == ".ply":
        points = load_ascii_ply(path, max_points)
    elif suffix == ".pcd":
        points = load_pcd(path, max_points)
    elif suffix == ".obj":
        points = load_obj_vertices(path, max_points)
    else:
        points = load_xyz_like(path, max_points)
    if not points:
        raise ValueError(f"no points loaded from {path}")
    return points


def load_xyz_like(path: Path, max_points: Optional[int]) -> List[Point3]:
    points: List[Point3] = []
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if max_points is not None and len(points) >= max_points:
                break
            clean = strip_comment(line)
            if not clean:
                continue
            parts = [p for p in re.split(r"[\s,;]+", clean) if p]
            if len(parts) < 3:
                continue
            try:
                points.append((float(parts[0]), float(parts[1]), float(parts[2])))
            except ValueError:
                continue
    return points


def load_obj_vertices(path: Path, max_points: Optional[int]) -> List[Point3]:
    points: List[Point3] = []
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if max_points is not None and len(points) >= max_points:
                break
            if not line.startswith("v "):
                continue
            parts = line.split()
            if len(parts) >= 4:
                points.append((float(parts[1]), float(parts[2]), float(parts[3])))
    return points


def load_ascii_ply(path: Path, max_points: Optional[int]) -> List[Point3]:
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        first = f.readline().strip()
        if first != "ply":
            raise ValueError(f"{path} is not a PLY file")
        vertex_count = None
        is_ascii = False
        properties: List[str] = []
        in_vertex = False
        for line in f:
            line = line.strip()
            if line.startswith("format ascii"):
                is_ascii = True
            elif line.startswith("element vertex"):
                vertex_count = int(line.split()[-1])
                in_vertex = True
            elif line.startswith("element ") and not line.startswith("element vertex"):
                in_vertex = False
            elif line.startswith("property") and in_vertex:
                properties.append(line.split()[-1])
            elif line == "end_header":
                break
        if not is_ascii:
            raise ValueError("only ASCII PLY is supported by the offline simulator")
        if vertex_count is None:
            raise ValueError("PLY header has no vertex count")
        try:
            ix, iy, iz = properties.index("x"), properties.index("y"), properties.index("z")
        except ValueError as exc:
            raise ValueError("PLY vertices must have x, y and z properties") from exc
        points: List[Point3] = []
        for _ in range(vertex_count):
            if max_points is not None and len(points) >= max_points:
                break
            parts = f.readline().split()
            if len(parts) <= max(ix, iy, iz):
                continue
            points.append((float(parts[ix]), float(parts[iy]), float(parts[iz])))
        return points


def load_pcd(path: Path, max_points: Optional[int]) -> List[Point3]:
    with path.open("rb") as f:
        header_lines: List[str] = []
        while True:
            line_bytes = f.readline()
            if not line_bytes:
                raise ValueError("PCD header ended before DATA line")
            line = line_bytes.decode("ascii", errors="ignore").strip()
            header_lines.append(line)
            if line.startswith("DATA"):
                data_mode = line.split(maxsplit=1)[1].strip().lower()
                data_start = f.tell()
                break

        header = parse_pcd_header(header_lines)
        if data_mode == "ascii":
            return load_ascii_pcd_payload(f, header, max_points)
        if data_mode == "binary":
            return load_binary_pcd_payload(f, header, data_start, max_points)
        raise ValueError(f"unsupported PCD DATA mode: {data_mode}")


def parse_pcd_header(header_lines: Sequence[str]) -> Dict[str, object]:
    header: Dict[str, object] = {
        "fields": [],
        "size": [],
        "type": [],
        "count": [],
        "points": None,
    }
    for line in header_lines:
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        key = parts[0].upper()
        values = parts[1:]
        if key == "FIELDS":
            header["fields"] = values
        elif key == "SIZE":
            header["size"] = [int(v) for v in values]
        elif key == "TYPE":
            header["type"] = values
        elif key == "COUNT":
            header["count"] = [int(v) for v in values]
        elif key == "POINTS":
            header["points"] = int(values[0])
        elif key == "WIDTH" and header["points"] is None:
            header["points"] = int(values[0])
    fields = header["fields"]
    if not isinstance(fields, list) or not fields:
        raise ValueError("PCD header must include FIELDS")
    if not header["count"]:
        header["count"] = [1] * len(fields)
    if not header["size"] or not header["type"]:
        raise ValueError("PCD header must include SIZE and TYPE")
    if not all(len(header[name]) == len(fields) for name in ("size", "type", "count")):  # type: ignore[arg-type]
        raise ValueError("PCD FIELDS, SIZE, TYPE and COUNT lengths do not match")
    if header["points"] is None:
        raise ValueError("PCD header must include POINTS or WIDTH")
    return header


def pcd_field_offsets(header: Dict[str, object]) -> Tuple[Dict[str, Tuple[int, int, str]], int]:
    fields = header["fields"]
    sizes = header["size"]
    types = header["type"]
    counts = header["count"]
    assert isinstance(fields, list)
    assert isinstance(sizes, list)
    assert isinstance(types, list)
    assert isinstance(counts, list)
    offsets: Dict[str, Tuple[int, int, str]] = {}
    offset = 0
    for field, size, typ, count in zip(fields, sizes, types, counts):
        offsets[str(field)] = (offset, int(size), str(typ).upper())
        offset += int(size) * int(count)
    return offsets, offset


def pcd_unpack_scalar(data: bytes, offset: int, size: int, typ: str) -> float:
    if typ == "F" and size == 4:
        return float(struct.unpack_from("<f", data, offset)[0])
    if typ == "F" and size == 8:
        return float(struct.unpack_from("<d", data, offset)[0])
    if typ == "I" and size == 1:
        return float(struct.unpack_from("<b", data, offset)[0])
    if typ == "I" and size == 2:
        return float(struct.unpack_from("<h", data, offset)[0])
    if typ == "I" and size == 4:
        return float(struct.unpack_from("<i", data, offset)[0])
    if typ == "U" and size == 1:
        return float(struct.unpack_from("<B", data, offset)[0])
    if typ == "U" and size == 2:
        return float(struct.unpack_from("<H", data, offset)[0])
    if typ == "U" and size == 4:
        return float(struct.unpack_from("<I", data, offset)[0])
    raise ValueError(f"unsupported PCD scalar type {typ}{size}")


def load_binary_pcd_payload(f, header: Dict[str, object], data_start: int, max_points: Optional[int]) -> List[Point3]:
    del data_start
    offsets, point_step = pcd_field_offsets(header)
    for required in ("x", "y", "z"):
        if required not in offsets:
            raise ValueError("PCD fields must include x, y and z")
    point_count = int(header["points"])
    read_count = point_count if max_points is None else min(point_count, max_points)
    points: List[Point3] = []
    for _ in range(read_count):
        raw = f.read(point_step)
        if len(raw) < point_step:
            break
        x_offset, x_size, x_type = offsets["x"]
        y_offset, y_size, y_type = offsets["y"]
        z_offset, z_size, z_type = offsets["z"]
        point = (
            pcd_unpack_scalar(raw, x_offset, x_size, x_type),
            pcd_unpack_scalar(raw, y_offset, y_size, y_type),
            pcd_unpack_scalar(raw, z_offset, z_size, z_type),
        )
        if all(math.isfinite(v) for v in point):
            points.append(point)
    return points


def load_ascii_pcd_payload(f, header: Dict[str, object], max_points: Optional[int]) -> List[Point3]:
    fields = header["fields"]
    assert isinstance(fields, list)
    try:
        ix, iy, iz = fields.index("x"), fields.index("y"), fields.index("z")
    except ValueError as exc:
        raise ValueError("PCD fields must include x, y and z") from exc
    point_count = int(header["points"])
    read_count = point_count if max_points is None else min(point_count, max_points)
    points: List[Point3] = []
    for raw_line in f:
        if len(points) >= read_count:
            break
        line = raw_line.decode("ascii", errors="ignore")
        parts = line.split()
        if len(parts) <= max(ix, iy, iz):
            continue
        point = (float(parts[ix]), float(parts[iy]), float(parts[iz]))
        if all(math.isfinite(v) for v in point):
            points.append(point)
    return points


def make_demo_cloud(resolution: float = 0.2) -> Tuple[List[Point3], Point3, Point3, State4]:
    """Create a tiny room-like point cloud for smoke tests."""
    points: List[Point3] = []
    xmin, xmax = 0.0, 12.0
    ymin, ymax = 0.0, 8.0
    zmin, zmax = 0.0, 3.0
    xs = [xmin + i * resolution for i in range(int((xmax - xmin) / resolution) + 1)]
    ys = [ymin + i * resolution for i in range(int((ymax - ymin) / resolution) + 1)]
    zs = [zmin + i * resolution for i in range(int((zmax - zmin) / resolution) + 1)]
    for x in xs:
        for z in zs:
            points.append((x, ymin, z))
            points.append((x, ymax, z))
    for y in ys:
        for z in zs:
            points.append((xmin, y, z))
            points.append((xmax, y, z))
    for cx, cy, radius in ((4.0, 3.5, 0.45), (8.0, 4.5, 0.65)):
        for x in xs:
            for y in ys:
                if (x - cx) ** 2 + (y - cy) ** 2 <= radius ** 2:
                    for z in zs:
                        if z <= 2.4:
                            points.append((x, y, z))
    return points, (xmin, ymin, zmin), (xmax, ymax, zmax), (1.0, 1.0, 1.2, 0.0)


class VoxelWorld:
    def __init__(self, params: OfflineParams, ground_truth_points: Sequence[Point3]):
        self.params = params
        self.resolution = params.resolution
        self.boundary_min = params.boundary_min
        self.boundary_max = params.boundary_max
        self.ground_truth_occupied = {self.key(p) for p in ground_truth_points if self.in_bounds_point(p)}
        self.observed_free: set[VoxelKey] = set()
        self.observed_occupied: set[VoxelKey] = set()
        self.collision_offsets = self._make_neighbor_offsets(params.bounding_radius)

    def key(self, point: Sequence[float]) -> VoxelKey:
        r = self.resolution
        return (math.floor(point[0] / r), math.floor(point[1] / r), math.floor(point[2] / r))

    def voxel_center(self, key: VoxelKey) -> Point3:
        r = self.resolution
        return ((key[0] + 0.5) * r, (key[1] + 0.5) * r, (key[2] + 0.5) * r)

    def in_bounds_point(self, point: Sequence[float]) -> bool:
        return (
            self.boundary_min[0] < point[0] < self.boundary_max[0]
            and self.boundary_min[1] < point[1] < self.boundary_max[1]
            and self.boundary_min[2] < point[2] < self.boundary_max[2]
        )

    def in_bounds_key(self, key: VoxelKey) -> bool:
        return self.in_bounds_point(self.voxel_center(key))

    def observed_state_key(self, key: VoxelKey) -> str:
        if key in self.observed_occupied:
            return "occupied"
        if key in self.observed_free:
            return "free"
        return "unknown"

    def observed_state_point(self, point: Sequence[float]) -> str:
        return self.observed_state_key(self.key(point))

    def is_known_free_point(self, point: Sequence[float]) -> bool:
        return self.observed_state_point(point) == "free"

    def mark_free(self, key: VoxelKey) -> bool:
        if key in self.observed_occupied:
            return False
        before = len(self.observed_free)
        self.observed_free.add(key)
        return len(self.observed_free) != before

    def mark_occupied(self, key: VoxelKey) -> bool:
        changed = key not in self.observed_occupied
        self.observed_occupied.add(key)
        self.observed_free.discard(key)
        return changed

    def reveal_fov(self, state: State4) -> int:
        """Reveal observed voxels using the current drone pose and yaw."""
        changed = 0
        origin = (state[0], state[1], state[2])
        yaw_deg = math.degrees(state[3])
        theta_start = yaw_deg - self.params.hfov / 2.0
        theta_stop = yaw_deg + self.params.hfov / 2.0
        phi_start = 90.0 - self.params.vfov / 2.0
        phi_stop = 90.0 + self.params.vfov / 2.0
        for theta in frange(theta_start, theta_stop, self.params.dtheta):
            theta_rad = math.radians(theta)
            for phi in frange(phi_start, phi_stop, self.params.dphi):
                phi_rad = math.radians(phi)
                changed += self._reveal_ray(origin, theta_rad, phi_rad)
        return changed

    def camera_occupied_points(self, state: State4, dtheta: float, dphi: float) -> List[Point3]:
        """Return visible occupied surface samples for the offline detector."""
        origin = (state[0], state[1], state[2])
        yaw_deg = math.degrees(state[3])
        theta_start = yaw_deg - self.params.hfov / 2.0
        theta_stop = yaw_deg + self.params.hfov / 2.0
        phi_start = 90.0 - self.params.vfov / 2.0
        phi_stop = 90.0 + self.params.vfov / 2.0
        points: List[Point3] = []
        for theta in frange(theta_start, theta_stop, max(0.1, dtheta)):
            theta_rad = math.radians(theta)
            for phi in frange(phi_start, phi_stop, max(0.1, dphi)):
                phi_rad = math.radians(phi)
                hit = self._raycast_occupied_point(origin, theta_rad, phi_rad)
                if hit is not None:
                    points.append(hit)
        return points

    def _raycast_occupied_point(self, origin: Point3, theta_rad: float, phi_rad: float) -> Optional[Point3]:
        for radius in frange(0.0, self.params.r_max + self.params.dr * 0.5, self.params.dr):
            point = (
                origin[0] + radius * math.cos(theta_rad) * math.sin(phi_rad),
                origin[1] + radius * math.sin(theta_rad) * math.sin(phi_rad),
                origin[2] + radius * math.cos(phi_rad),
            )
            if not self.in_bounds_point(point):
                break
            key = self.key(point)
            if key in self.ground_truth_occupied:
                return self.voxel_center(key)
        return None

    def _reveal_ray(self, origin: Point3, theta_rad: float, phi_rad: float) -> int:
        changed = 0
        for radius in frange(0.0, self.params.r_max + self.params.dr * 0.5, self.params.dr):
            point = (
                origin[0] + radius * math.cos(theta_rad) * math.sin(phi_rad),
                origin[1] + radius * math.sin(theta_rad) * math.sin(phi_rad),
                origin[2] + radius * math.cos(phi_rad),
            )
            if not self.in_bounds_point(point):
                break
            key = self.key(point)
            if key in self.ground_truth_occupied:
                if self.mark_occupied(key):
                    changed += 1
                break
            if self.mark_free(key):
                changed += 1
        return changed

    def _make_neighbor_offsets(self, radius: float) -> List[VoxelKey]:
        cells = max(0, int(math.ceil(radius / self.resolution)))
        offsets: List[VoxelKey] = []
        radius_sq = (radius + self.resolution * 0.5) ** 2
        for dx in range(-cells, cells + 1):
            for dy in range(-cells, cells + 1):
                for dz in range(-cells, cells + 1):
                    dist_sq = (dx * self.resolution) ** 2 + (dy * self.resolution) ** 2 + (dz * self.resolution) ** 2
                    if dist_sq <= radius_sq:
                        offsets.append((dx, dy, dz))
        return offsets or [(0, 0, 0)]

    def collision_line(self, start: State4, end: State4, radius: float) -> bool:
        length = distance3(start, end)
        if length < 1e-9:
            return self._near_observed_occupied(self.key(start))
        step = max(self.resolution * 0.5, min(self.resolution, radius))
        steps = max(1, int(math.ceil(length / step)))
        for i in range(steps + 1):
            t = i / float(steps)
            point = (
                start[0] + (end[0] - start[0]) * t,
                start[1] + (end[1] - start[1]) * t,
                start[2] + (end[2] - start[2]) * t,
            )
            if self._near_observed_occupied(self.key(point)):
                return True
        return False

    def _near_observed_occupied(self, key: VoxelKey) -> bool:
        for dx, dy, dz in self.collision_offsets:
            if (key[0] + dx, key[1] + dy, key[2] + dz) in self.observed_occupied:
                return True
        return False

    def point_on_xy_boundaries(self, point: Sequence[float]) -> bool:
        margin = self.params.boosted_boundary_length
        return (
            point[0] < self.boundary_min[0] + margin
            or point[0] > self.boundary_max[0] - margin
            or point[1] < self.boundary_min[1] + margin
            or point[1] > self.boundary_max[1] - margin
        )

    def observed_occupied_count_near_xy(self, x: float, y: float, radius: float, z_min: float, z_max: float) -> int:
        radius_sq = radius * radius
        count = 0
        for key in self.observed_occupied:
            cx, cy, cz = self.voxel_center(key)
            if cz < z_min or cz > z_max:
                continue
            if (cx - x) ** 2 + (cy - y) ** 2 <= radius_sq:
                count += 1
        return count

    def frontier_goal(self, current: State4, rng: random.Random) -> Optional[State4]:
        free_keys = list(self.observed_free)
        if len(free_keys) > self.params.frontier_sample_limit:
            free_keys = rng.sample(free_keys, self.params.frontier_sample_limit)
        best_key: Optional[VoxelKey] = None
        best_score = -1.0
        neighbor_offsets = (
            (1, 0, 0),
            (-1, 0, 0),
            (0, 1, 0),
            (0, -1, 0),
            (0, 0, 1),
            (0, 0, -1),
        )
        for key in free_keys:
            center = self.voxel_center(key)
            if not self.in_bounds_point(center):
                continue
            if self.collision_line(current, (center[0], center[1], center[2], current[3]), self.params.bounding_radius):
                continue
            unknown_neighbors = 0
            for dx, dy, dz in neighbor_offsets:
                nkey = (key[0] + dx, key[1] + dy, key[2] + dz)
                if self.in_bounds_key(nkey) and self.observed_state_key(nkey) == "unknown":
                    unknown_neighbors += 1
            if unknown_neighbors == 0:
                continue
            dist = max(0.1, distance3(current, center))
            score = float(unknown_neighbors) / (1.0 + dist)
            if score > best_score:
                best_score = score
                best_key = key
        if best_key is None:
            return None
        x, y, z = self.voxel_center(best_key)
        yaw = math.atan2(y - current[1], x - current[0])
        return (x, y, z, yaw)

    def volume_stats(self) -> Dict[str, float]:
        occupied = len(self.observed_occupied)
        free = len(self.observed_free - self.observed_occupied)
        voxel_volume = self.resolution ** 3
        known_volume = (free + occupied) * voxel_volume
        total_volume = (
            (self.boundary_max[0] - self.boundary_min[0])
            * (self.boundary_max[1] - self.boundary_min[1])
            * (self.boundary_max[2] - self.boundary_min[2])
        )
        coverage_pct = 100.0 * known_volume / total_volume if total_volume > 0 else 0.0
        return {
            "coverage_m3": known_volume,
            "coverage_pct": coverage_pct,
            "free_m3": free * voxel_volume,
            "occupied_m3": occupied * voxel_volume,
            "unmapped_m3": max(0.0, total_volume - known_volume),
            "known_voxels": free + occupied,
            "free_voxels": free,
            "occupied_voxels": occupied,
            "total_volume_m3": total_volume,
        }


@dataclass
class RRTNode:
    state: State4
    parent: Optional[int] = None
    gain: float = 0.0
    dynamic_gain: float = 0.0
    dfm_score: float = 0.0
    static_score: float = 0.0
    dynamic_score: float = 0.0
    cost: float = 0.0
    time_cost: float = 0.0


@dataclass
class PlanResult:
    goal: Optional[State4]
    path_label: str
    planner_mode: str
    is_clear: bool
    best_idx: Optional[int]
    selected_idx: Optional[int]
    tree_nodes: List[RRTNode]
    planning_time_sec: float


class OfflineTreeDetector:
    def __init__(self, params: OfflineParams, rng: random.Random):
        self.params = params
        self.rng = rng

    def detect(self, camera_points: Sequence[Point3]) -> List[Dict[str, object]]:
        points = list(camera_points)
        if len(points) < 20:
            return []
        trunk = [p for p in points if self.params.tree_detector_slice_z_min <= p[2] <= self.params.tree_detector_slice_z_max]
        if len(trunk) < 20:
            return []
        if self.params.tree_detector_clustering_mode == "grid_cc":
            return self._detect_grid_cc(trunk)
        return self._detect_dbscan(trunk)

    def _detect_dbscan(self, trunk: Sequence[Point3]) -> List[Dict[str, object]]:
        labels = self._dbscan([(p[0], p[1]) for p in trunk])
        detections: List[Dict[str, object]] = []
        cluster_id = 0
        for label in sorted({lb for lb in labels if lb >= 0}):
            indices = [i for i, lb in enumerate(labels) if lb == label]
            if len(indices) < self.params.tree_detector_min_cluster_points:
                continue
            cluster_points = [trunk[i] for i in indices]
            detection = self._detection_from_cluster(cluster_points, cluster_id)
            if detection is not None:
                detections.append(detection)
            cluster_id += 1
        return detections

    def _dbscan(self, points_xy: Sequence[Tuple[float, float]]) -> List[int]:
        eps_sq = self.params.tree_detector_dbscan_eps**2
        min_samples = self.params.tree_detector_dbscan_min_samples
        labels = [-99] * len(points_xy)
        cluster_id = 0

        def neighbors(index: int) -> List[int]:
            x, y = points_xy[index]
            out = []
            for j, (px, py) in enumerate(points_xy):
                if (px - x) ** 2 + (py - y) ** 2 <= eps_sq:
                    out.append(j)
            return out

        for index in range(len(points_xy)):
            if labels[index] != -99:
                continue
            seed = neighbors(index)
            if len(seed) < min_samples:
                labels[index] = -1
                continue
            labels[index] = cluster_id
            queue = list(seed)
            cursor = 0
            while cursor < len(queue):
                point_idx = queue[cursor]
                cursor += 1
                if labels[point_idx] == -1:
                    labels[point_idx] = cluster_id
                if labels[point_idx] != -99:
                    continue
                labels[point_idx] = cluster_id
                point_neighbors = neighbors(point_idx)
                if len(point_neighbors) >= min_samples:
                    for neighbor in point_neighbors:
                        if neighbor not in queue:
                            queue.append(neighbor)
            cluster_id += 1
        return labels

    def _detect_grid_cc(self, trunk: Sequence[Point3]) -> List[Dict[str, object]]:
        cells: Dict[Tuple[int, int], List[Point3]] = {}
        for point in trunk:
            key = self._to_cell(point[0], point[1])
            cells.setdefault(key, []).append(point)
        occupied = {cell for cell, pts in cells.items() if len(pts) >= self.params.tree_detector_min_points_per_cell}
        detections: List[Dict[str, object]] = []
        for cluster_id, component in enumerate(self._cluster_cells(occupied)):
            if len(component) < self.params.tree_detector_min_cells_per_cluster:
                continue
            if len(component) > self.params.tree_detector_max_cells_per_cluster:
                continue
            cluster_points: List[Point3] = []
            for cell in component:
                cluster_points.extend(cells[cell])
            if len(cluster_points) < self.params.tree_detector_min_cluster_points:
                continue
            detection = self._detection_from_cluster(cluster_points, cluster_id)
            if detection is not None:
                detections.append(detection)
        return detections

    def _to_cell(self, x: float, y: float) -> Tuple[int, int]:
        cell_size = max(1e-9, self.params.tree_detector_cell_size)
        return (math.floor(x / cell_size), math.floor(y / cell_size))

    def _cluster_cells(self, occupied: set[Tuple[int, int]]) -> List[List[Tuple[int, int]]]:
        visited: set[Tuple[int, int]] = set()
        components: List[List[Tuple[int, int]]] = []
        for start in occupied:
            if start in visited:
                continue
            queue = [start]
            visited.add(start)
            component: List[Tuple[int, int]] = []
            cursor = 0
            while cursor < len(queue):
                cx, cy = queue[cursor]
                cursor += 1
                component.append((cx, cy))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        neighbor = (cx + dx, cy + dy)
                        if neighbor in occupied and neighbor not in visited:
                            visited.add(neighbor)
                            queue.append(neighbor)
            components.append(component)
        return components

    def _detection_from_cluster(self, cluster_points: Sequence[Point3], cluster_id: int) -> Optional[Dict[str, object]]:
        fit = self._fit_circle_ransac([(p[0], p[1]) for p in cluster_points])
        if fit is None:
            return None
        cx, cy, radius, fit_error, inlier_ratio = fit
        z = sum(p[2] for p in cluster_points) / float(len(cluster_points))
        return {
            "x": cx,
            "y": cy,
            "z": z,
            "radius": radius,
            "diameter": 2.0 * radius,
            "cluster_label": cluster_id,
            "cluster_points": len(cluster_points),
            "fit_error": fit_error,
            "confidence": self._confidence(inlier_ratio, fit_error),
            "matched_tree_id": "",
            "match_distance_m": "",
        }

    def _fit_circle_ransac(self, points_xy: Sequence[Tuple[float, float]]) -> Optional[Tuple[float, float, float, float, float]]:
        if len(points_xy) < 3:
            return None
        best: Optional[Tuple[float, float, float]] = None
        best_inliers = 0
        best_fit_error = 0.0
        iterations = max(1, self.params.tree_detector_ransac_iterations)
        for _ in range(iterations):
            try:
                p1, p2, p3 = self.rng.sample(list(points_xy), 3)
                circle = self._circle_from_points(p1, p2, p3)
                if circle is None:
                    continue
                cx, cy, radius = circle
                diameter = 2.0 * radius
                if diameter < self.params.tree_detector_min_diameter or diameter > self.params.tree_detector_max_diameter:
                    continue
                residuals = [abs(math.hypot(px - cx, py - cy) - radius) for px, py in points_xy]
                inliers = [r for r in residuals if r < self.params.tree_detector_ransac_distance_threshold]
                if not inliers:
                    continue
                fit_error = sum(inliers) / float(len(inliers))
                if len(inliers) > best_inliers:
                    best = (cx, cy, radius)
                    best_inliers = len(inliers)
                    best_fit_error = fit_error
            except Exception:
                continue
        if best is None:
            return None
        inlier_ratio = best_inliers / float(max(1, len(points_xy)))
        if inlier_ratio < self.params.tree_detector_ransac_min_inlier_ratio:
            return None
        return best[0], best[1], best[2], best_fit_error, inlier_ratio

    @staticmethod
    def _circle_from_points(
        p1: Tuple[float, float], p2: Tuple[float, float], p3: Tuple[float, float]
    ) -> Optional[Tuple[float, float, float]]:
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        temp = x2 * x2 + y2 * y2
        bc = (x1 * x1 + y1 * y1 - temp) / 2.0
        cd = (temp - x3 * x3 - y3 * y3) / 2.0
        det = (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2)
        if abs(det) < 1e-8:
            return None
        cx = (bc * (y2 - y3) - cd * (y1 - y2)) / det
        cy = ((x1 - x2) * cd - (x2 - x3) * bc) / det
        radius = math.hypot(x1 - cx, y1 - cy)
        if not math.isfinite(radius):
            return None
        return cx, cy, radius

    def _confidence(self, inlier_ratio: float, fit_error: float) -> float:
        err_term = math.exp(-fit_error / max(2.0 * self.params.tree_detector_ransac_distance_threshold, 1e-6))
        conf = 0.62 * inlier_ratio + 0.38 * err_term
        return max(0.0, min(1.0, conf))


class TreeObservationTracker:
    def __init__(self, params: OfflineParams, trees: Sequence[TreeTruth]):
        self.params = params
        self.trees = list(trees)
        self.seen_by_index: Dict[int, Dict[str, object]] = {}
        self.match_counts: Dict[int, int] = {tree.index: 0 for tree in self.trees}

    def update(
        self,
        detections: Sequence[Dict[str, object]],
        iteration: int,
        elapsed_time: float,
        path_length: float,
    ) -> List[TreeTruth]:
        newly_seen: List[TreeTruth] = []
        pairs: List[Tuple[float, int, int]] = []
        for ti, tree in enumerate(self.trees):
            for di, detection in enumerate(detections):
                dist = math.hypot(float(detection["x"]) - tree.x, float(detection["y"]) - tree.y)
                pairs.append((dist, ti, di))
        pairs.sort(key=lambda item: item[0])
        used_trees: set[int] = set()
        used_detections: set[int] = set()
        for dist, tree_pos, detection_pos in pairs:
            if dist > self.params.tree_match_threshold:
                break
            if tree_pos in used_trees or detection_pos in used_detections:
                continue
            tree = self.trees[tree_pos]
            detection = detections[detection_pos]
            detection["matched_tree_id"] = "" if tree.tree_id is None else tree.tree_id
            detection["match_distance_m"] = dist
            used_trees.add(tree_pos)
            used_detections.add(detection_pos)
            self.match_counts[tree.index] = self.match_counts.get(tree.index, 0) + 1
            if tree.index in self.seen_by_index:
                continue
            self.seen_by_index[tree.index] = {
                "tree": tree,
                "first_seen_iteration": iteration,
                "first_seen_time_sec": elapsed_time,
                "first_seen_path_length_m": path_length,
                "detection": dict(detection),
            }
            newly_seen.append(tree)
        return newly_seen

    def coverage_pct(self) -> float:
        return 100.0 * len(self.seen_by_index) / float(len(self.trees)) if self.trees else 0.0

    def stats(self) -> Dict[str, object]:
        return {
            "trees_total": len(self.trees),
            "trees_seen": len(self.seen_by_index),
            "trees_seen_pct": self.coverage_pct(),
            "tree_match_threshold_m": self.params.tree_match_threshold,
            "tree_detector_slice_z_min": self.params.tree_detector_slice_z_min,
            "tree_detector_slice_z_max": self.params.tree_detector_slice_z_max,
        }

    def tree_rows(self) -> List[Dict[str, object]]:
        rows: List[Dict[str, object]] = []
        for tree in self.trees:
            seen = self.seen_by_index.get(tree.index)
            detection = seen.get("detection", {}) if seen else {}
            rows.append(
                {
                    "tree_id": "" if tree.tree_id is None else tree.tree_id,
                    "name": tree.name,
                    "x": tree.x,
                    "y": tree.y,
                    "z": tree.z,
                    "seen": 1 if seen else 0,
                    "first_seen_iteration": "" if not seen else seen["first_seen_iteration"],
                    "first_seen_time_sec": "" if not seen else seen["first_seen_time_sec"],
                    "first_seen_path_length_m": "" if not seen else seen["first_seen_path_length_m"],
                    "matched_detections": self.match_counts.get(tree.index, 0),
                    "first_detection_x": "" if not seen else detection.get("x", ""),
                    "first_detection_y": "" if not seen else detection.get("y", ""),
                    "first_detection_radius_m": "" if not seen else detection.get("radius", ""),
                    "first_detection_confidence": "" if not seen else detection.get("confidence", ""),
                    "first_detection_distance_m": "" if not seen else detection.get("match_distance_m", ""),
                }
            )
        return rows


class OfflineExplorer:
    def __init__(
        self,
        params: OfflineParams,
        world: VoxelWorld,
        output_dir: Path,
        start: State4,
        seed: int,
        trees: Optional[Sequence[TreeTruth]] = None,
    ):
        self.params = params
        self.world = world
        self.output_dir = output_dir
        self.fixed_z: Optional[float] = start[2] if params.fixed_z_from_start else None
        self.start_state = self._apply_fixed_z(start)
        self.current_state = self.start_state
        self.rng = random.Random(seed)
        self.seed = seed
        self.path_length = 0.0
        self.elapsed_time = 0.0
        self.total_planning_time = 0.0
        self.total_fly_time = 0.0
        self.tree_tracker = TreeObservationTracker(params, trees or []) if trees else None
        self.tree_detector = OfflineTreeDetector(params, self.rng) if params.tree_detector_enabled else None

    def _apply_fixed_z(self, state: State4) -> State4:
        if self.fixed_z is None:
            return state
        return (state[0], state[1], self.fixed_z, state[3])

    def run(self) -> Dict[str, object]:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.world.reveal_fov(self.current_state)

        with (
            (self.output_dir / "path.csv").open("w", newline="", encoding="utf-8") as path_f,
            (self.output_dir / "logfile.csv").open("w", newline="", encoding="utf-8") as log_f,
            (self.output_dir / "coverage.csv").open("w", newline="", encoding="utf-8") as coverage_f,
            (self.output_dir / "rrt_tree_log.csv").open("w", newline="", encoding="utf-8") as tree_f,
            (self.output_dir / "rrt_goal_log.csv").open("w", newline="", encoding="utf-8") as goal_f,
            (self.output_dir / "tree_coverage.csv").open("w", newline="", encoding="utf-8") as tree_coverage_f,
            (self.output_dir / "tree_detections.csv").open("w", newline="", encoding="utf-8") as tree_detections_f,
        ):
            path_writer = csv.writer(path_f)
            log_writer = csv.writer(log_f)
            coverage_writer = csv.writer(coverage_f)
            tree_writer = csv.writer(tree_f)
            goal_writer = csv.writer(goal_f)
            tree_coverage_writer = csv.writer(tree_coverage_f)
            tree_detections_writer = csv.writer(tree_detections_f)

            path_writer.writerow(["Goal x", "Goal y", "Goal z", "Planner"])
            log_writer.writerow(["Iteration", "Path length", "Time", "Planning", "Flying"])
            coverage_writer.writerow(["Time", "Coverage (m3)", "Coverage (%)", "Free space", "Occupied Space", "Unmapped Space"])
            self.write_rrt_tree_header(tree_writer)
            self.write_rrt_goal_header(goal_writer)
            self.write_tree_coverage_header(tree_coverage_writer)
            self.write_tree_detections_header(tree_detections_writer)

            initial_detections = self.detect_trees()
            initial_new_trees = self.update_tree_observations(initial_detections, 0)
            self.write_tree_detections_rows(tree_detections_writer, 0, initial_detections)
            self.write_log_row(log_writer, 0)
            self.write_coverage_row(coverage_writer)
            self.write_tree_coverage_row(tree_coverage_writer, 0, initial_new_trees)

            stop_reason = "max_iterations"
            for iteration in range(1, self.params.max_iterations + 1):
                plan = self.plan_once(iteration)
                self.write_rrt_logs(tree_writer, goal_writer, iteration, plan)
                self.total_planning_time += plan.planning_time_sec

                if plan.goal is None:
                    stop_reason = "no_goal"
                    break

                previous = self.current_state
                fly_time = self.time_to_reach(previous, plan.goal)
                self.path_length += distance3(previous, plan.goal)
                self.total_fly_time += fly_time
                self.elapsed_time += fly_time + plan.planning_time_sec
                self.current_state = plan.goal
                revealed = self.world.reveal_fov(self.current_state)
                detections = self.detect_trees()
                new_trees = self.update_tree_observations(detections, iteration)

                path_writer.writerow([f"{plan.goal[0]:.6f}", f"{plan.goal[1]:.6f}", f"{plan.goal[2]:.6f}", plan.path_label])
                self.write_log_row(log_writer, iteration)
                self.write_coverage_row(coverage_writer)
                self.write_tree_coverage_row(tree_coverage_writer, iteration, new_trees)
                self.write_tree_detections_rows(tree_detections_writer, iteration, detections)

                coverage = self.world.volume_stats()["coverage_pct"]
                if self.params.stop_on_coverage and coverage >= self.params.target_coverage_pct:
                    stop_reason = "target_coverage"
                    break
                if revealed == 0 and not plan.is_clear and plan.path_label != "rrt":
                    stop_reason = "no_new_voxels"
                    break

        summary = {
            "seed": self.seed,
            "stop_reason": stop_reason,
            "start_state": self.start_state,
            "final_state": self.current_state,
            "fixed_z_m": self.fixed_z,
            "path_length_m": self.path_length,
            "elapsed_time_sec": self.elapsed_time,
            "planning_time_sec": self.total_planning_time,
            "flying_time_sec": self.total_fly_time,
            "coverage": self.world.volume_stats(),
            "tree_coverage": self.tree_tracker.stats() if self.tree_tracker else None,
            "params": asdict(self.params),
        }
        with (self.output_dir / "summary.json").open("w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, sort_keys=True)
        self.write_tree_observations_csv()
        return summary

    def plan_once(self, iteration: int) -> PlanResult:
        start_time = time.perf_counter()
        nodes: List[RRTNode] = [RRTNode(state=self.current_state)]
        best_idx: Optional[int] = None
        n_valid = 0
        n_sampled = 0

        while self._should_keep_sampling(n_valid, n_sampled, best_idx, nodes):
            candidate = self.sample_candidate(nodes)
            n_sampled += 1
            if candidate is None:
                continue
            state, parent_idx = candidate
            parent = nodes[parent_idx]
            arrival = parent.time_cost + self.time_to_reach(parent.state, state)
            gain, dynamic_gain, yaw = self.gain_cubature((state[0], state[1], state[2], state[3]), arrival)
            state = (state[0], state[1], state[2], yaw)
            dist = distance3(parent.state, state)
            static_score = parent.static_score + gain * math.exp(-self.params.lambda_gain * dist)
            dynamic_score = parent.dynamic_score + dynamic_gain * math.exp(
                -self.params.lambda_gain * dist * (1.0 + self.params.zeta * 0.0)
            )
            node = RRTNode(
                state=state,
                parent=parent_idx,
                gain=gain,
                dynamic_gain=dynamic_gain,
                dfm_score=0.0,
                static_score=static_score,
                dynamic_score=dynamic_score,
                cost=parent.cost + dist,
                time_cost=arrival,
            )
            nodes.append(node)
            node_idx = len(nodes) - 1
            n_valid += 1
            if best_idx is None or node.dynamic_score > nodes[best_idx].dynamic_score:
                best_idx = node_idx

        planner_mode = "frontier_no_valid"
        selected_idx: Optional[int] = None
        goal: Optional[State4] = None
        path_label = "none"
        is_clear = False

        if best_idx is not None and nodes[best_idx].dynamic_score > self.params.zero_gain:
            selected_idx = self.first_child_on_branch(nodes, best_idx)
            if selected_idx is not None:
                goal = self._apply_fixed_z(nodes[selected_idx].state)
                planner_mode = "local_rrt"
                path_label = "aep"
                is_clear = True
        else:
            if best_idx is not None:
                planner_mode = "frontier_zero_gain"
            frontier = self.world.frontier_goal(self.current_state, self.rng)
            if frontier is not None:
                goal = self._apply_fixed_z(frontier)
                path_label = "rrt"

        planning_time = time.perf_counter() - start_time
        return PlanResult(
            goal=goal,
            path_label=path_label,
            planner_mode=planner_mode,
            is_clear=is_clear,
            best_idx=best_idx,
            selected_idx=selected_idx,
            tree_nodes=nodes,
            planning_time_sec=planning_time,
        )

    def _should_keep_sampling(self, n_valid: int, n_sampled: int, best_idx: Optional[int], nodes: List[RRTNode]) -> bool:
        first_phase = n_valid < self.params.init_iterations and n_sampled < self.params.max_sampled_initial_nodes
        second_phase = (
            n_valid > 0
            and n_sampled < self.params.max_sampled_nodes
            and best_idx is not None
            and nodes[best_idx].dynamic_score < self.params.zero_gain
        )
        return first_phase or second_phase

    def sample_candidate(self, nodes: List[RRTNode]) -> Optional[Tuple[State4, int]]:
        for _ in range(self.params.sample_attempts_per_node):
            offset = self.sample_new_point()
            raw = (
                self.current_state[0] + offset[0],
                self.current_state[1] + offset[1],
                self.current_state[2] + (0.0 if self.fixed_z is not None else offset[2]),
                self.current_state[3],
            )
            parent_idx = self.choose_parent(nodes, raw)
            if parent_idx is None:
                continue
            state = self.restrict_distance(nodes[parent_idx].state, raw)
            if not self.world.in_bounds_point(state):
                continue
            if not self.world.is_known_free_point(state):
                continue
            if self.world.collision_line(nodes[parent_idx].state, state, self.params.bounding_radius):
                continue
            return state, parent_idx
        return None

    def sample_new_point(self) -> Point3:
        radius = self.params.max_sampling_radius
        while True:
            z_offset = 0.0 if self.fixed_z is not None else radius * 2.0 * (self.rng.random() - 0.5)
            point = (
                radius * 2.0 * (self.rng.random() - 0.5),
                radius * 2.0 * (self.rng.random() - 0.5),
                z_offset,
            )
            if point[0] ** 2 + point[1] ** 2 + point[2] ** 2 <= radius**2:
                return point

    def choose_parent(self, nodes: List[RRTNode], state: State4) -> Optional[int]:
        in_range: List[Tuple[float, int]] = []
        nearest: Optional[Tuple[float, int]] = None
        limit = self.params.extension_range + 0.5
        for idx, node in enumerate(nodes):
            dist = distance3(node.state, state)
            if nearest is None or dist < nearest[0]:
                nearest = (dist, idx)
            if dist <= limit:
                in_range.append((node.cost, idx))
        if in_range:
            return min(in_range)[1]
        return nearest[1] if nearest is not None else None

    def restrict_distance(self, nearest: State4, new_pos: State4) -> State4:
        dx = new_pos[0] - nearest[0]
        dy = new_pos[1] - nearest[1]
        dz = new_pos[2] - nearest[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist > self.params.extension_range and dist > 1e-9:
            scale = self.params.extension_range / dist
            dx *= scale
            dy *= scale
            if self.fixed_z is None:
                dz *= scale
        next_z = nearest[2] if self.fixed_z is not None else nearest[2] + dz
        return (nearest[0] + dx, nearest[1] + dy, next_z, new_pos[3])

    def gain_cubature(self, state: State4, time_of_arrival: float) -> Tuple[float, float, float]:
        del time_of_arrival  # Dynamic obstacle blocking is not modeled in this offline version yet.
        origin = (state[0], state[1], state[2])
        gain_by_theta: Dict[int, float] = {}
        dynamic_gain_by_theta: Dict[int, float] = {}
        dphi_rad = math.radians(self.params.dphi)
        dtheta_rad = math.radians(self.params.dtheta)
        phi_start = 90.0 - self.params.vfov / 2.0
        phi_stop = 90.0 + self.params.vfov / 2.0

        for theta in range(-180, 180, int(self.params.dtheta)):
            theta_rad = math.radians(theta)
            theta_gain = 0.0
            theta_dynamic_gain = 0.0
            for phi in frange(phi_start, phi_stop, self.params.dphi):
                phi_rad = math.radians(phi)
                ray_gain = 0.0
                ray_dynamic_gain = 0.0
                for radius in frange(self.params.r_min, self.params.r_max, self.params.dr):
                    point = (
                        origin[0] + radius * math.cos(theta_rad) * math.sin(phi_rad),
                        origin[1] + radius * math.sin(theta_rad) * math.sin(phi_rad),
                        origin[2] + radius * math.cos(phi_rad),
                    )
                    if not self.world.in_bounds_point(point):
                        break
                    state_at_point = self.world.observed_state_point(point)
                    if state_at_point == "occupied":
                        break
                    if state_at_point == "free":
                        continue
                    current_gain = (
                        (2.0 * radius * radius * self.params.dr + (1.0 / 6.0) * self.params.dr**3)
                        * dtheta_rad
                        * math.sin(phi_rad)
                        * math.sin(dphi_rad / 2.0)
                    )
                    if self.world.point_on_xy_boundaries(point):
                        current_gain *= self.params.boost_magnitude
                    ray_gain += current_gain
                    ray_dynamic_gain += current_gain
                theta_gain += ray_gain
                theta_dynamic_gain += ray_dynamic_gain
            gain_by_theta[theta] = theta_gain
            dynamic_gain_by_theta[theta] = theta_dynamic_gain

        best_yaw = 0
        best_yaw_score = 0.0
        best_dynamic_yaw_score = 0.0
        half_hfov = self.params.hfov / 2.0
        for yaw in range(-180, 180):
            yaw_score = 0.0
            dynamic_yaw_score = 0.0
            for fov in range(int(-half_hfov), int(half_hfov)):
                theta = yaw + fov
                if theta < -180:
                    theta += 360
                if theta > 180:
                    theta -= 360
                yaw_score += gain_by_theta.get(theta, 0.0)
                dynamic_yaw_score += dynamic_gain_by_theta.get(theta, 0.0)
            if best_yaw_score < yaw_score:
                best_yaw_score = yaw_score
                best_dynamic_yaw_score = dynamic_yaw_score
                best_yaw = yaw
        return best_yaw_score, best_dynamic_yaw_score, math.radians(best_yaw)

    def first_child_on_branch(self, nodes: List[RRTNode], best_idx: int) -> Optional[int]:
        current = best_idx
        child = None
        while current is not None and current != 0:
            child = current
            parent = nodes[current].parent
            if parent is None:
                return None
            current = parent
        return child

    def time_to_reach(self, current: State4, target: State4) -> float:
        yaw_difference = normalize_angle(target[3] - current[3])
        rotation_time = abs(yaw_difference) / max(1e-9, self.params.drone_angular_velocity)
        linear_time = distance3(current, target) / max(1e-9, self.params.drone_linear_velocity)
        return rotation_time + linear_time

    def write_log_row(self, writer: csv.writer, iteration: int) -> None:
        writer.writerow(
            [
                iteration,
                f"{self.path_length:.6f}",
                f"{self.elapsed_time:.6f}",
                f"{self.total_planning_time:.6f}",
                f"{self.total_fly_time:.6f}",
            ]
        )

    def write_coverage_row(self, writer: csv.writer) -> None:
        stats = self.world.volume_stats()
        writer.writerow(
            [
                f"{self.elapsed_time:.6f}",
                f"{stats['coverage_m3']:.6f}",
                f"{stats['coverage_pct']:.6f}",
                f"{stats['free_m3']:.6f}",
                f"{stats['occupied_m3']:.6f}",
                f"{stats['unmapped_m3']:.6f}",
            ]
        )

    def detect_trees(self) -> List[Dict[str, object]]:
        if not self.tree_detector:
            return []
        camera_points = self.world.camera_occupied_points(
            self.current_state,
            self.params.tree_detector_dtheta,
            self.params.tree_detector_dphi,
        )
        return self.tree_detector.detect(camera_points)

    def update_tree_observations(self, detections: Sequence[Dict[str, object]], iteration: int) -> List[TreeTruth]:
        if not self.tree_tracker:
            return []
        return self.tree_tracker.update(detections, iteration, self.elapsed_time, self.path_length)

    def write_tree_coverage_header(self, writer: csv.writer) -> None:
        writer.writerow(
            [
                "Iteration",
                "Time",
                "Path length",
                "Trees seen",
                "Trees total",
                "Trees seen (%)",
                "New trees seen",
                "New tree ids",
            ]
        )

    def write_tree_coverage_row(self, writer: csv.writer, iteration: int, new_trees: Sequence[TreeTruth]) -> None:
        if self.tree_tracker:
            stats = self.tree_tracker.stats()
        else:
            stats = {"trees_seen": 0, "trees_total": 0, "trees_seen_pct": 0.0}
        new_ids = []
        for tree in new_trees:
            new_ids.append(str(tree.tree_id if tree.tree_id is not None else tree.index))
        writer.writerow(
            [
                iteration,
                f"{self.elapsed_time:.6f}",
                f"{self.path_length:.6f}",
                stats["trees_seen"],
                stats["trees_total"],
                f"{float(stats['trees_seen_pct']):.6f}",
                len(new_trees),
                ";".join(new_ids),
            ]
        )

    def write_tree_detections_header(self, writer: csv.writer) -> None:
        writer.writerow(
            [
                "Iteration",
                "Time",
                "Path length",
                "detection_id",
                "x",
                "y",
                "z",
                "radius_m",
                "diameter_m",
                "cluster_label",
                "cluster_points",
                "fit_error",
                "confidence",
                "matched_tree_id",
                "match_distance_m",
            ]
        )

    def write_tree_detections_rows(
        self, writer: csv.writer, iteration: int, detections: Sequence[Dict[str, object]]
    ) -> None:
        for detection_id, detection in enumerate(detections, start=1):
            match_distance = detection.get("match_distance_m", "")
            writer.writerow(
                [
                    iteration,
                    f"{self.elapsed_time:.6f}",
                    f"{self.path_length:.6f}",
                    detection_id,
                    f"{float(detection['x']):.6f}",
                    f"{float(detection['y']):.6f}",
                    f"{float(detection['z']):.6f}",
                    f"{float(detection['radius']):.6f}",
                    f"{float(detection['diameter']):.6f}",
                    int(detection["cluster_label"]),
                    int(detection["cluster_points"]),
                    f"{float(detection['fit_error']):.6f}",
                    f"{float(detection['confidence']):.6f}",
                    detection.get("matched_tree_id", ""),
                    "" if match_distance == "" else f"{float(match_distance):.6f}",
                ]
            )

    def write_tree_observations_csv(self) -> None:
        path = self.output_dir / "tree_observations.csv"
        fields = [
            "tree_id",
            "name",
            "x",
            "y",
            "z",
            "seen",
            "first_seen_iteration",
            "first_seen_time_sec",
            "first_seen_path_length_m",
            "matched_detections",
            "first_detection_x",
            "first_detection_y",
            "first_detection_radius_m",
            "first_detection_confidence",
            "first_detection_distance_m",
        ]
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            if not self.tree_tracker:
                return
            for row in self.tree_tracker.tree_rows():
                out = dict(row)
                for key in ("x", "y", "z"):
                    out[key] = "{:.6f}".format(float(out[key]))
                for key in ("first_seen_time_sec", "first_seen_path_length_m"):
                    if out[key] != "":
                        out[key] = "{:.6f}".format(float(out[key]))
                for key in (
                    "first_detection_x",
                    "first_detection_y",
                    "first_detection_radius_m",
                    "first_detection_confidence",
                    "first_detection_distance_m",
                ):
                    if out[key] != "":
                        out[key] = "{:.6f}".format(float(out[key]))
                writer.writerow(out)

    def write_rrt_tree_header(self, writer: csv.writer) -> None:
        writer.writerow(
            [
                "planning_iteration",
                "stamp_sec",
                "planner_mode",
                "node_id",
                "parent_id",
                "depth",
                "x",
                "y",
                "z",
                "yaw",
                "gain",
                "dynamic_gain",
                "dfm_score",
                "static_score",
                "dynamic_score",
                "cost",
                "time_cost",
                "is_root",
                "is_best_node",
                "is_selected_goal",
                "is_best_branch",
                "child_count",
            ]
        )

    def write_rrt_goal_header(self, writer: csv.writer) -> None:
        writer.writerow(
            [
                "planning_iteration",
                "stamp_sec",
                "planner_mode",
                "is_clear",
                "tree_node_count",
                "best_node_id",
                "selected_goal_node_id",
                "selected_goal_source",
                "best_x",
                "best_y",
                "best_z",
                "best_yaw",
                "best_gain",
                "best_dynamic_gain",
                "best_dfm_score",
                "best_static_score",
                "best_dynamic_score",
                "selected_x",
                "selected_y",
                "selected_z",
                "selected_yaw",
            ]
        )

    def write_rrt_logs(self, tree_writer: csv.writer, goal_writer: csv.writer, iteration: int, plan: PlanResult) -> None:
        stamp = self.elapsed_time + plan.planning_time_sec
        children: Dict[int, List[int]] = {idx: [] for idx in range(len(plan.tree_nodes))}
        for idx, node in enumerate(plan.tree_nodes):
            if node.parent is not None:
                children[node.parent].append(idx)

        best_branch = set()
        current = plan.best_idx
        while current is not None:
            best_branch.add(current)
            current = plan.tree_nodes[current].parent

        depths = self.compute_depths(plan.tree_nodes)
        for idx, node in enumerate(plan.tree_nodes):
            parent_id = node.parent if node.parent is not None else -1
            tree_writer.writerow(
                [
                    iteration,
                    f"{stamp:.6f}",
                    plan.planner_mode,
                    idx,
                    parent_id,
                    depths[idx],
                    f"{node.state[0]:.6f}",
                    f"{node.state[1]:.6f}",
                    f"{node.state[2]:.6f}",
                    f"{node.state[3]:.6f}",
                    f"{node.gain:.6f}",
                    f"{node.dynamic_gain:.6f}",
                    f"{node.dfm_score:.6f}",
                    f"{node.static_score:.6f}",
                    f"{node.dynamic_score:.6f}",
                    f"{node.cost:.6f}",
                    f"{node.time_cost:.6f}",
                    1 if idx == 0 else 0,
                    1 if idx == plan.best_idx else 0,
                    1 if idx == plan.selected_idx else 0,
                    1 if idx in best_branch else 0,
                    len(children[idx]),
                ]
            )

        best = plan.tree_nodes[plan.best_idx] if plan.best_idx is not None else None
        selected = plan.tree_nodes[plan.selected_idx] if plan.selected_idx is not None else None
        selected_source = "none"
        if selected is not None:
            selected_source = "rrt_tree"
        elif plan.path_label == "rrt":
            selected_source = "frontier"

        row: List[object] = [
            iteration,
            f"{stamp:.6f}",
            plan.planner_mode,
            1 if plan.is_clear else 0,
            len(plan.tree_nodes),
            plan.best_idx if plan.best_idx is not None else -1,
            plan.selected_idx if plan.selected_idx is not None else -1,
            selected_source,
        ]
        if best is not None:
            row.extend(
                [
                    f"{best.state[0]:.6f}",
                    f"{best.state[1]:.6f}",
                    f"{best.state[2]:.6f}",
                    f"{best.state[3]:.6f}",
                    f"{best.gain:.6f}",
                    f"{best.dynamic_gain:.6f}",
                    f"{best.dfm_score:.6f}",
                    f"{best.static_score:.6f}",
                    f"{best.dynamic_score:.6f}",
                ]
            )
        else:
            row.extend([""] * 9)
        if selected is not None:
            row.extend([f"{selected.state[0]:.6f}", f"{selected.state[1]:.6f}", f"{selected.state[2]:.6f}", f"{selected.state[3]:.6f}"])
        else:
            row.extend([""] * 4)
        goal_writer.writerow(row)

    def compute_depths(self, nodes: List[RRTNode]) -> Dict[int, int]:
        depths: Dict[int, int] = {}
        for idx in range(len(nodes)):
            depth = 0
            current = nodes[idx].parent
            while current is not None:
                depth += 1
                current = nodes[current].parent
            depths[idx] = depth
        return depths


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a fast offline DAEP/AEP-style point-cloud exploration simulation.")
    parser.add_argument("--cloud", type=Path, help="Point cloud path. Default: offline 10x10 PCD.")
    parser.add_argument("--demo", action="store_true", help="Use a small built-in synthetic point cloud.")
    parser.add_argument("--config", type=Path, help="ROS-style planner YAML. Default: world_jean_exploration.yaml.")
    parser.add_argument("--run-name", default=DEFAULT_RUN_NAME, help="Run folder name base. Default output is <run-name>-seed<seed>.")
    parser.add_argument("--output", type=Path, help="Output directory for CSV logs. Overrides --run-name.")
    parser.add_argument("--start", type=str, help="Initial state as x,y,z[,yaw_rad]. Default for the 10x10 scene: 0,0,1.5,0.")
    parser.add_argument("--seed", type=int, default=1, help="Random seed for RRT sampling.")
    parser.add_argument("--max-iterations", type=int, help="Maximum planning iterations.")
    parser.add_argument("--target-coverage-pct", type=float, help="Stop when observed known volume reaches this percentage.")
    parser.add_argument("--resolution", type=float, help="Override voxel resolution.")
    parser.add_argument("--boundary-min", type=parse_floats, help="Override boundary min as x,y,z.")
    parser.add_argument("--boundary-max", type=parse_floats, help="Override boundary max as x,y,z.")
    parser.add_argument("--fixed-z-from-start", dest="fixed_z_from_start", action="store_true", help="Keep z fixed at the initial altitude.")
    parser.add_argument("--free-z", dest="fixed_z_from_start", action="store_false", help="Allow z variation even if config enables fixed z.")
    parser.add_argument("--tree-ground-truth-csv", type=Path, help="Tree ground-truth CSV used for offline tree observation metrics.")
    parser.add_argument("--tree-match-threshold", type=float, help="Distance threshold used to match offline detector candidates to GT trees.")
    parser.add_argument("--tree-detector-ransac-distance-threshold", type=float, help="Override the offline detector RANSAC inlier threshold.")
    parser.add_argument("--max-points", type=int, help="Read at most N points from the input cloud.")
    parser.add_argument("--report", dest="report", action="store_true", help="Generate the offline report after the simulation finishes. Enabled by default.")
    parser.add_argument("--no-report", dest="report", action="store_false", help="Skip report generation after the simulation finishes.")
    parser.add_argument("--report-output", type=Path, help="Report output directory. Default: <output>/report.")
    parser.add_argument("--report-ground-truth-csv", type=Path, help="Shared tree ground-truth CSV used by the generated report.")
    parser.add_argument("--report-copy-inputs", action="store_true", help="Copy run CSV/JSON inputs into the generated report/data directory.")
    parser.set_defaults(fixed_z_from_start=None, report=True)
    return parser


def resolve_start(args: argparse.Namespace, params: OfflineParams, demo_start: Optional[State4]) -> State4:
    if args.start:
        values = parse_floats(args.start)
        if len(values) == 3:
            return (values[0], values[1], values[2], 0.0)
        if len(values) == 4:
            return (values[0], values[1], values[2], values[3])
        raise argparse.ArgumentTypeError("--start expects x,y,z or x,y,z,yaw_rad")
    if demo_start is not None:
        return demo_start
    if getattr(args, "using_default_scene", False):
        return DEFAULT_SCENE_START
    return (
        params.boundary_min[0] + 1.0,
        params.boundary_min[1] + 1.0,
        clamp((params.boundary_min[2] + params.boundary_max[2]) * 0.5, params.boundary_min[2] + 0.5, params.boundary_max[2] - 0.5),
        0.0,
    )


def generate_report_after_run(args: argparse.Namespace) -> None:
    from export_offline_report import main as export_report_main

    report_args = [
        "--run-dir",
        str(args.output),
    ]
    if args.cloud:
        report_args.extend(["--cloud", str(args.cloud)])
    if args.config:
        report_args.extend(["--config", str(args.config)])
    if args.report_output:
        report_args.extend(["--output-dir", str(args.report_output)])
    if args.report_ground_truth_csv:
        report_args.extend(["--ground-truth-csv", str(args.report_ground_truth_csv)])
    else:
        effective_tree_csv = getattr(args, "effective_tree_ground_truth_csv", None)
        if effective_tree_csv:
            report_args.extend(["--ground-truth-csv", str(effective_tree_csv)])
    if args.report_copy_inputs:
        report_args.append("--copy-inputs")

    rc = export_report_main(report_args)
    if rc != 0:
        raise RuntimeError("offline report generation failed with exit code {}".format(rc))


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    args.using_default_scene = False
    if not args.demo and args.cloud is None:
        args.cloud = default_cloud_path()
        args.using_default_scene = True
    if args.config is None and not args.demo:
        candidate_config = default_config_path()
        if candidate_config.exists():
            args.config = candidate_config
    if args.output is None:
        args.output = default_output_dir(args.run_name, args.seed)
    if args.max_iterations is None:
        args.max_iterations = DEFAULT_MAX_ITERATIONS
    params = OfflineParams.from_ros_yaml(args.config)
    params.apply_cli(args)

    demo_start: Optional[State4] = None
    if args.demo:
        points, bmin, bmax, demo_start = make_demo_cloud(params.resolution)
        if args.boundary_min is None:
            params.boundary_min = bmin
        if args.boundary_max is None:
            params.boundary_max = bmax
    else:
        assert args.cloud is not None
        points = load_point_cloud(args.cloud, args.max_points)

    tree_csv = args.tree_ground_truth_csv or args.report_ground_truth_csv
    if tree_csv is None and not args.demo:
        default_tree_csv = default_tree_ground_truth_csv()
        if default_tree_csv.exists():
            tree_csv = default_tree_csv
    trees: List[TreeTruth] = []
    if tree_csv is not None:
        tree_csv = tree_csv.expanduser().resolve()
        trees = load_tree_truth_csv(tree_csv)
    args.effective_tree_ground_truth_csv = tree_csv

    start = resolve_start(args, params, demo_start)
    world = VoxelWorld(params, points)
    explorer = OfflineExplorer(params=params, world=world, output_dir=args.output, start=start, seed=args.seed, trees=trees)
    summary = explorer.run()
    print(json.dumps(summary, indent=2, sort_keys=True))
    if args.report:
        generate_report_after_run(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
