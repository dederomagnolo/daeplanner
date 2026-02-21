#!/usr/bin/env python3

"""Generate a Gazebo .world from world-jean OBJ assets.

The script:
1) Parses `3d_scene_prepared_10x10.obj`.
2) Extracts:
   - one tree mesh (Wallnut_.002)
   - ground mesh (Ground)
3) Computes rigid 2D transforms (x, y, yaw) for all tree instances.
4) Writes Gazebo models + world + launch files under drone_gazebo.
"""

import os
import sys

# In some WSL sessions, cwd can be stale or inaccessible. Resolve script directory
# without assuming os.getcwd() always works.
def _safe_script_dir() -> str:
    file_token = globals().get("__file__", "") or (sys.argv[0] if sys.argv else "")
    candidates: list[str] = []

    if file_token:
        if os.path.isabs(file_token):
            candidates.append(file_token)
        else:
            pwd_env = os.environ.get("PWD")
            if pwd_env:
                candidates.append(os.path.join(pwd_env, file_token))
            try:
                candidates.append(os.path.join(os.getcwd(), file_token))
            except FileNotFoundError:
                pass

    for cand in candidates:
        try:
            full = os.path.realpath(cand)
        except FileNotFoundError:
            continue
        if os.path.exists(full):
            return os.path.dirname(full)

    if candidates:
        return os.path.dirname(candidates[0])
    return "."


_SCRIPT_DIR = _safe_script_dir()
try:
    os.getcwd()
except FileNotFoundError:
    os.chdir(_SCRIPT_DIR)

if sys.path and sys.path[0] and not os.path.isabs(sys.path[0]):
    sys.path[0] = _SCRIPT_DIR

import argparse
import math
import re
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

SCRIPT_DIR = Path(_SCRIPT_DIR).resolve()
REPO_ROOT = SCRIPT_DIR.parents[1] if len(SCRIPT_DIR.parents) >= 2 else SCRIPT_DIR


@dataclass
class FaceRef:
    v: int
    vt: Optional[int] = None
    vn: Optional[int] = None


@dataclass
class Face:
    refs: List[FaceRef]
    material: Optional[str]
    smooth: Optional[str]


@dataclass
class GroupData:
    vertex_positions: List[Tuple[float, float, float]] = field(default_factory=list)
    faces: List[Face] = field(default_factory=list)


@dataclass
class ObjData:
    vertices: List[Tuple[float, float, float]] = field(default_factory=list)
    texcoords: List[Tuple[float, float]] = field(default_factory=list)
    normals: List[Tuple[float, float, float]] = field(default_factory=list)
    groups: Dict[str, GroupData] = field(default_factory=dict)


def parse_index(raw: str, total: int) -> int:
    value = int(raw)
    if value > 0:
        return value
    return total + value + 1


def parse_face_ref(token: str, obj: ObjData) -> FaceRef:
    # OBJ reference formats:
    # v
    # v/vt
    # v//vn
    # v/vt/vn
    if "//" in token:
        v_str, vn_str = token.split("//")
        return FaceRef(
            v=parse_index(v_str, len(obj.vertices)),
            vt=None,
            vn=parse_index(vn_str, len(obj.normals)),
        )

    parts = token.split("/")
    if len(parts) == 1:
        return FaceRef(v=parse_index(parts[0], len(obj.vertices)))

    if len(parts) == 2:
        vt = parse_index(parts[1], len(obj.texcoords)) if parts[1] else None
        return FaceRef(v=parse_index(parts[0], len(obj.vertices)), vt=vt)

    if len(parts) == 3:
        vt = parse_index(parts[1], len(obj.texcoords)) if parts[1] else None
        vn = parse_index(parts[2], len(obj.normals)) if parts[2] else None
        return FaceRef(v=parse_index(parts[0], len(obj.vertices)), vt=vt, vn=vn)

    raise ValueError(f"Unsupported face token: {token}")


def parse_obj(path: Path) -> ObjData:
    obj = ObjData()
    current_group: Optional[str] = None
    current_material: Optional[str] = None
    current_smooth: Optional[str] = None

    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue

            if stripped.startswith("g "):
                current_group = stripped.split(maxsplit=1)[1]
                obj.groups.setdefault(current_group, GroupData())
                continue

            if stripped.startswith("usemtl "):
                current_material = stripped.split(maxsplit=1)[1]
                continue

            if stripped.startswith("s "):
                current_smooth = stripped.split(maxsplit=1)[1]
                continue

            if stripped.startswith("v "):
                _, xs, ys, zs = stripped.split()
                vertex = (float(xs), float(ys), float(zs))
                obj.vertices.append(vertex)
                if current_group is not None:
                    obj.groups[current_group].vertex_positions.append(vertex)
                continue

            if stripped.startswith("vt "):
                parts = stripped.split()
                obj.texcoords.append((float(parts[1]), float(parts[2])))
                continue

            if stripped.startswith("vn "):
                _, xs, ys, zs = stripped.split()
                obj.normals.append((float(xs), float(ys), float(zs)))
                continue

            if stripped.startswith("f "):
                if current_group is None:
                    continue
                tokens = stripped.split()[1:]
                refs = [parse_face_ref(token, obj) for token in tokens]
                obj.groups[current_group].faces.append(
                    Face(refs=refs, material=current_material, smooth=current_smooth)
                )
                continue

    return obj


def write_submesh_obj(
    obj: ObjData,
    face_group: str,
    output_obj: Path,
    output_mtl_name: str,
    output_group_name: str,
) -> None:
    group = obj.groups.get(face_group)
    if group is None or not group.faces:
        raise RuntimeError(f"Face group '{face_group}' not found or empty.")

    v_order: List[int] = []
    vt_order: List[int] = []
    vn_order: List[int] = []
    v_seen: Dict[int, int] = {}
    vt_seen: Dict[int, int] = {}
    vn_seen: Dict[int, int] = {}

    for face in group.faces:
        for ref in face.refs:
            if ref.v not in v_seen:
                v_seen[ref.v] = len(v_order) + 1
                v_order.append(ref.v)
            if ref.vt is not None and ref.vt not in vt_seen:
                vt_seen[ref.vt] = len(vt_order) + 1
                vt_order.append(ref.vt)
            if ref.vn is not None and ref.vn not in vn_seen:
                vn_seen[ref.vn] = len(vn_order) + 1
                vn_order.append(ref.vn)

    output_obj.parent.mkdir(parents=True, exist_ok=True)
    with output_obj.open("w", encoding="utf-8") as f:
        f.write("# Generated by generate_world_jean.py\n")
        f.write(f"mtllib {output_mtl_name}\n")
        f.write(f"g {output_group_name}\n")

        for old_idx in v_order:
            x, y, z = obj.vertices[old_idx - 1]
            f.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for old_idx in vt_order:
            u, v = obj.texcoords[old_idx - 1]
            f.write(f"vt {u:.6f} {v:.6f}\n")
        for old_idx in vn_order:
            nx, ny, nz = obj.normals[old_idx - 1]
            f.write(f"vn {nx:.6f} {ny:.6f} {nz:.6f}\n")

        last_material = None
        last_smooth = None
        for face in group.faces:
            if face.material and face.material != last_material:
                f.write(f"usemtl {face.material}\n")
                last_material = face.material
            if face.smooth != last_smooth:
                f.write(f"s {face.smooth if face.smooth is not None else 'off'}\n")
                last_smooth = face.smooth

            face_tokens = []
            for ref in face.refs:
                new_v = v_seen[ref.v]
                new_vt = vt_seen[ref.vt] if ref.vt is not None else None
                new_vn = vn_seen[ref.vn] if ref.vn is not None else None

                if new_vt is None and new_vn is None:
                    token = f"{new_v}"
                elif new_vt is not None and new_vn is None:
                    token = f"{new_v}/{new_vt}"
                elif new_vt is None and new_vn is not None:
                    token = f"{new_v}//{new_vn}"
                else:
                    token = f"{new_v}/{new_vt}/{new_vn}"
                face_tokens.append(token)
            f.write("f " + " ".join(face_tokens) + "\n")


def compute_tree_poses(
    groups: Dict[str, GroupData], reference_mesh_group: str
) -> List[Tuple[str, float, float, float, float]]:
    ref = groups[reference_mesh_group].vertex_positions
    if not ref:
        raise RuntimeError(f"Reference mesh group '{reference_mesh_group}' has no vertices.")

    n = len(ref)
    rcx = sum(p[0] for p in ref) / n
    rcy = sum(p[1] for p in ref) / n
    rcz = sum(p[2] for p in ref) / n

    tree_mesh_re = re.compile(r"^Wallnut_\.(\d{3})_Mesh$")
    tree_groups = [name for name in groups if tree_mesh_re.match(name)]
    tree_groups.sort(key=lambda name: int(tree_mesh_re.match(name).group(1)))

    poses: List[Tuple[str, float, float, float, float]] = []
    worst_rms = 0.0

    for name in tree_groups:
        pts = groups[name].vertex_positions
        if len(pts) != n:
            raise RuntimeError(
                f"Mesh '{name}' has {len(pts)} vertices, expected {n}."
            )

        cx = sum(p[0] for p in pts) / n
        cy = sum(p[1] for p in pts) / n
        cz = sum(p[2] for p in pts) / n

        num = 0.0
        den = 0.0
        for (x, y, _), (u, v, _) in zip(ref, pts):
            x0 = x - rcx
            y0 = y - rcy
            u0 = u - cx
            v0 = v - cy
            num += x0 * v0 - y0 * u0
            den += x0 * u0 + y0 * v0

        yaw = math.atan2(num, den)
        c = math.cos(yaw)
        s = math.sin(yaw)
        tx = cx - (c * rcx - s * rcy)
        ty = cy - (s * rcx + c * rcy)
        tz = cz - rcz

        err2 = 0.0
        for (x, y, z), (u, v, w) in zip(ref, pts):
            xp = c * x - s * y + tx
            yp = s * x + c * y + ty
            zp = z + tz
            dx = xp - u
            dy = yp - v
            dz = zp - w
            err2 += dx * dx + dy * dy + dz * dz
        rms = math.sqrt(err2 / n)
        worst_rms = max(worst_rms, rms)

        poses.append((name, tx, ty, tz, yaw))

    print(f"[info] Tree groups found: {len(poses)}")
    print(f"[info] Worst rigid-fit RMS: {worst_rms:.6e}")
    return poses


def write_model_files(model_dir: Path, model_name: str, mesh_name: str) -> None:
    model_dir.mkdir(parents=True, exist_ok=True)

    model_config = f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>auto-generated</name>
    <email>n/a</email>
  </author>
  <description>Generated from world-jean assets.</description>
</model>
"""
    (model_dir / "model.config").write_text(model_config, encoding="utf-8")

    model_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/{mesh_name}</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/{mesh_name}</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""
    (model_dir / "model.sdf").write_text(model_sdf, encoding="utf-8")


def write_world_file(world_path: Path, tree_poses: List[Tuple[str, float, float, float, float]]) -> None:
    world_path.parent.mkdir(parents=True, exist_ok=True)

    lines = [
        "<?xml version='1.0' ?>",
        "<sdf version='1.6'>",
        "  <world name='world_jean'>",
        "    <include>",
        "      <uri>model://sun</uri>",
        "    </include>",
        "",
        "    <include>",
        "      <uri>model://world_jean_ground</uri>",
        "      <name>world_jean_ground</name>",
        "      <pose>0 0 0 0 0 0</pose>",
        "    </include>",
        "",
    ]

    tree_mesh_re = re.compile(r"^Wallnut_\.(\d{3})_Mesh$")
    for source_name, tx, ty, tz, yaw in tree_poses:
        match = tree_mesh_re.match(source_name)
        tree_id = match.group(1) if match else source_name
        lines.extend(
            [
                "    <include>",
                "      <uri>model://world_jean_tree</uri>",
                f"      <name>world_jean_tree_{tree_id}</name>",
                f"      <pose>{tx:.6f} {ty:.6f} {tz:.6f} 0 0 {yaw:.6f}</pose>",
                "    </include>",
                "",
            ]
        )

    lines.extend(
        [
            "    <gui fullscreen='0'>",
            "      <camera name='user_camera'>",
            "        <pose>0 -18 12 0 0.55 1.57</pose>",
            "      </camera>",
            "    </gui>",
            "  </world>",
            "</sdf>",
            "",
        ]
    )
    world_path.write_text("\n".join(lines), encoding="utf-8")


def write_launch_file(launch_path: Path, world_rel_path: str) -> None:
    launch_path.parent.mkdir(parents=True, exist_ok=True)
    launch_content = f"""<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find drone_gazebo)/{world_rel_path}"/>
  </include>

  <arg name="avoidance" default="false" />
</launch>
"""
    launch_path.write_text(launch_content, encoding="utf-8")


def main() -> None:
    default_source_obj = SCRIPT_DIR / "3d_scene_prepared_10x10.obj"
    default_source_mtl = SCRIPT_DIR / "3d_scene_prepared_10x10.mtl"
    default_tree_texture = REPO_ROOT / "Biomass/scripts/pcd_files/tree_3_texture.png"
    default_grass_texture = REPO_ROOT / "Biomass/scripts/pcd_files/seamless_grass_texture.jpg"
    default_models_root = REPO_ROOT / "daep/catkin_ws/src/drone_gazebo/models"
    default_world_path = REPO_ROOT / "daep/catkin_ws/src/drone_gazebo/worlds/world_jean.world"
    default_launch_path = REPO_ROOT / "daep/catkin_ws/src/drone_gazebo/launch/world_jean_static.launch"

    parser = argparse.ArgumentParser(description="Generate world_jean Gazebo assets.")
    parser.add_argument(
        "--source-obj",
        type=Path,
        default=default_source_obj,
    )
    parser.add_argument(
        "--source-mtl",
        type=Path,
        default=default_source_mtl,
    )
    parser.add_argument(
        "--tree-texture",
        type=Path,
        default=default_tree_texture,
    )
    parser.add_argument(
        "--grass-texture",
        type=Path,
        default=default_grass_texture,
    )
    parser.add_argument(
        "--models-root",
        type=Path,
        default=default_models_root,
    )
    parser.add_argument(
        "--world-path",
        type=Path,
        default=default_world_path,
    )
    parser.add_argument(
        "--launch-path",
        type=Path,
        default=default_launch_path,
    )
    args = parser.parse_args()

    source_obj = args.source_obj
    source_mtl = args.source_mtl
    if not source_obj.is_file():
        raise SystemExit(f"Missing source OBJ: {source_obj}")
    if not source_mtl.is_file():
        raise SystemExit(f"Missing source MTL: {source_mtl}")
    if not args.tree_texture.is_file():
        raise SystemExit(f"Missing tree texture: {args.tree_texture}")
    if not args.grass_texture.is_file():
        raise SystemExit(f"Missing grass texture: {args.grass_texture}")

    print(f"[info] Parsing OBJ: {source_obj}")
    obj = parse_obj(source_obj)

    tree_reference_mesh = "Wallnut_.002_Mesh"
    tree_reference_faces = "Wallnut_.002_Walnut_L"
    ground_faces = "Ground_grass"

    if tree_reference_mesh not in obj.groups:
        raise SystemExit(f"Group not found: {tree_reference_mesh}")
    if tree_reference_faces not in obj.groups:
        raise SystemExit(f"Group not found: {tree_reference_faces}")
    if ground_faces not in obj.groups:
        raise SystemExit(f"Group not found: {ground_faces}")

    tree_poses = compute_tree_poses(obj.groups, tree_reference_mesh)

    tree_model_name = "world_jean_tree"
    ground_model_name = "world_jean_ground"

    tree_model_dir = args.models_root / tree_model_name
    ground_model_dir = args.models_root / ground_model_name
    tree_mesh_dir = tree_model_dir / "meshes"
    ground_mesh_dir = ground_model_dir / "meshes"

    tree_mesh_dir.mkdir(parents=True, exist_ok=True)
    ground_mesh_dir.mkdir(parents=True, exist_ok=True)

    tree_obj_name = "world_jean_tree.obj"
    ground_obj_name = "world_jean_ground.obj"
    mtl_name = "world_jean_assets.mtl"

    print("[info] Writing extracted OBJ meshes")
    write_submesh_obj(
        obj=obj,
        face_group=tree_reference_faces,
        output_obj=tree_mesh_dir / tree_obj_name,
        output_mtl_name=mtl_name,
        output_group_name="world_jean_tree",
    )
    write_submesh_obj(
        obj=obj,
        face_group=ground_faces,
        output_obj=ground_mesh_dir / ground_obj_name,
        output_mtl_name=mtl_name,
        output_group_name="world_jean_ground",
    )

    print("[info] Copying materials and textures")
    shutil.copy2(source_mtl, tree_mesh_dir / mtl_name)
    shutil.copy2(source_mtl, ground_mesh_dir / mtl_name)
    shutil.copy2(args.tree_texture, tree_mesh_dir / args.tree_texture.name)
    shutil.copy2(args.grass_texture, tree_mesh_dir / args.grass_texture.name)
    shutil.copy2(args.tree_texture, ground_mesh_dir / args.tree_texture.name)
    shutil.copy2(args.grass_texture, ground_mesh_dir / args.grass_texture.name)

    print("[info] Writing model.sdf/model.config")
    write_model_files(tree_model_dir, tree_model_name, tree_obj_name)
    write_model_files(ground_model_dir, ground_model_name, ground_obj_name)

    print(f"[info] Writing world: {args.world_path}")
    write_world_file(args.world_path, tree_poses)

    world_rel_path = str(args.world_path).replace("\\", "/")
    if "/src/drone_gazebo/" in world_rel_path:
        world_rel_path = world_rel_path.split("/src/drone_gazebo/", 1)[1]
    print(f"[info] Writing launch: {args.launch_path}")
    write_launch_file(args.launch_path, world_rel_path)

    print("[done] world_jean generation completed")
    print(f"[done] Trees exported: {len(tree_poses)}")


if __name__ == "__main__":
    main()
