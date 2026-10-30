"""Convert youBot meshes from ROS youbot_description (GPLv3) into OBJ files for sim/robot.

    git clone https://github.com/youbot/youbot_description /tmp/youbot_description
    python tools/convert_youbot_meshes.py /tmp/youbot_description

Each Collada file is split by material into one OBJ per colour (MuJoCo ignores .mtl colours,
so colours are set per geom in youbot_lift.xml). Vertices are moved into the URDF *link*
frame (the URDF <visual><origin> is applied), so a mesh goes into the matching MuJoCo body
with pos 0. arm2/arm3 are stretched in the middle only (joint hubs keep their shape) to the
lengthened links of youbot_lift.xml.
"""

import argparse
from pathlib import Path

import numpy as np
import trimesh

OUT = Path(__file__).resolve().parent.parent / "sim" / "robot" / "meshes" / "youbot"

# name: (source file, URDF visual origin in the link frame, stretch (old_len, new_len) or None)
PARTS = {
    "base": ("youbot_base/base.dae", (0.0, 0.0, 0.0), None),
    "arm0": ("youbot_arm/arm0.dae", (0.0, 0.0, 0.0), None),
    "arm1": ("youbot_arm/arm1.dae", (0.0, 0.0, 0.0), None),
    "arm2": ("youbot_arm/arm2.dae", (0.0, -0.032, 0.078), (0.155, 0.23)),
    "arm3": ("youbot_arm/arm3.dae", (0.0, 0.028, 0.079), (0.135, 0.20)),
    "arm4": ("youbot_arm/arm4.dae", (0.0, -0.010, 0.029), None),
    "arm5": ("youbot_arm/arm5.dae", (0.003, 0.0, -0.034), None),
}
HUB = 0.045      # m kept rigid at each end of a stretched link
TARGET_FACES = 4000  # per OBJ, to keep the repo small


def stretch(z, old, new):
    """Piecewise map: [.., HUB] fixed, [old-HUB, ..] shifted by new-old, linear in between."""
    lo, hi = HUB, old - HUB
    out = z.copy()
    mid = (z > lo) & (z < hi)
    out[mid] = lo + (z[mid] - lo) * (hi - lo + new - old) / (hi - lo)
    out[z >= hi] = z[z >= hi] + (new - old)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("youbot_description", type=Path)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    for name, (src, origin, stretch_to) in PARTS.items():
        scene = trimesh.load(args.youbot_description / "meshes" / src, force="scene")
        by_colour: dict[tuple, list] = {}
        for node in scene.graph.nodes_geometry:
            T, gname = scene.graph[node]
            g = scene.geometry[gname].copy()
            g.apply_transform(T)
            mat = g.visual.material
            rgba = getattr(mat, "diffuse", None)
            if rgba is None:
                rgba = mat.baseColorFactor
            colour = tuple(int(c) for c in rgba[:3])
            by_colour.setdefault(colour, []).append(g)
        for k, (colour, parts) in enumerate(sorted(by_colour.items(), key=lambda kv: -sum(len(p.faces) for p in kv[1]))):
            mesh = trimesh.util.concatenate(parts)
            mesh = trimesh.Trimesh(mesh.vertices, mesh.faces, process=True)
            if len(mesh.faces) > TARGET_FACES:
                mesh = mesh.simplify_quadric_decimation(face_count=TARGET_FACES)
            v = mesh.vertices + np.asarray(origin)
            if stretch_to:
                v[:, 2] = stretch(v[:, 2], *stretch_to)
            mesh = trimesh.Trimesh(v, mesh.faces, process=False)
            path = OUT / f"{name}_{k}.obj"
            mesh.export(path, include_normals=False)
            rgb = " ".join(f"{c / 255:.2f}" for c in colour)
            print(f"{path.name:12s} faces {len(mesh.faces):5d}  rgba {rgb} 1")


if __name__ == "__main__":
    main()
