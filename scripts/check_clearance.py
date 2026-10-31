"""Run a tree and report every time the robot touches or visually overlaps the box or shelf.

    python scripts/check_clearance.py trees/b4_all_cups.json --seeds 0,1,2,3,4
    python scripts/check_clearance.py trees/b5_recovery.json --seeds 0,1 --inject seal:0.3,drop-carry:2

Two checks, per running node:
  physical  contacts between the robot's collision geoms and the furniture (the physics)
  visual    the youBot meshes (visual only) penetrating the furniture, measured with
            mj_geomDistance every 0.1 s. Collision shapes smaller than the meshes let the
            robot look as if it went through the box stand while the physics saw no contact.
Cups are ignored (touching them is the job). An empty report means no contact at all.
"""

import argparse
import contextlib
import io
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import shelving.actions  # noqa: E402,F401
from shelving.faults import FaultInjector  # noqa: E402
from behavioros.core.builder import TreeBuilder  # noqa: E402
from sim.kinematics import ArmIK  # noqa: E402
from sim.nav import GridPlanner  # noqa: E402
from sim.world import PREFIX, World  # noqa: E402


def check(tree_json, seed, inject=""):
    world = World.create()
    info = world.reset(seed)
    m, d = world.m, world.d
    bb = {"world": world, "ik": ArmIK(m), "planner": GridPlanner(info)}
    tree = TreeBuilder(tree_json, bb, devices={}, clock=lambda: d.time).build()
    faults = FaultInjector(inject, world, seed)

    def body(g):
        return m.body(m.geom_bodyid[g]).name

    meshes = [g for g in range(m.ngeom) if body(g).startswith(PREFIX)
              and m.geom_contype[g] == 0 and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
    furniture = [g for g in range(m.ngeom) if body(g) in ("box", "shelf") and m.geom_contype[g]]

    def running_node():
        return next((n for n in tree.py_tree.root.iterate()
                     if n.status.name == "RUNNING" and not n.children), None)

    def running_leaf():
        n = running_node()
        return n.name if n else "-"

    physical, visual, fromto, k = {}, {}, np.zeros(6), 0
    with contextlib.redirect_stdout(io.StringIO()):
        while tree.status not in ("SUCCESS", "FAILURE") and d.time < 600:
            if k % 10 == 0:
                tree.tick()
                faults.on_tick(running_node(), bb)
                for c in d.contact[: d.ncon]:
                    n1, n2 = m.geom(c.geom1).name, m.geom(c.geom2).name
                    if n1.startswith(PREFIX) != n2.startswith(PREFIX) and "cup_" not in n1 + n2:
                        robot, other = (n1, n2) if n1.startswith(PREFIX) else (n2, n1)
                        key = (running_leaf(), robot, other)
                        physical[key] = min(physical.get(key, 0.0), c.dist)
            if k % 50 == 0:
                for gv in meshes:
                    for gf in furniture:
                        dist = mujoco.mj_geomDistance(m, d, gv, gf, 0.05, fromto)
                        if dist < 0:
                            key = (running_leaf(), body(gv), m.geom(gf).name)
                            visual[key] = min(visual.get(key, 0.0), dist)
            mujoco.mj_step(m, d)
            k += 1
    print(f"seed {seed}: tree {tree.status}, "
          f"{len(physical)} physical contact(s), {len(visual)} visual overlap(s)")
    for label, found in (("physical", physical), ("visual", visual)):
        for (node, a, b), dist in sorted(found.items(), key=lambda kv: kv[1]):
            print(f"   {label:8s} {node:18s} {a:18s} x {b:22s} {1000 * dist:6.1f} mm")
    return not physical and not visual


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tree", type=Path)
    ap.add_argument("--seeds", default="0")
    ap.add_argument("--inject", default="", help="faults, see shelving/faults.py")
    args = ap.parse_args()
    tree_json = json.loads(args.tree.read_text())
    clean = [check(tree_json, int(s), args.inject) for s in args.seeds.split(",")]
    print(f"\nclear on {sum(clean)}/{len(clean)} seeds")


if __name__ == "__main__":
    main()
