"""Reachability check: can the tool reach every cup lid in the box and every shelf slot?

    python scripts/check_reach.py --seeds 0
    python scripts/check_reach.py --seeds 0,1,2,3,4

For each target the base is teleported to a parking pose (box: try the four sides, nearest
first; shelf: in front of the slot), IK is solved with the tool pointing down, and the
resulting pose is checked for collisions with the scene. Purely kinematic, no physics steps.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np  # noqa: E402

from shelving.kinematics import ArmIK, level_reach_seed  # noqa: E402
from shelving.actions.go_to_box import box_side_poses  # noqa: E402
from shelving.actions.go_to_shelf import SHELF_GAPS, shelf_front_pose  # noqa: E402
from sim.world import STOW, World  # noqa: E402

CUP_HEIGHT = 0.11


def try_pose(world, ik, pose, target):
    world.set_base_pose(pose.x, pose.y, pose.yaw)
    world.set_joint_positions(STOW)
    if ik.contacts_at(world.d, STOW):
        return None, "base collides"
    res = ik.solve(world.d, target, seed=level_reach_seed(target))
    if not res.ok:
        return res, f"IK err {res.pos_err * 1000:.0f} mm / {np.degrees(res.axis_err):.0f} deg"
    hits = [c for c in ik.contacts_at(world.d, res.q) if "cup_" not in c[0] + c[1]]
    if hits:
        return res, f"arm hits {hits[0][1] if hits[0][0].startswith('robot/') else hits[0][0]}"
    return res, "ok"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="0")
    ap.add_argument("--hover", type=float, default=0.03, help="tool tip height above the lid / placed lid (m)")
    args = ap.parse_args()

    world = World.create()
    ik = ArmIK(world.m)
    total, reached = 0, 0
    for seed in [int(s) for s in args.seeds.split(",")]:
        info = world.reset(seed)
        box, shelf = info["box"], info["shelf"]
        print(f"\n== seed {seed}")
        snapshot = world.d.qpos.copy()
        for cup in world.cup_names():
            world.d.qpos[:] = snapshot
            target = world.cup_top(cup) + [0, 0, args.hover]
            status = "no side"
            for pose in box_side_poses(box["pos"], box["yaw"], target[:2]):
                res, status = try_pose(world, ik, pose, target)
                if status == "ok":
                    break
            total += 1
            reached += status == "ok"
            lift = f"lift {res.q['lift_z']:.2f}" if res else ""
            print(f"  {cup:24s} side {pose.side:2s}  {status:28s} {lift}")
        for k in range(1, 7):
            world.d.qpos[:] = snapshot
            slot = world.d.site(f"slot_{k}").xpos.copy()
            target = slot + [0, 0, CUP_HEIGHT + args.hover]
            for gap in SHELF_GAPS:
                pose = shelf_front_pose(shelf["pos"], shelf["yaw"], slot[:2], gap)
                res, status = try_pose(world, ik, pose, target)
                if status == "ok":
                    break
            total += 1
            reached += status == "ok"
            lift = f"lift {res.q['lift_z']:.2f}" if res else ""
            print(f"  slot_{k:<19d} {pose.side:9s} {status:28s} {lift}")
    print(f"\nreachable {reached}/{total}")


if __name__ == "__main__":
    main()
