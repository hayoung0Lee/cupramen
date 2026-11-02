"""Show a benchmark layout with the robot.

    mjpython scripts/view_episode.py --seed 0                 # robot at its start pose
    mjpython scripts/view_episode.py --seed 0 --reach-cup 0   # parked at the box, arm over cup 0
    mjpython scripts/view_episode.py --seed 0 --reach-slot 4  # parked at the shelf, arm over slot_4

With --reach-*, the base is teleported near the parking pose and the arm then moves to the IK
pose on its own (position actuators), so you can watch it. For a shelf slot the base starts
backed off, the arm takes its pose clear of the shelf, and only then the base drives in -
swinging the arm in front of the shelf would hit the boards. Otherwise move the robot with
the viewer's Control panel sliders.
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import mujoco.viewer  # noqa: E402

from shelving.kinematics import ArmIK, level_reach_seed  # noqa: E402
from shelving.go_to_box import box_side_poses  # noqa: E402
from shelving.go_to_shelf import shelf_front_pose  # noqa: E402
from sim.world import POSITION_ACTUATOR, World, r  # noqa: E402

HOVER = 0.03
CUP_HEIGHT = 0.11
SHELF_BACKOFF = 0.35   # m the base starts back from its shelf parking pose
DRIVE_IN_AFTER = 3.0   # s of sim time for the arm to settle before driving in
DRIVE_IN_SPEED = 0.10  # m/s


def park_and_reach(world: World, args):
    """Returns (heading, distance) the base still has to drive in, or None."""
    info, ik = world.info, ArmIK(world.m)
    if args.reach_cup is not None:
        cup = world.cup_names()[args.reach_cup]
        target = world.cup_top(cup) + [0, 0, HOVER]
        poses = box_side_poses(info["box"]["pos"], info["box"]["yaw"], target[:2])
        label = cup
    else:
        slot = world.d.site(f"slot_{args.reach_slot}").xpos.copy()
        target = slot + [0, 0, CUP_HEIGHT + HOVER]
        poses = [shelf_front_pose(info["shelf"]["pos"], info["shelf"]["yaw"], slot[:2])]
        label = f"slot_{args.reach_slot}"
    for pose in poses:
        world.set_base_pose(pose.x, pose.y, pose.yaw)
        mujoco.mj_forward(world.m, world.d)
        res = ik.solve(world.d, target, seed=level_reach_seed(target))
        if res.ok:
            break
    print(f"{label}: base side {pose.side}, IK ok={res.ok}, "
          + ", ".join(f"{j}={v:.2f}" for j, v in res.q.items()))
    for j, v in res.q.items():  # targets only: the arm drives there itself
        world.d.actuator(r(POSITION_ACTUATOR[j])).ctrl = v
    if args.reach_slot is None:
        return None
    heading = np.array([np.cos(pose.yaw), np.sin(pose.yaw)])
    back = np.array([pose.x, pose.y]) - SHELF_BACKOFF * heading
    world.set_base_pose(back[0], back[1], pose.yaw)
    return heading, SHELF_BACKOFF


def drive_step(world: World, drive_in) -> None:
    """Drive the base straight in by the given distance once the arm has settled."""
    if drive_in is None:
        return
    heading, dist = drive_in
    d = world.d
    if not hasattr(drive_step, "start"):
        drive_step.start = np.array(world.base_pose()[:2])
    moved = float(np.linalg.norm(np.array(world.base_pose()[:2]) - drive_step.start))
    v = DRIVE_IN_SPEED if d.time > DRIVE_IN_AFTER and moved < dist else 0.0
    d.actuator(r("base_vx")).ctrl = v * heading[0]
    d.actuator(r("base_vy")).ctrl = v * heading[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--reach-cup", type=int, choices=range(6), help="cup index 0-5")
    g.add_argument("--reach-slot", type=int, choices=range(1, 7), help="slot 1-6")
    args = ap.parse_args()

    world = World.create()
    info = world.reset(args.seed)
    mob = info["mobility"]
    print(f"seed {args.seed}: box {info['box']['pos'][:2]}, shelf {info['shelf']['pos'][:2]}")
    print(f"start->box approach {mob['start_to_box_approach_m']:.2f} m, "
          f"box->shelf approach {mob['box_to_shelf_approach_m']:.2f} m")
    drive_in = None
    if args.reach_cup is not None or args.reach_slot is not None:
        drive_in = park_and_reach(world, args)

    m, d = world.m, world.d
    with mujoco.viewer.launch_passive(m, d) as viewer:
        viewer.cam.lookat[:] = d.body(r("base_link")).xpos + [0, 0, 0.7]
        viewer.cam.distance = 3.0 if args.reach_cup is not None or args.reach_slot is not None else 7.0
        viewer.cam.elevation = -30
        while viewer.is_running():
            t0 = time.time()
            for _ in range(8):  # 16 ms of sim per frame ~ real time
                drive_step(world, drive_in)
                mujoco.mj_step(m, d)
            viewer.sync()
            time.sleep(max(0.0, 0.016 - (time.time() - t0)))


if __name__ == "__main__":
    main()
