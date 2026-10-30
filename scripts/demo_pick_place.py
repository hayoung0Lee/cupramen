"""Connected demo: start -> box -> pick one cup with suction -> shelf -> place it on a slot.

    mjpython scripts/demo_pick_place.py --seed 0                    # watch in the viewer
    python   scripts/demo_pick_place.py --seed 0 --headless         # run fast, print only
    python   scripts/demo_pick_place.py --seed 0 --cup 2 --slot 5 --headless

Scripted sequence of incremental motion primitives (sim/motion.py); no Behavior Tree yet and
no failure handling. IK targets are planned once at the start from the known layout.
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from sim.kinematics import ArmIK, level_reach_seed  # noqa: E402
from sim.motion import BaseMove, JointMove, Lazy, Suction, Wait  # noqa: E402
from sim.nav import GridPlanner  # noqa: E402
from sim.placement import box_side_poses, shelf_front_pose  # noqa: E402
from sim.world import STOW, World, r, randomize  # noqa: E402

CONTROL_EVERY = 10   # physics steps per control tick (20 ms)
HOVER = 0.03         # tool above lid / cup bottom above shelf board while moving in
PRESS = 0.003        # tool target this far into the lid so the pad really touches
LIFT = 0.10          # lift the cup this much out of the box
BACKOFF = 0.35       # base distance back from a parking pose before driving in / after leaving
CARRY_TIP = (0.35, 0.0, 1.05)  # tool tip in the base frame while driving with a cup: tucked in


def backed_off(pose, dist=BACKOFF):
    return (pose.x - dist * np.cos(pose.yaw), pose.y - dist * np.sin(pose.yaw), pose.yaw)


def plan(world: World, cup_idx: int, slot: int):
    """Choose parking poses and solve every IK pose the demo needs. Returns (phases, meta)."""
    info, ik, d = world.info, ArmIK(world.m), world.d
    cup = world.cup_names()[cup_idx]
    top = world.cup_top(cup)
    cup_h = float(world.m.site(f"{cup}_top").pos[2])

    def solve(base, target):
        res = ik.solve(d, target, seed=level_reach_seed(target), base=base)
        return res if res.ok else None

    planner = GridPlanner(info)
    start = world.base_pose()

    # Box side: first side (nearest first) that the base can drive to and where the hover,
    # press and lift poses all solve.
    for park in box_side_poses(info["box"]["pos"], info["box"]["yaw"], top[:2]):
        if planner.plan(start, backed_off(park)) is None:
            continue
        b = (park.x, park.y, park.yaw)
        q_hover, q_press, q_lift = (solve(b, top + [0, 0, dz]) for dz in (HOVER, -PRESS, LIFT))
        if q_hover and q_press and q_lift:
            break
    else:
        raise RuntimeError(f"{cup}: no box side reachable")

    def clear(base, q):
        """No robot-vs-furniture contact at this base pose and joint pose (cups ignored)."""
        return not [c for c in ik.contacts_at(d, q, base) if "cup_" not in c[0] + c[1]]

    # Shelf: park in front of the slot. If the spot to back off to is blocked (e.g. the box
    # stands close), back off less, then try shifting toward the shelf centre and swinging
    # arm1 sideways. Every candidate must be reachable and collision-free both where the arm
    # unfolds (backed off) and over the slot.
    slot_xyz = d.site(f"slot_{slot}").xpos.copy()
    sh_pos, sh_yaw = info["shelf"]["pos"], info["shelf"]["yaw"]
    along = np.array([np.cos(sh_yaw), np.sin(sh_yaw)])          # shelf +x (its width)
    lateral = float((np.asarray(sh_pos[:2]) - slot_xyz[:2]) @ along)
    found = False
    for shift in (0.0, 0.125, 0.25):
        align_xy = slot_xyz[:2] + along * np.clip(lateral, -shift, shift)
        shelf_park = shelf_front_pose(sh_pos, sh_yaw, align_xy)
        s = (shelf_park.x, shelf_park.y, shelf_park.yaw)
        q_slot_hover = solve(s, slot_xyz + [0, 0, cup_h + HOVER])
        q_slot_place = solve(s, slot_xyz + [0, 0, cup_h + 0.003])
        if not (q_slot_hover and q_slot_place and clear(s, q_slot_hover.q)):
            continue
        for backoff in (BACKOFF, 0.25, 0.15):
            shelf_out = backed_off(shelf_park, backoff)
            if clear(shelf_out, q_slot_hover.q) and planner.plan(backed_off(park), shelf_out):
                found = True
                break
        if found:
            break
    if not found:
        raise RuntimeError(f"slot_{slot}: no reachable parking pose")

    # Carry pose: same joint values work at any base pose, so solve it for the base at the origin.
    q_carry = solve((0.0, 0.0, 0.0), np.array(CARRY_TIP))
    if not q_carry:
        raise RuntimeError("carry pose IK failed")

    box_out = backed_off(park)

    def route(a, b):
        pts = planner.plan(a, b)
        if pts is None:
            raise RuntimeError(f"no path from {np.round(a[:2], 2)} to {np.round(b[:2], 2)}")
        return [(x, y, b[2]) for x, y in pts[1:]]

    def aligned(dz):
        """JointMove that puts the *cup bottom* (not the tool) dz above the slot.

        The cup can sit off-centre on the pad or shift during transport, so measure the
        tool-to-cup-bottom offset now and correct the tool target by it.
        """
        def factory(w):
            offset = w.tool_tip() - w.d.site(f"{cup}_bottom").xpos
            target = slot_xyz + offset + [0, 0, dz]
            res = ik.solve(w.d, target, seed=w.joint_positions())
            print(f"    cup offset on pad {1000 * np.linalg.norm(offset[:2]):.0f} mm -> corrected, "
                  f"IK err {1000 * res.pos_err:.1f} mm")
            return JointMove(res.q, 1.5)
        return Lazy(factory)

    slow = dict(vmax=0.12)
    phases = [
        ("drive to box", BaseMove(route(start, box_out))),
        ("drive in to box", BaseMove([(park.x, park.y, park.yaw)], **slow)),
        ("arm over cup", JointMove(q_hover.q, 2.5)),
        ("descend onto lid", JointMove(q_press.q, 1.5)),
        ("suction on", Suction(True)),
        ("let suction grip", Wait(0.3)),
        ("lift cup", JointMove(q_lift.q, 1.5)),
        ("back away from box", BaseMove([box_out], **slow)),
        ("tuck arm with cup", JointMove(q_carry.q, 2.0)),
        ("drive to shelf", BaseMove(route(box_out, shelf_out))),
        ("arm to shelf-entry pose", JointMove(q_slot_hover.q, 2.5)),
        ("drive in to shelf", BaseMove([s], **slow)),
        ("align cup over slot", aligned(HOVER)),
        ("lower cup onto board", aligned(0.003)),
        ("suction off", Suction(False)),
        ("let cup settle", Wait(0.8)),
        ("raise tool", JointMove(q_slot_hover.q, 1.0)),
        ("back away from shelf", BaseMove([shelf_out], **slow)),
        ("stow arm", JointMove(STOW, 2.5)),
    ]
    meta = dict(cup=cup, slot=slot, box_side=park.side)
    return phases, meta


class Runner:
    def __init__(self, world, phases, log):
        self.world, self.phases, self.log, self.i = world, phases, log, 0

    @property
    def done(self):
        return self.i >= len(self.phases)

    def tick(self):
        if self.done:
            return
        name, prim = self.phases[self.i]
        if prim.step(self.world):
            print(f"  t={self.world.d.time:6.2f}s  done: {name}")
            self.i += 1


def report(world, meta, log):
    cup_z = world.d.body(meta["cup"]).xpos
    placed = randomize.count_placed(world.m, world.d)
    s = log.summary()
    print(f"\n{meta['cup']} -> slot_{meta['slot']} (box side {meta['box_side']})")
    print(f"  placed on shelf: {placed['success_count']} / 6, cup at {np.round(cup_z, 3)}")
    print(f"  sim time {world.d.time:.1f} s, base path {s['path_length_m']:.2f} m")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--cup", type=int, default=0, choices=range(6))
    ap.add_argument("--slot", type=int, default=2, choices=range(1, 7))
    ap.add_argument("--headless", action="store_true")
    args = ap.parse_args()

    world = World.create()
    info = world.reset(args.seed)
    phases, meta = plan(world, args.cup, args.slot)
    print(f"seed {args.seed}: {meta['cup']} from box side {meta['box_side']} -> slot_{args.slot}")
    log = randomize.MobilityLog(world.m, r("base_link"), info)
    runner = Runner(world, phases, log)
    m, d = world.m, world.d

    def step_once(k):
        if k % CONTROL_EVERY == 0:
            runner.tick()
        mujoco.mj_step(m, d)
        log.update(d)

    k = 0
    if args.headless:
        while not runner.done and d.time < 180:
            step_once(k)
            k += 1
        for _ in range(500):  # let things settle before judging
            mujoco.mj_step(m, d)
        report(world, meta, log)
        return

    from mujoco import viewer as mj_viewer
    with mj_viewer.launch_passive(m, d) as viewer:
        viewer.cam.lookat[:] = [*np.mean([info["box"]["pos"][:2], info["shelf"]["pos"][:2]], axis=0), 0.6]
        viewer.cam.distance = 5.0
        viewer.cam.elevation = -35
        reported = False
        while viewer.is_running():
            t0 = time.time()
            for _ in range(8):  # ~real time
                step_once(k)
                k += 1
            if runner.done and not reported:
                report(world, meta, log)
                reported = True
            viewer.sync()
            time.sleep(max(0.0, 0.016 - (time.time() - t0)))


if __name__ == "__main__":
    main()
