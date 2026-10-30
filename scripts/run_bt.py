"""Run a Behavior Tree (JSON) on the MuJoCo cup-shelving scene with BehaviorOS.

    mjpython scripts/run_bt.py trees/b2_go_to_box.json --seed 0          # watch in the viewer
    python   scripts/run_bt.py trees/b2_go_to_box.json --seed 0 --headless

Loop (same role as the Webots controller's execution_loop, in one process):
  every 2 ms   mj_step          physics
  every 20 ms  tree.tick()      BehaviorOS: runtime resolution hook, then py_trees tick
The tree is printed whenever any node changes status, so you can follow where it is.
"""

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402
import py_trees  # noqa: E402

import shelving.actions  # noqa: E402,F401  (registers the actions)
from behavioros.core.builder import TreeBuilder  # noqa: E402
from sim.kinematics import ArmIK  # noqa: E402
from sim.nav import GridPlanner  # noqa: E402
from sim.world import World, r, randomize  # noqa: E402

CONTROL_EVERY = 10      # physics steps per tick: 10 x 2 ms = 20 ms
TIME_LIMIT_S = 600.0    # simulation seconds before giving up


def tree_snapshot(root):
    return tuple((n.name, n.status.name) for n in root.iterate())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tree", type=Path, help="JSON tree file")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--headless", action="store_true")
    args = ap.parse_args()

    world = World.create()
    info = world.reset(args.seed)
    m, d = world.m, world.d
    blackboard = {"world": world, "ik": ArmIK(m), "planner": GridPlanner(info)}
    tree = (
        TreeBuilder(json.loads(args.tree.read_text()), blackboard, devices={"world": world},
                    clock=lambda: d.time)
        .with_runtime()
        .build()
    )
    log = randomize.MobilityLog(m, r("base_link"), info)
    print(f"seed {args.seed}: tree {args.tree.name}")

    state = {"k": 0, "last": None, "result": None}

    def step():
        """One physics step; tick the tree every CONTROL_EVERY steps."""
        if state["result"] is None and state["k"] % CONTROL_EVERY == 0:
            res = tree.tick()
            snap = tree_snapshot(tree.py_tree.root)
            if snap != state["last"]:
                print(f"\n--- t={d.time:6.2f}s  tick {res.tick_count}: {res.status}")
                print(py_trees.display.unicode_tree(tree.py_tree.root, show_status=True).rstrip())
                state["last"] = snap
            if res.status in ("SUCCESS", "FAILURE"):
                state["result"] = res.status
        mujoco.mj_step(m, d)
        log.update(d)
        state["k"] += 1

    def report():
        result = tree.get_execution_result()
        s = log.summary()
        print(f"\nresult {result.status}: {result.total_ticks} ticks, "
              f"{result.duration:.1f} s sim time, base path {s['path_length_m']:.2f} m")
        for f in blackboard.get("failure_log", []):
            print(f"  failure at t={f['time']:.1f}s in '{f['node']}': {f['reason']} {f['detail']}")

    if args.headless:
        while state["result"] is None and d.time < TIME_LIMIT_S:
            step()
        report()
        return

    from mujoco import viewer as mj_viewer
    with mj_viewer.launch_passive(m, d) as viewer:
        viewer.cam.lookat[:] = [*info["box"]["pos"][:2], 0.6]
        viewer.cam.distance = 5.0
        viewer.cam.elevation = -35
        reported = False
        while viewer.is_running():
            t0 = time.time()
            for _ in range(8):  # ~real time
                step()
            if state["result"] and not reported:
                report()
                reported = True
            viewer.sync()
            time.sleep(max(0.0, 0.016 - (time.time() - t0)))


if __name__ == "__main__":
    main()
