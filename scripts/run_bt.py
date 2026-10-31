"""Run a Behavior Tree (JSON) on the MuJoCo cup-shelving scene with BehaviorOS.

    mjpython scripts/run_bt.py trees/b2_go_to_box.json --seed 0          # watch in the viewer
    python   scripts/run_bt.py trees/b2_go_to_box.json --seed 0 --headless
    python   scripts/run_bt.py trees/b5_recovery.json --seed 0 --headless --inject seal:0.3
             (faults: shelving/faults.py)
    python   scripts/run_bt.py trees/b5_recovery.json --seed 0 --headless --csv results/results.csv
             (writes this seed's row in the results_template.csv format; other rows kept)

Loop (same role as the Webots controller's execution_loop, in one process):
  every 2 ms   mj_step          physics
  every 20 ms  tree.tick()      BehaviorOS: runtime resolution hook, then py_trees tick
The tree is printed whenever any node changes status, so you can follow where it is.
"""

import argparse
import collections
import csv
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402
import py_trees  # noqa: E402

import shelving.actions  # noqa: E402,F401  (registers the actions)
from shelving.faults import FaultInjector  # noqa: E402
from behavioros.core.builder import TreeBuilder  # noqa: E402
from sim.kinematics import ArmIK  # noqa: E402
from sim.nav import GridPlanner  # noqa: E402
from sim.world import World, r, randomize  # noqa: E402

CONTROL_EVERY = 10      # physics steps per tick: 10 x 2 ms = 20 ms
TIME_LIMIT_S = 600.0    # simulation seconds before giving up


def tree_snapshot(root):
    return tuple((n.name, n.status.name) for n in root.iterate())


CSV_FIELDS = ["seed", "success_count", "total_cups", "duration_s", "path_length_m",
              "reference_path_m", "path_vs_reference", "human_interventions", "failures",
              "recoveries", "notes"]          # column order of the provided results_template.csv


def results_row(seed, blackboard, placed, summary, reference_m, duration_s, inject):
    """One CSV row. failures: every failure the tree recorded (end of work excluded).
    recoveries: failures whose cup still ended up on the shelf."""
    log = [f for f in blackboard.get("failure_log", []) if f["reason"] != "nothing_left"]
    cups = blackboard.get("cups", {})
    recovered = [f for f in log if cups.get(f["detail"].get("cup")) == "placed"]
    counts = collections.Counter(f["reason"] for f in log)
    notes = [f"{r} x{n}" for r, n in counts.most_common()]
    left = {c.removeprefix("cup_"): s for c, s in cups.items() if s != "placed"}
    if left:
        notes.append("not shelved: " + " ".join(f"{c}={s}" for c, s in left.items()))
    if blackboard.get("blocked_slots"):
        notes.append("blocked slots: " + " ".join(map(str, sorted(blackboard["blocked_slots"]))))
    if inject:
        notes.append(f"injected faults: {inject}")
    return {
        "seed": seed, "success_count": placed, "total_cups": 6,
        "duration_s": round(duration_s, 1),
        "path_length_m": round(summary["path_length_m"], 2),
        "reference_path_m": round(reference_m, 2),
        "path_vs_reference": round(summary["path_vs_reference"], 2),
        "human_interventions": blackboard.get("human_interventions", 0),
        "failures": len(log), "recoveries": len(recovered),
        "notes": "; ".join(notes) or "no failures",
    }


def write_row(path: Path, row):
    """Replace this seed's row (or add it), keep the others, sorted by seed."""
    rows = []
    if path.exists():
        with path.open() as f:
            rows = [r for r in csv.DictReader(f) if r["seed"] != str(row["seed"])]
    rows.append(row)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()
        w.writerows(sorted(rows, key=lambda r: int(r["seed"])))


def running_leaf(root):
    return next((n for n in root.iterate() if n.status.name == "RUNNING" and not n.children),
                None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tree", type=Path, help="JSON tree file")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--inject", default="", help="faults, e.g. seal-fail:1,drop-carry:4 (shelving/faults.py)")
    ap.add_argument("--csv", type=Path, help="write this run's row to a results CSV")
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
    faults = FaultInjector(args.inject, world, args.seed)
    log = randomize.MobilityLog(m, r("base_link"), info)
    print(f"seed {args.seed}: tree {args.tree.name}")

    state = {"k": 0, "last": None, "result": None}

    def step():
        """One physics step; tick the tree every CONTROL_EVERY steps."""
        if state["result"] is None and state["k"] % CONTROL_EVERY == 0:
            res = tree.tick()
            faults.on_tick(running_leaf(tree.py_tree.root), blackboard)
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
        placed = randomize.count_placed(m, d)["success_count"]   # the provided judge
        print(f"\nresult {result.status}: {result.total_ticks} ticks, "
              f"{result.duration:.1f} s sim time, base path {s['path_length_m']:.2f} m "
              f"(x{s['path_vs_reference']:.2f} of straight-line reference), "
              f"count_placed {placed}/6")
        for t, what in faults.events:
            print(f"  injected at t={t:.1f}s: {what}")
        for f in blackboard.get("failure_log", []):
            print(f"  failure at t={f['time']:.1f}s in '{f['node']}': {f['reason']} {f['detail']}")
        print(f"  recoveries (recover node): {blackboard.get('recoveries', 0)}")
        if "cups" in blackboard:
            print("  ledger: " + ", ".join(f"{c.removeprefix('cup_')}={s}"
                                           for c, s in blackboard["cups"].items()))
        if args.csv:
            write_row(args.csv, results_row(args.seed, blackboard, placed, s,
                                            info["mobility"]["reference_path_6_cups_m"],
                                            result.duration, args.inject))
            print(f"  row written to {args.csv}")

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
