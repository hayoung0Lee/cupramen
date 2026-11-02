"""Run a Behavior Tree (JSON) on the MuJoCo cup-shelving scene with BehaviorOS.

    mjpython scripts/run_bt.py trees/b2_go_to_box.json --seed 0          # watch in the viewer
    python   scripts/run_bt.py trees/b2_go_to_box.json --seed 0 --headless
    python   scripts/run_bt.py trees/b5_recovery.json --seed 0 --headless --inject seal:0.3
             (faults: shelving/faults.py)
    python   scripts/run_bt.py trees/b5_recovery.json --seed 0 --headless --csv results/results.csv
             (writes this seed's row in the results_template.csv format; other rows kept)
    mjpython scripts/run_bt.py trees/b6_takeover.json --seed 0
             (stage 2: press t in this terminal to take over at the shelf, ? for keys)

Loop (same role as the Webots controller's execution_loop, in one process), every 20 ms:
  tick_tree()     the running node does one slice of its work (while the tree runs)
  run_physics()   mj_step x 10, 2 ms each (always)
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
import numpy as np  # noqa: E402
import py_trees  # noqa: E402

import shelving  # noqa: E402,F401  (registers the Behavior Tree nodes)
from shelving.faults import FaultInjector  # noqa: E402
from shelving.teleop import NoOperator, Operator, ScriptedOperator, start_terminal_keys  # noqa: E402
from behavioros.core.builder import TreeBuilder  # noqa: E402
from shelving.kinematics import ArmIK  # noqa: E402
from shelving.nav import GridPlanner  # noqa: E402
from sim.world import World, r, randomize  # noqa: E402

KEYS_LEFT = "type in the TERMINAL\nw / s\na / d\nr / f\nW S A D R F\ni k j l / u o\ng / space\nh"
KEYS_RIGHT = ("\ntool forward / back 1 cm\ntool left / right 1 cm\ntool up / down 1 cm\n"
              "same, 2 mm\nbase 5 cm / turn 5 deg\ngrip / release\nhand back to the robot")
PHYSICS_STEPS_PER_TICK = 10   # physics 2 ms x 10 = one tree tick every 20 ms
TIME_LIMIT_S = 600.0          # simulation seconds before giving up (headless)


def tree_snapshot(root):
    return tuple((n.name, n.status.name) for n in root.iterate())


CSV_FIELDS = ["seed", "success_count", "total_cups", "duration_s", "path_length_m",
              "reference_path_m", "path_vs_reference", "human_interventions", "failures",
              "recoveries", "notes"]          # column order of the provided results_template.csv


def results_row(seed, blackboard, placed, summary, reference_m, duration_s, inject):
    """One CSV row. failures: every failure the tree recorded (end of work excluded).
    recoveries: failures whose cup still ended up on the shelf."""
    not_faults = ("nothing_left", "operator_takeover")   # end of work; the person asked
    log = [f for f in blackboard.get("failure_log", []) if f["reason"] not in not_faults]
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
    ap.add_argument("--full-tree", action="store_true",
                    help="print the whole tree on every change (default unless a person drives)")
    ap.add_argument("--operator", default=None,
                    help="keyboard (default with the viewer), none (default headless), "
                         "or script:KEYS e.g. script:t,s,f,space,h (shelving/teleop.py)")
    args = ap.parse_args()

    world = World.create()
    info = world.reset(args.seed)
    m, d = world.m, world.d
    mode = args.operator or ("none" if args.headless else "keyboard")
    if mode == "keyboard":
        operator = Operator()
        start_terminal_keys(operator)
    elif mode.startswith("script:"):
        operator = ScriptedOperator(mode.removeprefix("script:"))
    else:
        operator = NoOperator()
    blackboard = {"world": world, "ik": ArmIK(m), "planner": GridPlanner(info),
                  "operator": operator, "human_interventions": 0}
    tree = (
        TreeBuilder(json.loads(args.tree.read_text()), blackboard, devices={"world": world},
                    clock=lambda: d.time)
        .with_runtime()
        .build()
    )
    faults = FaultInjector(args.inject, world, args.seed)
    mobility = randomize.MobilityLog(m, r("base_link"), info)   # provided: path length, time
    print(f"seed {args.seed}: tree {args.tree.name}")

    # With a person at the keyboard the full tree printouts would bury the teleop messages:
    # print one line per running action instead.
    full_tree = args.full_tree or mode != "keyboard"
    root = tree.py_tree.root
    shown = {"tree": None, "leaf": None}         # what show_progress printed last

    def show_progress(res):
        """Print the tree when any node changed status (or one line per new running action)."""
        if full_tree:
            snap = tree_snapshot(root)
            if snap != shown["tree"]:
                print(f"\n--- t={d.time:6.2f}s  tick {res.tick_count}: {res.status}")
                print(py_trees.display.unicode_tree(root, show_status=True).rstrip())
                shown["tree"] = snap
        else:
            leaf = running_leaf(root)
            if leaf is not None and leaf is not shown["leaf"]:
                print(f"  t={d.time:6.1f}s  {leaf.name}")
                shown["leaf"] = leaf

    def run_physics():
        """20 ms of physics: 10 steps of 2 ms."""
        for _ in range(PHYSICS_STEPS_PER_TICK):
            mujoco.mj_step(m, d)
            mobility.update(d)

    def tick_tree() -> str:
        """One tree tick: the running node does one slice of its work.
        Returns the tree status: RUNNING, SUCCESS or FAILURE."""
        res = tree.tick()
        faults.on_tick(running_leaf(root), blackboard)     # (optional) injected faults
        if isinstance(operator, ScriptedOperator):
            operator.tick()                                # (optional) replayed keys
        show_progress(res)
        return res.status

    def report():
        result = tree.get_execution_result()
        s = mobility.summary()
        placed = randomize.count_placed(m, d)["success_count"]   # the provided judge
        print(f"\nresult {result.status}: {result.total_ticks} ticks, "
              f"{result.duration:.1f} s sim time, base path {s['path_length_m']:.2f} m "
              f"(x{s['path_vs_reference']:.2f} of straight-line reference), "
              f"count_placed {placed}/6")
        for t, what in faults.events:
            print(f"  injected at t={t:.1f}s: {what}")
        for f in blackboard.get("failure_log", []):
            print(f"  failure at t={f['time']:.1f}s in '{f['node']}': {f['reason']} {f['detail']}")
        print(f"  recoveries (recover node): {blackboard.get('recoveries', 0)}, "
              f"human interventions: {blackboard.get('human_interventions', 0)}")
        if "cups" in blackboard:
            print("  ledger: " + ", ".join(f"{c.removeprefix('cup_')}={s}"
                                           for c, s in blackboard["cups"].items()))
        if args.csv:
            write_row(args.csv, results_row(args.seed, blackboard, placed, s,
                                            info["mobility"]["reference_path_6_cups_m"],
                                            result.duration, args.inject))
            print(f"  row written to {args.csv}")

    if args.headless:
        status = "RUNNING"
        while status == "RUNNING" and d.time < TIME_LIMIT_S:
            status = tick_tree()          # decide
            run_physics()                 # the world moves 20 ms
        report()
        return

    from mujoco import viewer as mj_viewer
    with mj_viewer.launch_passive(m, d) as viewer:
        viewer.cam.lookat[:] = [*info["box"]["pos"][:2], 0.6]
        viewer.cam.distance = 5.0
        viewer.cam.elevation = -35
        camera = {"mode": "auto", "overview": None}

        def follow_operator():
            """Close-up while a person drives (from behind and to the right of the robot,
            looking down at the tool, so 2 mm steps are visible); back to the overview after."""
            if operator.mode == camera["mode"]:
                return
            if operator.mode == "human":
                camera["overview"] = (viewer.cam.lookat.copy(), viewer.cam.distance,
                                      viewer.cam.azimuth, viewer.cam.elevation)
                viewer.cam.lookat[:] = world.tool_tip() - [0, 0, 0.08]
                viewer.cam.distance = 0.9
                viewer.cam.azimuth = float(np.degrees(world.base_pose()[2])) - 40
                viewer.cam.elevation = -25
            elif camera["mode"] == "human":
                (viewer.cam.lookat[:], viewer.cam.distance, viewer.cam.azimuth,
                 viewer.cam.elevation) = camera["overview"]
            camera["mode"] = operator.mode

        def show_texts(status):
            """Top left: who is in control (or DONE). Bottom left: the keys, while a person drives."""
            title, detail = operator.overlay() if status == "RUNNING" else ("DONE", status)
            texts = [(None, mujoco.mjtGridPos.mjGRID_TOPLEFT, title, detail)]
            if operator.mode == "human":
                texts.append((mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                              KEYS_LEFT, KEYS_RIGHT))
            viewer.set_texts(texts)

        tick_s = PHYSICS_STEPS_PER_TICK * m.opt.timestep    # 20 ms of simulated time
        status = "RUNNING"
        while viewer.is_running():
            started = time.time()
            if status == "RUNNING":       # decide, while the tree still runs
                status = tick_tree()
                if status != "RUNNING":
                    report()
            run_physics()                 # always: the world moves 20 ms (cups settle after the end)
            follow_operator()
            show_texts(status)
            viewer.sync()
            time.sleep(max(0.0, tick_s - (time.time() - started)))   # real time


if __name__ == "__main__":
    main()
