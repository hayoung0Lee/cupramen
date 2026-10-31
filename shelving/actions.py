"""BehaviorOS actions for the cup-shelving task.

Every action follows the same pattern:
  reset()    per-run state; called by BehaviorOS each time the node starts (again)
  execute()  called every tick (20 ms); on the first tick it plans from the *current*
             simulation state, then advances its stages a little and returns RUNNING,
             or SUCCESS when done, or self.fail(reason) when it cannot finish.

Shared objects come from the blackboard (put there by scripts/run_bt.py):
  world    sim.world.World      the MuJoCo scene and robot
  ik       sim.kinematics.ArmIK
  planner  sim.nav.GridPlanner  occupancy grid for this episode's layout
"""

from __future__ import annotations

import numpy as np
import py_trees

from behavioros.core.actions import BaseAction, action
from shelving import ledger
from shelving.skills import (
    BACKOFF, CARRY_TIP, HOVER, LIFT, PRESS, READY_REACH, backed_off, base_frame_point,
    carry_pose, plan_box_approach, plan_shelf_approach, route, solve_down,
)
from sim.motion import BaseMove, JointMove, Lazy, ToolLine, Wait
from sim.placement import BOX_STAND_HALF, BasePose, _rot
from sim.world import PREFIX

RUNNING = py_trees.common.Status.RUNNING
SUCCESS = py_trees.common.Status.SUCCESS
FAILURE = py_trees.common.Status.FAILURE


# ---------------------------------------------------------------------------
# Stages: what an action runs, one after another
# ---------------------------------------------------------------------------
class Check:
    """Inspect the world without moving it. fn(world) -> None if fine, else (reason, detail)."""

    def __init__(self, fn):
        self.fn = fn


class Do:
    """Change something instantly (e.g. suction off); finished at once."""

    def __init__(self, fn):
        self.fn = fn


class Guarded:
    """A motion primitive that must stop the moment a condition breaks.

    guard(world) -> None if fine, else (reason, detail); checked before every step.
    E.g. while carrying, fail with cup_dropped as soon as the pad loses the cup.
    """

    def __init__(self, inner, guard):
        self.inner, self.guard = inner, guard


def holding(cup):
    """Guard / check: the pad still holds `cup`."""
    def check(world):
        held = world.held_cup()
        if held == cup:
            return None
        return ("cup_dropped" if held is None else "wrong_cup"), {"cup": cup, "held": held}
    return check


class SimAction(BaseAction):
    """Common helpers for actions that run stages in the simulation.

    self.stages: list of motion primitives (sim/motion.py), Check, Do or Guarded.
    run_stages(): call once per tick from execute().
    """

    timeout_s = 60.0

    @property
    def world(self):
        return self.read_blackboard("world")

    @property
    def ik(self):
        return self.read_blackboard("ik")

    @property
    def planner(self):
        return self.read_blackboard("planner")

    def reset(self):
        self.stages = None          # None = not planned yet (first tick plans)
        self.started_at = self.now()

    def run_stages(self):
        """Advance by one tick. SUCCESS when every stage is done; FAILURE on a broken check."""
        world = self.world
        if self.now() - self.started_at > self.param("timeout_s", self.timeout_s):
            BaseMove.stop(world)
            return self.fail("timeout", after_s=round(self.now() - self.started_at, 1))
        # Instant stages run back to back in the same tick.
        while self.stages and isinstance(self.stages[0], (Check, Do)):
            stage = self.stages.pop(0)
            problem = stage.fn(world)
            if isinstance(stage, Check) and problem is not None:
                BaseMove.stop(world)
                return self.fail(problem[0], **problem[1])
        if not self.stages:
            return SUCCESS
        stage = self.stages[0]
        if isinstance(stage, Guarded):
            problem = stage.guard(world)
            if problem is not None:
                BaseMove.stop(world)
                return self.fail(problem[0], **problem[1])
            stage = stage.inner
        if stage.step(world):
            self.stages.pop(0)
        elif getattr(stage, "stuck", False):          # BaseMove made no progress
            BaseMove.stop(world)
            touching = {n for pair in world.robot_scene_contacts() for n in pair
                        if not n.startswith(PREFIX) and "floor" not in n}
            return self.fail("blocked", touching=sorted(touching))
        return SUCCESS if not self.stages else RUNNING


# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------
@action(
    "go_to_box",
    "Drive to a parking pose beside the box from which the cup can be picked.",
    reads=["world", "ik", "planner"],
    writes=["box_plan"],
)
class GoToBox(SimAction):
    """params: cup (name or "${current_cup}"), optional timeout_s.

    Picks the nearest box side that the base can reach and from which the arm can reach the
    cup's lid, drives to a point backed off from it along the A* route, then creeps straight
    in. Writes the chosen plan to blackboard["box_plan"] for the pick that follows.
    Only the base moves.
    """

    def execute(self):
        if self.stages is None:
            world = self.world
            cup = self.param("cup")
            start = world.base_pose()
            plan = plan_box_approach(world, self.ik, self.planner, cup, start)
            if plan is None:
                return self.fail("no_box_side", cup=cup)
            path = route(self.planner, start, plan.out) or [plan.out]  # [] if already there
            self.write_blackboard("box_plan", plan)
            park = (plan.park.x, plan.park.y, plan.park.yaw)
            self.stages = [BaseMove(path), BaseMove([park], vmax=0.12)]
            print(f"    [{self.name}] {cup}: box side {plan.park.side}, "
                  f"{len(path)} waypoint(s) then drive in")
        return self.run_stages()


@action(
    "pick_cup",
    "At the box: lower the pad onto the cup lid, seal, lift, back away and tuck the arm.",
    reads=["world", "ik", "box_plan"],
)
class PickCup(SimAction):
    """params: cup.

    Arm poses are solved again from the cup's *current* position (it may have moved since
    go_to_box planned) and from where the base actually stopped.
    """

    def execute(self):
        if self.stages is None:
            world, cup = self.world, self.param("cup")
            plan = self.read_blackboard("box_plan")
            if plan is None or plan.cup != cup:
                return self.fail("no_box_plan", cup=cup)
            base, top = world.base_pose(), world.cup_top(cup)
            q = {name: solve_down(world, self.ik, base, top + [0, 0, dz])
                 for name, dz in (("hover", HOVER), ("press", -PRESS), ("lift", LIFT))}
            q["carry"] = carry_pose(world, self.ik)
            missing = [k for k, v in q.items() if v is None]
            if missing:
                return self.fail("ik_failed", cup=cup, poses=missing)

            def seal(w):
                held = w.suction_on()
                if held != cup:
                    w.suction_off()
                    return "seal_failed", {"cup": cup, "sealed_on": held}
                return None

            def lifted(w):
                rise = float(w.cup_top(cup)[2] - top[2])
                return None if rise > LIFT / 2 else ("not_lifted", {"cup": cup, "rise_m": round(rise, 3)})

            self.stages = [
                JointMove(q["hover"].q, 2.5),
                JointMove(q["press"].q, 1.5),
                Check(seal),
                Wait(0.3),
                Guarded(JointMove(q["lift"].q, 1.5), holding(cup)),
                Check(lifted),
                Guarded(BaseMove([plan.out], vmax=0.12), holding(cup)),
                Guarded(JointMove(q["carry"].q, 2.0), holding(cup)),
            ]
        return self.run_stages()


@action(
    "go_to_shelf",
    "Carry the held cup to a parking pose in front of the slot and creep in, arm ready.",
    reads=["world", "ik", "planner"],
    writes=["shelf_plan"],
)
class GoToShelf(SimAction):
    """params: cup, slot (1-6), optional timeout_s.

    Plans from the current pose: A* to a backed-off point, unfold the arm there (clear of
    the shelf), then drive straight in. Fails with cup_dropped as soon as the cup is lost.
    """

    timeout_s = 90.0

    def execute(self):
        if self.stages is None:
            world, cup, slot = self.world, self.param("cup"), int(self.param("slot"))
            cup_h = float(world.m.site(f"{cup}_top").pos[2])
            start = world.base_pose()
            plan = plan_shelf_approach(world, self.ik, self.planner, slot, cup_h, start)
            if plan is None:
                return self.fail("no_shelf_spot", slot=slot)
            self.write_blackboard("shelf_plan", plan)
            park = (plan.park.x, plan.park.y, plan.park.yaw)
            # Tool positions with the base at `out`: in front of the shelf at slot height,
            # and the slot-entry pose. Near the shelf the tool moves only along straight
            # lines (ToolLine): joint interpolation swings the tip on an arc into the boards.
            ready_tip = self.ik.tool_tip_at(world.d, plan.q_ready.q, plan.out)
            reach_in_tip = self.ik.tool_tip_at(world.d, plan.q_hover.q, plan.out)
            self.stages = [
                Guarded(BaseMove(route(self.planner, start, plan.out) or [plan.out]), holding(cup)),
                Guarded(ToolLine(self.ik, ready_tip, 2.0), holding(cup)),     # down, outside
                Guarded(ToolLine(self.ik, reach_in_tip, 1.5), holding(cup)),  # reach in level
                Guarded(BaseMove([park], vmax=0.12), holding(cup)),
            ]
            print(f"    [{self.name}] slot_{slot}: parking {plan.park.side}")
        return self.run_stages()


@action(
    "place_cup",
    "Lower the held cup onto the slot (correcting for where it sits on the pad), release, "
    "back away and fold the arm up outside the shelf.",
    reads=["world", "ik", "shelf_plan"],
)
class PlaceCup(SimAction):
    """params: cup, slot."""

    def execute(self):
        if self.stages is None:
            world, cup, slot = self.world, self.param("cup"), int(self.param("slot"))
            plan = self.read_blackboard("shelf_plan")
            if plan is None or plan.slot != slot:
                return self.fail("no_shelf_plan", slot=slot)
            slot_xyz = world.d.site(f"slot_{slot}").xpos.copy()
            # The cup can sit off-centre on the pad: aim the cup bottom, not the tool.
            offset = world.tool_tip() - world.d.site(f"{cup}_bottom").xpos
            base = world.base_pose()
            align_tip = slot_xyz + offset + [0, 0, HOVER]
            down_tip = slot_xyz + offset + [0, 0, 0.003]
            if solve_down(world, self.ik, base, align_tip) is None or \
                    solve_down(world, self.ik, base, down_tip) is None:
                return self.fail("ik_failed", slot=slot,
                                 offset_mm=round(1000 * float(np.linalg.norm(offset[:2]))))
            # After backing out: pull the tool out level, then straight up to the travel pose.
            ready_tip = self.ik.tool_tip_at(world.d, plan.q_ready.q, plan.out)
            travel_tip = base_frame_point(plan.out, CARRY_TIP)
            self.stages = [
                Guarded(ToolLine(self.ik, align_tip, 1.5), holding(cup)),
                Guarded(ToolLine(self.ik, down_tip, 1.5), holding(cup)),
                Do(lambda w: w.suction_off()),
                Wait(0.8),
                ToolLine(self.ik, align_tip, 1.0),
                BaseMove([plan.out], vmax=0.12),
                ToolLine(self.ik, ready_tip, 1.5),
                ToolLine(self.ik, travel_tip, 1.5),
            ]
        return self.run_stages()


@action(
    "verify_placed",
    "Check the cup stands on the slot: centre within 15 mm and tilt within 5 deg "
    "(the same test as the provided count_placed).",
    reads=["world"],
)
class VerifyPlaced(SimAction):
    """params: cup, slot."""

    TOL_XY = 0.015
    MAX_TILT_DEG = 5.0

    def execute(self):
        world, cup, slot = self.world, self.param("cup"), int(self.param("slot"))
        bottom = world.d.site(f"{cup}_bottom").xpos
        err = float(np.linalg.norm(bottom[:2] - world.d.site(f"slot_{slot}").xpos[:2]))
        up = world.d.body(cup).xmat.reshape(3, 3)[:, 2]
        tilt = float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))
        if err > self.TOL_XY or tilt > self.MAX_TILT_DEG:
            return self.fail("misplaced", cup=cup, slot=slot,
                             err_mm=round(1000 * err, 1), tilt_deg=round(tilt, 1))
        print(f"    [{self.name}] {cup} on slot_{slot}: {1000 * err:.1f} mm, {tilt:.1f} deg")
        return SUCCESS


# ---------------------------------------------------------------------------
# Task bookkeeping: the world-state ledger on the blackboard
#   cups           {cup name: state}, states in shelving/ledger.py
#   slots          {slot number: cup name or None}
#   attempts       {cup name: recoveries spent on it}
#   blocked_slots  slots the robot could not get to (e.g. a dropped cup in front of them)
#   current_cup    cup being handled now      (nodes read it as "${current_cup}")
#   current_slot   slot it is going to        (nodes read it as "${current_slot}")
#   all_cups_done  True when nothing is left to do (repeat_until stops on it)
#   recoveries     how many times recover ran
# The ledger is always rebuilt from the simulation (ledger.observe), never edited by
# hand: after a failure the plan says little about where the cups really are.
# ---------------------------------------------------------------------------
def _print_ledger(bb):
    cups = ", ".join(f"{c.removeprefix('cup_')}={s}" for c, s in bb["cups"].items())
    slots = " ".join(f"{k}:{(v or '-').removeprefix('cup_')}" for k, v in bb["slots"].items())
    print(f"    ledger | cups: {cups}\n           | slots: {slots}")


class LedgerAction(SimAction):
    def refresh_ledger(self):
        cups, slots = ledger.observe(self.world, self.read_blackboard("attempts"))
        self.write_blackboard("cups", cups)
        self.write_blackboard("slots", slots)
        blocked = self.read_blackboard("blocked_slots", set())
        self.write_blackboard("all_cups_done", ledger.nothing_left(cups, slots, blocked))
        _print_ledger(self.blackboard)


@action(
    "init_episode",
    "Start the ledger from what the scene shows (normally: every cup in the box).",
    reads=["world"],
    writes=["cups", "slots", "attempts", "blocked_slots", "recoveries", "all_cups_done"],
)
class InitEpisode(LedgerAction):
    def execute(self):
        self.write_blackboard("attempts", {})
        self.write_blackboard("blocked_slots", set())
        self.write_blackboard("recoveries", 0)
        self.refresh_ledger()
        return SUCCESS


@action(
    "select_next_task",
    "Choose the next cup (the one in the hand, else the nearest still in the box) and "
    "slot (lowest empty).",
    reads=["world", "cups", "slots", "blocked_slots"],
    writes=["current_cup", "current_slot", "all_cups_done"],
)
class SelectNextTask(SimAction):
    def execute(self):
        world, cups = self.world, self.read_blackboard("cups")
        held = [c for c, s in cups.items() if s == "held"]
        in_box = [c for c, s in cups.items() if s == "in_box"]
        blocked = self.read_blackboard("blocked_slots", set())
        empty = [k for k, v in self.read_blackboard("slots").items()
                 if v is None and k not in blocked]
        if not (held or in_box) or not empty:
            self.write_blackboard("all_cups_done", True)
            return self.fail("nothing_left", cups_in_box=len(in_box), empty_slots=len(empty))
        base = np.array(world.base_pose()[:2])
        cup = held[0] if held else \
            min(in_box, key=lambda c: float(np.linalg.norm(world.cup_top(c)[:2] - base)))
        slot = min(empty)
        self.write_blackboard("current_cup", cup)
        self.write_blackboard("current_slot", slot)
        print(f"    [{self.name}] next: {cup} -> slot_{slot}")
        return SUCCESS


@action(
    "is_holding",
    "Condition: SUCCESS if the pad holds the cup, else FAILURE (not an error).",
    reads=["world"],
)
class IsHolding(SimAction):
    """params: cup. Lets a fallback skip fetching a cup that is already in the hand,
    e.g. after recovering from a failure at the shelf."""

    def execute(self):
        return SUCCESS if self.world.held_cup() == self.param("cup") else FAILURE


@action(
    "mark_placed",
    "Look at the scene again after a verified placement; set all_cups_done when finished.",
    reads=["world", "attempts"],
    writes=["cups", "slots", "all_cups_done"],
)
class MarkPlaced(LedgerAction):
    def execute(self):
        self.refresh_ledger()
        return SUCCESS


@action(
    "recover",
    "After a failed cup: stop, bring the arm back to the carry pose clear of the "
    "furniture, count the attempt, and rebuild the ledger from the scene.",
    reads=["world", "ik", "planner", "last_failure", "current_cup", "current_slot", "attempts"],
    writes=["cups", "slots", "attempts", "blocked_slots", "recoveries", "all_cups_done",
            "last_failure"],
)
class Recover(LedgerAction):
    """Runs as the fallback of "shelve one cup". Always SUCCESS, so the loop goes on and
    the next round re-plans from wherever the robot and the cups are now.

    Making the robot safe, whatever it was doing:
      1. tool over the box  -> raise it straight up (do not sweep through the cups)
      2. near box or shelf  -> back the base straight out (reverses the creep in, pulls the
                               tool out of a shelf level)
      3. tool in to the ready distance at its height, then up to the carry pose
    A held cup stays held: the next round sees it in the ledger and carries it on.
    If the slot could not be reached (no parking spot, blocked, timeout on the way), the
    slot goes on blocked_slots and the next round picks another one.
    """

    SLOT_FAILURES = ("no_shelf_spot", "blocked", "timeout")

    NEAR_FURNITURE = 1.0   # base centre within this of the box or shelf centre

    def execute(self):
        if self.stages is None:
            world = self.world
            BaseMove.stop(world)
            if self.read_blackboard("all_cups_done"):        # e.g. select found nothing
                return SUCCESS
            failure = self.read_blackboard("last_failure") or {}
            cup = self.read_blackboard("current_cup")
            print(f"    [{self.name}] after '{failure.get('reason')}' in "
                  f"'{failure.get('node')}' ({cup}): making safe, then re-planning")
            slot = None
            if failure.get("action") == "go_to_shelf" and failure.get("reason") in self.SLOT_FAILURES:
                slot = self.read_blackboard("current_slot")
            self.stages = self.safe_stages(world) + [Do(lambda w: self.finish(cup, slot))]
        return self.run_stages()

    def safe_stages(self, world):
        info, ik = world.info, self.ik
        stages = []
        x, y, yaw = world.base_pose()
        box_local = _rot(info["box"]["yaw"]).T @ (world.tool_tip()[:2] - np.asarray(info["box"]["pos"][:2]))
        if abs(box_local[0]) < BOX_STAND_HALF[0] + 0.05 and abs(box_local[1]) < BOX_STAND_HALF[1] + 0.05:
            tip = world.tool_tip()
            stages.append(ToolLine(ik, [tip[0], tip[1], max(tip[2], CARRY_TIP[2])], 1.5))
        near = min(np.linalg.norm(np.array([x, y]) - np.asarray(info[k]["pos"][:2]))
                   for k in ("box", "shelf"))
        back = backed_off(BasePose(x, y, yaw, ""), BACKOFF)
        if near < self.NEAR_FURNITURE and self.planner.free(back[:2]):
            stages.append(BaseMove([back], vmax=0.12))

        def tool_in(w):
            base, tip = w.base_pose(), w.tool_tip()
            c, s = np.cos(base[2]), np.sin(base[2])
            left = float((tip[:2] - np.asarray(base[:2])) @ [-s, c])
            return ToolLine(ik, base_frame_point(base, (READY_REACH, left, tip[2])), 1.5)

        stages.append(Lazy(tool_in))
        stages.append(Lazy(lambda w: ToolLine(ik, base_frame_point(w.base_pose(), CARRY_TIP), 1.5)))
        return stages

    def finish(self, cup, blocked_slot=None):
        if blocked_slot is not None:
            self.read_blackboard("blocked_slots").add(int(blocked_slot))
            print(f"    [{self.name}] slot_{blocked_slot} blocked, will use another slot")
        attempts = self.read_blackboard("attempts")
        if cup is not None:
            attempts[cup] = attempts.get(cup, 0) + 1
        self.write_blackboard("recoveries", self.read_blackboard("recoveries", 0) + 1)
        self.write_blackboard("last_failure", None)          # handled
        self.refresh_ledger()
