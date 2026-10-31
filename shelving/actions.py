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
from shelving.teleop import BASE_STEP, HELP, STEP_COARSE, STEP_FINE, TURN_STEP, NoOperator
from shelving.skills import (
    BACKOFF, CARRY_TIP, HOVER, LIFT, PRESS, READY_REACH, backed_off, base_frame_point,
    carry_pose, clear_of_furniture, plan_box_approach, plan_shelf_approach, route, solve_down,
)
from sim.motion import BaseMove, JointMove, Lazy, ToolLine, Wait
from sim.placement import BOX_STAND_HALF, BasePose, _rot
from sim.world import PREFIX

RUNNING = py_trees.common.Status.RUNNING
SUCCESS = py_trees.common.Status.SUCCESS
FAILURE = py_trees.common.Status.FAILURE
NO_OPERATOR = NoOperator()


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

    def fail(self, reason, **detail):
        """BaseAction.fail, plus the cup being handled, so the report can tell which
        failures were recovered from (the cup still ended up on the shelf)."""
        detail.setdefault("cup", self.read_blackboard("current_cup"))
        return super().fail(reason, **detail)

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


    NEAR_FURNITURE = 1.0   # base centre within this of the box or shelf centre

    def safe_stages(self, world):
        """Stages that bring the robot clear of the furniture, whatever it was doing:
          1. tool over the box  -> raise it straight up (do not sweep through the cups)
          2. near box or shelf  -> back the base straight out (reverses the creep in, pulls
                                   the tool out of a shelf level)
          3. tool in to the ready distance at its height, then up to the carry pose"""
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

    @property
    def operator(self):
        """The stage-2 operator (shelving/teleop.py); nobody if the runner set none."""
        return self.read_blackboard("operator") or NO_OPERATOR


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
            op = self.operator

            def holding_no_takeover(w):
                if op.want_takeover:                     # operator pressed t: stop here
                    return "operator_takeover", {"slot": slot}
                return holding(cup)(w)

            self.stages = [
                Guarded(ToolLine(self.ik, align_tip, 1.5), holding_no_takeover),
                Guarded(ToolLine(self.ik, down_tip, 1.5), holding_no_takeover),
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

    Makes the robot safe first (SimAction.safe_stages). A held cup stays held: the next round sees it in the ledger and carries it on.
    If the slot could not be reached (no parking spot, blocked, timeout on the way), the
    slot goes on blocked_slots and the next round picks another one.
    """

    SLOT_FAILURES = ("no_shelf_spot", "blocked", "timeout")

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


# ---------------------------------------------------------------------------
# Stage 2: a person finishes the placement by keyboard (shelving/teleop.py)
#
#   Place on the slot (selector)
#   ├─ Robot places:  check_place_ready → place_cup → verify_placed
#   ├─ operator_available → retry: Human places:
#   │                          request_takeover → human_control → verify_placed → back_away
#   └─ Robot places anyway (nobody answered): place_cup → verify_placed
#
# The robot asks for help only at the last step and only when it cannot promise an
# accurate placement; the operator can also take over there by pressing t. Control goes
# back when the operator presses h; the robot then checks the placement itself and asks
# again if it is not right.
# ---------------------------------------------------------------------------
def describe(failure) -> str:
    """Why the robot asks for help, in words for the operator."""
    reason, det = failure.get("reason"), failure.get("detail", {})
    slot = det.get("slot", "?")
    return {
        "cup_tilted_on_pad": f"cup hangs {det.get('tilt_deg')} deg tilted on the pad; "
                             f"the robot can only put it down upright",
        "cup_off_centre_on_pad": f"cup sits {det.get('offset_mm')} mm off the pad centre",
        "ik_failed": f"no arm pose to put the cup down on slot {slot}",
        "misplaced": f"cup is {det.get('err_mm')} mm / {det.get('tilt_deg')} deg off slot {slot} "
                     f"(needs 15 mm / 5 deg)",
        "cup_dropped": "the cup slipped off while placing",
    }.get(reason, str(reason))


@action(
    "check_place_ready",
    "Condition before the robot puts the cup down: FAILURE (-> human) if the operator "
    "wants to take over or the cup sits too tilted / off-centre on the pad.",
    reads=["world", "operator"],
)
class CheckPlaceReady(SimAction):
    """params: cup, optional max_tilt_deg, max_offset_m.

    The robot aims the cup's bottom, so an off-centre cup is corrected for, but the tool
    always points straight down: a cup hanging tilted lands tilted (count_placed allows
    5 deg). In the runs without faults the cup hangs within 0.5 deg."""

    MAX_TILT_DEG = 4.0
    MAX_OFFSET = 0.02

    def execute(self):
        if self.operator.want_takeover:
            return FAILURE                      # not a fault: the operator asked
        world, cup = self.world, self.param("cup")
        up = world.d.body(cup).xmat.reshape(3, 3)[:, 2]
        tilt = float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))
        if tilt > self.param("max_tilt_deg", self.MAX_TILT_DEG):
            return self.fail("cup_tilted_on_pad", tilt_deg=round(tilt, 1))
        offset = float(np.linalg.norm((world.tool_tip() - world.cup_top(cup))[:2]))
        if offset > self.param("max_offset_m", self.MAX_OFFSET):
            return self.fail("cup_off_centre_on_pad", offset_mm=round(1000 * offset))
        return SUCCESS


@action(
    "operator_available",
    "Condition: SUCCESS if someone is at the keyboard (else the robot places by itself).",
    reads=["operator"],
)
class OperatorAvailable(SimAction):
    def execute(self):
        return SUCCESS if self.operator.available() else FAILURE


@action(
    "request_takeover",
    "Stop and ask the operator for help (the reason is the last failure); SUCCESS once "
    "the operator has taken over, FAILURE if nobody is at the keyboard.",
    reads=["world", "operator", "last_failure"],
    writes=["human_interventions"],
)
class RequestTakeover(SimAction):
    timeout_s = 600.0

    def execute(self):
        op = self.operator
        if self.stages is None:
            self.stages = []
            BaseMove.stop(self.world)
            if not op.available():
                return self.fail("no_operator")
            if not op.want_takeover:            # the robot asks; else the operator did
                op.request(describe(self.read_blackboard("last_failure") or {}))
        if self.now() - self.started_at > self.param("timeout_s", self.timeout_s):
            op.hand_back()
            return self.fail("no_answer", after_s=round(self.now() - self.started_at))
        if op.accept():
            n = self.read_blackboard("human_interventions", 0) + 1
            self.write_blackboard("human_interventions", n)
            print(f"\n  >> YOU HAVE CONTROL (intervention {n}). Keys, typed in this terminal:\n{HELP}")
            return SUCCESS
        return RUNNING


@action(
    "human_control",
    "The operator drives with the keyboard (tool, base, grip/release); SUCCESS when they hand "
    "back (h). Moves that would hit the box, the shelf or a dropped cup are refused.",
    reads=["world", "ik", "operator", "current_cup", "current_slot"],
)
class HumanControl(SimAction):
    timeout_s = 1e9
    SAFETY_MARGIN = 0.003

    def reset(self):
        super().reset()
        self.motion = None
        self.target = None      # commanded tool tip: steps add to it, not to where the tool
                                # happens to be (else small steps drift by the tracking error)

    def execute(self):
        world, op = self.world, self.operator
        if self.stages is None:
            self.stages = []
            BaseMove.stop(world)
            self.target = world.tool_tip()
            op.status = self.where()
            print("  " + op.status)
        if self.motion is not None:
            if self.motion.step(world):
                if isinstance(self.motion, BaseMove):      # the tool moved with the base
                    self.target = world.tool_tip()
                self.motion = None
                op.status = self.where()
                print("  " + op.status)
            return RUNNING
        key = op.next_key()
        if key is None:
            return RUNNING
        if key == "h":
            op.hand_back()
            print("\n  >> handed back to the robot: checking the placement")
            return SUCCESS
        if key == " ":
            world.suction_off()
            print("  released" if world.held_cup() is None else "  still holding?")
            op.status = self.where()
            return RUNNING
        if key == "g":
            held = world.held_cup() or world.suction_on()
            print(f"  {'holding ' + held if held else 'no seal: lower the pad flat onto a lid (F)'}")
            op.status = self.where()
            return RUNNING
        self.motion = self.command(world, key)
        return RUNNING

    def command(self, world, key):
        """A motion primitive for the key, or None (unknown key, out of reach, unsafe)."""
        x, y, yaw = world.base_pose()
        fwd, left = np.array([np.cos(yaw), np.sin(yaw)]), np.array([-np.sin(yaw), np.cos(yaw)])
        step = STEP_FINE if key.isupper() else STEP_COARSE
        tool = {"w": (step, 0, 0), "s": (-step, 0, 0), "a": (0, step, 0), "d": (0, -step, 0),
                "r": (0, 0, step), "f": (0, 0, -step)}.get(key.lower())
        base = {"i": (BASE_STEP, 0, 0), "k": (-BASE_STEP, 0, 0), "j": (0, BASE_STEP, 0),
                "l": (0, -BASE_STEP, 0), "u": (0, 0, TURN_STEP), "o": (0, 0, -TURN_STEP)}.get(key)
        q = world.joint_positions()
        if tool is not None:
            target = self.target + np.r_[tool[0] * fwd + tool[1] * left, tool[2]]
            res = self.ik.solve(world.d, target, seed=q, axis_tol=0.003)
            if not res.ok:
                print("  refused: out of the arm's reach")
                return None
            if not clear_of_furniture(world, self.ik, (x, y, yaw), res.q, self.SAFETY_MARGIN,
                                      held_cup=False):
                print("  refused: the arm would touch the furniture")
                return None
            self.target = target
            return ToolLine(self.ik, target, 0.3, tol=0.001, axis_tol=0.003, settle=0.2)
        if base is not None:
            xy = np.array([x, y]) + base[0] * fwd + base[1] * left
            goal = (float(xy[0]), float(xy[1]), yaw + base[2])
            if not clear_of_furniture(world, self.ik, goal, q, self.SAFETY_MARGIN, held_cup=False):
                print("  refused: the robot would touch the furniture")
                return None
            return BaseMove([goal], vmax=0.1)
        return None

    def where(self) -> str:
        """Where the cup is relative to the slot, in the robot's frame (for the operator)."""
        world, d = self.world, self.world.d
        cup, slot = self.read_blackboard("current_cup"), self.read_blackboard("current_slot")
        if cup is None or slot is None:
            return ""
        _, _, yaw = world.base_pose()
        c, s = np.cos(yaw), np.sin(yaw)
        sp = d.site(f"slot_{slot}").xpos
        if world.held_cup() == cup:
            e = sp - d.site(f"{cup}_bottom").xpos
            return (f"cup -> slot_{slot}: forward {1000 * (c * e[0] + s * e[1]):+.0f} mm, "
                    f"left {1000 * (-s * e[0] + c * e[1]):+.0f} mm, down {-1000 * e[2]:.0f} mm")
        e = world.cup_top(cup) - world.tool_tip()
        off = 1000 * float(np.linalg.norm(d.site(f"{cup}_bottom").xpos[:2] - sp[:2]))
        return (f"cup is {off:.0f} mm off slot_{slot}; pad -> lid: forward "
                f"{1000 * (c * e[0] + s * e[1]):+.0f} mm, left {1000 * (-s * e[0] + c * e[1]):+.0f} mm, "
                f"down {-1000 * e[2]:.0f} mm")


@action(
    "back_away",
    "Bring the robot clear of the shelf after the operator handed back (arm out of the "
    "shelf, carry pose).",
    reads=["world", "ik", "planner"],
)
class BackAway(SimAction):
    def execute(self):
        if self.stages is None:
            self.stages = self.safe_stages(self.world)
        return self.run_stages()
