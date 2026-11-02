"""Stage 2: a person finishes the placement by keyboard (operator: shelving/teleop.py).

    Place on the slot (selector)
    ├─ Robot places:  check_place_ready → place_cup → verify_placed
    ├─ operator_available → retry: Human places:
    │                          request_takeover → human_control → verify_placed → back_away
    └─ Robot places anyway (nobody answered): place_cup → verify_placed

The robot asks for help only at the last step and only when it cannot promise an accurate
placement; the operator can also take over there by pressing t. Control goes back when the
operator presses h; the robot then checks the placement itself and asks again if it is
not right.
"""

from __future__ import annotations

import numpy as np

from behavioros.core.actions import action
from shelving.actions.base import FAILURE, RUNNING, SUCCESS, SimAction
from shelving.common import clear_of_furniture
from shelving.motion import BaseMove, ToolLine
from shelving.teleop import BASE_STEP, HELP, STEP_COARSE, STEP_FINE, TURN_STEP


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
