"""The frame every shelving action is built on.

An action runs a list of *stages* one after another, a little each tick:
  motion primitives (shelving/utils/motion.py)  BaseMove, ToolLine, JointMove, Wait: take time
  Check(fn)                               look at the world; fail with a reason if wrong
  Do(fn)                                  change something at once (e.g. suction off)
  Guarded(motion, guard)                  a motion that stops the moment guard() complains

SimAction (a BehaviorOS BaseAction) runs them: execute() plans on the first tick and fills
self.stages, then calls run_stages() every tick, which returns RUNNING / SUCCESS / FAILURE.
"""

from __future__ import annotations

import numpy as np
import py_trees

from behavioros.core.actions import BaseAction
from shelving.utils.common import (
    BACKOFF, BOX_STAND_HALF, CARRY_TIP, READY_REACH, BasePose, backed_off, base_frame_point, rot2d,
)
from shelving.utils.motion import BaseMove, Lazy, ToolLine
from shelving.utils.teleop import NoOperator
from sim.world import PREFIX

RUNNING = py_trees.common.Status.RUNNING
SUCCESS = py_trees.common.Status.SUCCESS
FAILURE = py_trees.common.Status.FAILURE
NO_OPERATOR = NoOperator()


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

    Objects come from the tree's devices (self.devices, set up by scripts/run_bt.py); the
    blackboard only carries data between nodes (current cup, ledger, failures ...):
      world     sim.world.World                   the MuJoCo scene and robot
      ik        shelving.utils.kinematics.ArmIK   arm math
      planner   shelving.utils.planner.GridPlanner    map and paths for this episode's layout
      operator  shelving.utils.teleop.Operator    stage 2 (nobody if absent)
    """

    timeout_s = 60.0
    NEAR_FURNITURE = 1.0   # base centre within this of the box or shelf centre

    @property
    def world(self):
        return self.devices["world"]

    @property
    def ik(self):
        return self.devices["ik"]

    @property
    def planner(self):
        return self.devices["planner"]

    @property
    def operator(self):
        """The stage-2 operator (shelving/utils/teleop.py); nobody if the runner set none."""
        return self.devices.get("operator") or NO_OPERATOR

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

    def safe_stages(self, world):
        """Stages that bring the robot clear of the furniture, whatever it was doing
        (used by recover and back_away):
          1. tool over the box  -> raise it straight up (do not sweep through the cups)
          2. near box or shelf  -> back the base straight out (reverses the creep in, pulls
                                   the tool out of a shelf level)
          3. tool in to the ready distance at its height, then up to the carry pose"""
        info, ik = world.info, self.ik
        stages = []
        x, y, yaw = world.base_pose()
        box_local = rot2d(info["box"]["yaw"]).T @ (world.tool_tip()[:2] - np.asarray(info["box"]["pos"][:2]))
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
