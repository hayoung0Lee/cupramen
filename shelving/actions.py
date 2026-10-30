"""BehaviorOS actions for the cup-shelving task.

Every action follows the same pattern:
  reset()    per-run state; called by BehaviorOS each time the node starts (again)
  execute()  called every tick (20 ms); on the first tick it plans from the *current*
             simulation state, then advances its motion primitives a little and returns
             RUNNING, or SUCCESS when done, or self.fail(reason) when it cannot finish.

Shared objects come from the blackboard (put there by scripts/run_bt.py):
  world    sim.world.World      the MuJoCo scene and robot
  ik       sim.kinematics.ArmIK
  planner  sim.nav.GridPlanner  occupancy grid for this episode's layout
"""

from __future__ import annotations

import py_trees

from behavioros.core.actions import BaseAction, action
from shelving.skills import plan_box_approach, route
from sim.motion import BaseMove

RUNNING = py_trees.common.Status.RUNNING
SUCCESS = py_trees.common.Status.SUCCESS


class SimAction(BaseAction):
    """Common helpers: blackboard objects, a list of motion stages, a time limit."""

    timeout_s = 60.0

    @property
    def world(self):
        return self.read_blackboard("world")

    def reset(self):
        self.stages = None          # motion primitives still to run; None = not planned yet
        self.started_at = self.now()

    def run_stages(self):
        """Advance the current stage by one tick. SUCCESS when all stages are done."""
        if self.now() - self.started_at > self.param("timeout_s", self.timeout_s):
            BaseMove.stop(self.world)
            return self.fail("timeout", after_s=round(self.now() - self.started_at, 1))
        if self.stages and self.stages[0].step(self.world):
            self.stages.pop(0)
        return SUCCESS if not self.stages else RUNNING


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
    """

    def execute(self):
        if self.stages is None:
            world = self.world
            cup = self.param("cup")
            start = world.base_pose()
            plan = plan_box_approach(world, self.read_blackboard("ik"),
                                     self.read_blackboard("planner"), cup, start)
            if plan is None:
                return self.fail("no_box_side", cup=cup)
            path = route(self.read_blackboard("planner"), start, plan.out)
            self.write_blackboard("box_plan", plan)
            park = (plan.park.x, plan.park.y, plan.park.yaw)
            self.stages = [BaseMove(path), BaseMove([park], vmax=0.12)]
            print(f"    [{self.name}] {cup}: box side {plan.park.side}, "
                  f"{len(path)} waypoint(s) then drive in")
        return self.run_stages()
