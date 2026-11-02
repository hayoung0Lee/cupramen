"""go_to_box: drive to the side of the box from which the cup can be picked.

Also here: choosing that side (box_side_poses lists the four candidates, plan_box_approach
checks them and picks one). pick_cup reads the chosen plan from blackboard["box_plan"].
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from behavioros.core.actions import action
from shelving.action_base import SimAction
from shelving.utils.common import (
    BASE_HALF_LENGTH, BOX_INTERIOR_HALF, BOX_STAND_HALF, GAP, HOVER, LIFT, PRESS, BasePose, Pose,
    avoid_floor_cups, backed_off, creep_clear, floor_cups, rot2d, route, solve_down, to_world,
)
from shelving.utils.kinematics import ArmIK, IKResult
from shelving.utils.motion import BaseMove
from shelving.utils.nav import GridPlanner
from sim.world import World

REACH_MARGIN = 0.03  # a box side "has reach to spare" if the arm reaches 3 cm farther too


def box_side_poses(box_pos, box_yaw, target_xy) -> list[BasePose]:
    """Candidate base poses on all four sides of the box stand, nearest side first.

    Each faces the box, its chassis front GAP from the stand, lined up with the target
    (within the box interior)."""
    local = rot2d(box_yaw).T @ (np.asarray(target_xy) - np.asarray(box_pos[:2]))
    hx, hy = BOX_STAND_HALF
    ix, iy = BOX_INTERIOR_HALF
    stand = BASE_HALF_LENGTH + GAP
    # (name, outward normal in box frame, half depth, lateral axis index, lateral limit)
    sides = [
        ("-y", np.array([0.0, -1.0]), hy, 0, ix),
        ("+y", np.array([0.0, 1.0]), hy, 0, ix),
        ("-x", np.array([-1.0, 0.0]), hx, 1, iy),
        ("+x", np.array([1.0, 0.0]), hx, 1, iy),
    ]
    out = []
    for name, n, h, lat_i, lat_lim in sides:
        p = n * (h + stand)
        p[lat_i] = np.clip(local[lat_i], -lat_lim, lat_lim)
        depth = h - float(n @ local)  # how far the target is inside from this face
        heading_local = np.arctan2(-n[1], -n[0])  # robot faces the box
        w = to_world(box_pos[:2], box_yaw, p)
        out.append((depth, BasePose(float(w[0]), float(w[1]), float(box_yaw + heading_local), name)))
    out.sort(key=lambda t: t[0])
    return [p for _, p in out]


@dataclass
class BoxPlan:
    cup: str
    park: BasePose          # where the base stops to pick
    out: Pose               # backed-off pose: drive here, then straight in
    q_hover: IKResult       # tool HOVER above the lid
    q_press: IKResult       # pad pressed onto the lid
    q_lift: IKResult        # cup LIFT above where it stood


def plan_box_approach(world: World, ik: ArmIK, planner: GridPlanner, cup: str,
                      start: Pose, skip_sides: tuple[str, ...] = ()) -> BoxPlan | None:
    """Choose the box side to pick `cup` from; None if no side works.

    Among the sides the base can drive to and pick from, prefer those with reach to spare
    (the arm still reaches with the cup REACH_MARGIN farther away), then the shortest drive
    from `start`. A side at the edge of reach is used only when no other side works.

    Why both: the side nearest the cup sent the robot around the whole box when that side
    faced away from it (3.26 m instead of 0.60 m, seed 0 second cup); the shortest drive
    alone picked sides at the edge of reach, where the ~1 cm the base stops off its target
    made pick_cup's IK fail.
    """
    info = world.info
    top = world.cup_top(cup)
    avoid_floor_cups(world, planner)
    floor = floor_cups(world)
    best, best_key = None, None
    for park in box_side_poses(info["box"]["pos"], info["box"]["yaw"], top[:2]):
        if park.side in skip_sides:
            continue
        path = planner.plan(start, backed_off(park))
        if path is None:
            continue
        b = (park.x, park.y, park.yaw)
        q = [solve_down(world, ik, b, top + [0, 0, dz]) for dz in (HOVER, -PRESS, LIFT)]
        if not all(q):
            continue
        if floor and not creep_clear(world, ik, backed_off(park), b, q[0].q):
            continue                                     # a dropped cup is in the way
        # Same targets with the base REACH_MARGIN farther back = the cup that much farther.
        spare = all(solve_down(world, ik, backed_off(park, REACH_MARGIN), top + [0, 0, dz])
                    for dz in (HOVER, -PRESS))
        key = (not spare, path_length(start, path))      # spare reach first, then distance
        if best_key is None or key < best_key:
            best, best_key = BoxPlan(cup, park, backed_off(park), *q), key
    return best


def path_length(start: Pose, path) -> float:
    pts = np.vstack([np.asarray(start[:2], float), np.asarray(path, float)[:, :2]])
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())


@action(
    "go_to_box",
    "Drive to a parking pose beside the box from which the cup can be picked.",
    reads=["world", "ik", "planner"],
    writes=["box_plan"],
)
class GoToBox(SimAction):
    """params: cup (name or "${current_cup}"), optional timeout_s.

    Picks a box side (plan_box_approach), drives to a point backed off from it along the
    A* route, then creeps straight in. Writes the chosen plan to blackboard["box_plan"] for
    the pick that follows. Only the base moves.
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
