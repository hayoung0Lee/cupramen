"""What several actions share: scene sizes, distances, and checks.

Layer: shelving, shared helpers. Computes only; never moves anything. Each Behavior Tree
node file in shelving/ keeps the helpers only it uses; what two or more need is here.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from shelving.utils.ik import ArmIK, IKResult, level_reach_seed
from sim.world import PREFIX, World

if TYPE_CHECKING:
    from shelving.utils.planner import GridPlanner

# ---- scene sizes (from the provided scene files) --------------------------------------
BASE_HALF_LENGTH = 0.29           # chassis half size along robot x (youbot_lift.xml)
BOX_STAND_HALF = (0.253, 0.198)   # box stand half sizes (x, y), box.xml
BOX_INTERIOR_HALF = (0.225, 0.17)
SHELF_HALF_DEPTH = 0.20           # shelf 900 x 400 mm, front = -y
SHELF_HALF_WIDTH = 0.45

# ---- distances the actions use ---------------------------------------------------------
# Chassis front edge to the box stand / shelf front. The youBot arm base and first link
# stick out ~5 cm past the chassis front; with 3 cm they sat 2 cm inside the box stand and
# the shelf's upper board.
GAP = 0.06
HOVER = 0.03         # tool above lid / cup bottom above shelf board while moving in
PRESS = 0.003        # tool target this far into the lid so the pad really touches
LIFT = 0.15          # lift the cup this much: its bottom must clear the neighbours' lids (cups are ~0.11 m tall)
BACKOFF = 0.35       # base distance back from a parking pose before driving in / after leaving
READY_REACH = 0.40   # base centre to tool when the arm waits outside the shelf
CARRY_TIP = (0.40, 0.0, 1.10)  # tool tip in the base frame while driving with a cup: tucked in, cup clear of the arm

Pose = tuple[float, float, float]


@dataclass
class BasePose:
    x: float
    y: float
    yaw: float
    side: str       # which box side ("+x", "-y", ...) or "front" of the shelf


def rot2d(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def to_world(origin_xy, yaw, local_xy) -> np.ndarray:
    """A point given in the frame of something at origin_xy turned by yaw, in world xy."""
    return np.asarray(origin_xy) + rot2d(yaw) @ np.asarray(local_xy)


def backed_off(pose: BasePose, dist: float = BACKOFF) -> Pose:
    return (pose.x - dist * np.cos(pose.yaw), pose.y - dist * np.sin(pose.yaw), pose.yaw)


def base_frame_point(base: Pose, p) -> np.ndarray:
    """World position of point p = (forward, left, up) given in the frame of a base at `base`."""
    c, s = np.cos(base[2]), np.sin(base[2])
    return np.array([base[0] + c * p[0] - s * p[1], base[1] + s * p[0] + c * p[1], p[2]])


def route(planner: GridPlanner, start: Pose, goal: Pose) -> list[Pose] | None:
    """A* waypoints from start to goal, every waypoint carrying the goal heading."""
    pts = planner.plan(start, goal)
    if pts is None:
        return None
    return [(float(x), float(y), goal[2]) for x, y in pts[1:]]


def solve_down(world: World, ik: ArmIK, base: Pose, target) -> IKResult | None:
    """Tool-down IK as if the base stood at `base`; None if it does not converge."""
    res = ik.solve(world.d, np.asarray(target, float), seed=level_reach_seed(target), base=base)
    return res if res.ok else None


# ---- collisions, and cups dropped on the floor -----------------------------------------
FLOOR_Z = 0.3            # a cup whose origin is below this lies on the floor
FLOOR_CUP_CLEARANCE = 0.42  # grid radius around it: chassis half-diagonal 0.345 + cup 0.07.
                            # A cup on the floor only meets the chassis, not the carried cup,
                            # so the full ROBOT_RADIUS (0.45 + cup) would block too much.
SWEEP_MARGIN = 0.01   # keep this far from everything along a checked motion


def floor_cups(world: World) -> list[str]:
    """Cups lying on the floor (dropped): obstacles for the base, out of the arm's reach."""
    return [c for c in world.cup_names() if world.d.body(c).xpos[2] < FLOOR_Z]


def avoid_floor_cups(world: World, planner: GridPlanner) -> None:
    """Put the dropped cups into the planner's grid, so routes go around them."""
    planner.set_obstacles([world.d.body(c).xpos[:2] for c in floor_cups(world)], FLOOR_CUP_CLEARANCE)


def clear_of_furniture(world: World, ik: ArmIK, base: Pose, q: dict[str, float],
                       margin: float = 0.0, held_cup: bool = True) -> bool:
    """No robot contact with the box, the shelf or a cup on the floor at this base and joint
    pose (nothing closer than `margin`). Cups in the box or on the shelf are ignored
    (touching them is the job). The cup in the hand counts as part of the robot unless
    held_cup=False (a person putting it down on the board is meant to touch the board)."""
    floor = floor_cups(world)

    def counts(name):
        return "cup_" not in name or any(name.startswith(c) for c in floor)

    # contacts_at gives (robot or held cup, other)
    return not any(counts(other) for mine, other in ik.contacts_at(world.d, q, base, margin)
                   if held_cup or mine.startswith(PREFIX))


def creep_clear(world: World, ik: ArmIK, out: Pose, park: Pose, q: dict[str, float],
                step: float = 0.02) -> bool:
    """The straight drive from `out` in to `park` with the arm at q touches nothing."""
    n = max(1, int(np.ceil(np.hypot(park[0] - out[0], park[1] - out[1]) / step)))
    return all(clear_of_furniture(world, ik, tuple(np.add(out, np.subtract(park, out) * k / n)), q,
                                  SWEEP_MARGIN)
               for k in range(n + 1))
