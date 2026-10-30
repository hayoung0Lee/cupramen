"""Planning helpers shared by the BT actions and the scripted demo.

They answer "where should the base stand and which joint poses will the arm need" for one
cup or one slot, using the current simulation state. They do not move anything.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from sim.kinematics import ArmIK, IKResult, level_reach_seed
from sim.nav import GridPlanner
from sim.placement import BasePose, box_side_poses, shelf_front_pose
from sim.world import World

HOVER = 0.03         # tool above lid / cup bottom above shelf board while moving in
PRESS = 0.003        # tool target this far into the lid so the pad really touches
LIFT = 0.10          # lift the cup this much out of the box
BACKOFF = 0.35       # base distance back from a parking pose before driving in / after leaving
CARRY_TIP = (0.35, 0.0, 1.05)  # tool tip in the base frame while driving with a cup: tucked in

Pose = tuple[float, float, float]


def backed_off(pose: BasePose, dist: float = BACKOFF) -> Pose:
    return (pose.x - dist * np.cos(pose.yaw), pose.y - dist * np.sin(pose.yaw), pose.yaw)


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


def clear_of_furniture(world: World, ik: ArmIK, base: Pose, q: dict[str, float]) -> bool:
    """No robot-vs-box/shelf contact at this base and joint pose (cups ignored)."""
    return not [c for c in ik.contacts_at(world.d, q, base) if "cup_" not in c[0] + c[1]]


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
    """First box side (nearest first) the base can reach and pick `cup` from, or None."""
    info = world.info
    top = world.cup_top(cup)
    for park in box_side_poses(info["box"]["pos"], info["box"]["yaw"], top[:2]):
        if park.side in skip_sides or planner.plan(start, backed_off(park)) is None:
            continue
        b = (park.x, park.y, park.yaw)
        q = [solve_down(world, ik, b, top + [0, 0, dz]) for dz in (HOVER, -PRESS, LIFT)]
        if all(q):
            return BoxPlan(cup, park, backed_off(park), *q)
    return None


@dataclass
class ShelfPlan:
    slot: int
    park: BasePose
    out: Pose
    q_hover: IKResult       # cup bottom HOVER above the board (enter / leave pose)
    q_place: IKResult       # cup bottom just above the board


def plan_shelf_approach(world: World, ik: ArmIK, planner: GridPlanner, slot: int,
                        cup_h: float, start: Pose) -> ShelfPlan | None:
    """Parking in front of `slot`. If the spot to back off to is blocked (e.g. the box
    stands close), back off less, then try shifting toward the shelf centre and swinging
    arm1 sideways. A candidate must be reachable and collision-free both where the arm
    unfolds (backed off) and over the slot."""
    info, d = world.info, world.d
    slot_xyz = d.site(f"slot_{slot}").xpos.copy()
    sh_pos, sh_yaw = info["shelf"]["pos"], info["shelf"]["yaw"]
    along = np.array([np.cos(sh_yaw), np.sin(sh_yaw)])          # shelf +x (its width)
    lateral = float((np.asarray(sh_pos[:2]) - slot_xyz[:2]) @ along)
    for shift in (0.0, 0.125, 0.25):
        park = shelf_front_pose(sh_pos, sh_yaw, slot_xyz[:2] + along * np.clip(lateral, -shift, shift))
        s = (park.x, park.y, park.yaw)
        q_hover = solve_down(world, ik, s, slot_xyz + [0, 0, cup_h + HOVER])
        q_place = solve_down(world, ik, s, slot_xyz + [0, 0, cup_h + 0.003])
        if not (q_hover and q_place and clear_of_furniture(world, ik, s, q_hover.q)):
            continue
        for backoff in (BACKOFF, 0.25, 0.15):
            out = backed_off(park, backoff)
            if clear_of_furniture(world, ik, out, q_hover.q) and planner.plan(start, out):
                return ShelfPlan(slot, park, out, q_hover, q_place)
    return None


def carry_pose(world: World, ik: ArmIK) -> IKResult | None:
    """Arm tucked with a cup hanging from the pad; base-relative, so solved at the origin."""
    return solve_down(world, ik, (0.0, 0.0, 0.0), CARRY_TIP)
