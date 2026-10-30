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
LIFT = 0.15          # lift the cup this much: its bottom must clear the neighbours' lids (cups are ~0.11 m tall)
BACKOFF = 0.35       # base distance back from a parking pose before driving in / after leaving
READY_REACH = 0.40   # base centre to tool when the arm waits outside the shelf
REACH_MARGIN = 0.03  # a box side "has reach to spare" if the arm reaches 3 cm farther too
CARRY_TIP = (0.40, 0.0, 1.10)  # tool tip in the base frame while driving with a cup: tucked in, cup clear of the arm

Pose = tuple[float, float, float]


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


@dataclass
class ShelfPlan:
    slot: int
    park: BasePose
    out: Pose
    q_hover: IKResult       # cup bottom HOVER above the board (enter / leave pose)
    q_place: IKResult       # cup bottom just above the board
    q_ready: IKResult       # at `out`: tool at the hover height but pulled back, outside the
                            # shelf. Change height only here, then reach in / pull out level;
                            # rising or dropping with the arm under a board scrapes it.


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
            if not (clear_of_furniture(world, ik, out, q_hover.q) and planner.plan(start, out)):
                continue
            heading = np.array([np.cos(out[2]), np.sin(out[2])])
            ready_xy = np.asarray(out[:2]) + READY_REACH * heading
            hover_z = float(slot_xyz[2] + cup_h + HOVER)
            q_ready = solve_down(world, ik, out, [ready_xy[0], ready_xy[1], hover_z])
            if q_ready and clear_of_furniture(world, ik, out, q_ready.q):
                return ShelfPlan(slot, park, out, q_hover, q_place, q_ready)
    return None


def carry_pose(world: World, ik: ArmIK) -> IKResult | None:
    """Arm tucked with a cup hanging from the pad; base-relative, so solved at the origin."""
    return solve_down(world, ik, (0.0, 0.0, 0.0), CARRY_TIP)
