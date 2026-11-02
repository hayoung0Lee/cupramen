"""Planning helpers shared by the BT actions and the scripted demo.

Layer: shelving — task planning. Combines the sim calculations (placement, nav, kinematics)
for one cup or one slot. Computes only: doesn't move anything or decide the order of work.

They answer "where should the base stand and which joint poses will the arm need" for one
cup or one slot, using the current simulation state. They do not move anything.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from shelving.kinematics import ArmIK, IKResult, level_reach_seed
from shelving.nav import GridPlanner
from shelving.placement import BasePose, _rot, box_side_poses, shelf_front_pose
from sim.world import PREFIX, World

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


FLOOR_Z = 0.3            # a cup whose origin is below this lies on the floor
FLOOR_CUP_CLEARANCE = 0.42  # grid radius around it: chassis half-diagonal 0.345 + cup 0.07.
                            # A cup on the floor only meets the chassis, not the carried cup,
                            # so the full ROBOT_RADIUS (0.45 + cup) would block too much.


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


SWEEP_MARGIN = 0.01   # keep this far from everything along a checked motion


def creep_clear(world: World, ik: ArmIK, out: Pose, park: Pose, q: dict[str, float],
                step: float = 0.02) -> bool:
    """The straight drive from `out` in to `park` with the arm at q touches nothing."""
    n = max(1, int(np.ceil(np.hypot(park[0] - out[0], park[1] - out[1]) / step)))
    return all(clear_of_furniture(world, ik, tuple(np.add(out, np.subtract(park, out) * k / n)), q,
                                  SWEEP_MARGIN)
               for k in range(n + 1))


def tool_line(world: World, ik: ArmIK, base: Pose, q_from: dict[str, float],
              tip_to, n: int = 8) -> dict[str, float] | None:
    """Replay a ToolLine from the pose q_from to tip_to with the base at `base`: IK along
    the line seeded from the previous step, as ToolLine does. Returns the joints it ends
    with, or None if a pose on the way touches something.

    The end joints matter: the arm can put the tool at one point with several poses (more
    lift and a straighter arm, or less lift and a raised elbow), and ToolLine ends on
    whichever its steps lead to, not necessarily on the pose a plan solved for."""
    a = ik.tool_tip_at(world.d, q_from, base)
    q = q_from
    for k in range(1, n + 1):
        q = ik.solve(world.d, a + (np.asarray(tip_to) - a) * k / n, seed=q, base=base).q
        if not clear_of_furniture(world, ik, base, q, SWEEP_MARGIN):
            return None
    return q


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
    unfolds (backed off) and over the slot.

    Only if all of those are blocked (a dropped cup lying in front of the shelf), try the
    wider set: shifted up to 0.35 m either way and turned up to ~30 deg, nearest first."""
    info, d = world.info, world.d
    slot_xyz = d.site(f"slot_{slot}").xpos.copy()
    sh_pos, sh_yaw = info["shelf"]["pos"], info["shelf"]["yaw"]
    along = np.array([np.cos(sh_yaw), np.sin(sh_yaw)])          # shelf +x (its width)
    lateral = float((np.asarray(sh_pos[:2]) - slot_xyz[:2]) @ along)
    avoid_floor_cups(world, planner)
    floor = floor_cups(world)

    def pose(offset: float, turn: float = 0.0) -> Pose:
        """In front of the slot, `offset` along the shelf, turned `turn` about the slot."""
        p = shelf_front_pose(sh_pos, sh_yaw, slot_xyz[:2] + along * offset)
        xy = slot_xyz[:2] + _rot(turn) @ (np.array([p.x, p.y]) - slot_xyz[:2])
        return (float(xy[0]), float(xy[1]), float(p.yaw + turn))

    usual = [pose(float(np.clip(lateral, -shift, shift))) for shift in (0.0, 0.125, 0.25)]
    wider = sorted(((off, turn) for off in np.arange(-0.35, 0.351, 0.05)
                    for turn in (0.0, 0.25, -0.25, 0.5, -0.5)),
                   key=lambda c: abs(c[0]) + 0.5 * abs(c[1]))
    candidates = usual + ([pose(off, turn) for off, turn in wider] if floor else [])
    for s in candidates:
        park = BasePose(*s, "front")
        q_hover = solve_down(world, ik, s, slot_xyz + [0, 0, cup_h + HOVER])
        q_place = solve_down(world, ik, s, slot_xyz + [0, 0, cup_h + 0.003])
        if not (q_hover and q_place and clear_of_furniture(world, ik, s, q_hover.q)):
            continue
        for backoff in (BACKOFF, 0.25, 0.15):
            out = backed_off(park, backoff)
            if not (clear_of_furniture(world, ik, out, q_hover.q) and planner.plan(start, out)):
                continue
            if floor and not creep_clear(world, ik, out, s, q_hover.q):
                continue                                 # a dropped cup is in the way
            heading = np.array([np.cos(out[2]), np.sin(out[2])])
            ready_xy = np.asarray(out[:2]) + READY_REACH * heading
            hover_z = float(slot_xyz[2] + cup_h + HOVER)
            q_ready = solve_down(world, ik, out, [ready_xy[0], ready_xy[1], hover_z])
            if not (q_ready and clear_of_furniture(world, ik, out, q_ready.q)):
                continue
            # With a dropped cup about, unusual parking poses come into play: check the
            # whole way in (arm down outside, reach in level, drive in), not just its ends.
            if floor:
                q_carry = solve_down(world, ik, out, base_frame_point(out, CARRY_TIP))
                q1 = q_carry and tool_line(world, ik, out, q_carry.q,
                                           ik.tool_tip_at(d, q_ready.q, out))
                q2 = q1 and tool_line(world, ik, out, q1, ik.tool_tip_at(d, q_hover.q, out))
                if not (q2 and creep_clear(world, ik, out, s, q2)):
                    continue
            return ShelfPlan(slot, park, out, q_hover, q_place, q_ready)
    return None


def carry_pose(world: World, ik: ArmIK) -> IKResult | None:
    """Arm tucked with a cup hanging from the pad; base-relative, so solved at the origin."""
    return solve_down(world, ik, (0.0, 0.0, 0.0), CARRY_TIP)
