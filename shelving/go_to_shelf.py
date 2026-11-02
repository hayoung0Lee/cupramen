"""go_to_shelf: carry the held cup to a parking pose in front of the slot and creep in.

Also here: choosing that pose (shelf_front_pose is the plain candidate, plan_shelf_approach
checks candidates and picks one). place_cup reads the plan from blackboard["shelf_plan"].
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from behavioros.core.actions import action
from shelving.action_base import Guarded, SimAction, holding
from shelving.common import (
    BACKOFF, BASE_HALF_LENGTH, CARRY_TIP, GAP, HOVER, READY_REACH, SHELF_HALF_DEPTH, SWEEP_MARGIN,
    BasePose, Pose, avoid_floor_cups, backed_off, base_frame_point, clear_of_furniture,
    creep_clear, floor_cups, rot2d, route, solve_down, to_world,
)
from shelving.kinematics import ArmIK, IKResult
from shelving.motion import BaseMove, ToolLine
from shelving.nav import GridPlanner
from sim.world import World

# Standoffs to try in front of the shelf (scripts/check_reach.py). Close in, the long arm has
# to fold and its elbow hits the board above; further out it reaches in nearly straight.
SHELF_GAPS = (0.03, 0.10, 0.15, 0.20, 0.25, 0.30)


def shelf_front_pose(shelf_pos, shelf_yaw, target_xy, gap: float = GAP) -> BasePose:
    """Base pose in front of the shelf, aligned with the slot, `gap` from the shelf front."""
    local = rot2d(shelf_yaw).T @ (np.asarray(target_xy) - np.asarray(shelf_pos[:2]))
    p = np.array([local[0], -(SHELF_HALF_DEPTH + BASE_HALF_LENGTH + gap)])
    w = to_world(shelf_pos[:2], shelf_yaw, p)
    return BasePose(float(w[0]), float(w[1]), float(shelf_yaw + np.pi / 2), f"front+{gap:.2f}")


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
        xy = slot_xyz[:2] + rot2d(turn) @ (np.array([p.x, p.y]) - slot_xyz[:2])
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
