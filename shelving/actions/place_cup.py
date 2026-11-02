"""place_cup and verify_placed: put the held cup down on the slot, then check it stands there."""

from __future__ import annotations

import numpy as np

from behavioros.core.actions import action
from shelving.actions.base import SUCCESS, Do, Guarded, SimAction, holding
from shelving.common import CARRY_TIP, HOVER, base_frame_point, solve_down
from shelving.motion import BaseMove, ToolLine, Wait


@action(
    "place_cup",
    "Lower the held cup onto the slot (correcting for where it sits on the pad), release, "
    "back away and fold the arm up outside the shelf.",
    reads=["world", "ik", "shelf_plan"],
)
class PlaceCup(SimAction):
    """params: cup, slot. Stops (fails with operator_takeover) before the release if the
    operator pressed t, so the person can finish (stage 2)."""

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
