"""pick_cup: at the box, lower the pad onto the lid, seal, lift, back away, tuck the arm."""

from __future__ import annotations

from behavioros.core.actions import action
from shelving.action_base import Check, Guarded, SimAction, holding
from shelving.utils.common import CARRY_TIP, HOVER, LIFT, PRESS, solve_down
from shelving.utils.ik import ArmIK, IKResult
from shelving.utils.motion import BaseMove, JointMove, Wait
from sim.world import World


def carry_pose(world: World, ik: ArmIK) -> IKResult | None:
    """Arm tucked with a cup hanging from the pad; base-relative, so solved at the origin."""
    return solve_down(world, ik, (0.0, 0.0, 0.0), CARRY_TIP)


@action(
    "pick_cup",
    "At the box: lower the pad onto the cup lid, seal, lift, back away and tuck the arm.",
    reads=["box_plan"],
)
class PickCup(SimAction):
    """params: cup.

    Arm poses are solved again from the cup's *current* position (it may have moved since
    go_to_box planned) and from where the base actually stopped.
    """

    def execute(self):
        if self.stages is None:
            world, cup = self.world, self.param("cup")
            plan = self.read_blackboard("box_plan")
            if plan is None or plan.cup != cup:
                return self.fail("no_box_plan", cup=cup)
            base, top = world.base_pose(), world.cup_top(cup)
            q = {name: solve_down(world, self.ik, base, top + [0, 0, dz])
                 for name, dz in (("hover", HOVER), ("press", -PRESS), ("lift", LIFT))}
            q["carry"] = carry_pose(world, self.ik)
            missing = [k for k, v in q.items() if v is None]
            if missing:
                return self.fail("ik_failed", cup=cup, poses=missing)

            def seal(w):
                held = w.suction_on()
                if held != cup:
                    w.suction_off()
                    return "seal_failed", {"cup": cup, "sealed_on": held}
                return None

            def lifted(w):
                rise = float(w.cup_top(cup)[2] - top[2])
                return None if rise > LIFT / 2 else ("not_lifted", {"cup": cup, "rise_m": round(rise, 3)})

            self.stages = [
                JointMove(q["hover"].q, 2.5),
                JointMove(q["press"].q, 1.5),
                Check(seal),
                Wait(0.3),
                Guarded(JointMove(q["lift"].q, 1.5), holding(cup)),
                Check(lifted),
                Guarded(BaseMove([plan.out], vmax=0.12), holding(cup)),
                Guarded(JointMove(q["carry"].q, 2.0), holding(cup)),
            ]
        return self.run_stages()
