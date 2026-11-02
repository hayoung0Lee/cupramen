"""recover: after a failed cup, make the robot safe and look at the scene again."""

from __future__ import annotations

from behavioros.core.actions import action
from shelving.action_base import SUCCESS, Do
from shelving.ledger_nodes import LedgerAction
from shelving.motion import BaseMove


@action(
    "recover",
    "After a failed cup: stop, bring the arm back to the carry pose clear of the "
    "furniture, count the attempt, and rebuild the ledger from the scene.",
    reads=["world", "ik", "planner", "last_failure", "current_cup", "current_slot", "attempts"],
    writes=["cups", "slots", "attempts", "blocked_slots", "recoveries", "all_cups_done",
            "last_failure"],
)
class Recover(LedgerAction):
    """Runs as the fallback of "shelve one cup". Always SUCCESS, so the loop goes on and
    the next round re-plans from wherever the robot and the cups are now.

    Makes the robot safe first (SimAction.safe_stages in action_base.py). A held cup stays
    held: the next round sees it in the ledger and carries it on. If the slot could not be reached (no
    parking spot, blocked, timeout on the way), the slot goes on blocked_slots and the next
    round picks another one.
    """

    SLOT_FAILURES = ("no_shelf_spot", "blocked", "timeout")

    def execute(self):
        if self.stages is None:
            world = self.world
            BaseMove.stop(world)
            if self.read_blackboard("all_cups_done"):        # e.g. select found nothing
                return SUCCESS
            failure = self.read_blackboard("last_failure") or {}
            cup = self.read_blackboard("current_cup")
            print(f"    [{self.name}] after '{failure.get('reason')}' in "
                  f"'{failure.get('node')}' ({cup}): making safe, then re-planning")
            slot = None
            if failure.get("action") == "go_to_shelf" and failure.get("reason") in self.SLOT_FAILURES:
                slot = self.read_blackboard("current_slot")
            self.stages = self.safe_stages(world) + [Do(lambda w: self.finish(cup, slot))]
        return self.run_stages()

    def finish(self, cup, blocked_slot=None):
        if blocked_slot is not None:
            self.read_blackboard("blocked_slots").add(int(blocked_slot))
            print(f"    [{self.name}] slot_{blocked_slot} blocked, will use another slot")
        attempts = self.read_blackboard("attempts")
        if cup is not None:
            attempts[cup] = attempts.get(cup, 0) + 1
        self.write_blackboard("recoveries", self.read_blackboard("recoveries", 0) + 1)
        self.write_blackboard("last_failure", None)          # handled
        self.refresh_ledger()
