"""recover: after a failed cup, make the robot safe and look at the scene again."""

from __future__ import annotations

from behavioros.core.actions import action
from shelving.sim_action import SUCCESS, Do
from shelving.ledger import LedgerAction
from shelving.utils.base_action import BaseMove


@action(
    "recover",
    "After a failed cup: stop, bring the arm back to the carry pose clear of the "
    "furniture, count the attempt, and rebuild the ledger from the scene.",
    reads=["last_failure", "current_cup", "current_slot", "attempts"],
    writes=["cups", "slots", "attempts", "blocked_slots", "recoveries", "all_cups_done",
            "last_failure"],
)
class Recover(LedgerAction):
    """Runs as the fallback of "shelve one cup". Always SUCCESS, so the loop goes on and
    the next round re-plans from wherever the robot and the cups are now.

    When it runs: any node inside "shelve one cup" failed (after its own retries, if it
    has them) and left the reason on the blackboard as last_failure, e.g.
      {"node": "Carry to shelf", "action": "go_to_shelf", "reason": "cup_dropped", ...}

    What it does, as one node with its own list of stages (like every SimAction):
      first tick   stop the base, then write the list once:
                     safe_stages(world)      moves that depend on where the robot is now
                                             (sim_action.py): tool over the box -> lift it
                                             straight up; near the box or shelf -> back the
                                             base straight out; then arm to the carry pose
                     Do(finish)              the bookkeeping, run when the moves are done
                                             (a lambda, so it does not run while writing)
      every tick   run_stages() moves the first stage on by one tick and drops it when
                   done -> RUNNING; an instant stage (Do) runs in the same tick; the list
                   empty -> SUCCESS

    finish (the bookkeeping):
      - attempts[cup] += 1. At MAX_ATTEMPTS (ledger.py) a cup still in the box is
        "skipped", so one cup cannot fail forever.
      - go_to_shelf could not reach the slot (no parking spot, blocked, timeout on the
        way) -> the slot goes on blocked_slots and the next round picks another one.
      - recoveries += 1, last_failure cleared.
      - refresh_ledger: look at the scene again for every cup and slot, instead of
        trusting what was planned. That decides the next round:
          cup still on the pad      -> "held": the next round carries it on
          cup fell back into box    -> "in_box": picked again from where it lies now
          cup on the floor          -> "lost": out of reach, given up
          cup released off its slot -> "misplaced": that slot is blocked
        and all_cups_done goes True when nothing is left, which ends the loop.
    """

    SLOT_FAILURES = ("no_shelf_spot", "blocked", "timeout")

    def execute(self):
        if self.stages is None:                              # first tick: plan the stages
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
        return self.run_stages()                             # every tick: one step on

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
