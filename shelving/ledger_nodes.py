"""Ledger nodes: start it, choose the next cup and slot, check the hand, record a placement.

The ledger lives on the blackboard:
  cups           {cup name: state}, states in shelving/ledger.py
  slots          {slot number: cup name or None}
  attempts       {cup name: recoveries spent on it}
  blocked_slots  slots the robot could not get to (e.g. a dropped cup in front of them)
  current_cup    cup being handled now      (nodes read it as "${current_cup}")
  current_slot   slot it is going to        (nodes read it as "${current_slot}")
  all_cups_done  True when nothing is left to do (repeat_until stops on it)
  recoveries     how many times recover ran
It is always rebuilt from the simulation (ledger.observe), never edited by hand: after a
failure the plan says little about where the cups really are.
"""

from __future__ import annotations

import numpy as np

from behavioros.core.actions import action
from shelving import ledger
from shelving.action_base import FAILURE, SUCCESS, SimAction


def print_ledger(bb):
    cups = ", ".join(f"{c.removeprefix('cup_')}={s}" for c, s in bb["cups"].items())
    slots = " ".join(f"{k}:{(v or '-').removeprefix('cup_')}" for k, v in bb["slots"].items())
    print(f"    ledger | cups: {cups}\n           | slots: {slots}")


class LedgerAction(SimAction):
    """An action that rebuilds the ledger from the scene (also used by recover)."""

    def refresh_ledger(self):
        cups, slots = ledger.observe(self.world, self.read_blackboard("attempts"))
        self.write_blackboard("cups", cups)
        self.write_blackboard("slots", slots)
        blocked = self.read_blackboard("blocked_slots", set())
        self.write_blackboard("all_cups_done", ledger.nothing_left(cups, slots, blocked))
        print_ledger(self.blackboard)


@action(
    "init_episode",
    "Start the ledger from what the scene shows (normally: every cup in the box).",
    reads=["world"],
    writes=["cups", "slots", "attempts", "blocked_slots", "recoveries", "all_cups_done"],
)
class InitEpisode(LedgerAction):
    def execute(self):
        self.write_blackboard("attempts", {})
        self.write_blackboard("blocked_slots", set())
        self.write_blackboard("recoveries", 0)
        self.refresh_ledger()
        return SUCCESS


@action(
    "select_next_task",
    "Choose the next cup (the one in the hand, else the nearest still in the box) and "
    "slot (lowest empty).",
    reads=["world", "cups", "slots", "blocked_slots"],
    writes=["current_cup", "current_slot", "all_cups_done"],
)
class SelectNextTask(SimAction):
    def execute(self):
        world, cups = self.world, self.read_blackboard("cups")
        held = [c for c, s in cups.items() if s == "held"]
        in_box = [c for c, s in cups.items() if s == "in_box"]
        blocked = self.read_blackboard("blocked_slots", set())
        empty = [k for k, v in self.read_blackboard("slots").items()
                 if v is None and k not in blocked]
        if not (held or in_box) or not empty:
            self.write_blackboard("all_cups_done", True)
            return self.fail("nothing_left", cups_in_box=len(in_box), empty_slots=len(empty))
        base = np.array(world.base_pose()[:2])
        cup = held[0] if held else \
            min(in_box, key=lambda c: float(np.linalg.norm(world.cup_top(c)[:2] - base)))
        slot = min(empty)
        self.write_blackboard("current_cup", cup)
        self.write_blackboard("current_slot", slot)
        print(f"    [{self.name}] next: {cup} -> slot_{slot}")
        return SUCCESS


@action(
    "is_holding",
    "Condition: SUCCESS if the pad holds the cup, else FAILURE (not an error).",
    reads=["world"],
)
class IsHolding(SimAction):
    """params: cup. Lets a fallback skip fetching a cup that is already in the hand,
    e.g. after recovering from a failure at the shelf."""

    def execute(self):
        return SUCCESS if self.world.held_cup() == self.param("cup") else FAILURE


@action(
    "mark_placed",
    "Look at the scene again after a verified placement; set all_cups_done when finished.",
    reads=["world", "attempts"],
    writes=["cups", "slots", "all_cups_done"],
)
class MarkPlaced(LedgerAction):
    def execute(self):
        self.refresh_ledger()
        return SUCCESS
