"""The ledger (장부): which cups are where and which slots are taken, and its tree nodes.

It lives on the blackboard:
  cups           {cup name: state}, see below
  slots          {slot number: cup name or None}
  attempts       {cup name: recoveries spent on it}
  blocked_slots  slots the robot could not get to (e.g. a dropped cup in front of them)
  current_cup    cup being handled now      (nodes read it as "${current_cup}")
  current_slot   slot it is going to        (nodes read it as "${current_slot}")
  all_cups_done  True when nothing is left to do (repeat_until stops on it)
  recoveries     how many times recover ran

It is always rebuilt from what the simulation shows (observe), never edited step by step:
after a failure the robot cannot trust what it *planned* (a dropped cup may have fallen back
into the box or onto the floor, a released cup may stand off its slot). Cup states:

  held       the pad holds it
  placed     stands on a slot (same test as the provided count_placed) -> slot filled
  in_box     inside the box walls (upright or not): can be picked (again)
  misplaced  near a slot but off it / tilted -> that slot is blocked, the cup is lost
  lost       anywhere else (floor): out of the arm's reach, given up
  skipped    still in the box, but failed too often (attempts >= MAX_ATTEMPTS)

Nodes: init_episode, select_next_task, is_holding, mark_placed (recover reuses LedgerAction).
"""

from __future__ import annotations

import numpy as np

from behavioros.core.actions import action
from shelving.action_base import FAILURE, SUCCESS, SimAction
from shelving.utils.common import BOX_INTERIOR_HALF, rot2d
from sim.world import World

MAX_ATTEMPTS = 3         # recoveries for one cup before it is skipped
SLOT_TOL_XY = 0.015      # count_placed defaults
SLOT_TOL_Z = 0.01
SLOT_MAX_TILT = 5.0
SLOT_BLOCK_XY = 0.07     # a cup this close to a slot centre (and on that board) blocks it


def tilt_deg(world: World, cup: str) -> float:
    up = world.d.body(cup).xmat.reshape(3, 3)[:, 2]
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))


def in_box(world: World, cup: str) -> bool:
    box = world.info["box"]
    local = rot2d(box["yaw"]).T @ (world.d.body(cup).xpos[:2] - np.asarray(box["pos"][:2]))
    return bool(abs(local[0]) <= BOX_INTERIOR_HALF[0] and abs(local[1]) <= BOX_INTERIOR_HALF[1])


def observe(world: World, attempts: dict[str, int] | None = None):
    """(cups, slots) from the current simulation state. See the module docstring."""
    attempts = attempts or {}
    d = world.d
    held = world.held_cup()
    slots: dict[int, str | None] = {k: None for k in range(1, 7)}
    cups: dict[str, str] = {}
    for cup in world.cup_names():
        if cup == held:
            cups[cup] = "held"
            continue
        pos = d.body(cup).xpos
        for k in range(1, 7):
            sp = d.site(f"slot_{k}").xpos
            dxy = float(np.linalg.norm(pos[:2] - sp[:2]))
            on_board = abs(pos[2] - sp[2]) <= 0.03
            if slots[k] is None and on_board and dxy <= SLOT_BLOCK_XY:
                ok = dxy <= SLOT_TOL_XY and abs(pos[2] - sp[2]) <= SLOT_TOL_Z \
                    and tilt_deg(world, cup) <= SLOT_MAX_TILT
                cups[cup] = "placed" if ok else "misplaced"
                slots[k] = cup
                break
        else:
            if in_box(world, cup):
                cups[cup] = "skipped" if attempts.get(cup, 0) >= MAX_ATTEMPTS else "in_box"
            else:
                cups[cup] = "lost"
    return cups, slots


def nothing_left(cups: dict[str, str], slots: dict[int, str | None],
                 blocked: set[int] = frozenset()) -> bool:
    """True when no cup can still be shelved: none in the box or held, or no empty slot
    the robot can get to."""
    todo = [c for c, s in cups.items() if s in ("in_box", "held")]
    usable = [k for k, v in slots.items() if v is None and k not in blocked]
    return not todo or not usable


# ---- tree nodes -------------------------------------------------------------------------
def print_ledger(bb):
    cups = ", ".join(f"{c.removeprefix('cup_')}={s}" for c, s in bb["cups"].items())
    slots = " ".join(f"{k}:{(v or '-').removeprefix('cup_')}" for k, v in bb["slots"].items())
    print(f"    ledger | cups: {cups}\n           | slots: {slots}")


class LedgerAction(SimAction):
    """An action that rebuilds the ledger from the scene (also used by recover)."""

    def refresh_ledger(self):
        cups, slots = observe(self.world, self.read_blackboard("attempts"))
        self.write_blackboard("cups", cups)
        self.write_blackboard("slots", slots)
        blocked = self.read_blackboard("blocked_slots", set())
        self.write_blackboard("all_cups_done", nothing_left(cups, slots, blocked))
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
