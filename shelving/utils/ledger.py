"""The task ledger, rebuilt from what the simulation actually shows.

Layer: shelving — task state. Reads the scene only.

After a failure the robot cannot trust what it *planned*: a dropped cup may have fallen
back into the box or onto the floor, a released cup may stand off its slot. So instead of
editing the ledger step by step, look at every cup and classify it:

  held       the pad holds it
  placed     stands on a slot (same test as the provided count_placed) -> slot filled
  in_box     inside the box walls (upright or not): can be picked (again)
  misplaced  near a slot but off it / tilted -> that slot is blocked, the cup is lost
  lost       anywhere else (floor): out of the arm's reach, given up
  skipped    still in the box, but failed too often (attempts >= MAX_ATTEMPTS)
"""

from __future__ import annotations

import numpy as np

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
