"""Fault injection: make things go wrong on purpose, to exercise the recovery branches.

    python scripts/run_bt.py trees/b5_recovery.json --seed 0 --inject seal:0.3
    python scripts/run_bt.py trees/b5_recovery.json --seed 0 --inject seal-fail:1,drop-lift:3,drop-carry:4

N is the cup's turn: the N-th cup the tree chose (a cup chosen again after a failure keeps
its number). Each fault fires once, on the first try at that cup.

  seal:P          every suction attempt fails with probability P (dented or dusty lid).
                  Seeded from the episode seed, so a run is reproducible.
  seal-fail:N     the first suction attempt on cup N fails.
  drop-lift:N     cup N slips off the pad while being lifted: it falls back into the box,
                  somewhere physics decides, and has to be picked again from there.
  drop-carry:N    cup N slips off the pad on the way to the shelf: it falls to the floor.

The robot is not told about any of this; it only sees the consequences.
"""

from __future__ import annotations

import numpy as np

from sim.world import World

DROP_LIFT_AFTER_S = 1.0  # after the seal: mid-lift
DROP_CARRY_AFTER_S = 3.0  # after go_to_shelf starts: driving
KINDS = ("seal", "seal-fail", "drop-lift", "drop-carry")


class FaultInjector:
    def __init__(self, spec: str, world: World, seed: int):
        self.world, self.events = world, []
        self.seal_p = 0.0
        self.planned: set[tuple[str, int]] = set()   # (kind, cup turn) still to fire
        for item in filter(None, (spec or "").split(",")):
            kind, _, arg = item.partition(":")
            if kind not in KINDS:
                raise ValueError(f"unknown fault '{kind}' (one of {', '.join(KINDS)})")
            if kind == "seal":
                self.seal_p = float(arg)
            else:
                self.planned.add((kind, int(arg)))
        self.rng = np.random.default_rng(1000 + seed)
        self.order: list[str] = []      # cups in the order the tree chose them
        self.current = None             # blackboard current_cup at the last tick
        self.drop_at = None             # (sim time, cup)
        self.last_leaf = None
        self._suction_on, world.suction_on = world.suction_on, self.suction_on

    def turn(self, cup) -> int | None:
        return self.order.index(cup) + 1 if cup in self.order else None

    def fire(self, kind, cup) -> bool:
        """True (once) if fault `kind` is planned for this cup's turn."""
        key = (kind, self.turn(cup))
        if key in self.planned:
            self.planned.discard(key)
            return True
        return False

    def log(self, what):
        self.events.append((round(float(self.world.d.time), 1), what))
        print(f"    !! fault t={self.world.d.time:.1f}s: {what}")

    def suction_on(self):
        cup = self.current
        if self.fire("seal-fail", cup) or (self.seal_p and self.rng.random() < self.seal_p):
            self.log(f"suction did not seal on {cup}")
            return None
        held = self._suction_on()
        if held is not None and self.fire("drop-lift", held):
            self.drop_at = (self.world.d.time + DROP_LIFT_AFTER_S, held)
        return held

    def on_tick(self, running_leaf, blackboard):
        """Call after every tree tick with the running action node (or None)."""
        w, d = self.world, self.world.d
        self.current = blackboard.get("current_cup")
        if self.current is not None and self.current not in self.order:
            self.order.append(self.current)
        if running_leaf is not self.last_leaf:
            self.last_leaf = running_leaf
            if type(running_leaf).__name__ == "GoToShelf" and w.held_cup() \
                    and self.fire("drop-carry", w.held_cup()):
                self.drop_at = (d.time + DROP_CARRY_AFTER_S, w.held_cup())
        if self.drop_at is not None and d.time >= self.drop_at[0]:
            cup = self.drop_at[1]
            self.drop_at = None
            if w.held_cup() == cup:
                self.log(f"pad lost {cup} (cup {self.turn(cup)})")
                w.suction_off()

