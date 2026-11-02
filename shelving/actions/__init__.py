"""BehaviorOS actions (Behavior Tree nodes) for the cup-shelving task.

Layer: shelving — the nodes decide what to do, in which order, and check that it worked
(SUCCESS / FAILURE with a reason). One file per node (or pair), with the helpers only that
node uses; what several share is in shelving/common.py.

    base.py          the frame: SimAction, stages (Check / Do / Guarded), safe_stages
    go_to_box.py     go_to_box      + choosing the box side
    pick_cup.py      pick_cup
    go_to_shelf.py   go_to_shelf    + choosing where to stand at the shelf
    place_cup.py     place_cup, verify_placed
    ledger_nodes.py  init_episode, select_next_task, is_holding, mark_placed
    recover.py       recover
    takeover.py      stage 2: check_place_ready, operator_available, request_takeover,
                     human_control, back_away

They use:
  move     shelving/motion.py   BaseMove, ToolLine, ... advanced a little each tick
  act      sim/world.py         instant buttons: suction_on / suction_off, queries
  observe  shelving/ledger.py   which cups are where
e.g. pick_cup: solve arm poses -> lower the arm (motion) -> world.suction_on() -> check it
returned this cup, else fail("seal_failed") -> lift (motion) -> check still held.

Importing this package registers every node with BehaviorOS (the @action decorators), so
a JSON tree can name them by "type".
"""

from shelving.actions import (  # noqa: F401
    go_to_box, go_to_shelf, ledger_nodes, pick_cup, place_cup, recover, takeover,
)
