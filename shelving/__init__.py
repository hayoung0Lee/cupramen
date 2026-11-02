"""The cup-shelving robot: Behavior Tree nodes and what they use.

Behavior Tree nodes (BehaviorOS actions), one file per node or pair, each with the helpers
only it uses:
    sim_action.py    the frame every node is built on: SimAction, stages, safe_stages
    go_to_box.py     go_to_box      + choosing the box side
    pick_cup.py      pick_cup
    go_to_shelf.py   go_to_shelf    + choosing where to stand at the shelf
    place_cup.py     place_cup, verify_placed
    ledger.py        the ledger (which cups are where) + init_episode, select_next_task,
                     is_holding, mark_placed
    recover.py       recover
    takeover.py      stage 2: check_place_ready, operator_available, request_takeover,
                     human_control, back_away

What they use: shelving/utils/ (common, ik, planner, base_action, teleop;
see its __init__.py).

The nodes decide what to do, in which order, and check that it worked (SUCCESS / FAILURE
with a reason); they don't compute geometry or drive motors themselves. E.g. pick_cup:
solve arm poses (ik) -> lower the arm (motion) -> world.suction_on() -> check it
returned this cup, else fail("seal_failed") -> lift (motion) -> check still held.

Importing this package registers every node with BehaviorOS (the @action decorators), so a
JSON tree can name them by "type".
"""

from shelving import (  # noqa: F401
    go_to_box, go_to_shelf, ledger, pick_cup, place_cup, recover, takeover,
)
