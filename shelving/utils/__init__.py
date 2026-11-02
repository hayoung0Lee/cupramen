"""Helpers the Behavior Tree nodes in shelving/ use (no nodes here).

    common.py      what several nodes share: scene sizes, distances, collision checks
    ik.py          arm math, computes only: joint angles for a tool point (IK), tool point
                   for joint angles, "would this pose touch anything?"
    planner.py     map and paths for the base (A*)
    base_action.py basic moves several actions share: BaseMove, ToolLine, Wait (write
                   motor commands each tick; ToolLine asks ik.py where the joints go)
    teleop.py      stage 2: the operator and the keyboard
"""
