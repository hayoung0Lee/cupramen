"""
Webots-specific examples for BehaviorOS
These examples are loaded when using Webots controller
"""

from behavioros.prompts import example


@example(
    name="webots_move_forward",
    request="move forward",
    response={
        "initial_values": {"move_speed": 1.0, "move_duration": 3.0},
        "tree": {"type": "move_forward", "name": "Move Forward"},
    },
    description="Basic forward movement in Webots",
    tags=["webots", "movement", "simple"],
)
def webots_move_forward():
    pass


@example(
    name="webots_turn_left",
    request="turn left for 2 seconds",
    response={
        "initial_values": {"turn_speed": 3.0, "turn_duration": 2.0},
        "tree": {"type": "turn_left", "name": "Turn Left"},
    },
    description="Turn robot left in place",
    tags=["webots", "movement", "turn"],
)
def webots_turn_left():
    pass


@example(
    name="webots_sequence_movement",
    request="move forward then turn right",
    response={
        "initial_values": {
            "move_speed": 5.0,
            "move_duration": 3.0,
            "turn_speed": 3.0,
            "turn_duration": 1.5,
        },
        "tree": {
            "type": "sequence",
            "name": "Forward and Turn",
            "children": [
                {"type": "move_forward", "name": "Move"},
                {"type": "turn_right", "name": "Turn"},
            ],
        },
    },
    description="Sequential movement actions",
    tags=["webots", "movement", "sequence"],
)
def webots_sequence_movement():
    pass


@example(
    name="webots_patrol",
    request="patrol pattern: forward, left, forward, right",
    response={
        "initial_values": {
            "move_speed": 4.0,
            "move_duration": 2.0,
            "turn_speed": 3.0,
            "turn_duration": 1.5,
        },
        "tree": {
            "type": "sequence",
            "name": "Patrol Route",
            "children": [
                {"type": "move_forward", "name": "Forward 1"},
                {"type": "turn_left", "name": "Left Turn"},
                {"type": "move_forward", "name": "Forward 2"},
                {"type": "turn_right", "name": "Right Turn"},
            ],
        },
    },
    description="Patrol movement pattern",
    tags=["webots", "movement", "patrol", "sequence"],
)
def webots_patrol():
    pass


@example(
    name="webots_stop",
    request="stop the robot",
    response={"tree": {"type": "stop", "name": "Stop Robot"}},
    description="Emergency stop",
    tags=["webots", "stop", "simple"],
)
def webots_stop():
    pass


@example(
    name="webots_dance",
    request="do a simple dance",
    response={
        "initial_values": {
            "move_speed": 3.0,
            "move_duration": 1.0,
            "turn_speed": 5.0,
            "turn_duration": 1.0,
        },
        "tree": {
            "type": "sequence",
            "name": "Dance Routine",
            "children": [
                {"type": "move_forward", "name": "Step 1"},
                {"type": "turn_left", "name": "Spin Left"},
                {"type": "move_forward", "name": "Step 2"},
                {"type": "turn_right", "name": "Spin Right"},
                {"type": "turn_right", "name": "Spin Right Again"},
                {"type": "stop", "name": "Finish"},
            ],
        },
    },
    description="Simple dance sequence",
    tags=["webots", "movement", "dance", "sequence"],
)
def webots_dance():
    pass
