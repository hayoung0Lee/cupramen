"""
YouBot prompt examples with resolver templates
"""

from behavioros.prompts import example


@example(
    name="youbot_pick_color",
    request="pick up the color can",
    response={
        "tree": {
            "type": "pick_can",
            "name": "Pick Color Can",
            "nearest_can": "${nearest_color_can}",  # Resolver template
        }
    },
    description="Color can pickup with resolver",
    tags=["youbot", "pick", "resolver"],
)
def pick_color_example():
    pass


@example(
    name="youbot_visit_color",
    request="pick up the color can",
    response={
        "tree": {
            "type": "visit_can",
            "name": "Visit Color Can",
            "nearest_can": "${nearest_color_can}",  # Resolver template
        }
    },
    description="Color can visit with resolver",
    tags=["youbot", "visit", "resolver"],
)
def visit_color_example():
    pass


# --- NEW: Approach work area ---
@example(
    name="youbot_approach_work_area",
    request="approach the work area",
    response={
        "tree": {
            "type": "approach_work_area",
            "name": "Approach Work Area",
        },
    },
    description="Drive to the predefined work area pose (world frame) using resolver-provided pose",
    tags=["youbot", "approach", "work_area", "resolver"],
)
def approach_work_area_example():
    pass


# --- NEW: Approach box (generic or nearest) ---
@example(
    name="youbot_approach_box",
    request="approach the target box",
    response={
        # 상황에 맞게 박스 포즈를 선택해 넣는 리졸버(예: 최근접 박스)
        "initial_values": {
            "box_pose": "${nearest_box_pose}"  # or "${box_pose}" / "${box_pose_from_color}"
        },
        "tree": {
            "type": "approach_box",
            "name": "Approach Box",
        },
    },
    description="Drive to a box pose chosen by the resolver (e.g., nearest box pose)",
    tags=["youbot", "approach", "box", "resolver"],
)
def approach_box_example():
    pass


# --- NEW: Drop can (generic or nearest) ---
@example(
    name="youbot_drop_can",
    request="drop the target can into the box",
    response={
        "tree": {
            "type": "drop_can",
            "name": "Drop Can",
            "drop_goal": "${drop_goal}",
        },
    },
    description="Drop a can at a target",
    tags=["youbot", "drop", "can", "resolver"],
)
def drop_can_example():
    pass


@example(
    name="youbot_visiting_sequence",
    request="visit a red can and then a green can`",
    response={
        "tree": {
            "type": "sequence",
            "name": "Visit Multiple Cans",
            "children": [
                {
                    "type": "visit_can",
                    "name": "Visit Red Can",
                    "nearest_can": "${nearest_red_can}",  # Resolver
                },
                {
                    "type": "visit_can",
                    "name": "Visit Green Can",
                    "nearest_can": "${nearest_green_can}",  # Resolver
                },
            ],
        },
    },
    description="Sequence with resolvers",
    tags=["youbot", "sequence"],
)
def sequence_visiting_example():
    pass


@example(
    name="youbot_fallback_example",
    request="try to pick a red can, if not available pick a green can",
    response={
        "tree": {
            "type": "fallback",
            "name": "Pick Red or Green Can",
            "children": [
                {
                    "type": "pick_can",
                    "name": "Pick Red Can",
                    "nearest_can": "${nearest_red_can}",  # Resolver
                },
                {
                    "type": "pick_can",
                    "name": "Pick Green Can",
                    "nearest_can": "${nearest_green_can}",  # Resolver
                },
            ],
        },
    },
    description="Fallback with resolvers",
    tags=["youbot", "fallback"],
)
def fallback_example():
    pass


@example(
    name="youbot_fallback_approach_box",
    request="try pick a red can, if not available return to box",
    response={
        "tree": {
            "type": "fallback",
            "name": "Pick Red Can or Return to Box",
            "children": [
                {
                    "type": "pick_can",
                    "name": "Pick Red Can",
                    "nearest_can": "${nearest_red_can}",  # Resolver
                },
                {
                    "type": "approach_box",
                    "name": "Return to Box",
                },
            ],
        },
    },
    description="Fallback with resolvers",
    tags=["youbot", "fallback", "approach_box"],
)
def fallback_approach_box_example():
    pass
