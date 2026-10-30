"""
TurtleBot3 Prompt Examples - Practical Scenarios
Real-world behaviors that work with 2D LiDAR limitations
"""

from behavioros.prompts import example

# ========== OBJECT INTERACTION ==========


@example(
    name="turtlebot_find_object",
    request="find and go to the nearest object",
    response={
        "initial_values": {"approach_distance": 0.3},
        "tree": {
            "type": "search_and_approach_object",
            "name": "Find Object",
            "nearest_object": "${nearest_object}",
            "obstacle": "${obstacle}",
        },
    },
    description="360° search and approach with obstacle avoidance",
    tags=["turtlebot", "object", "search"],
)
def find_object():
    pass


@example(
    name="turtlebot_simple_object",
    request="go to the nearest object",
    response={
        "initial_values": {"approach_distance": 0.35},
        "tree": {
            "type": "search_and_approach_object",
            "name": "Go to Object",
            "nearest_object": "${nearest_object}",
            "obstacle": "${obstacle}",
        },
    },
    description="Direct object approach with search fallback",
    tags=["turtlebot", "object", "simple"],
)
def simple_object():
    pass


@example(
    name="turtlebot_investigate_objects",
    request="investigate multiple objects around the room",
    response={
        "initial_values": {
            "approach_distance": 0.4,
            "investigation_duration": 15.0,
        },
        "tree": {
            "type": "sequence",
            "name": "Object Investigation",
            "children": [
                {
                    "type": "search_and_approach_object",
                    "name": "Find First Object",
                    "nearest_object": "${nearest_object}",
                    "obstacle": "${obstacle}",
                },
                {
                    "type": "move_backward",
                    "name": "Back Away",
                    "move_speed": 0.05,
                    "move_duration": 1.5,
                },
                {
                    "type": "turn_left",
                    "name": "Look Around",
                    "turn_speed": 1.0,
                    "turn_duration": 2.0,
                },
                {
                    "type": "search_and_approach_object",
                    "name": "Find Next Object",
                    "nearest_object": "${nearest_object}",
                    "obstacle": "${obstacle}",
                },
            ],
        },
    },
    description="Systematic object investigation with repositioning",
    tags=["turtlebot", "investigate", "objects"],
)
def investigate_objects():
    pass


# ========== EXPLORATION ==========


@example(
    name="turtlebot_explore",
    request="explore the environment",
    response={
        "initial_values": {
            "exploration_duration": 20.0,
            "min_safe_distance": 0.5,
        },
        "tree": {
            "type": "smart_exploration",
            "name": "Smart Exploration",
            "obstacle": "${obstacle}",
            "safe_speed": "${safe_speed}",
        },
    },
    description="Intelligent exploration with adaptive behaviors",
    tags=["turtlebot", "explore", "adaptive"],
)
def explore():
    pass


@example(
    name="turtlebot_room_search",
    request="search the room for interesting things",
    response={
        "initial_values": {
            "exploration_duration": 25.0,
            "approach_distance": 0.35,
            "min_safe_distance": 0.4,
        },
        "tree": {
            "type": "selector",
            "name": "Room Search Strategy",
            "children": [
                {
                    "type": "search_and_approach_object",
                    "name": "Direct Object Search",
                    "nearest_object": "${nearest_object}",
                    "obstacle": "${obstacle}",
                },
                {
                    "type": "sequence",
                    "name": "Exploration Phase",
                    "children": [
                        {
                            "type": "smart_exploration",
                            "name": "Explore Area",
                            "obstacle": "${obstacle}",
                            "safe_speed": "${safe_speed}",
                        },
                        {
                            "type": "search_and_approach_object",
                            "name": "Search After Exploration",
                            "nearest_object": "${nearest_object}",
                            "obstacle": "${obstacle}",
                        },
                    ],
                },
            ],
        },
    },
    description="Combined exploration and object search strategy",
    tags=["turtlebot", "search", "exploration"],
)
def room_search():
    pass


# ========== NAVIGATION ==========


@example(
    name="turtlebot_adaptive_move",
    request="move forward carefully",
    response={
        "initial_values": {
            "move_duration": 5.0,
            "move_speed": 0.15,
        },
        "tree": {
            "type": "adaptive_navigation",
            "name": "Careful Movement",
            "obstacle": "${obstacle}",
            "safe_speed": "${safe_speed}",
        },
    },
    description="Adaptive forward movement with real-time obstacle avoidance",
    tags=["turtlebot", "navigation", "adaptive"],
)
def adaptive_move():
    pass


@example(
    name="turtlebot_safe_forward",
    request="move forward with obstacle checking",
    response={
        "initial_values": {
            "move_duration": 3.0,
            "move_speed": 0.12,
        },
        "tree": {
            "type": "move_forward",
            "name": "Safe Forward",
            "move_speed": "${safe_speed}",
            "obstacle": "${obstacle}",
        },
    },
    description="Forward movement with adaptive speed control",
    tags=["turtlebot", "movement", "safety"],
)
def safe_forward():
    pass


# ========== WALL/OBSTACLE FOLLOWING ==========


@example(
    name="turtlebot_follow_wall",
    request="follow the wall on the right",
    response={
        "initial_values": {
            "follow_duration": 12.0,
            "proximity_threshold": 1.0,
        },
        "tree": {
            "type": "selector",
            "name": "Wall Following Strategy",
            "children": [
                {
                    "type": "sequence",
                    "name": "Follow if Wall Present",
                    "children": [
                        {
                            "type": "check_wall_proximity",
                            "name": "Wall Nearby?",
                            "wall_distance": "${wall_distance}",
                        },
                        {
                            "type": "proximity_wall_follow",
                            "name": "Follow Right Side",
                            "wall_distance": "${wall_distance}",
                            "obstacle": "${obstacle}",
                        },
                    ],
                },
                {
                    "type": "sequence",
                    "name": "Search for Wall",
                    "children": [
                        {
                            "type": "turn_right",
                            "name": "Look Right",
                            "turn_duration": 2.0,
                        },
                        {
                            "type": "adaptive_navigation",
                            "name": "Move to Wall",
                            "obstacle": "${obstacle}",
                            "safe_speed": "${safe_speed}",
                        },
                    ],
                },
            ],
        },
    },
    description="Right-side obstacle following with wall search",
    tags=["turtlebot", "wall", "follow"],
)
def follow_wall():
    pass


@example(
    name="turtlebot_perimeter_patrol",
    request="patrol around the perimeter",
    response={
        "initial_values": {
            "follow_duration": 8.0,
            "turn_duration": 1.5,
            "patrol_cycles": 3,
        },
        "tree": {
            "type": "sequence",
            "name": "Perimeter Patrol",
            "children": [
                {
                    "type": "proximity_wall_follow",
                    "name": "Follow Perimeter",
                    "wall_distance": "${wall_distance}",
                    "obstacle": "${obstacle}",
                },
                {
                    "type": "turn_right",
                    "name": "Corner Turn",
                    "turn_speed": 1.2,
                },
                {
                    "type": "move_forward",
                    "name": "Corner Clearance",
                    "move_speed": "${safe_speed}",
                    "move_duration": 1.0,
                    "obstacle": "${obstacle}",
                },
                {
                    "type": "proximity_wall_follow",
                    "name": "Continue Patrol",
                    "wall_distance": "${wall_distance}",
                    "obstacle": "${obstacle}",
                },
            ],
        },
    },
    description="Systematic perimeter patrol with corner handling",
    tags=["turtlebot", "patrol", "perimeter"],
)
def perimeter_patrol():
    pass


# ========== SAFETY BEHAVIORS ==========


@example(
    name="turtlebot_cautious",
    request="move very carefully and slowly",
    response={
        "initial_values": {
            "min_safe_distance": 0.8,
            "move_speed": 0.08,
            "move_duration": 4.0,
        },
        "tree": {
            "type": "selector",
            "name": "Ultra-Cautious Movement",
            "children": [
                {
                    "type": "sequence",
                    "name": "Check and Move",
                    "children": [
                        {
                            "type": "check_clear_path",
                            "name": "Path Safe?",
                            "obstacle": "${obstacle}",
                        },
                        {
                            "type": "move_forward",
                            "name": "Cautious Advance",
                            "move_speed": 0.05,
                            "obstacle": "${obstacle}",
                        },
                    ],
                },
                {
                    "type": "sequence",
                    "name": "Obstacle Avoidance",
                    "children": [
                        {
                            "type": "move_backward",
                            "name": "Retreat",
                            "move_speed": 0.03,
                            "move_duration": 1.5,
                        },
                        {
                            "type": "turn_left",
                            "name": "Find New Path",
                            "turn_speed": 0.8,
                            "turn_duration": 2.0,
                        },
                    ],
                },
            ],
        },
    },
    description="Ultra-cautious movement with retreat and reposition",
    tags=["turtlebot", "safety", "cautious"],
)
def cautious():
    pass


@example(
    name="turtlebot_emergency_stop",
    request="stop immediately",
    response={"tree": {"type": "stop", "name": "Emergency Stop"}},
    description="Immediate motor stop",
    tags=["turtlebot", "stop", "emergency"],
)
def emergency_stop():
    pass


# ========== COMBINED BEHAVIORS ==========


@example(
    name="turtlebot_office_helper",
    request="patrol the office and check for objects",
    response={
        "initial_values": {
            "patrol_duration": 15.0,
            "approach_distance": 0.4,
            "follow_duration": 6.0,
        },
        "tree": {
            "type": "sequence",
            "name": "Office Helper Routine",
            "children": [
                {
                    "type": "selector",
                    "name": "Initial Object Search",
                    "children": [
                        {
                            "type": "search_and_approach_object",
                            "name": "Find Objects",
                            "nearest_object": "${nearest_object}",
                            "obstacle": "${obstacle}",
                        },
                        {
                            "type": "smart_exploration",
                            "name": "Explore Office",
                            "obstacle": "${obstacle}",
                            "safe_speed": "${safe_speed}",
                        },
                    ],
                },
                {
                    "type": "selector",
                    "name": "Perimeter Check",
                    "children": [
                        {
                            "type": "proximity_wall_follow",
                            "name": "Follow Walls",
                            "wall_distance": "${wall_distance}",
                            "obstacle": "${obstacle}",
                        },
                        {
                            "type": "adaptive_navigation",
                            "name": "Navigate Freely",
                            "obstacle": "${obstacle}",
                            "safe_speed": "${safe_speed}",
                        },
                    ],
                },
                {
                    "type": "search_and_approach_object",
                    "name": "Final Object Check",
                    "nearest_object": "${nearest_object}",
                    "obstacle": "${obstacle}",
                },
            ],
        },
    },
    description="Complete office patrol with object detection and wall following",
    tags=["turtlebot", "office", "patrol", "complex"],
)
def office_helper():
    pass


@example(
    name="turtlebot_security_patrol",
    request="patrol the area like a security robot",
    response={
        "initial_values": {
            "patrol_duration": 20.0,
            "investigation_distance": 0.5,
            "turn_duration": 3.0,
        },
        "tree": {
            "type": "sequence",
            "name": "Security Patrol",
            "children": [
                {
                    "type": "smart_exploration",
                    "name": "Area Sweep",
                    "obstacle": "${obstacle}",
                    "safe_speed": "${safe_speed}",
                },
                {
                    "type": "selector",
                    "name": "Investigate Anomalies",
                    "children": [
                        {
                            "type": "sequence",
                            "name": "Object Investigation",
                            "children": [
                                {
                                    "type": "check_for_object",
                                    "name": "Anomaly Detected?",
                                    "nearest_object": "${nearest_object}",
                                },
                                {
                                    "type": "search_and_approach_object",
                                    "name": "Investigate",
                                    "nearest_object": "${nearest_object}",
                                    "obstacle": "${obstacle}",
                                },
                                {
                                    "type": "turn_left",
                                    "name": "Circle Inspect",
                                    "turn_speed": 0.5,
                                },
                            ],
                        },
                        {
                            "type": "proximity_wall_follow",
                            "name": "Perimeter Check",
                            "wall_distance": "${wall_distance}",
                            "obstacle": "${obstacle}",
                        },
                    ],
                },
            ],
        },
    },
    description="Security-style patrol with anomaly investigation",
    tags=["turtlebot", "security", "patrol", "investigation"],
)
def security_patrol():
    pass


# ========== BASIC MOVEMENTS ==========


@example(
    name="turtlebot_turn_around",
    request="turn around",
    response={
        "initial_values": {"turn_duration": 2.0},
        "tree": {
            "type": "turn_left",
            "name": "Turn Around",
            "turn_speed": 1.57,  # 90°/s for 2s = 180°
        },
    },
    description="180 degree turn",
    tags=["turtlebot", "turn", "simple"],
)
def turn_around():
    pass


@example(
    name="turtlebot_backup",
    request="back up slowly",
    response={
        "initial_values": {
            "move_speed": 0.05,
            "move_duration": 3.0,
        },
        "tree": {
            "type": "move_backward",
            "name": "Slow Backup",
        },
    },
    description="Slow backward movement",
    tags=["turtlebot", "backup", "slow"],
)
def backup():
    pass
