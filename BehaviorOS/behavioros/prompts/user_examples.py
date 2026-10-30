"""
User custom examples for BehaviorOS
Add your own domain-specific examples here
"""

from behavioros.prompts import example


# Example: Custom robotics safety movement
@example(
    name="safe_robot_navigation",
    request="Move robot safely to kitchen with obstacle avoidance",
    response={
        "initial_values": {
            "destination": "kitchen",
            "max_speed": 0.5,
            "safety_distance": 1.0,
        },
        "tree": {
            "type": "sequence",
            "name": "Safe Navigation",
            "children": [
                {"type": "CheckDistanceSensors", "name": "Initial Safety Check"},
                {"type": "PlanPath", "name": "Calculate Route"},
                {
                    "type": "selector",
                    "name": "Navigate with Fallback",
                    "children": [
                        {"type": "MoveToLocation", "name": "Try Direct Path"},
                        {
                            "type": "sequence",
                            "name": "Alternative Route",
                            "children": [
                                {"type": "FindAlternativePath", "name": "Recalculate"},
                                {"type": "MoveToLocation", "name": "Try Alternative"},
                            ],
                        },
                    ],
                },
                {"type": "ReportStatus", "name": "Confirm Arrival"},
            ],
        },
    },
    description="Safe robot navigation with obstacle handling",
    tags=["robotics", "movement", "safety", "navigation", "custom"],
)
def safe_robot_navigation():
    """Custom example for safe robot movement"""
    pass


# Example: Data analysis pipeline
@example(
    name="analyze_sensor_data",
    request="Collect sensor data for 10 seconds and analyze",
    response={
        "initial_values": {
            "collection_duration": 10,
            "sample_rate": 100,
            "output_format": "json",
        },
        "tree": {
            "type": "sequence",
            "name": "Sensor Analysis Pipeline",
            "children": [
                {"type": "StartDataCollection", "name": "Begin Collection"},
                {"type": "WaitDuration", "name": "Collect for Duration"},
                {"type": "StopDataCollection", "name": "End Collection"},
                {"type": "ProcessSensorData", "name": "Analyze Data"},
                {"type": "GenerateVisualization", "name": "Create Plots"},
                {"type": "SaveResults", "name": "Export Results"},
            ],
        },
    },
    description="Data collection and analysis workflow",
    tags=["data", "analysis", "sensor", "custom"],
)
def analyze_sensor_data():
    """Custom example for data analysis pipeline"""
    pass


# Example: Home automation
@example(
    name="morning_routine",
    request="Run morning automation routine",
    response={
        "type": "parallel",
        "name": "Morning Routine",
        "children": [
            {
                "type": "sequence",
                "name": "Lights",
                "children": [
                    {"type": "TurnOnLights", "name": "Bedroom Lights"},
                    {"type": "SetBrightness", "name": "Dim to 50%"},
                ],
            },
            {"type": "StartCoffeeMaker", "name": "Brew Coffee"},
            {"type": "SetThermostat", "name": "Warm House"},
            {"type": "PlayMusic", "name": "Morning Playlist"},
        ],
    },
    description="Home automation parallel tasks",
    tags=["home", "automation", "parallel", "custom"],
)
def morning_routine():
    """Custom example for home automation"""
    pass


# Add more custom examples below...
# @example(
#     name="your_example",
#     request="Your request here",
#     response={...},
#     tags=["your", "tags"]
# )
# def your_example():
#     pass
