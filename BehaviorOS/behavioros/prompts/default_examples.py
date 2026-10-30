"""
Default few-shot examples for BehaviorOS
"""

from behavioros.prompts import example


@example(
    name="simple_action",
    request="Check system diagnostics",
    response={"type": "DiagnosticInfo", "name": "System Check"},
    description="Simple single action execution",
    tags=["simple", "diagnostic", "basic"],
)
def simple_action_example():
    """Single action example - no tree structure needed"""
    pass


@example(
    name="file_operation",
    request="Search for txt files and read the first one",
    response={
        "initial_values": {"search_pattern": "*.txt", "search_directory": "."},
        "tree": {
            "type": "sequence",
            "name": "File Operation",
            "children": [
                {"type": "SearchFiles", "name": "Find Files"},
                {"type": "ReadFileContent", "name": "Read First File"},
            ],
        },
    },
    description="File search and read with initial values",
    tags=["file", "search", "sequence", "blackboard"],
)
def file_operation_example():
    """Example showing initial_values and sequence usage"""
    pass


@example(
    name="parallel_sensors",
    request="Check all sensors at once",
    response={
        "type": "parallel",
        "name": "Sensor Check",
        "children": [
            {"type": "CheckDistanceSensors", "name": "Distance"},
            {"type": "CheckBattery", "name": "Battery"},
            {"type": "CheckTemperature", "name": "Temperature"},
        ],
    },
    description="Parallel execution of multiple sensors",
    tags=["sensor", "parallel", "robotics"],
)
def parallel_sensor_example():
    """Example of parallel node for concurrent execution"""
    pass


@example(
    name="conditional_fallback",
    request="Try to connect to WiFi or use cellular",
    response={
        "type": "selector",
        "name": "Network Connection",
        "children": [
            {"type": "ConnectWiFi", "name": "Try WiFi"},
            {"type": "ConnectCellular", "name": "Try Cellular"},
            {"type": "ReportOffline", "name": "Report No Connection"},
        ],
    },
    description="Selector/fallback for trying alternatives",
    tags=["network", "selector", "fallback"],
)
def conditional_fallback_example():
    """Example of selector node - stops on first success"""
    pass


@example(
    name="complex_workflow",
    request="Download files from URL and process them",
    response={
        "initial_values": {
            "url": "https://example.com/data.zip",
            "output_dir": "./downloads",
        },
        "tree": {
            "type": "sequence",
            "name": "Download and Process",
            "children": [
                {"type": "CreateDirectory", "name": "Setup Output Dir"},
                {"type": "DownloadFile", "name": "Download from URL"},
                {"type": "ExtractArchive", "name": "Extract Files"},
                {"type": "ProcessFiles", "name": "Process Extracted Files"},
                {"type": "GenerateReport", "name": "Create Summary Report"},
            ],
        },
    },
    description="Multi-step workflow with data pipeline",
    tags=["workflow", "download", "process", "sequence"],
)
def complex_workflow_example():
    """Example of complex multi-step data processing workflow"""
    pass


@example(
    name="robot_movement",
    request="Move robot forward for 5 seconds",
    response={
        "initial_values": {"speed": 1.0, "duration": 5.0},
        "tree": {"type": "MoveForward", "name": "Move Forward"},
    },
    description="Simple robot movement with parameters",
    tags=["robotics", "movement", "simple"],
)
def robot_movement_example():
    """Basic robot movement with speed and duration"""
    pass


@example(
    name="error_handling",
    request="Read config file with fallback to defaults",
    response={
        "type": "selector",
        "name": "Load Configuration",
        "children": [
            {
                "type": "sequence",
                "name": "Try Config File",
                "children": [
                    {"type": "ReadConfigFile", "name": "Read config.json"},
                    {"type": "ValidateConfig", "name": "Validate Settings"},
                ],
            },
            {"type": "LoadDefaultConfig", "name": "Use Defaults"},
        ],
    },
    description="Error handling with fallback behavior",
    tags=["config", "error-handling", "selector", "fallback"],
)
def error_handling_example():
    """Example of graceful error handling with fallback"""
    pass
