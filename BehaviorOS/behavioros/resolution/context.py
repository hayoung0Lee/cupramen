"""
Resolution context shared by both static and runtime resolvers
"""

from dataclasses import dataclass
from typing import Any, Dict


@dataclass
class ResolutionContext:
    """Context information needed for resolution"""

    devices: Dict[str, Any]  # Robot devices
    blackboard: Dict[str, Any]  # Shared data store
    robot: Any = None  # Robot instance
    sensor_cache: Dict[str, Any] = None  # Sensor value cache

    def __post_init__(self):
        if self.sensor_cache is None:
            self.sensor_cache = {}
