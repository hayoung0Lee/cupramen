"""
Simplified action system with useful metadata.
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional

import py_trees


class ActionRegistry:
    """Registry for action types with metadata."""

    def __init__(self):
        self._actions = {}

    def register(self, action_id: str, action_class, metadata: Dict[str, Any]):
        """Register an action with its metadata."""
        self._actions[action_id] = {"class": action_class, "metadata": metadata}

    def get_class(self, action_id: str):
        """Get action class by ID."""
        info = self._actions.get(action_id)
        return info["class"] if info else None

    def get_metadata(self, action_id: str) -> Dict[str, Any]:
        """Get action metadata."""
        info = self._actions.get(action_id)
        return info["metadata"] if info else {}

    def list_actions(self):
        """List all actions with their metadata."""
        result = []
        for action_id, info in self._actions.items():
            meta = info["metadata"]
            result.append(
                {
                    "id": action_id,
                    "description": meta.get("description", ""),
                    "reads": meta.get("reads", []),
                    "writes": meta.get("writes", []),
                }
            )
        return result


# Global registry
_registry = ActionRegistry()


class BaseAction(py_trees.behaviour.Behaviour, ABC):
    """Base action class."""

    def __init__(
        self, name: str, blackboard: Optional[Dict[str, Any]] = None, **kwargs
    ):
        """
        Args:
            name: Instance name for logging (from JSON 'name' field)
            blackboard: Shared data storage
        """
        super().__init__(name=name, **kwargs)
        self.blackboard = blackboard or {}

    @abstractmethod
    def execute(self) -> py_trees.common.Status:
        """Execute the action logic."""
        pass

    def update(self) -> py_trees.common.Status:
        """Called by py_trees during tick."""
        try:
            return self.execute()
        except Exception as e:
            print(f"Action {self.name} failed: {e}")
            return py_trees.common.Status.FAILURE

    def read_blackboard(self, key: str, default=None):
        """Read value from blackboard."""
        return self.blackboard.get(key, default)

    def write_blackboard(self, key: str, value):
        """Write value to blackboard."""
        self.blackboard[key] = value


def action(
    action_id: str,
    description: str = "",
    reads: List[str] = None,
    writes: List[str] = None,
):
    """
    Register an action with metadata.

    Args:
        action_id: Unique identifier for this action type (matches JSON 'type')
        description: What this action does
        reads: List of blackboard keys this action reads
        writes: List of blackboard keys this action writes

    Example:
        @action(
            action_id="move_forward",
            description="Move the robot forward",
            reads=["robot", "speed", "duration"],
            writes=["position", "movement_status"]
        )
        class MoveForwardAction(BaseAction):
            def execute(self):
                speed = self.read_blackboard("speed", default=1.0)
                # ...

    In JSON:
        {"type": "move_forward", "name": "Go to kitchen"}
    """

    def decorator(cls):
        metadata = {
            "description": description,
            "reads": reads or [],
            "writes": writes or [],
        }
        _registry.register(action_id, cls, metadata)
        cls._action_id = action_id
        cls._metadata = metadata
        return cls

    return decorator


def get_registry() -> ActionRegistry:
    """Get the global action registry."""
    return _registry


def validate_blackboard_usage(tree_json: Dict[str, Any]) -> List[str]:
    """
    Validate that all actions have required blackboard keys.

    Returns list of warnings/errors.
    """
    warnings = []
    initial_values = tree_json.get("initial_values", {})

    def check_node(node, available_keys):
        node_type = node.get("type")
        node_name = node.get("name", node_type)

        # Check if it's an action
        metadata = _registry.get_metadata(node_type)
        if metadata:
            # Check required reads
            for key in metadata.get("reads", []):
                if key not in available_keys:
                    warnings.append(
                        f"Action '{node_name}' reads '{key}' but it's not in blackboard"
                    )

            # Add writes to available keys
            for key in metadata.get("writes", []):
                available_keys.add(key)

        # Recurse for children
        for child in node.get("children", []):
            check_node(child, available_keys)

    # Start checking from root
    available_keys = set(initial_values.keys())
    tree = tree_json.get("tree", tree_json)
    check_node(tree, available_keys)

    return warnings


def generate_action_docs() -> str:
    """Generate markdown documentation for all registered actions."""
    docs = ["# Available Actions\n"]

    for action_info in _registry.list_actions():
        docs.append(f"## {action_info['id']}\n")
        docs.append(f"{action_info['description']}\n")

        if action_info["reads"]:
            docs.append("**Reads from blackboard:**")
            for key in action_info["reads"]:
                docs.append(f"- `{key}`")
            docs.append("")

        if action_info["writes"]:
            docs.append("**Writes to blackboard:**")
            for key in action_info["writes"]:
                docs.append(f"- `{key}`")
            docs.append("")

    return "\n".join(docs)


# Convenience status constants
SUCCESS = py_trees.common.Status.SUCCESS
FAILURE = py_trees.common.Status.FAILURE
RUNNING = py_trees.common.Status.RUNNING
