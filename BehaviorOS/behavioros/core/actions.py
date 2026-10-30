"""
Simplified action system with useful metadata.
"""

import re
import time
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional

import py_trees

_REFERENCE = re.compile(r"\$\{([^}]+)\}")


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
        self.blackboard = {} if blackboard is None else blackboard
        # This node's own parameters from its JSON node (set by TreeExecutor).
        self.params: Dict[str, Any] = {}
        # The tree's clock (set by TreeExecutor); wall clock when used on its own.
        self.clock = time.time

    @abstractmethod
    def execute(self) -> py_trees.common.Status:
        """Execute the action logic."""
        pass

    def reset(self) -> None:
        """Set up per-run state. Called each time the node starts (again).

        An action that spans several ticks keeps its progress (current phase, start
        time, targets) here rather than in __init__; otherwise a repeated or retried
        run resumes from where the previous run ended and finishes without working.
        """

    def initialise(self) -> None:
        """py_trees hook: runs when the node is ticked while not already RUNNING."""
        try:
            self.reset()
        except Exception as e:
            # Same policy as update(): an action error must not crash the tree. The node
            # then fails on its first execute(), which sees the incomplete state.
            print(f"Action {self.name} reset failed: {e}")

    def update(self) -> py_trees.common.Status:
        """Called by py_trees during tick."""
        try:
            return self.execute()
        except Exception as e:
            print(f"Action {self.name} failed: {e}")
            return self.fail("exception", error=f"{type(e).__name__}: {e}")

    def now(self) -> float:
        """Current time on the tree's clock (simulation time when one was given)."""
        return self.clock()

    def fail(self, reason: str, **detail) -> py_trees.common.Status:
        """Record why this action failed, then return FAILURE.

        A FAILURE status only says *that* a node failed. Recovery nodes and run reports
        need *why*, so it goes to the blackboard in one shape:
          blackboard["last_failure"]  the most recent failure
          blackboard["failure_log"]   every failure so far, in order
        Usage in execute(): return self.fail("seal_failed", cup=cup)
        """
        record = {
            "node": self.name,
            "action": getattr(self, "_action_id", type(self).__name__),
            "reason": reason,
            "detail": detail,
            "time": self.now(),
        }
        self.blackboard["last_failure"] = record
        self.blackboard.setdefault("failure_log", []).append(record)
        return py_trees.common.Status.FAILURE

    def param(self, key: str, default=None):
        """Read this node's own parameter.

        A parameter written as a single reference, e.g. "${current_cup}", is looked up in
        the blackboard at the moment of reading, so values kept current by runtime
        resolution are seen as they are now.
        """
        value = self.params.get(key, default)
        if isinstance(value, str):
            match = _REFERENCE.fullmatch(value)
            if match:
                # Same key the runtime processor writes: "name:default" / "name(args)" -> "name"
                bb_key = match.group(1).split(":")[0].split("(")[0]
                return self.blackboard.get(bb_key, default)
        return value

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
