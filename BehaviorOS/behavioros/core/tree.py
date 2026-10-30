"""
TreeExecutor with minimal hook support
"""

from __future__ import annotations

import datetime
import time
from dataclasses import dataclass
from typing import Any, Callable

import py_trees

from .decorators import RepeatUntil


class UnknownNodeTypeError(ValueError):
    """A JSON node's type is neither a composite nor a registered action."""


@dataclass
class TickResult:
    """Single tick result"""

    tick_count: int
    status: str
    timestamp: str


@dataclass
class ExecutionResult:
    """Final execution result"""

    status: str
    total_ticks: int
    duration: float
    blackboard: dict


class TreeExecutor:
    """TreeExecutor with hook support"""

    def __init__(
        self,
        json_data: dict,
        blackboard: dict = None,
        tick_callback: Callable | None = None,  # Legacy support
        clock: Callable[[], float] | None = None,
    ):
        self.blackboard = {} if blackboard is None else blackboard
        self.tick_count = 0
        self.tick_callback = tick_callback
        # Source of time for durations and for actions (self.now()). Pass the simulation
        # time when running in a simulator; defaults to the wall clock.
        self.clock = clock or time.time
        self.start_time: float | None = None
        self.end_time: float | None = None

        # Minimal hook system
        self.before_tick_hooks: list[Callable] = []
        self.after_tick_hooks: list[Callable] = []

        # Parse and build tree
        self._parse_json(json_data)
        self.py_tree = self._build_py_tree()
        self.tree_data = json_data

    def add_before_tick(self, hook: Callable) -> TreeExecutor:
        """Add hook to run before tick - hook(blackboard, py_tree)"""
        self.before_tick_hooks.append(hook)
        return self

    def add_after_tick(self, hook: Callable) -> TreeExecutor:
        """Add hook to run after tick - hook(tick_result, blackboard, py_tree)"""
        self.after_tick_hooks.append(hook)
        return self

    def tick(self) -> TickResult:
        """Execute one tick with hooks"""
        if self.start_time is None:
            self.start_time = self.clock()

        # Before-tick hooks
        for hook in self.before_tick_hooks:
            try:
                hook(self.blackboard, self.py_tree)
            except Exception as e:
                print(f"[Hook Error] {e}")

        self.tick_count += 1
        self.py_tree.tick()
        if self.status in ("SUCCESS", "FAILURE"):
            self.end_time = self.clock()

        result = TickResult(
            tick_count=self.tick_count,
            status=self.status,
            timestamp=datetime.datetime.now().isoformat(),
        )

        # After-tick hooks
        for hook in self.after_tick_hooks:
            try:
                hook(result, self.blackboard, self.py_tree)
            except Exception as e:
                print(f"[Hook Error] {e}")

        # Legacy callback
        if self.tick_callback:
            self.tick_callback(result, self.py_tree)

        return result

    @property
    def status(self) -> str:
        """Get current tree status"""
        return self.py_tree.root.status.name

    def get_execution_result(self) -> ExecutionResult:
        """Get current execution result"""
        # `is None`, not truthiness: a simulation clock starts at 0.0.
        if self.start_time is None:
            duration = 0.0
        else:
            end = self.end_time if self.end_time is not None else self.clock()
            duration = end - self.start_time
        return ExecutionResult(
            status=self.status,
            total_ticks=self.tick_count,
            duration=duration,
            blackboard=self.blackboard.copy(),
        )

    def _parse_json(self, json_data: dict):
        """Parse JSON data"""
        if "initial_values" in json_data:
            self.blackboard.update(json_data["initial_values"])
            self.tree_data = json_data["tree"]
        elif "tree" in json_data:
            self.tree_data = json_data["tree"]
        else:
            self.tree_data = json_data

    def _build_py_tree(self) -> py_trees.trees.BehaviourTree:
        """Build py_trees structure"""
        root = self._build_node(self.tree_data)
        py_tree = py_trees.trees.BehaviourTree(root)
        py_tree.setup()
        return py_tree

    def _build_node(self, node_data: dict[str, Any]):
        """Convert JSON node to py_trees node"""
        from .actions import get_registry

        node_type = node_data.get("type")
        name = node_data.get("name", node_type)

        # Composite nodes
        if node_type == "sequence":
            children = [
                self._build_node(child) for child in node_data.get("children", [])
            ]
            composite = py_trees.composites.Sequence(name=name, memory=True)
            for child in children:
                composite.add_child(child)
            return composite

        elif node_type == "selector" or node_type == "fallback":
            children = [
                self._build_node(child) for child in node_data.get("children", [])
            ]
            composite = py_trees.composites.Selector(name=name, memory=True)
            for child in children:
                composite.add_child(child)
            return composite

        elif node_type == "parallel":
            children = [
                self._build_node(child) for child in node_data.get("children", [])
            ]
            policy = py_trees.common.ParallelPolicy.SuccessOnAll()
            composite = py_trees.composites.Parallel(name=name, policy=policy)
            for child in children:
                composite.add_child(child)
            return composite

        # Decorators: exactly one child
        elif node_type == "retry":
            return py_trees.decorators.Retry(
                name=name,
                child=self._single_child(node_data),
                num_failures=int(self._required(node_data, "num_failures")),
            )

        elif node_type == "repeat_until":
            max_iterations = node_data.get("max_iterations")
            return RepeatUntil(
                name=name,
                child=self._single_child(node_data),
                blackboard=self.blackboard,
                until=self._required(node_data, "until"),
                max_iterations=None if max_iterations is None else int(max_iterations),
            )

        elif node_type == "inverter":
            return py_trees.decorators.Inverter(name=name, child=self._single_child(node_data))

        else:
            # Action node
            registry = get_registry()
            action_class = registry.get_class(node_type)

            if action_class is None:
                # Fail at build time: silently substituting a Success node let a typo
                # skip a step while the tree still reported SUCCESS.
                known = sorted(a["id"] for a in registry.list_actions())
                raise UnknownNodeTypeError(
                    f"Node '{name}': unknown type '{node_type}'. "
                    f"Composites: sequence, selector/fallback, parallel. "
                    f"Decorators: retry, repeat_until, inverter. "
                    f"Registered actions: {', '.join(known) or '(none)'}"
                )
            params = {
                k: v for k, v in node_data.items() if k not in ("type", "name", "children")
            }
            # v0.1 behaviour, kept for actions that read their params from the blackboard.
            # Nodes sharing a key overwrite each other here; new actions use self.param().
            self.blackboard.update(params)
            node = action_class(name=name, blackboard=self.blackboard)
            node.params = params
            node.clock = self.clock
            return node

    def _single_child(self, node_data: dict[str, Any]):
        children = node_data.get("children", [])
        if len(children) != 1:
            raise ValueError(
                f"Node '{node_data.get('name', node_data.get('type'))}': "
                f"'{node_data.get('type')}' needs exactly one child, got {len(children)}"
            )
        return self._build_node(children[0])

    @staticmethod
    def _required(node_data: dict[str, Any], key: str):
        if key not in node_data:
            raise ValueError(
                f"Node '{node_data.get('name', node_data.get('type'))}': "
                f"'{node_data.get('type')}' needs '{key}'"
            )
        return node_data[key]
