"""
Execution state tracking (memory only, no file I/O).
"""

from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Optional

import py_trees


@dataclass
class ExecutionState:
    """Complete execution state at a point in time."""

    execution_id: str
    tree_name: str
    tick_count: int = 0
    tree_status: str = "IDLE"
    current_node: str = "Unknown"
    blackboard: Dict[str, Any] = field(default_factory=dict)
    actions: List[Dict[str, Any]] = field(default_factory=list)
    node_statuses: Dict[str, str] = field(default_factory=dict)
    start_time: Optional[datetime] = None
    tree_json: Dict[str, Any] = field(default_factory=dict)


class ExecutionTracker:
    """
    Tracks execution state in memory.
    Analyzes py_trees to understand what's happening.
    """

    def __init__(self):
        self.state: Optional[ExecutionState] = None
        self.current_node = "Unknown"
        self.current_actions = []

    def start_execution(self, tree_name: str, tree_json: Dict[str, Any] = None) -> str:
        """Start tracking new execution."""
        execution_id = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.state = ExecutionState(
            execution_id=execution_id,
            tree_name=tree_name,
            start_time=datetime.now(),
            tree_json=tree_json or {},
            tree_status="RUNNING",
        )

        return execution_id

    def update_from_tick(self, tick_result, py_tree=None, blackboard=None):
        """
        Update state from tick result.

        Args:
            tick_result: TickResult from TreeExecutor
            py_tree: py_trees.trees.BehaviourTree for analysis
            blackboard: Current blackboard state
        """
        if not self.state:
            return

        # Update basic info from tick_result
        self.state.tick_count = tick_result.tick_count
        self.state.tree_status = tick_result.status

        # Analyze py_tree to get current execution state
        if py_tree:
            self._analyze_tree(py_tree)

        # Update state with analyzed data
        self.state.current_node = self.current_node
        self.state.actions = self.current_actions.copy()

        # Update blackboard if provided
        if blackboard:
            self.state.blackboard = blackboard.copy()

    def _analyze_tree(self, py_tree):
        """Analyze py_trees to understand current execution state."""
        tip = py_tree.tip()
        if tip:
            self.current_node = tip.name

        self.current_actions = []

        class NodeCollector:
            def __init__(self, tracker):
                self.tracker = tracker
                self.node_statuses = {}

            def run(self, node):
                # Record all node statuses
                self.node_statuses[node.name] = node.status.name

                # Add node info to actions list
                if node.status != py_trees.common.Status.INVALID:
                    self.tracker.current_actions.append(
                        {
                            "name": node.name,
                            "type": type(node).__name__,
                            "status": node.status.name,
                        }
                    )

                # Return True to continue visiting children
                return True

        # Create visitor and visit all nodes
        collector = NodeCollector(self)

        # py_trees는 visit 대신 traverse를 사용할 수도 있음
        if hasattr(py_tree.root, "traverse"):
            for node in py_tree.root.traverse():
                collector.run(node)
        else:
            # 또는 iterate를 사용
            for node in py_tree.root.iterate():
                collector.run(node)

        # Update state's node_statuses
        if self.state:
            self.state.node_statuses = collector.node_statuses.copy()

    def complete_execution(self, final_status: str = None):
        """Mark execution as complete."""
        if self.state:
            self.state.tree_status = final_status or "COMPLETED"

    def get_current_state(self) -> Optional[ExecutionState]:
        """Get current execution state."""
        return self.state
