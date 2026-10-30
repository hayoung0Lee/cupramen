"""
Pure file I/O for logging.
"""

import json
import os
from datetime import datetime
from pathlib import Path
from typing import Optional

from behavioros.logging.tracker import ExecutionState


class FileLogger:
    """Pure file logger - only handles file I/O."""

    def __init__(self, log_dir: Optional[str] = None):
        """Initialize with log directory."""
        if log_dir:
            self.log_dir = Path(log_dir)
        else:
            self.log_dir = Path(os.getenv("BEHAVIOROS_LOG_DIR", "logs"))

        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.current_execution_dir: Optional[Path] = None
        self.summary = {}

    def start_execution(self, state: ExecutionState) -> Path:
        """Start logging a new execution."""
        self.current_execution_dir = self.log_dir / f"execution_{state.execution_id}"
        self.current_execution_dir.mkdir(parents=True, exist_ok=True)

        # Save initial summary
        self.summary = {
            "session_id": state.execution_id,
            "tree_name": state.tree_name,
            "start_time": state.start_time.isoformat() if state.start_time else None,
            "status": state.tree_status,
            "tree_json": state.tree_json,
            "initial_blackboard": state.blackboard,
        }

        summary_file = self.current_execution_dir / "summary.json"
        with open(summary_file, "w") as f:
            json.dump(self.summary, f, indent=2, default=str)

        return self.current_execution_dir

    def log_tick(self, state: "ExecutionState"):
        """Log a single tick."""
        if not self.current_execution_dir or not state:
            return

        tick_data = {
            "tick_count": state.tick_count,
            "timestamp": datetime.now().isoformat(),
            "tree_status": state.tree_status,
            "current_node": state.current_node,
            "blackboard": state.blackboard,
            "actions": state.actions,
            "node_statuses": state.node_statuses,
        }

        tick_file = self.current_execution_dir / f"tick_{state.tick_count:04d}.json"
        with open(tick_file, "w") as f:
            json.dump(tick_data, f, indent=2, default=str)

    def complete_execution(self, state: ExecutionState, duration: float = 0):
        """Complete the execution log."""
        if not self.current_execution_dir or not state:
            return

        # Update summary with final data
        self.summary.update(
            {
                "session_id": state.execution_id,
                "tree_name": state.tree_name,
                "start_time": state.start_time.isoformat()
                if state.start_time
                else None,
                "end_time": datetime.now().isoformat(),
                "status": state.tree_status,
                "total_ticks": state.tick_count,
                "duration": duration,
                "final_blackboard": state.blackboard,
            }
        )

        summary_file = self.current_execution_dir / "summary.json"
        with open(summary_file, "w") as f:
            json.dump(self.summary, f, indent=2, default=str)
