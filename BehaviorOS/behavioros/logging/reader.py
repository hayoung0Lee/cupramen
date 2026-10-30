"""
Log reader for analysis and visualization.
"""

import json
from pathlib import Path
from typing import Any, Dict, List, Optional


class LogReader:
    """Read logs for analysis."""

    def __init__(self, log_dir: Optional[str] = None):
        """Initialize log reader."""
        if log_dir:
            self.log_dir = Path(log_dir)
        else:
            import os

            self.log_dir = Path(os.getenv("BEHAVIOROS_LOG_DIR", "logs"))

    def get_sessions(self) -> List[Dict[str, Any]]:
        """Get all execution sessions."""
        sessions = []

        if not self.log_dir.exists():
            return sessions

        for execution_dir in sorted(self.log_dir.iterdir()):
            if execution_dir.is_dir() and execution_dir.name.startswith("execution_"):
                summary_file = execution_dir / "summary.json"
                if summary_file.exists():
                    try:
                        with open(summary_file) as f:
                            summary = json.load(f)
                            summary["path"] = str(execution_dir)
                            sessions.append(summary)
                    except Exception:
                        continue

        return sessions

    def get_session_summary(self, session_id: str) -> Optional[Dict[str, Any]]:
        """Get summary for specific session."""
        execution_dir = self.log_dir / f"execution_{session_id}"
        summary_file = execution_dir / "summary.json"

        if summary_file.exists():
            with open(summary_file) as f:
                return json.load(f)
        return None

    def get_ticks(
        self, session_id: str, start: int = 0, end: Optional[int] = None
    ) -> List[Dict[str, Any]]:
        """Get tick data for session."""
        execution_dir = self.log_dir / f"execution_{session_id}"

        if not execution_dir.exists():
            return []

        ticks = []
        tick_files = sorted(execution_dir.glob("tick_*.json"))

        # Apply range
        if end is not None:
            tick_files = tick_files[start:end]
        else:
            tick_files = tick_files[start:]

        for tick_file in tick_files:
            try:
                with open(tick_file) as f:
                    ticks.append(json.load(f))
            except Exception:
                continue

        return ticks
