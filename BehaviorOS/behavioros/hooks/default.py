"""
Core hooks only - logging and runtime resolution
"""

from behavioros.logging import ExecutionTracker, FileLogger


def create_logging_hook(log_dir: str = "logs", resolved_tree: dict = None):
    """
    Create logging hook for execution tracking

    Returns:
        after_tick_hook function
    """

    tracker = ExecutionTracker()
    logger = FileLogger(log_dir=log_dir)
    state = {"started": False}

    def log_tick(result, blackboard, py_tree):
        """Log each tick"""
        # Initialize on first tick
        if not state["started"]:
            tracker.start_execution("Tree", resolved_tree)
            logger.start_execution(tracker.state)
            state["started"] = True

        # Update and log
        tracker.update_from_tick(result, py_tree, blackboard)
        logger.log_tick(tracker.state)

        # Complete on finish
        if result.status in ["SUCCESS", "FAILURE"]:
            tracker.complete_execution(result.status)
            logger.complete_execution(tracker.state, 0)
            state["started"] = False

    return log_tick


def create_runtime_hook(tree_json: dict, devices: dict):
    """
    Create runtime resolution hook

    Args:
        tree_json: Resolved tree (after static resolution)
        devices: Robot devices

    Returns:
        before_tick_hook function or None
    """
    from behavioros.resolution import RuntimeProcessor

    runtime = RuntimeProcessor()
    runtime.scan_tree(tree_json)

    if runtime.runtime_expressions:
        return runtime.create_hook(devices)
    return None
