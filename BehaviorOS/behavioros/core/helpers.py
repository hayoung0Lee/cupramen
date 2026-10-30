"""
Helper functions for easy tree creation
"""

from typing import Any, Callable, Dict

from behavioros.core import TreeExecutor
from behavioros.hooks.default import create_logging_hook, create_runtime_hook
from behavioros.resolution import StaticProcessor


def create_tree(
    tree_json: Dict[str, Any],
    blackboard: Dict[str, Any],
    devices: Dict[str, Any],
    enable_logging: bool = True,
    enable_runtime: bool = True,
    log_dir: str = "logs",
    clock: Callable[[], float] | None = None,
) -> TreeExecutor:
    """
    Create tree with essential features

    Args:
        tree_json: Tree definition from LLM
        blackboard: Shared data
        devices: Robot devices
        enable_logging: Add logging
        enable_runtime: Add runtime resolution
        log_dir: Log directory
        clock: Time source for durations and actions (default: wall clock)

    Returns:
        Configured TreeExecutor

    Usage:
        tree = create_tree(tree_json, blackboard, devices)
    """
    # Static resolution
    static = StaticProcessor(devices, blackboard)
    resolved_tree = static.process(tree_json)

    # Create tree
    tree = TreeExecutor(resolved_tree, blackboard, clock=clock)

    # Add runtime resolution
    if enable_runtime:
        runtime_hook = create_runtime_hook(resolved_tree, devices)
        if runtime_hook:
            tree.add_before_tick(runtime_hook)
            print("✓ Runtime resolution enabled")

    # Add logging
    if enable_logging:
        log_hook = create_logging_hook(log_dir, resolved_tree)
        tree.add_after_tick(log_hook)
        print("✓ Logging enabled")

    return tree
