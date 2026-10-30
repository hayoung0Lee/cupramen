"""
Helper functions for easy tree creation with resolution
"""

from typing import Any, Dict

from .runtime_processor import RuntimeProcessor
from .static_processor import StaticProcessor


def create_resolved_tree(
    tree_json: Dict[str, Any],
    blackboard: Dict[str, Any],
    devices: Dict[str, Any],
    enable_runtime: bool = True,
) -> tuple:
    """
    Create a resolved tree with optional runtime hook

    Args:
        tree_json: Raw tree from LLM
        blackboard: Shared data
        devices: Robot devices
        enable_runtime: Whether to create runtime hook

    Returns:
        Tuple of (resolved_tree, runtime_hook or None)
    """
    # Static resolution
    static = StaticProcessor(devices, blackboard)
    resolved_tree = static.process(tree_json)
    print("[Resolution] Static templates resolved")

    # Runtime setup
    runtime_hook = None
    if enable_runtime:
        runtime = RuntimeProcessor()
        runtime_expressions = runtime.scan_tree(resolved_tree)

        if runtime_expressions:
            runtime_hook = runtime.create_hook(devices)
            print(
                f"[Resolution] Runtime hook created for: {list(runtime_expressions.keys())}"
            )

    return resolved_tree, runtime_hook
