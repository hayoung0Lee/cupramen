"""
Builder pattern for clean tree configuration
"""

from __future__ import annotations

from typing import Any, Callable

from behavioros.core import TreeExecutor
from behavioros.hooks.default import create_logging_hook, create_runtime_hook
from behavioros.hooks.simple import create_status_printer
from behavioros.resolution import StaticProcessor


class TreeBuilder:
    """Builder for TreeExecutor with hooks"""

    def __init__(
        self,
        tree_json: dict[str, Any],
        blackboard: dict[str, Any],
        devices: dict[str, Any],
        clock: Callable[[], float] | None = None,
    ):
        # Static resolution first
        static = StaticProcessor(devices, blackboard)
        resolved_tree = static.process(tree_json)

        # Create tree
        self.tree = TreeExecutor(resolved_tree, blackboard, clock=clock)
        self.resolved_tree = resolved_tree
        self.devices = devices

    def with_runtime(self) -> TreeBuilder:
        """Add runtime resolution"""
        hook = create_runtime_hook(self.resolved_tree, self.devices)
        if hook:
            self.tree.add_before_tick(hook)
            print("✓ Runtime resolution enabled")
        return self

    def with_logging(self, log_dir: str = "logs") -> TreeBuilder:
        """Add logging"""
        hook = create_logging_hook(log_dir, self.resolved_tree)
        self.tree.add_after_tick(hook)
        print("✓ Logging enabled")
        return self

    def with_status_printer(self, every: int = 10) -> TreeBuilder:
        """Add status printer"""
        hook = create_status_printer(every)
        self.tree.add_after_tick(hook)
        print("✓ Status printer enabled")
        return self

    def with_hook(self, hook: callable, before: bool = False) -> TreeBuilder:
        """Add custom hook"""
        if before:
            self.tree.add_before_tick(hook)
        else:
            self.tree.add_after_tick(hook)
        return self

    def build(self) -> TreeExecutor:
        """Return configured tree"""
        return self.tree
