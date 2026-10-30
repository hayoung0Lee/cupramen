"""
Runtime processor - handles per-tick resolution
Creates hooks for TreeExecutor
"""

import re
from typing import Any, Callable, Dict

from .context import ResolutionContext
from .registry import get_global_registry


class RuntimeProcessor:
    """
    Handles runtime template resolution
    Creates hooks that update values every tick
    """

    def __init__(self):
        """Initialize runtime processor"""
        self.registry = get_global_registry()
        self.runtime_expressions: Dict[str, str] = {}  # key -> expression

    def scan_tree(self, tree_data: Dict[str, Any]) -> Dict[str, str]:
        """
        Scan tree for runtime templates

        Args:
            tree_data: Processed tree from StaticProcessor

        Returns:
            Dictionary of key -> expression for runtime templates
        """
        runtime_found = {}

        def scan(obj, path=""):
            if isinstance(obj, str) and "${" in obj:
                matches = re.findall(r"\$\{([^}]+)\}", obj)
                for match in matches:
                    if self.registry.is_runtime(match):
                        # Extract simple key
                        key = match.split(":")[0].split("(")[0]
                        runtime_found[key] = match
                        print(f"[Runtime] Found: {key} = ${{{match}}} at {path}")

            elif isinstance(obj, dict):
                for k, v in obj.items():
                    new_path = f"{path}.{k}" if path else k
                    scan(v, new_path)

            elif isinstance(obj, list):
                for i, item in enumerate(obj):
                    scan(item, f"{path}[{i}]")

        scan(tree_data)
        self.runtime_expressions = runtime_found
        return runtime_found

    def create_hook(self, devices: Dict[str, Any]) -> Callable:
        """
        Create a before-tick hook for TreeExecutor

        Args:
            devices: Robot devices dictionary

        Returns:
            Hook function that updates runtime values
        """

        def update_runtime_values(blackboard, py_tree):
            """Hook function called before each tick"""
            context = ResolutionContext(
                devices=devices, blackboard=blackboard, robot=blackboard.get("robot")
            )

            for key, expression in self.runtime_expressions.items():
                try:
                    value = self._evaluate_runtime(expression, context)
                    if value is not None:
                        blackboard[key] = value

                        # Debug output for detected objects
                        if isinstance(value, dict) and "detected" in value:
                            if value.get("detected"):
                                distance = value.get("distance", "?")
                                print(f"[Runtime] {key}: detected at {distance}m")

                except Exception as e:
                    print(f"[Runtime Error] {key}: {e}")

        return update_runtime_values

    def _evaluate_runtime(self, expr: str, context: ResolutionContext) -> Any:
        """Evaluate runtime expression"""
        # Check symbol
        symbol = self.registry.get_symbol(expr)
        if symbol and symbol.get("runtime", False):
            handler = symbol["handler"]
            return handler(context) if callable(handler) else handler

        # Check patterns
        for pattern_info in self.registry.get_patterns():
            if pattern_info.get("runtime", False):
                pattern = pattern_info["pattern"]
                match = pattern.match(expr)
                if match:
                    return pattern_info["handler"](match, context)

        # Check namespace
        if ":" in expr:
            namespace_name = expr.split(":")[0]
            namespace = self.registry.get_namespace(namespace_name)
            if namespace and namespace.get("runtime", False):
                handler = namespace["handler"]
                if hasattr(handler, "resolve"):
                    return handler.resolve(expr.split(":", 1)[1], context)

        return None
