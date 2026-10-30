"""
Static processor - handles one-time resolution at initialization
Converts LLM response to executable tree
"""

import re
from typing import Any, Dict

from .context import ResolutionContext
from .registry import get_global_registry


class StaticProcessor:
    """
    Processes static templates at initialization
    Runtime templates are left as-is for later processing
    """

    TEMPLATE_PATTERN = re.compile(r"\$\{([^}]+)\}")

    def __init__(self, devices: Dict[str, Any], blackboard: Dict[str, Any] = None):
        """
        Initialize static processor

        Args:
            devices: Robot devices dictionary
            blackboard: Shared data store
        """
        self.devices = devices
        self.blackboard = blackboard or {}
        self.registry = get_global_registry()

    def process(self, llm_response: Dict[str, Any]) -> Dict[str, Any]:
        """
        Process LLM response, resolving static templates only

        Args:
            llm_response: JSON tree from LLM

        Returns:
            Tree with static templates resolved, runtime templates preserved
        """
        # Create temporary blackboard for resolution
        temp_blackboard = self.blackboard.copy()

        # Add initial_values to temp blackboard
        if "initial_values" in llm_response:
            temp_blackboard.update(llm_response["initial_values"])

        # Create resolution context
        context = ResolutionContext(
            devices=self.devices,
            blackboard=temp_blackboard,
            robot=self.blackboard.get("robot"),
        )

        # Resolve static templates only
        resolved_tree = self._resolve(llm_response, context)

        # Post-process
        resolved_tree = self._post_process(resolved_tree)

        return resolved_tree

    def _resolve(self, value: Any, context: ResolutionContext) -> Any:
        """Recursively resolve static templates"""
        if isinstance(value, str):
            if "${" in value:
                return self._resolve_template(value, context)
            return value
        elif isinstance(value, dict):
            return {k: self._resolve(v, context) for k, v in value.items()}
        elif isinstance(value, list):
            return [self._resolve(item, context) for item in value]
        return value

    def _resolve_template(self, template: str, context: ResolutionContext) -> Any:
        """Process template strings, resolving static ones only"""

        def replacer(match):
            expr = match.group(1)

            # Skip runtime resolvers
            if self.registry.is_runtime(expr):
                return match.group(0)  # Return ${xxx} unchanged

            # Handle default values
            default_value = None
            if ":" in expr and "(" not in expr:
                parts = expr.split(":", 1)
                expr = parts[0]
                default_value = parts[1]

            # Evaluate static expression
            result = self._evaluate_static(expr, context)

            # Use default if needed
            if result is None and default_value is not None:
                try:
                    if "." in default_value:
                        return float(default_value)
                    else:
                        return int(default_value)
                except ValueError:
                    return default_value

            # Not known yet: keep the reference for read-time lookup (BaseAction.param)
            return str(result) if result is not None else match.group(0)

        # Special case: single template preserves type
        if (
            template.startswith("${")
            and template.endswith("}")
            and template.count("${") == 1
        ):
            expr = template[2:-1]

            # Skip runtime
            if self.registry.is_runtime(expr):
                return template

            # Handle default
            default_value = None
            if ":" in expr and "(" not in expr:
                parts = expr.split(":", 1)
                expr = parts[0]
                default_value = parts[1]

            result = self._evaluate_static(expr, context)

            if result is None and default_value is not None:
                try:
                    if "." in default_value:
                        return float(default_value)
                    else:
                        return int(default_value)
                except ValueError:
                    return default_value

            # A value that is not known yet (e.g. chosen by a node while the tree runs)
            # used to become None here for good. Keep the reference; BaseAction.param()
            # looks it up in the blackboard when the node reads it.
            return template if result is None else result

        # Multiple templates or mixed content
        return self.TEMPLATE_PATTERN.sub(replacer, template)

    def _evaluate_static(self, expr: str, context: ResolutionContext) -> Any:
        """Evaluate static expression"""
        # Check symbol
        symbol = self.registry.get_symbol(expr)
        if symbol and not symbol.get("runtime", False):
            handler = symbol["handler"]
            return handler(context) if callable(handler) else handler

        # Check patterns
        for pattern_info in self.registry.get_patterns():
            if not pattern_info.get("runtime", False):
                pattern = pattern_info["pattern"]
                match = pattern.match(expr)
                if match:
                    return pattern_info["handler"](match, context)

        # Check namespace
        if ":" in expr:
            namespace_name = expr.split(":")[0]
            namespace = self.registry.get_namespace(namespace_name)
            if namespace and not namespace.get("runtime", False):
                handler = namespace["handler"]
                if hasattr(handler, "resolve"):
                    return handler.resolve(expr.split(":", 1)[1], context)

        # Check blackboard
        if expr in context.blackboard:
            return context.blackboard[expr]

        # Handle dot notation
        if "." in expr:
            parts = expr.split(".")
            value = context.blackboard.get(parts[0])

            for part in parts[1:]:
                if value is None:
                    break
                if isinstance(value, dict):
                    value = value.get(part)
                else:
                    value = getattr(value, part, None)

            return value

        return None

    def _post_process(self, tree: Dict[str, Any]) -> Dict[str, Any]:
        """Post-processing: type conversions, etc."""

        def convert_numbers(value):
            if isinstance(value, str):
                try:
                    # Skip templates
                    if "${" in value:
                        return value
                    # Try conversion
                    if "." in value:
                        return float(value)
                    else:
                        return int(value)
                except ValueError:
                    return value
            elif isinstance(value, dict):
                return {k: convert_numbers(v) for k, v in value.items()}
            elif isinstance(value, list):
                return [convert_numbers(item) for item in value]
            return value

        return convert_numbers(tree)
