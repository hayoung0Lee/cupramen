"""
Resolver registry - manages all registered resolvers
Now tracks runtime flag for each resolver
"""

import re
from typing import Any, Callable, Dict, List, Optional, Set


class ResolverRegistry:
    """
    Central registry for all resolver types
    Tracks whether each resolver is static or runtime
    """

    def __init__(self):
        self.symbols: Dict[str, Dict[str, Any]] = {}
        self.patterns: List[Dict[str, Any]] = []
        self.namespaces: Dict[str, Dict[str, Any]] = {}
        self.runtime_keys: Set[str] = set()  # Track which are runtime

    def register_symbol(
        self, name: str, handler: Callable, description: str = "", runtime: bool = False
    ):
        """Register a symbol resolver with runtime flag"""
        self.symbols[name] = {
            "handler": handler,
            "description": description,
            "runtime": runtime,
        }
        if runtime:
            self.runtime_keys.add(f"symbol:{name}")

    def register_pattern(
        self,
        pattern: str,
        handler: Callable,
        description: str = "",
        runtime: bool = False,
    ):
        """Register a pattern resolver with runtime flag"""
        self.patterns.append(
            {
                "pattern": re.compile(pattern),
                "handler": handler,
                "description": description,
                "pattern_str": pattern,
                "runtime": runtime,
            }
        )
        if runtime:
            self.runtime_keys.add(f"pattern:{pattern}")

    def register_namespace(
        self, name: str, handler: Any, description: str = "", runtime: bool = False
    ):
        """Register a namespace resolver with runtime flag"""
        # Instantiate if it's a class
        if isinstance(handler, type):
            handler = handler()

        self.namespaces[name] = {
            "handler": handler,
            "description": description,
            "runtime": runtime,
        }
        if runtime:
            self.runtime_keys.add(f"namespace:{name}")

    def is_runtime(self, expression: str) -> bool:
        """Check if an expression is a runtime resolver"""
        # Check symbol
        if expression in self.symbols:
            return self.symbols[expression].get("runtime", False)

        # Check patterns
        for pattern_info in self.patterns:
            if pattern_info["pattern"].match(expression):
                return pattern_info.get("runtime", False)

        # Check namespace
        if ":" in expression:
            namespace_name = expression.split(":")[0]
            if namespace_name in self.namespaces:
                return self.namespaces[namespace_name].get("runtime", False)

        return False

    def get_symbol(self, name: str) -> Optional[Dict[str, Any]]:
        """Get a symbol resolver by name"""
        return self.symbols.get(name)

    def get_patterns(self) -> List[Dict[str, Any]]:
        """Get all pattern resolvers"""
        return self.patterns

    def get_namespace(self, name: str) -> Optional[Dict[str, Any]]:
        """Get a namespace resolver by name"""
        return self.namespaces.get(name)

    def list_all(self) -> Dict[str, Any]:
        """List all registered resolvers with runtime info"""
        return {
            "symbols": self.symbols,
            "patterns": [
                {
                    "pattern": p["pattern_str"],
                    "description": p["description"],
                    "runtime": p.get("runtime", False),
                }
                for p in self.patterns
            ],
            "namespaces": self.namespaces,
        }


# Global registry singleton
_global_registry = ResolverRegistry()


def get_global_registry() -> ResolverRegistry:
    """Get the global resolver registry"""
    return _global_registry


def reset_registry():
    """Reset the registry (mainly for testing)"""
    global _global_registry
    _global_registry = ResolverRegistry()
