"""
BehaviorOS Resolution System
Static and Runtime value resolution for behavior trees
"""

from .context import ResolutionContext
from .decorators import list_resolvers, runtime_resolver, static_resolver
from .registry import get_global_registry, reset_registry
from .runtime_processor import RuntimeProcessor
from .static_processor import StaticProcessor

__all__ = [
    # Processors
    "StaticProcessor",
    "RuntimeProcessor",
    # Context
    "ResolutionContext",
    # Decorators
    "static_resolver",
    "runtime_resolver",
    "list_resolvers",
    # Registry
    "get_global_registry",
    "reset_registry",
]
