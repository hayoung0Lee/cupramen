"""
Decorators for registering static and runtime resolvers
"""

from .registry import get_global_registry


def static_resolver(key: str, resolver_type: str = "symbol", description: str = ""):
    """
    Register a static resolver - resolved once at initialization

    Usage:
        @static_resolver("robot_model")
        def resolve_robot_model(context):
            return "TurtleBot3"

    Args:
        key: Symbol name, regex pattern, or namespace name
        resolver_type: Type of resolver (symbol, pattern, or namespace)
        description: Human-readable description
    """

    def decorator(func_or_class):
        registry = get_global_registry()

        if resolver_type == "symbol":
            registry.register_symbol(key, func_or_class, description, runtime=False)
        elif resolver_type == "pattern":
            registry.register_pattern(key, func_or_class, description, runtime=False)
        elif resolver_type == "namespace":
            registry.register_namespace(key, func_or_class, description, runtime=False)

        return func_or_class

    return decorator


def runtime_resolver(key: str, resolver_type: str = "symbol", description: str = ""):
    """
    Register a runtime resolver - resolved every tick

    Usage:
        @runtime_resolver("nearest_object", description="Detect nearest object")
        def resolve_nearest_object(context):
            # Read sensor and return current value
            return {"detected": True, "distance": 0.5}

    Args:
        key: Symbol name, regex pattern, or namespace name
        resolver_type: Type of resolver (symbol, pattern, or namespace)
        description: Human-readable description
    """

    def decorator(func_or_class):
        registry = get_global_registry()

        if resolver_type == "symbol":
            registry.register_symbol(key, func_or_class, description, runtime=True)
        elif resolver_type == "pattern":
            registry.register_pattern(key, func_or_class, description, runtime=True)
        elif resolver_type == "namespace":
            registry.register_namespace(key, func_or_class, description, runtime=True)

        return func_or_class

    return decorator


def list_resolvers():
    """Print all registered resolvers with their types"""
    registry = get_global_registry()
    all_resolvers = registry.list_all()

    print("\n=== Static Resolvers ===")
    print("Symbol Resolvers:")
    for name, info in all_resolvers["symbols"].items():
        if not info.get("runtime", False):
            print(f"  ${{{name}}} - {info['description']}")

    print("\nPattern Resolvers:")
    for pattern in all_resolvers["patterns"]:
        if not pattern.get("runtime", False):
            print(f"  {pattern['pattern']} - {pattern['description']}")

    print("\nNamespace Resolvers:")
    for name, info in all_resolvers["namespaces"].items():
        if not info.get("runtime", False):
            print(f"  {name}:* - {info['description']}")

    print("\n=== Runtime Resolvers ===")
    print("Symbol Resolvers:")
    for name, info in all_resolvers["symbols"].items():
        if info.get("runtime", False):
            print(f"  ${{{name}}} - {info['description']}")

    print("\nPattern Resolvers:")
    for pattern in all_resolvers["patterns"]:
        if pattern.get("runtime", False):
            print(f"  {pattern['pattern']} - {pattern['description']}")

    print("\nNamespace Resolvers:")
    for name, info in all_resolvers["namespaces"].items():
        if info.get("runtime", False):
            print(f"  {name}:* - {info['description']}")
