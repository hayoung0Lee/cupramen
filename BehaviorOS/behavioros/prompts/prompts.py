"""
Decorator-based prompt management for BehaviorOS
"""

import json
from typing import Any, Dict, List


class ExampleRegistry:
    """Few-shot example registry"""

    def __init__(self):
        self._examples = {}

    def register(self, name: str, example_data: Dict[str, Any]):
        """Register an example"""
        self._examples[name] = example_data

    def get_all(self) -> List[Dict[str, Any]]:
        """Get all registered examples"""
        return list(self._examples.values())

    def get_by_tags(self, tags: List[str]) -> List[Dict[str, Any]]:
        """Filter examples by tags"""
        results = []
        for example in self._examples.values():
            if any(tag in example.get("tags", []) for tag in tags):
                results.append(example)
        return results


# Global registry
_example_registry = ExampleRegistry()


def example(
    name: str,
    request: str,
    response: Dict[str, Any],
    description: str = "",
    tags: List[str] = None,
):
    """
    Decorator to register few-shot examples

    Args:
        name: Unique identifier for the example
        request: The user request text
        response: The expected JSON response
        description: Brief description of what this example shows
        tags: List of tags for filtering examples

    Usage:
        @example(
            name="file_search",
            request="Search for txt files and read the first one",
            response={
                "initial_values": {"search_pattern": "*.txt"},
                "tree": {...}
            },
            tags=["file", "search"]
        )
        def file_search_example():
            pass  # Function body not used, just for registration
    """

    def decorator(func):
        example_data = {
            "name": name,
            "request": request,
            "response": response,
            "description": description,
            "tags": tags or [],
        }
        _example_registry.register(name, example_data)
        return func

    return decorator


def build_prompt(
    user_request: str, max_examples: int = 2, example_tags: List[str] = None
) -> str:
    """
    Build the complete prompt for LLM

    Args:
        user_request: The user's request to convert to behavior tree
        max_examples: Maximum number of examples to include
        example_tags: Filter examples by these tags (if provided)

    Returns:
        Complete prompt string
    """
    # System prompt
    system = """You are a behavior tree generator for BehaviorOS.

BLACKBOARD-ONLY SYSTEM:
- Actions communicate through blackboard (key-value store)
- Use "initial_values" to set blackboard values before tree execution
- Actions read required values from blackboard keys automatically
- Chain actions so outputs become inputs for next actions"""

    # Available actions
    actions = _get_available_actions()

    # Few-shot examples
    examples = _format_examples(max_examples, example_tags)

    # Generation rules
    rules = """GENERATION RULES:
1. Return ONLY valid JSON (no explanations, no markdown, no comments)
2. Use only actions listed above
3. Actions have NO parameters - they read from blackboard automatically
4. Use "initial_values" to populate required blackboard keys
5. Create proper tree structure (sequence/selector/parallel/fallback)
6. NEVER add sensor checks or safety actions unless explicitly requested
7. Execute ONLY the exact actions needed for the user's request

CRITICAL FORMAT:
- Always use "tree" key (NOT "root")
- Always use "type" and "children" structure for composites
- Single action: {"tree": {"type": "action_name", "name": "Display Name"}}
- Sequence: {"tree": {"type": "sequence", "name": "Name", "children": [...]}}
- Each child must have "type" field

Example structure:
{
  "initial_values": {...},
  "tree": {
    "type": "sequence",
    "name": "Main",
    "children": [
      {"type": "action1", "name": "First"},
      {"type": "action2", "name": "Second"}
    ]
  }
}"""

    return f"""{system}

AVAILABLE ACTIONS:
{actions}

{examples}

{rules}

USER REQUEST: {user_request}

JSON OUTPUT:"""


def _format_examples(max_count: int = 2, tags: List[str] = None) -> str:
    """Format examples for inclusion in prompt"""
    if tags:
        examples = _example_registry.get_by_tags(tags)
    else:
        examples = _example_registry.get_all()

    if not examples:
        return ""

    lines = ["EXAMPLES:"]
    for i, ex in enumerate(examples[:max_count], 1):
        lines.append(f"\n{i}. {ex['description'] or ex['name']}:")
        lines.append(f'Request: "{ex["request"]}"')
        lines.append("Response:")
        lines.append(json.dumps(ex["response"], indent=2))

    return "\n".join(lines)


def _get_available_actions() -> str:
    """Get formatted list of available actions from registry"""
    try:
        from behavioros.core.actions import get_registry

        registry = get_registry()
        lines = []

        for action in registry.list_actions():
            lines.append(f"- {action['id']}: {action['description']}")
            if action["reads"]:
                lines.append(f"  Reads: {', '.join(action['reads'])}")
            if action["writes"]:
                lines.append(f"  Writes: {', '.join(action['writes'])}")

        return "\n".join(lines) if lines else "No actions registered yet"
    except ImportError:
        return "Actions not available (core module not found)"


def get_registry() -> ExampleRegistry:
    """Get the global example registry"""
    return _example_registry


def list_examples() -> List[Dict[str, Any]]:
    """List all registered examples"""
    return _example_registry.get_all()
