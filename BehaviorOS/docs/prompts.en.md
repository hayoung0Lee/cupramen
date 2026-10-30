# Prompt System

The prompt layer asks an LLM to select from the Actions that the current robot has
registered and return a JSON Behavior Tree.

## Prompt construction

```python
from behavioros.prompts import build_prompt

prompt = build_prompt(
    user_request="pick up the red can",
    max_examples=3,
    example_tags=["youbot"],
)
```

`build_prompt()` combines four inputs:

1. the natural-language request;
2. Action IDs, descriptions, and Blackboard reads/writes;
3. tagged few-shot examples;
4. JSON formatting rules.

## Register a few-shot example

```python
from behavioros.prompts import example

@example(
    name="pick_red_can",
    request="pick up the red can",
    response={
        "tree": {
            "type": "pick_can",
            "name": "Pick red can",
            "nearest_can": "${nearest_red_can}",
        }
    },
    tags=["youbot", "pick"],
)
def pick_red_can_example():
    pass
```

Examples are registered at import time and can be filtered by robot or task tags.

## Generated format

```json
{
  "initial_values": {
    "move_speed": 0.15
  },
  "tree": {
    "type": "sequence",
    "name": "Main task",
    "children": [
      {"type": "move_forward", "name": "Approach"},
      {"type": "stop", "name": "Stop"}
    ]
  }
}
```

Action fields are read from the shared Blackboard. Values that need runtime
recomputation can be written as placeholders such as `${nearest_red_can}`.

## Robot-specific context

Each client appends constraints and examples for its robot. This reduces invalid
Action selection, but it is prompt guidance rather than a security boundary.

## OpenAI integration

`OpenAIClient` currently uses the Chat Completions API and requests JSON-object output.
The optional base interface allows another client to be implemented later, but v0.1
ships only the OpenAI implementation.

## v0.1 limitations

- A valid JSON response may still contain an invalid tree.
- Prompt rules are not a replacement for schema and semantic validation.
- Example selection is tag filtering, not semantic retrieval.
- Prompt quality is not benchmarked in this repository.
- Claude, Ollama, and other providers are not implemented clients in v0.1.

The next architecture plans to parse the model response into a constrained typed Plan
IR and validate it before compilation and execution.
