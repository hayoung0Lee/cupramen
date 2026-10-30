# API Reference

This is the small public surface of the v0.1 Python prototype.

## TreeExecutor

```python
from behavioros.core import TreeExecutor

executor = TreeExecutor(json_data, blackboard)
tick_result = executor.tick()
```

| API | Purpose |
| --- | --- |
| `tick()` | Tick the `py_trees` tree once |
| `add_before_tick(hook)` | Register a pre-tick hook |
| `add_after_tick(hook)` | Register a post-tick hook |
| `get_execution_result()` | Read current status, ticks, duration, and Blackboard |
| `status` | Read the root node status name |

## TreeBuilder

```python
from behavioros.core.builder import TreeBuilder

tree = (
    TreeBuilder(tree_json, blackboard, devices)
    .with_runtime()
    .with_logging()
    .build()
)
```

| API | Purpose |
| --- | --- |
| `with_runtime()` | Add Runtime Resolution when placeholders are present |
| `with_logging(log_dir="logs")` | Add tick-level file logging |
| `with_status_printer(every=10)` | Print status periodically |
| `with_hook(hook, before=False)` | Add a custom hook |
| `build()` | Return the configured `TreeExecutor` |

## Actions

```python
from behavioros.core import BaseAction, action

@action(
    "pick_can",
    description="Pick a detected can",
    reads=["devices", "nearest_can"],
    writes=["pick_status"],
)
class PickCanAction(BaseAction):
    def execute(self):
        ...
```

`BaseAction` provides `read_blackboard(key, default)` and
`write_blackboard(key, value)`. Subclasses implement `execute()` and return a
`py_trees.common.Status`.

## Resolvers

```python
from behavioros.resolution import runtime_resolver, static_resolver

@static_resolver("drop_goal")
def drop_goal(context):
    return {"x": 0.3, "y": 0.0, "z": 0.15}

@runtime_resolver("nearest_red_can")
def nearest_red_can(context):
    return detect(context.devices["camera"])
```

Resolver handlers receive a `ResolutionContext` containing `devices`, `blackboard`,
an optional `robot`, and an optional `sensor_cache`.

## Prompts and LLM

```python
from behavioros.llm import OpenAIClient
from behavioros.prompts import build_prompt

prompt = build_prompt("pick up the red can", example_tags=["youbot"])
response = await OpenAIClient(model="gpt-4o-mini").generate(prompt)
```

The response is JSON text. The caller is responsible for parsing it and v0.1 does not
provide an enforced plan-validation API.
