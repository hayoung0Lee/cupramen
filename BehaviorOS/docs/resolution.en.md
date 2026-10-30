# Value Resolution

> Connect a name in the plan to a concrete value. The key question is: **when should
> that value be read?**

An LLM-generated tree may contain placeholders such as `${drop_goal}` or
`${nearest_red_can}`. BehaviorOS separates them by **when they are computed**, not by
their type or source.

## Static and Runtime Resolution

| Type | When it runs | Examples | Intended use |
| --- | --- | --- | --- |
| Static | Once while building the tree | `${drop_goal}`, `${transport_speed}` | Values that need one injection |
| Runtime | Before every tree tick | `${nearest_red_can}`, `${elapsed_time}` | Values that need recomputation |

### Static Resolution

```python
@static_resolver("drop_goal")
def resolve_drop_goal(context):
    return {"x": 0.53, "y": 0.54, "z": 0.15}
```

The placeholder is replaced in the JSON tree before `TreeExecutor` builds the
`py_trees` object. The resulting type is preserved when the whole field is one
placeholder.

### Runtime Resolution

```python
@runtime_resolver("nearest_red_can")
def resolve_nearest_red_can(context):
    return detect_red_can(context.devices["camera"])
```

The placeholder stays in the tree. `RuntimeProcessor` discovers it, creates a
before-tick hook, and writes the latest resolver result to the shared Blackboard.

## Example

The LLM produces a symbolic reference:

```json
{
  "tree": {
    "type": "pick_can",
    "name": "Pick red can",
    "nearest_can": "${nearest_red_can}"
  }
}
```

Before a tick, Runtime Resolution updates:

```python
blackboard["nearest_red_can"] = {
    "detected": True,
    "x_robot": 0.35,
    "distance": 0.36,
}
```

The YouBot Action reads `nearest_can`, sees the `${nearest_red_can}` reference, and
dereferences the latest Blackboard value. The plan structure stays unchanged while
the resolver result can change.

## Choosing a type

```text
Can the value change while this tree is running?
├── No  → Static Resolution
└── Yes → Runtime Resolution
```

Use Static Resolution when one computation and injection is sufficient. Use Runtime
Resolution when a value must be recomputed, whether it comes from a sensor, the
Blackboard, elapsed time, robot state, or another calculation.

## ResolutionContext

Resolvers receive a context rather than importing one robot directly:

```python
@dataclass
class ResolutionContext:
    devices: dict
    blackboard: dict
    robot: object | None = None
    sensor_cache: dict | None = None
```

## Registration

`@static_resolver` and `@runtime_resolver` register handlers when their module is
imported. A robot controller must import its resolver module before it builds a tree.

## Practical limits in v0.1

- A runtime resolver is executed for every referenced runtime key on every tick,
  regardless of where its value comes from.
- Expensive perception work needs its own caching or lower update rate.
- Resolver exceptions are printed; they do not automatically fail or cancel the tree.
- Resolution connects values. It is not plan validation or a safety policy.
