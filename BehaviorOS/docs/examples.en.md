# Examples

The examples below show the JSON plan format used by v0.1. Actual available Actions
depend on which robot module the controller imports.

## Single Action

```json
{
  "initial_values": {
    "move_speed": 0.15,
    "move_duration": 3.0
  },
  "tree": {
    "type": "move_forward",
    "name": "Move forward"
  }
}
```

## Sequence

```json
{
  "tree": {
    "type": "sequence",
    "name": "Approach and stop",
    "children": [
      {"type": "move_forward", "name": "Approach"},
      {"type": "stop", "name": "Stop"}
    ]
  }
}
```

## Fallback

```json
{
  "tree": {
    "type": "fallback",
    "name": "Choose an available can",
    "children": [
      {
        "type": "pick_can",
        "name": "Try red can",
        "nearest_can": "${nearest_red_can}"
      },
      {
        "type": "pick_can",
        "name": "Try green can",
        "nearest_can": "${nearest_green_can}"
      }
    ]
  }
}
```

Fallback is a predefined execution path. It does not call the LLM again or dynamically
create a new plan.

## Static and Runtime values together

```json
{
  "tree": {
    "type": "sequence",
    "name": "Pick and place",
    "children": [
      {
        "type": "pick_can",
        "name": "Pick nearest red can",
        "nearest_can": "${nearest_red_can}"
      },
      {
        "type": "drop_can",
        "name": "Drop in box",
        "drop_goal": "${drop_goal}"
      }
    ]
  }
}
```

`${drop_goal}` is connected once during Static Resolution.
`${nearest_red_can}` is refreshed during Runtime Resolution before every tick.

## Custom resolver

```python
from behavioros.resolution import runtime_resolver

@runtime_resolver("battery_state")
def battery_state(context):
    sensor = context.devices.get("battery")
    if sensor is None:
        return {"available": False}
    return {"available": True, "level": sensor.getValue()}
```

The module containing the decorator must be imported before the tree is built.

!!! warning "Validate examples against the selected robot"
    v0.1 does not enforce a universal Action schema. A JSON example is executable only
    when every referenced Action and resolver has been registered by the controller.
