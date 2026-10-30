# Actions

Actions are the leaf nodes that contain robot-specific behavior. The LLM may select
an Action by ID, but only Python classes registered in the process can execute code.

## Define an Action

```python
import py_trees
from behavioros.core import BaseAction, action

@action(
    "move_forward",
    description="Move the robot forward",
    reads=["devices", "move_speed"],
    writes=["movement_status"],
)
class MoveForwardAction(BaseAction):
    def execute(self) -> py_trees.common.Status:
        devices = self.read_blackboard("devices", {})
        speed = self.read_blackboard("move_speed", 0.1)

        # Send a command through robot-specific device APIs.
        self.write_blackboard("movement_status", "running")
        return py_trees.common.Status.RUNNING
```

The decorator registers:

- an Action ID used by the JSON `type` field;
- a description used in the LLM prompt;
- Blackboard keys read and written by the Action.

## JSON representation

```json
{
  "type": "move_forward",
  "name": "Approach the work area"
}
```

`TreeExecutor` looks up `move_forward` in the Action registry and instantiates the
class with the shared Blackboard.

## Status contract

An Action returns one of the `py_trees` statuses:

| Status | Meaning |
| --- | --- |
| `RUNNING` | The Action needs another tick |
| `SUCCESS` | The Action finished successfully |
| `FAILURE` | The Action could not complete |

Internal Action state should be reset when the Action finishes or fails so a later
tree can run it predictably.

## Blackboard I/O

```python
target = self.read_blackboard("nearest_can")
self.write_blackboard("pick_status", "success")
```

The metadata documents this contract and is included in prompts. In v0.1 it is not a
runtime type system: missing keys are not prevented by an enforced validator.

## Registration at import time

The decorator runs when Python imports the module:

```python
import controllers.youbot_controller.youbot_actions  # registers YouBot Actions
```

The controller must import the correct robot module before generating a prompt or
building a tree.

## Control-flow nodes

The JSON parser also recognizes:

- `sequence`: run children in order until one fails;
- `selector` or `fallback`: try children until one succeeds;
- `parallel`: tick children together and succeed when all succeed.

These are execution structures, not automatic LLM replanning.

## v0.1 limitations

- Action inputs are shared Blackboard keys rather than typed ports.
- Metadata describes reads and writes but is not enforced at runtime.
- Multiple nodes using the same field can overwrite one shared key while the tree is
  built.
- Unknown Action IDs are not currently rejected with a fail-closed validator.
