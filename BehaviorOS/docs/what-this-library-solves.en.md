# What This Library Solves

BehaviorOS v0.1 does not replace a robot's low-level controller. It is an
orchestration prototype between a natural-language goal and existing robot Actions,
managing **action order and execution-time values**.

## Two layers of robot control

| Layer | Question | Examples |
| --- | --- | --- |
| Low-level control | How should the robot move? | Motor control, IK, grasping, locomotion |
| High-level behavior | What should run, in what order? | Task plan, sequence, fallback, parallel |

BehaviorOS focuses on the second layer. Robot-specific Actions perform movement and
grasping; BehaviorOS selects their execution structure.

## Why combine an LLM with a Behavior Tree?

An LLM can structure natural-language intent, but its output is not an execution
guarantee. A Behavior Tree makes action order and success/failure flow explicit.

```text
Sequence
├── FindRedCan
├── PickCan
└── PlaceInBox
```

In v0.1, the LLM generates this structure as JSON, while only registered Actions
contain robot code. The current prototype does not enforce a JSON schema or parameter
constraints on the generated plan.

## The central experiment

Some values should be computed by the execution environment rather than embedded in
the plan. One computation may be enough, or the value may need continuous
recomputation. For example, BehaviorOS can leave `${nearest_red_can}` in the plan and
let a camera resolver connect it to the latest value before every tick.

```text
Planning:  nearest_can = ${nearest_red_can}
                            ↓
Execution: nearest_red_can = {detected: true, x: 0.35, ...}
```

This example uses **Runtime Resolution**. Runtime values can also come from the
Blackboard, time, robot state, or another computation. Static Resolution computes a
value once while the tree is built. This site uses **Value Resolution** as the
umbrella term for both.

## What the prototype provides

- Action-aware prompt construction for natural-language requests
- OpenAI JSON response parsing
- JSON-to-`py_trees` Behavior Tree conversion
- Sequence, Selector/Fallback, and Parallel control flow
- Values computed once during tree construction or recomputed before every tick
- Blackboard-based Action data sharing
- Tick-level execution logs
- Webots examples for YouBot, TurtleBot3, and TIAGo

## What it does not provide

- A validated safety-control layer
- LLM replanning during execution
- A BehaviorTree.CPP or ROS 2 executor
- Deployment-grade timeout, cancellation, or approval policy

These are not presented as v0.1 features. The next version is planned as a separate
redesign based on a typed Plan IR, validation, BehaviorTree.CPP, and ROS 2.
