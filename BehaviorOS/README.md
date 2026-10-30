# BehaviorOS

> A Webots-based prototype that executes LLM-generated JSON plans as Behavior Trees
> and resolves symbolic plan values either once at construction or repeatedly at runtime.

[English documentation](docs/index.en.md) · [한국어 문서](docs/index.md)

## Overview

BehaviorOS v0.1 explores a boundary between language-model planning and robot
execution:

```text
Natural-language command
  → Action-aware prompt
  → OpenAI JSON Behavior Tree
  → Static Resolution
  → py_trees execution object
  → Runtime Resolution before every tick
  → Robot Actions and execution logs
```

The LLM defines the execution structure while registered resolvers provide concrete
values. A value that should not be embedded in the plan remains a symbolic reference,
such as `${drop_goal}` or `${nearest_red_can}`. BehaviorOS either resolves it once
while building the tree or recomputes it before every tick.

BehaviorOS calls this mechanism **Value Resolution**:

- **Static Resolution** computes and injects a value once while the tree is built.
- **Runtime Resolution** recomputes a value before every tick when it must stay current.

A Runtime value can come from a sensor, the Blackboard, time, robot state, or any
other registered computation. The distinction is about **when a value is computed**,
not where it comes from.

## Implemented in v0.1

- Prompt construction from registered Actions and tagged few-shot examples
- OpenAI JSON-object response handling
- JSON-to-`py_trees` conversion for Sequence, Selector/Fallback, and Parallel nodes
- Static and Runtime Resolution with a shared Blackboard
- Decorator-based Action and resolver registries
- Tick-level file logging
- Webots controllers for KUKA YouBot, TurtleBot3, and TIAGo

## Current boundaries

The following are not implemented in v0.1:

- Enforced JSON Schema or semantic plan validation
- Parameter constraints or a safety-policy layer
- LLM replanning during execution
- A BehaviorTree.CPP executor
- A ROS 2 package or physical robot integration
- LLM provider clients other than OpenAI

v0.1 is therefore a research and design prototype, not a safety-certified execution
framework for physical robots.

## Project structure

```text
behavioros/             # Execution, Actions, Value Resolution, prompts, logging
controllers/            # Webots clients, controllers, and robot-specific Actions
worlds/                 # YouBot, TurtleBot3, and TIAGo demo worlds
docs/                   # Korean and English MkDocs documentation
```

## Run locally

### Requirements

- Python 3.10+
- Webots R2023b+
- An OpenAI API key

### Install

```bash
python -m venv venv
source venv/bin/activate
pip install -e ".[llm]"
export OPENAI_API_KEY="your-api-key"
```

Open `worlds/youbot_can_demo.wbt` in Webots, start the simulation, and run the client
from the repository root:

```bash
python controllers/youbot_controller/youbot_client.py
```

Then enter a supported command:

```text
pick up the red can
```

See [Getting Started](docs/getting-started.en.md) for the complete flow and prototype
limitations.
