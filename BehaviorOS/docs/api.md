# API 레퍼런스

v0.1 Python 프로토타입의 주요 API입니다.

## TreeExecutor

```python
from behavioros.core import TreeExecutor

executor = TreeExecutor(json_data, blackboard)
tick_result = executor.tick()
```

| API | 용도 |
| --- | --- |
| `tick()` | `py_trees` 트리를 한 번 tick |
| `add_before_tick(hook)` | tick 전 hook 등록 |
| `add_after_tick(hook)` | tick 후 hook 등록 |
| `get_execution_result()` | 상태, tick 수, 실행 시간, Blackboard 반환 |
| `status` | root node의 상태 이름 반환 |

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

| API | 용도 |
| --- | --- |
| `with_runtime()` | runtime placeholder가 있을 때 Runtime Resolution 추가 |
| `with_logging(log_dir="logs")` | tick 단위 파일 로깅 추가 |
| `with_status_printer(every=10)` | 일정 주기마다 상태 출력 |
| `with_hook(hook, before=False)` | custom hook 추가 |
| `build()` | 설정된 `TreeExecutor` 반환 |

## Actions

```python
from behavioros.core import BaseAction, action

@action(
    "pick_can",
    description="감지된 캔 집기",
    reads=["devices", "nearest_can"],
    writes=["pick_status"],
)
class PickCanAction(BaseAction):
    def execute(self):
        ...
```

`BaseAction`은 `read_blackboard(key, default)`와
`write_blackboard(key, value)`를 제공합니다. 하위 클래스는 `execute()`를 구현하고
`py_trees.common.Status`를 반환합니다.

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

Resolver에는 `devices`, `blackboard`, 선택적인 `robot`, `sensor_cache`를 담은
`ResolutionContext`가 전달됩니다.

## Prompts and LLM

```python
from behavioros.llm import OpenAIClient
from behavioros.prompts import build_prompt

prompt = build_prompt("pick up the red can", example_tags=["youbot"])
response = await OpenAIClient(model="gpt-4o-mini").generate(prompt)
```

응답은 JSON 문자열입니다. 호출하는 쪽에서 파싱해야 하며 v0.1에는 이 계획을
강제로 검증하는 API가 없습니다.
