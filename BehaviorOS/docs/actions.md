# 액션

Action은 로봇별 동작을 담는 Behavior Tree의 leaf node입니다. LLM은 ID로 Action을
선택할 수 있지만, 실제 코드를 실행할 수 있는 것은 현재 프로세스에 등록된 Python
클래스뿐입니다.

## Action 정의

```python
import py_trees
from behavioros.core import BaseAction, action

@action(
    "move_forward",
    description="로봇을 앞으로 이동",
    reads=["devices", "move_speed"],
    writes=["movement_status"],
)
class MoveForwardAction(BaseAction):
    def execute(self) -> py_trees.common.Status:
        devices = self.read_blackboard("devices", {})
        speed = self.read_blackboard("move_speed", 0.1)

        # 로봇별 device API를 통해 명령을 보냅니다.
        self.write_blackboard("movement_status", "running")
        return py_trees.common.Status.RUNNING
```

데코레이터는 다음 정보를 등록합니다.

- JSON의 `type` 필드에서 사용하는 Action ID
- LLM prompt에 포함할 설명
- Action이 읽고 쓰는 Blackboard key

## JSON 표현

```json
{
  "type": "move_forward",
  "name": "작업 영역으로 접근"
}
```

`TreeExecutor`는 Action registry에서 `move_forward`를 찾아 공유 Blackboard와 함께
클래스를 생성합니다.

## 상태 반환 규칙

Action은 `py_trees` 상태 중 하나를 반환합니다.

| 상태 | 의미 |
| --- | --- |
| `RUNNING` | 다음 tick에도 계속 실행해야 함 |
| `SUCCESS` | 동작을 성공적으로 완료함 |
| `FAILURE` | 동작을 완료할 수 없음 |

나중에 같은 Action을 다시 실행할 수 있도록 완료나 실패 시 내부 상태를 정리해야
합니다.

## Blackboard 입출력

```python
target = self.read_blackboard("nearest_can")
self.write_blackboard("pick_status", "success")
```

reads/writes 메타데이터는 이 계약을 문서화하고 prompt에도 포함됩니다. 그러나
v0.1에서는 이를 강제하는 runtime type system이나 validator가 아닙니다.

## import 시 등록

데코레이터는 Python이 모듈을 import할 때 실행됩니다.

```python
import controllers.youbot_controller.youbot_actions  # YouBot Action 등록
```

Controller는 prompt를 만들거나 트리를 생성하기 전에 해당 로봇의 Action 모듈을
import해야 합니다.

## Control-flow node

JSON parser는 다음 composite도 인식합니다.

- `sequence`: 자식을 순서대로 실행하고 하나가 실패하면 실패
- `selector` 또는 `fallback`: 성공하는 자식이 나올 때까지 순서대로 시도
- `parallel`: 자식을 함께 tick하고 모두 성공하면 성공

이 구조는 미리 정한 실행 흐름이며 LLM을 다시 호출하는 동적 재계획은 아닙니다.

## v0.1 한계

- Action 입력은 typed port가 아니라 공유 Blackboard key입니다.
- reads/writes 메타데이터는 실행 시 강제되지 않습니다.
- 여러 node가 같은 field를 사용하면 트리 생성 중 하나의 공유 key를 덮어쓸 수
  있습니다.
- 알 수 없는 Action ID를 fail-closed 방식으로 차단하는 validator가 없습니다.
