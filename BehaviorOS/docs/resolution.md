# Value Resolution

> 계획 속 참조를 실제 값으로 해석합니다. 핵심 질문은 **그 값을 언제 계산하는가?**입니다.

LLM이 만든 tree에는 `${drop_goal}`, `${nearest_red_can}` 같은 placeholder가 들어갈
수 있습니다. BehaviorOS는 값의 종류나 출처가 아니라 **계산 시점**을 두 가지로
나눕니다.

## Static과 Runtime Resolution

| 타입 | 실행 시점 | 예시 | 용도 |
| --- | --- | --- | --- |
| Static | 트리를 만드는 동안 한 번 | `${drop_goal}`, `${transport_speed}` | 한 번 주입하면 되는 값 |
| Runtime | 매 tree tick 전 | `${nearest_red_can}`, `${elapsed_time}` | 계속 다시 계산해야 하는 값 |

### Static Resolution

```python
@static_resolver("drop_goal")
def resolve_drop_goal(context):
    return {"x": 0.53, "y": 0.54, "z": 0.15}
```

`TreeExecutor`가 `py_trees` 객체를 만들기 전에 JSON tree 안의 placeholder를 실제
값으로 바꿉니다. Field 전체가 하나의 placeholder라면 반환값의 type도 보존합니다.

### Runtime Resolution

```python
@runtime_resolver("nearest_red_can")
def resolve_nearest_red_can(context):
    return detect_red_can(context.devices["camera"])
```

Runtime placeholder는 tree에 남습니다. `RuntimeProcessor`가 이를 찾아 before-tick
hook을 만들고 resolver의 최신 결과를 공유 Blackboard에 기록합니다.

## 예제

LLM은 실제 좌표 대신 이름으로 된 reference를 만듭니다.

```json
{
  "tree": {
    "type": "pick_can",
    "name": "Pick red can",
    "nearest_can": "${nearest_red_can}"
  }
}
```

Tick 전에 Runtime Resolution이 다음 값을 갱신합니다.

```python
blackboard["nearest_red_can"] = {
    "detected": True,
    "x_robot": 0.35,
    "distance": 0.36,
}
```

YouBot Action은 `nearest_can`에서 `${nearest_red_can}` reference를 읽고 Blackboard의
최신 값을 다시 조회합니다. 계획 구조는 그대로지만 resolver 결과는 계속 변할 수
있습니다.

## 타입 선택

```text
이 tree가 실행되는 동안 값이 바뀔 수 있나요?
├── 아니오 → Static Resolution
└── 예     → Runtime Resolution
```

한 번 계산해 주입하면 충분한 값에는 Static Resolution을 사용합니다. Sensor,
Blackboard 상태, 경과 시간, 다른 계산 결과처럼 계속 다시 계산해야 하는 값에는
Runtime Resolution을 사용합니다.

## ResolutionContext

Resolver가 한 로봇을 직접 import하지 않도록 context를 전달합니다.

```python
@dataclass
class ResolutionContext:
    devices: dict
    blackboard: dict
    robot: object | None = None
    sensor_cache: dict | None = None
```

## 등록

`@static_resolver`, `@runtime_resolver`는 해당 모듈을 import할 때 handler를
등록합니다. Robot controller는 tree를 만들기 전에 resolver 모듈을 import해야
합니다.

## v0.1에서 주의할 점

- 참조된 runtime key의 resolver는 값의 출처와 관계없이 매 tick 실행됩니다.
- 무거운 perception 연산에는 별도의 cache나 낮은 갱신 주기가 필요합니다.
- Resolver 예외는 출력되지만 자동으로 tree를 실패시키거나 취소하지 않습니다.
- Value Resolution은 계획의 참조를 계산 결과로 해석하는 기능이며 plan
  validation이나 safety policy가 아닙니다.
