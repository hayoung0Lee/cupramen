# 예제

아래 예제는 v0.1의 JSON 계획 형식을 보여줍니다. 실제로 사용할 수 있는 Action은
controller가 어떤 로봇 모듈을 import했는지에 따라 달라집니다.

## 단일 Action

```json
{
  "initial_values": {
    "move_speed": 0.15,
    "move_duration": 3.0
  },
  "tree": {
    "type": "move_forward",
    "name": "앞으로 이동"
  }
}
```

## Sequence

```json
{
  "tree": {
    "type": "sequence",
    "name": "접근 후 정지",
    "children": [
      {"type": "move_forward", "name": "접근"},
      {"type": "stop", "name": "정지"}
    ]
  }
}
```

## Fallback

```json
{
  "tree": {
    "type": "fallback",
    "name": "사용 가능한 캔 선택",
    "children": [
      {
        "type": "pick_can",
        "name": "빨간 캔 시도",
        "nearest_can": "${nearest_red_can}"
      },
      {
        "type": "pick_can",
        "name": "초록 캔 시도",
        "nearest_can": "${nearest_green_can}"
      }
    ]
  }
}
```

Fallback은 미리 정의된 실행 경로입니다. LLM을 다시 호출하거나 새로운 계획을
동적으로 만들지는 않습니다.

## Static과 Runtime 값을 함께 사용

```json
{
  "tree": {
    "type": "sequence",
    "name": "Pick and Place",
    "children": [
      {
        "type": "pick_can",
        "name": "가장 가까운 빨간 캔 집기",
        "nearest_can": "${nearest_red_can}"
      },
      {
        "type": "drop_can",
        "name": "박스에 놓기",
        "drop_goal": "${drop_goal}"
      }
    ]
  }
}
```

`${drop_goal}`은 Static Resolution에서 한 번 연결됩니다.
`${nearest_red_can}`은 매 tick 전 Runtime Resolution에서 갱신됩니다.

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

데코레이터가 있는 모듈은 트리를 생성하기 전에 import해야 합니다.

!!! warning "선택한 로봇을 기준으로 확인"
    v0.1에는 모든 로봇에 공통으로 강제되는 Action schema가 없습니다. 예제에 나온
    모든 Action과 resolver를 controller가 등록한 경우에만 실행할 수 있습니다.
