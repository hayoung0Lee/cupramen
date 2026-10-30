# 프롬프트 시스템

Prompt layer는 현재 로봇에 등록된 Action 중에서 LLM이 필요한 동작을 선택하고 JSON
Behavior Tree를 반환하도록 요청합니다.

## Prompt 생성

```python
from behavioros.prompts import build_prompt

prompt = build_prompt(
    user_request="pick up the red can",
    max_examples=3,
    example_tags=["youbot"],
)
```

`build_prompt()`는 네 가지 정보를 결합합니다.

1. 자연어 사용자 요청
2. Action ID, 설명, Blackboard reads/writes
3. tag로 선택한 few-shot example
4. JSON 형식 생성 규칙

## Few-shot example 등록

```python
from behavioros.prompts import example

@example(
    name="pick_red_can",
    request="pick up the red can",
    response={
        "tree": {
            "type": "pick_can",
            "name": "Pick red can",
            "nearest_can": "${nearest_red_can}",
        }
    },
    tags=["youbot", "pick"],
)
def pick_red_can_example():
    pass
```

Example은 모듈 import 시 등록되며 로봇이나 작업 tag로 필터링할 수 있습니다.

## 생성 형식

```json
{
  "initial_values": {
    "move_speed": 0.15
  },
  "tree": {
    "type": "sequence",
    "name": "Main task",
    "children": [
      {"type": "move_forward", "name": "Approach"},
      {"type": "stop", "name": "Stop"}
    ]
  }
}
```

Action field는 공유 Blackboard를 통해 읽습니다. Runtime에 다시 계산할 값은
`${nearest_red_can}` 같은 placeholder로 표현할 수 있습니다.

## 로봇별 context

각 client는 해당 로봇에서 사용할 수 있는 Action과 제약을 추가합니다. 이는 잘못된
Action 선택을 줄이기 위한 prompt guidance이며 security boundary는 아닙니다.

## OpenAI 연동

`OpenAIClient`는 현재 Chat Completions API를 사용하고 JSON object 출력을
요청합니다. Base interface를 통해 다른 client를 구현할 수 있지만 v0.1에 포함된
실제 provider 구현은 OpenAI뿐입니다.

## v0.1 한계

- 올바른 JSON도 실행할 수 없는 tree일 수 있습니다.
- Prompt 규칙은 schema/semantic validation을 대신하지 않습니다.
- Example 선택은 tag filtering이며 semantic retrieval이 아닙니다.
- Prompt 품질을 측정하는 benchmark가 없습니다.
- Claude, Ollama 등 다른 provider client는 구현되어 있지 않습니다.

다음 아키텍처는 model 응답을 제한되고 typed된 Plan IR로 파싱하고, compile과 실행
전에 이를 명시적으로 검증할 계획입니다.
