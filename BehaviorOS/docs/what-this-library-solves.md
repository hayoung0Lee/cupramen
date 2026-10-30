# 해결하려는 문제

BehaviorOS v0.1은 로봇의 저수준 제어기를 새로 만드는 프로젝트가 아닙니다.
자연어 목표와 기존 로봇 Action 사이에서 **행동의 순서와 실행 시점의 값**을
관리하는 오케스트레이션 프로토타입입니다.

## 두 개의 로봇 제어 층

| 층 | 질문 | 예시 |
| --- | --- | --- |
| 저수준 제어 | 어떻게 움직일 것인가? | 모터 제어, IK, grasp, locomotion |
| 고수준 행동 | 무엇을 어떤 순서로 할 것인가? | task plan, sequence, fallback, parallel |

BehaviorOS는 두 번째 층에 집중합니다. 실제 이동과 grasp는 각 로봇의 Action이
담당하고, BehaviorOS는 어떤 Action을 어떤 구조로 실행할지 정합니다.

## 왜 LLM과 Behavior Tree를 함께 쓰는가

LLM은 자연어 의도를 구조화하는 데 유용하지만 출력이 곧 실행 규칙은 아닙니다.
Behavior Tree는 실행 순서와 성공·실패 흐름을 명시적으로 표현할 수 있습니다.

```text
Sequence
├── FindRedCan
├── PickCan
└── PlaceInBox
```

v0.1에서는 LLM이 이 구조를 JSON으로 생성하고, 등록된 Action만 실제 로봇 코드를
갖습니다. 다만 현재 구현은 LLM 출력에 대한 강제 schema validation이나 parameter
constraint를 제공하지 않습니다.

## 이 프로토타입의 핵심 실험

어떤 값은 계획에 직접 넣는 대신 실행 환경에서 계산해야 합니다. 한 번 계산하면
충분할 수도 있고, 실행 중 계속 다시 계산해야 할 수도 있습니다. 예를 들어
BehaviorOS는 계획에 `${nearest_red_can}` 같은 이름을 남기고, 매 tick 전에 카메라
resolver가 그 이름에 최신 값을 연결할 수 있습니다.

```text
계획 시점: nearest_can = ${nearest_red_can}
                           ↓
실행 시점: nearest_red_can = {detected: true, x: 0.35, ...}
```

이 예시는 **Runtime Resolution**입니다. Sensor 외에도 Blackboard 상태, 시간,
다른 계산 결과처럼 계속 갱신할 값에 사용할 수 있습니다. 트리를 만들 때 한 번만
계산하는 과정은 Static Resolution입니다. 이 문서에서는 두 방식을 합쳐
**Value Resolution**이라고 부릅니다.

## 현재 제공하는 것

- 자연어 요청을 위한 Action-aware prompt 구성
- OpenAI JSON 응답 파싱
- JSON에서 `py_trees` Behavior Tree 생성
- Sequence, Selector/Fallback, Parallel control flow
- 트리 생성 시 한 번 계산할 값과 매 tick 다시 계산할 값의 반영
- Blackboard 기반 Action 데이터 공유
- tick 단위 실행 로그
- Webots용 YouBot, TurtleBot3, TIAGo 예제

## 제공하지 않는 것

- 검증된 안전 제어 계층
- 실행 중 LLM 재계획
- BehaviorTree.CPP 또는 ROS 2 실행기
- 실제 로봇 배포 수준의 timeout, cancellation, approval policy

이 항목들은 v0.1의 기능으로 표현하지 않습니다.
