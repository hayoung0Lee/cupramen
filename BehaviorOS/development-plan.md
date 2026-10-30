# BehaviorOS 3개월 개발 계획

> **장기 목표:** 자연어 로봇 작업을 실행 가능한 BehaviorTree.CPP behavior로
> 변환하는 ROS 2 Jazzy-native 오픈소스 프레임워크

## 1. 목표와 전제

### 장기 목표

```text
자연어 작업
    ↓
Python LLM Planner
    ↓
검증된 Plan IR
    ↓
BehaviorTree.CPP XML
    ↓
C++ BehaviorTree.CPP Executor
    ↓
ROS 2 Action / Service
    ↓
Isaac Sim 또는 실제 로봇
```

Isaac Sim은 BehaviorOS의 일부가 아니라 첫 번째 실행 환경이다. BehaviorOS
코어는 Isaac Sim 없이도 빌드하고 테스트할 수 있어야 한다.

### 3개월 목표

다음과 같은 작업을 Isaac Sim에서 시연한다.

```text
"가장 가까운 빨간 큐브를 찾아 보관함에 넣어."
```

데모에서 다음 과정이 모두 보여야 한다.

1. 자연어 입력
2. 사용 가능한 capability 확인
3. prompt와 LLM 응답
4. 구조화된 Plan IR 생성
5. grounding 및 계획 검증
6. BehaviorTree.CPP XML 생성
7. C++ executor 실행
8. ROS 2 capability 호출
9. Isaac Sim 로봇 동작
10. 실행 결과와 전체 trace 확인

### 작업 시간

- 하루 3~4시간, 주 5일
- 주당 약 15~20시간
- 월 약 60~80시간
- 3개월 총 약 180~240시간
- 논문 조사일에는 구현 진척이 없다고 가정한다.
- 논문 조사는 질문별로 time-box하고 결정 기록을 남긴 뒤 종료한다.

## 2. 범위

### 3개월 안에 반드시 포함할 것

- Python 기반 자연어 planner
- C++ BehaviorTree.CPP executor
- Python과 C++ 사이의 ROS 2 경계
- 제한되고 typed된 Plan IR
- capability/entity/spatial grounding
- schema 및 semantic validation
- Plan IR → BT XML compiler
- prompt부터 action 결과까지 연결되는 로깅
- 시뮬레이터 없는 mock end-to-end 데모
- Isaac Sim pick-and-place vertical slice
- timeout, failure, cancellation
- 설치 및 실행 문서

### 3개월 이후로 미룰 것

- 실행 중 dynamic replanning
- plan critique와 repair
- 구조화된 deliberation
- dynamic BT expansion
- 여러 LLM provider의 완전한 지원
- 실제 perception model
- 실제 로봇 통합
- 멀티 로봇 협업
- policy/approval engine
- ROS build farm 및 `apt` 배포
- 논문용 대규모 benchmark

## 3. 기본 아키텍처

### Python의 책임

- CLI 및 향후 Python SDK
- LLM provider 호출
- prompt 구성과 버전 관리
- capability catalog 구성
- 자연어 개념의 planning-time grounding
- Plan IR parsing
- schema 및 semantic validation
- 결정적인 BT XML 생성
- 향후 critique, repair, replanning

### C++의 책임

- BehaviorTree.CPP factory와 executor
- C++ BT node plugin
- tick lifecycle
- BehaviorTree.CPP ports와 blackboard
- ROS 2 Action/Service client
- timeout, halt, cancellation
- runtime validation
- node transition event 생성

Python에서 BehaviorTree.CPP를 `pybind11`로 직접 감싸지 않는다. Python planner와
C++ executor는 별도 ROS 2 node로 실행하고 Action 또는 Service로 통신한다.

```text
Python planner
    │
    │ ExecuteTree Action
    ▼
C++ BT executor
    │
    │ ROS 2 Actions / Services
    ▼
Capability providers
```

### 패키지 구성

처음에는 경계를 검증하기 위해 두 패키지로 시작한다.

```text
BehaviorOS/
├── behavioros_planner/       # Python
└── behavioros_bt_executor/   # C++
```

인터페이스가 안정된 뒤 다음과 같이 분리한다.

```text
BehaviorOS/
├── behavioros_interfaces/     # ROS Action, Service, Message
├── behavioros_planner/        # 자연어 → Plan IR → BT XML
├── behavioros_bt_executor/    # BehaviorTree.CPP 실행
├── behavioros_bt_nodes/       # 범용 ROS-backed BT plugin
├── behavioros_observability/  # event schema, sink, reader
├── behavioros_mock_demo/      # 시뮬레이터 없는 mock 환경
└── behavioros_isaac_demo/     # Isaac Sim adapter와 launch
```

패키지들은 하나의 Git 저장소에 둔다. 사용자는 저장소를 자신의 기존 ROS
workspace의 `src/`에 clone하거나 별도 workspace에서 빌드할 수 있다.

## 4. Plan IR과 Grounding

Plan IR은 실행 순서, capability 호출, 입출력과 grounding 정보를 함께 표현하는
하나의 중간 모델이다. Grounding은 Plan IR과 별개의 포맷이 아니라 Plan IR의
입력값 타입을 정의하는 개념이다.

```text
Plan IR
├── control flow
├── capability invocation
├── typed inputs and outputs
└── grounded value
    ├── entity_query
    ├── semantic_entity
    ├── output_reference
    ├── literal
    └── config_reference
```

### 4.1 통합 Plan IR 예시

```json
{
  "version": "behavioros/v1alpha1",
  "goal": "가장 가까운 빨간 큐브를 보관함에 넣기",
  "root": {
    "type": "sequence",
    "children": [
      {
        "id": "detect_target",
        "capability": "perception.detect_object",
        "inputs": {
          "query": {
            "kind": "entity_query",
            "entity_type": "object",
            "constraints": {
              "shape": "cube",
              "color": "red"
            },
            "selection": "nearest"
          }
        },
        "outputs": {
          "object": "target_object",
          "pose": "target_pose"
        }
      },
      {
        "id": "pick_target",
        "capability": "manipulation.pick",
        "inputs": {
          "object": {
            "kind": "output_reference",
            "step": "detect_target",
            "port": "object"
          },
          "target_pose": {
            "kind": "output_reference",
            "step": "detect_target",
            "port": "pose"
          }
        }
      },
      {
        "id": "locate_destination",
        "capability": "world_model.locate_entity",
        "inputs": {
          "entity": {
            "kind": "semantic_entity",
            "entity_type": "location",
            "id": "storage_bin"
          }
        },
        "outputs": {
          "pose": "destination_pose"
        }
      },
      {
        "id": "place_target",
        "capability": "manipulation.place",
        "inputs": {
          "target_pose": {
            "kind": "output_reference",
            "step": "locate_destination",
            "port": "pose"
          }
        }
      }
    ]
  }
}
```

### 4.2 Grounded value 종류

| `kind` | 용도 | 예시 |
|---|---|---|
| `entity_query` | 실행 중 찾아야 하는 대상 | 가장 가까운 빨간 큐브 |
| `semantic_entity` | 알려진 의미적 대상 | `storage_bin` |
| `output_reference` | 앞선 step의 출력 | `detect_target.pose` |
| `literal` | 계획에 직접 포함된 값 | 속도 `0.2` |
| `config_reference` | 설정에서 가져올 값 | 기본 motion profile |

### 4.3 Capability grounding

자연어 동작을 현재 로봇이 제공하는 capability에 연결한다.

```text
"집어", "들어 올려", "grasp"
        ↓
manipulation.pick
        ↓
/pick_object ROS 2 Action
```

Capability catalog 예시:

```yaml
name: manipulation.pick
description: pose로 지정된 물체를 집는다
inputs:
  target_pose:
    type: geometry_msgs/msg/PoseStamped
outputs:
  grasped:
    type: boolean
execution:
  type: ros_action
  endpoint: /pick_object
```

Planner는 catalog에 등록된 capability만 선택할 수 있다. Validator는 현재
로봇의 capability 제공 여부와 입출력 타입을 확인한다.

### 4.4 Entity grounding

자연어 대상 표현을 구조화된 query 또는 실제 world entity에 연결한다.

```text
"가장 가까운 빨간 큐브"
        ↓ planning time
{shape: cube, color: red, selection: nearest}
        ↓ execution time
object_id: cube_17, pose: PoseStamped(...)
```

Planning 단계에서는 query를 만들고, 실행 단계에서는 `DetectObject` 같은
명시적인 BT node와 perception capability가 실제 대상과 pose를 찾는다.

### 4.5 Spatial grounding

장소 표현을 semantic entity, frame 또는 pose에 연결한다.

```text
"보관함"
    ↓
semantic location: storage_bin
    ↓
map-frame PoseStamped
```

정적인 위치는 config에서 binding할 수 있다. 움직이거나 불확실한 위치는
`LocateEntity` 같은 명시적인 runtime node가 찾아야 한다.

### 4.6 Grounding 단계

#### Planning-time grounding

- 자연어 동사 → capability ID
- 객체 표현 → `entity_query`
- “천천히” → named motion profile
- 장소 표현 → semantic location ID

#### Compile-time binding

- capability ID → C++ BT plugin
- Plan IR input/output → BT port
- static config → initial blackboard value
- `output_reference` → 명시적인 blackboard key

#### Execution-time grounding

- entity query → 관측된 object ID
- object → 현재 `PoseStamped`
- 동적 semantic location → 현재 pose
- robot/world state → condition 결과

Execution-time grounding은 숨겨진 resolver hook이 아니라 tree에 보이는 BT node
또는 ROS capability 호출로 표현한다.

### 4.7 BT XML 변환 예시

통합 Plan IR은 결정적으로 다음과 같은 XML로 컴파일된다.

```xml
<BehaviorTree ID="PickAndPlace">
  <Sequence>
    <DetectObject
      shape="cube"
      color="red"
      selection="nearest"
      object="{target_object}"
      pose="{target_pose}" />
    <PickObject
      object="{target_object}"
      target_pose="{target_pose}" />
    <LocateEntity
      entity_id="storage_bin"
      pose="{destination_pose}" />
    <PlaceObject
      target_pose="{destination_pose}" />
  </Sequence>
</BehaviorTree>
```

```text
Plan IR entity_query     → DetectObject 입력 port
Plan IR output_reference → BehaviorTree.CPP blackboard 연결
Plan IR semantic_entity  → LocateEntity 입력 port
Capability ID            → 등록된 C++ BT plugin
```

BehaviorTree.CPP blackboard를 실행 중 유일한 BT blackboard로 사용한다. 별도의
공유 Python runtime blackboard나 모든 값을 매 tick 갱신하는 resolver loop는
두지 않는다.

### 4.8 지속적으로 변하는 상태

연속 상태가 필요하면 다음 중 하나로 명시한다.

- ROS topic subscriber BT node
- state cache를 읽는 condition node
- ROS-backed world-state provider
- freshness를 검사하는 decorator

Runtime observation에는 가능한 경우 다음 정보를 포함한다.

- ROS timestamp
- coordinate frame
- source capability
- freshness 또는 expiry
- confidence

### 4.9 모호성 처리

Grounding 결과는 성공만 있는 것이 아니다.

```text
GROUNDED
AMBIGUOUS
NOT_FOUND
UNSUPPORTED
REQUIRES_OBSERVATION
REQUIRES_USER_INPUT
```

첫 버전에서 완전한 clarification 대화를 구현할 필요는 없지만, 모호한 표현을
임의로 선택해 실행해서는 안 된다.

## 5. 로깅과 관찰 가능성

로깅은 확장 기능이 아니라 BehaviorOS의 핵심 기능이다. 하나의 execution ID로
planning과 runtime을 연결한다.

```text
사용자 요청
    ↓
prompt
    ↓
LLM request/response
    ↓
Plan IR
    ↓
grounding/validation
    ↓
BT XML
    ↓
BT node transition
    ↓
ROS capability goal/feedback/result
    ↓
최종 결과
```

### 5.1 실행 artifact

```text
artifacts/
└── <execution-id>/
    ├── manifest.json
    ├── task.json
    ├── prompt.json
    ├── llm-response.json
    ├── plan.json
    ├── grounding.json
    ├── validation.json
    ├── tree.xml
    ├── events.jsonl
    └── result.json
```

초기에는 JSON/JSONL 파일을 사용한다. 저장 기능은 event sink interface 뒤에
두어 향후 rosbag, SQLite, OpenTelemetry 등으로 교체할 수 있게 한다.

### 5.2 Event envelope

```json
{
  "schema_version": "behavioros.events/v1alpha1",
  "execution_id": "uuid",
  "timestamp": "RFC3339 timestamp",
  "source": "planner|executor|capability",
  "event_type": "plan.generated",
  "payload": {}
}
```

초기 event 종류:

- `task.received`
- `prompt.rendered`
- `llm.requested`
- `llm.responded`
- `plan.parsed`
- `grounding.query_created`
- `grounding.entity_selected`
- `grounding.ambiguous`
- `grounding.failed`
- `plan.validation_failed`
- `plan.validated`
- `tree.compiled`
- `tree.started`
- `tree.node_transition`
- `capability.goal_sent`
- `capability.feedback`
- `capability.result`
- `tree.halted`
- `tree.completed`
- `task.completed`

### 5.3 Grounding provenance

```json
{
  "event_type": "grounding.entity_selected",
  "execution_id": "task-001",
  "payload": {
    "expression": "가장 가까운 빨간 큐브",
    "query": {
      "shape": "cube",
      "color": "red",
      "selection": "nearest"
    },
    "selected": "cube_17",
    "source": "isaac_ground_truth"
  }
}
```

### 5.4 Prompt 로깅과 보안

- prompt logging mode: `off`, `metadata`, `full`
- prompt template version과 hash
- model/provider와 generation parameters
- 응답 hash, latency, token metadata
- secret 및 환경변수 redaction
- multimodal 입력 저장 정책
- API key와 인증 header는 절대 기록하지 않음

### 5.5 Runtime 로깅 원칙

- logging 실패가 BT 결과를 조용히 바꾸면 안 된다.
- audit sink 실패 시 실행을 막을지는 policy로 설정한다.
- 파일 I/O가 BT tick을 막지 않도록 buffer 또는 비동기 sink를 사용한다.
- blackboard 전체가 아니라 선택된 serializable 값만 기록한다.
- prompt, plan, tree, capability 호출은 같은 execution ID를 사용한다.

## 6. Isaac Sim 연결 전 테스트 전략

Isaac Sim은 mock capability를 교체하는 단계다. 첫 실행 환경이 되어서는 안 된다.

```text
Pure unit tests
    ↓
Plan IR / XML snapshot tests
    ↓
C++ dummy BT nodes
    ↓
Mock ROS 2 Action servers
    ↓
Failure / timeout / cancellation tests
    ↓
Isaac Sim integration
```

### 6.1 Planner와 compiler 테스트

ROS, LLM API, simulator 없이 실행한다.

- Plan IR parsing
- unknown capability rejection
- required input 검증
- output-before-definition 검증
- capability input/output type 검증
- grounding value kind 검증
- 결정적 XML serialization
- XML snapshot 비교
- prompt snapshot과 template version 테스트

LLM 테스트와 compiler 테스트를 분리한다. 녹화된 LLM response fixture로 전체
planning pipeline을 offline 실행할 수 있어야 한다.

### 6.2 C++ dummy BT node

상태를 설정할 수 있는 dummy node를 등록한다.

```text
DetectObject → mock pose를 쓰고 SUCCESS
PickObject   → SUCCESS, FAILURE 또는 N tick 동안 RUNNING
PlaceObject  → SUCCESS
```

이를 통해 XML load, plugin 등록, ports, blackboard, Sequence/Fallback, halt와
event 생성까지 검증한다.

### 6.3 Mock ROS 2 capability server

다음 Action server를 simulator 없이 제공한다.

- `/detect_object`
- `/pick_object`
- `/place_object`

```bash
ros2 launch behavioros_mock_demo demo.launch.py \
  detect_result:=success \
  pick_result:=failure \
  pick_delay:=1.0
```

| 시나리오 | 예상 결과 |
|---|---|
| 모든 capability 성공 | Tree `SUCCESS` |
| 대상 없음 | Fallback 또는 Tree `FAILURE` |
| Pick 1회 실패 | Retry 또는 설정된 Fallback |
| Action timeout | Goal cancel 후 node `FAILURE` |
| 사용자 취소 | 현재 Action halt 및 cancel |
| 알 수 없는 capability | 실행 전 validation 실패 |
| 잘못된 port 연결 | compile/load 단계 실패 |

### 6.4 시뮬레이터 없는 시각적 결과

CLI에서 최소한 다음 상태를 표시한다.

```text
PickAndPlace [RUNNING]
└── Sequence [RUNNING]
    ├── DetectObject [SUCCESS]
    ├── PickObject [RUNNING]
    └── PlaceObject [IDLE]
```

동시에 artifact 디렉터리에 prompt, plan, grounding, XML, node transition과
최종 결과를 저장한다. Groot 호환 XML/tree model도 생성하되, 실시간 Groot
연동은 필수 범위에 포함하지 않는다.

## 7. 1개월 차 — Core와 observable mock demo

**예상 시간:** 60~80시간

**완료 상태:** 자연어나 recorded LLM response가 검증된 Plan IR과 BT XML로
변환되고, C++ BehaviorTree.CPP가 mock ROS capability를 호출하며 전체 trace를
남긴다.

### 1주 차 — 조사와 설계

- LLM 기반 BT 생성 논문 조사
- direct XML과 typed Plan IR 비교
- BehaviorTree.CPP 및 BehaviorTree.ROS2 조사
- Nav2 BT plugin/cancellation 사례 조사
- plan grounding, validation, repair 연구 조사
- ROS 2 logging, rosbag, tracing 선택지 조사
- Plan IR v1alpha1 확정
- capability catalog schema 확정
- event schema와 redaction policy 확정
- Architecture Decision Record 작성

조사는 관련 논문과 primary source 5~8개를 검토하고 구현 선택을 기록하면
종료한다. 남은 의문은 구현을 중단하지 않고 별도 목록으로 관리한다.

### 2주 차 — C++ executor와 runtime event

- `behavioros_bt_executor` 생성
- BehaviorTree.CPP XML load
- dummy C++ node
- tick lifecycle
- timeout, halt, cancellation
- unknown node rejection
- node transition event
- execution ID 전달
- JSONL sink prototype

완료 기준:

- fixture XML이 `SUCCESS`까지 실행된다.
- invalid XML과 unknown node가 실행 전에 거부된다.
- cancel이 active node의 halt까지 전달된다.
- 모든 node transition에 execution ID가 기록된다.

### 3주 차 — Python planner, grounding과 compiler

- `behavioros_planner` 생성
- 통합 Plan IR model
- capability catalog
- capability/entity/spatial grounding
- schema 및 semantic validation
- capability/plugin binding
- output reference와 BT port binding
- deterministic XML compiler
- prompt versioning과 redacted logging
- offline LLM fixture

완료 기준:

- fixture plan이 안정적인 XML snapshot으로 변환된다.
- 잘못된 capability, grounded value, 데이터 참조가 거부된다.
- prompt, response, plan, grounding, validation, XML이 execution ID로 연결된다.

### 4주 차 — Mock end-to-end demo

- mock detect/pick/place Action server
- ROS-backed C++ BT node
- planner → executor Action
- feedback와 result
- configurable success/failure/delay/timeout
- CLI tree view
- end-to-end artifact writer
- mock demo launch

완료 기준:

- Isaac Sim 없이 전체 pipeline이 실행된다.
- success, failure, timeout, cancellation이 보인다.
- runtime 값이 명시적인 BT port로 흐른다.
- 실행 후 전체 trace를 검사할 수 있다.

## 8. 2개월 차 — Isaac Sim vertical slice

**예상 시간:** 60~80시간

**완료 상태:** mock provider를 Isaac provider로 하나씩 교체해 첫 자연어
pick-and-place 작업을 실행한다.

### 5주 차 — Isaac 환경 조사와 ROS 연결

- Isaac Sim 지원 버전 결정
- ROS 2 Jazzy 연결 방식 결정
- robot asset과 manipulation interface 결정
- `/clock`, joint state, TF 검증
- object pose source 결정
- simulator reset/replay 절차 작성

### 6주 차 — Perception capability

첫 데모는 학습 기반 perception 대신 simulator ground truth를 사용한다.

- `/detect_object` Action provider
- `entity_query` 입력
- object ID와 `PoseStamped` 출력
- frame, timestamp, source 기록
- target-not-found와 timeout
- offline replay fixture

같은 `DetectObject` BT node가 mock 또는 Isaac provider와 변경 없이 동작해야
한다.

### 7주 차 — Pick capability

다음 순서로 구현한다.

1. hard-coded pose pick
2. ground-truth pose pick
3. `DetectObject` 출력 pose pick
4. BehaviorTree.CPP 전체 실행

feedback, timeout, cancellation, failure reason과 safe terminal state를 지원한다.

### 8주 차 — Place와 첫 전체 데모

- `/place_object` Action provider
- named drop zone grounding
- place 완료 판정
- object release
- ready/home recovery
- Isaac launch/config package
- simulator capability event까지 포함한 trace

완료 기준:

- 자연어로 단일 pick-and-place가 완료된다.
- 동일한 planner/executor가 mock 환경에서도 동작한다.
- LLM 실패 시 recorded plan으로 데모를 계속할 수 있다.

## 9. 3개월 차 — 복구, 평가와 공개

**예상 시간:** 60~80시간

### 9주 차 — 실패 처리

- target-not-found
- pick retry/fallback
- capability timeout
- user cancellation
- invalid plan
- unavailable capability
- grounding ambiguity/failure
- safe stop/recovery
- machine-readable failure code

실패는 crash가 아니라 BT status와 구조화된 result가 되어야 한다.

### 10주 차 — 평가와 로그 분석

평가 항목:

- Plan IR parse 성공률
- grounding/validation 성공률
- XML compile 성공률
- unknown capability 생성률
- mock 실행 성공률
- Isaac 실행 성공률
- planning/execution latency
- failure classification 정확도
- trace completeness

최소 실행:

- mock 50회
- 대표 Isaac 작업 10회
- simulator reset 후 replay
- LLM 변동성을 제외한 fixed-plan runtime 테스트

### 11주 차 — 문서와 설치 검증

작성할 문서:

- Getting Started
- Architecture
- Plan IR과 Grounding
- Capability 작성법
- Runtime data flow
- Observability와 Logging
- Mock demo
- Isaac Sim demo
- Prompt logging/privacy policy
- Architecture Decision Records

깨끗한 ROS 2 Jazzy workspace에서 다음을 검증한다.

```bash
mkdir -p ~/behavioros_ws/src
git clone <repository> ~/behavioros_ws/src/BehaviorOS
cd ~/behavioros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Isaac Sim이 없는 사용자도 mock demo와 artifact inspection을 실행할 수 있어야
한다.

### 12주 차 — 릴리스와 데모

- `v0.1.0-alpha`
- 오픈소스 라이선스
- Contribution Guide
- Issue template
- 테스트 환경 표
- 알려진 제한사항
- architecture diagram
- mock demo 영상
- Isaac Sim demo 영상
- 민감정보를 제거한 execution artifact 예제

## 10. 논문 조사 방식

논문 조사는 무기한 선행 단계로 두지 않는다.

```text
결정할 질문
    ↓
관련 primary source 3~5개 선정
    ↓
근거와 대안 정리
    ↓
Architecture Decision 작성
    ↓
구현 재개
```

- 작은 기술 결정: 작업일 1일
- 핵심 구조 결정: 작업일 2~3일
- 확신이 부족하면 논문을 계속 추가하지 말고, 대안을 구분할 최소 실험을 만든다.
- 논문을 읽는 동안에는 구현 진척이 없다는 전제로 월별 범위를 유지한다.

## 11. 일정 보호

일정이 밀리면 다음 순서로 제거한다.

1. Groot 실시간 연동
2. 복수 LLM provider
3. 학습 기반 perception
4. dynamic replanning
5. 복잡한 recovery tree 자동 생성
6. ROS build farm 배포
7. Web UI

다음은 제거하지 않는다.

- 로깅과 execution trace
- Plan IR과 grounding 일관성
- validation
- mock execution
- timeout과 cancellation
- 명시적인 BT port data flow
- Isaac Sim vertical slice

## 12. 최종 성공 기준

3개월 후 다음 조건을 만족해야 한다.

- 실제 실행 엔진이 C++ BehaviorTree.CPP다.
- Python planner와 C++ executor를 독립적으로 테스트할 수 있다.
- Isaac Sim 없이 mock capability로 전체 pipeline을 시연할 수 있다.
- LLM 없이 fixture로 deterministic test를 실행할 수 있다.
- Plan IR과 grounding이 하나의 일관된 schema를 사용한다.
- invalid plan과 unavailable capability가 실행 전에 차단된다.
- runtime observation에 frame, timestamp, source가 기록된다.
- ROS Action cancellation이 active BT node와 provider까지 전달된다.
- Isaac Sim을 제거해도 BehaviorOS core가 빌드된다.
- prompt, plan, grounding, validation, XML, node transition, capability 호출과
  결과가 execution ID로 연결된다.
- 새 사용자가 문서만 보고 clone, build, mock demo 실행을 완료할 수 있다.
