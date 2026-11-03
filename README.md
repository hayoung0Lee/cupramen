# 컵라면 진열 — BehaviorOS 행동 트리로 동작하는 바퀴형 로봇 (MuJoCo)

| | 링크 |
| --- | --- |
| 발표 자료 (PPT) | [`presentation/cupramen_deck.pptx`](presentation/cupramen_deck.pptx) (시연 영상 포함, PowerPoint에서 재생) |
| 시연 영상 1 — 1단계 자율 진열 | [Google Drive](https://drive.google.com/file/d/18KwTsCxmqHOXhp8NDLHtEb0RAk91i_ji/view?usp=drive_link) |
| 시연 영상 2 — 1단계 실패 복구 | [Google Drive](https://drive.google.com/file/d/1mXBa8EdBgkdYa95MGEwYV54UjXXVnBLJ/view?usp=drive_link) |
| 시연 영상 3 — 2단계 사람 개입 | [Google Drive](https://drive.google.com/file/d/1biMCKDI7z-blqV4Vi-rYDF6TaYEyvG41/view?usp=drive_link) |
| 반복 실험 결과 (seed 0~4) | [`results/results_template.csv`](results/results_template.csv) (제공 양식에 기록, [`results/results.csv`](results/results.csv)와 같은 내용) |

바퀴형 로봇이 무작위로 놓인 상자에서 컵라면 6개를 하나씩 집어 2단 선반의 진열 위치 6곳에 놓습니다.
작업 순서는 [BehaviorOS](BehaviorOS/)(개인 프로젝트, 이번 과제에서 확장)의 JSON Behavior Tree로 정의하고 실행합니다.

- **1단계 자율 진열**: seed 0~4 모두 **6/6** 진열, 로봇과 상자·선반 접촉 0건
- **실패 복구**: 흡착 실패, 들다가 놓침, 운반 중 놓침을 일부러 일으켜도 트리가 끝까지 진행 (재시도 → 안전하게 정리 → 장면을 다시 보고 재계획)
- **2단계 사람 개입**: 진열 마지막 단계에서 사람이 키보드로 제어권을 넘겨받아 놓고 돌려주면, 로봇이 놓인 상태를 검사하고 이어서 진행

---

## 1. 설치

macOS (Apple Silicon), Python 3.11, MuJoCo 3.17.0에서 개발·검증했습니다.

```bash
git clone https://github.com/hayoung0Lee/cupramen.git && cd cupramen
python3.11 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

**시뮬레이션 자산은 저장소에 없습니다.** 과제 안내에 따라 회사가 제공한 `cupramen_sim_assets`는 올리지 않았습니다.
압축을 푼 폴더를 `guide/cupramen_sim_assets`에 두거나, 다른 곳에 두고 경로를 알려 주세요.

```bash
export CUPRAMEN_ASSETS=/path/to/cupramen_sim_assets
```

> 화면(뷰어)이 있는 실행은 macOS에서 `mjpython`으로 합니다(`.venv/bin/mjpython`). Linux에서는 `python`을 쓰면 됩니다. 화면 없는 실행(`--headless`)은 어디서나 `python`입니다.

## 2. 실행

```bash
# 최종 트리를 화면으로 (약 5분). 6개를 모두 진열하면 왼쪽 위에 DONE / SUCCESS
.venv/bin/mjpython scripts/run_bt.py trees/b6_takeover.json --seed 0

# 화면 없이 (약 1분), 결과 요약 출력
.venv/bin/python scripts/run_bt.py trees/b6_takeover.json --seed 0 --headless
```

| 옵션              | 뜻                                                                                                    |
| ----------------- | ----------------------------------------------------------------------------------------------------- |
| `--seed N`        | 배치 번호. 제공된 `randomize.episode_for_seed(N)`로 상자·선반·컵·로봇 출발점을 놓습니다               |
| `--headless`      | 화면 없이 최대 속도로 실행                                                                            |
| `--csv PATH`      | 결과를 `results_template.csv`와 같은 열 순서로 한 줄 기록 (같은 seed 줄은 교체)                       |
| `--inject FAULTS` | 실패를 일부러 일으킴 (아래 3절)                                                                       |
| `--operator`      | 사람 조종자: `keyboard`(화면 실행의 기본), `none`(화면 없는 실행의 기본), `script:키,…`(키 입력 재생) |

### 1단계 실패 복구 시연

```bash
.venv/bin/mjpython scripts/run_bt.py trees/b5_recovery.json --seed 0 --inject seal-fail:1,drop-lift:3,drop-carry:4
```

| 주입           | 일어나는 일                             | 로봇의 대응                                                                                                      |
| -------------- | --------------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| `seal-fail:1`  | 1번째 컵의 첫 흡착이 실패               | 그 자리에서 다시 집기 (`retry`)                                                                                  |
| `drop-lift:3`  | 3번째 컵이 들어 올리는 중 상자로 떨어짐 | 컵의 **새 위치**로 다시 계산해 집기                                                                              |
| `drop-carry:4` | 4번째 컵이 운반 중 바닥에 떨어짐        | 로봇을 안전하게 정리(`recover`) → 컵 위치를 다시 확인해 바닥 컵은 포기(팔이 닿지 않음) → 바닥 컵을 길찾기 장애물로 등록하고 피해서 나머지 진열 |

그 밖에 `seal:P`(모든 흡착이 확률 P로 실패, seed로 재현 가능). 숫자 N은 "몇 번째로 고른 컵인지"이며 컵마다 한 번만 일어납니다([`scripts/faults.py`](scripts/faults.py)).

### 2단계 사람 개입

```bash
.venv/bin/mjpython scripts/run_bt.py trees/b6_takeover.json --seed 0
```

1. 뷰어가 뜨면 **터미널을 클릭**합니다. 키는 터미널에서 받습니다(MuJoCo 뷰어는 대부분의 알파벳 키를 화면 단축키로 씀).
2. `t`를 누르면 로봇이 다음 컵을 들고 칸 앞(내려놓기 직전)에서 멈추고 제어권을 넘깁니다. 화면 왼쪽 위가 `HUMAN CONTROL`이 되고 카메라가 손끝 가까이로 갑니다.
3. 터미널 안내(`cup -> slot_1: forward +4 mm, left +4 mm, down 29 mm`)를 보며 조작합니다.

| 키                          | 동작                                                           |
| --------------------------- | -------------------------------------------------------------- |
| `w` `s` / `a` `d` / `r` `f` | 손끝 앞뒤 / 좌우 / 위아래 1 cm (대문자는 2 mm), 로봇 기준 방향 |
| `i` `k` `j` `l` / `u` `o`   | 베이스 5 cm 이동 / 5° 회전                                     |
| `g` / `space`               | 흡착 켜기(잡기) / 끄기(놓기)                                   |
| `h` 또는 Enter              | 로봇에게 돌려주기                                              |
| `?`                         | 도움말                                                         |

4. `h`를 누르면 로봇이 놓인 상태를 검사합니다(칸 중심 15 mm, 기울기 5° 이내). 통과하면 팔을 빼고 남은 컵을 이어서 진열하고, 벗어났으면 이유를 말하며 다시 요청합니다.

사람이 조종해도 팔·차체가 상자나 선반에 3 mm 안으로 다가가는 명령은 거부됩니다.

## 3. 반복 실험 재현 (seed 0~4)

```bash
for s in 0 1 2 3 4; do
  .venv/bin/python scripts/run_bt.py trees/b6_takeover.json --seed $s --headless --csv results/results.csv
done
```

| seed | 진열 | 소요 시간 (시뮬레이션) | 이동 거리 | 직선 기준 | 사람 개입 | 실패 |
| ---- | ---- | ---------------------- | --------- | --------- | --------- | ---- |
| 0    | 6/6  | 271.0 s                | 23.23 m   | 22.88 m   | 0         | 0    |
| 1    | 6/6  | 270.6 s                | 22.18 m   | 19.95 m   | 0         | 0    |
| 2    | 6/6  | 273.6 s                | 26.26 m   | 19.54 m   | 0         | 0    |
| 3    | 6/6  | 295.0 s                | 34.44 m   | 41.01 m   | 0         | 0    |
| 4    | 6/6  | 254.4 s                | 14.33 m   | 16.45 m   | 0         | 0    |

- 배치는 제공된 `randomize.py`(`episode_for_seed`)로, 진열 판정은 제공된 `count_placed`로, 이동 거리는 제공된 `MobilityLog`로 측정합니다. 시간은 시뮬레이션 시간이라 화면 유무와 관계없이 같습니다.
- 실패 없이 돌린 결과입니다. 로봇이 사람을 부를 조건(아래 5절)이 걸리지 않아 사람 개입은 0회입니다.
- 실패 주입 결과(복구 시연, `seal:0.3,drop-lift:3,drop-carry:2`): [`results/results_with_faults.csv`](results/results_with_faults.csv). 5회 모두 끝까지 진행, 진열 5/6 ×4회, 4/6 ×1회(바닥에 떨어진 컵은 팔이 닿지 않아 포기).

```bash
for s in 0 1 2 3 4; do
  .venv/bin/python scripts/run_bt.py trees/b5_recovery.json --seed $s --headless \
    --inject seal:0.3,drop-lift:3,drop-carry:2 --csv results/results_with_faults.csv
done
```

### 검사

```bash
.venv/bin/python -m pytest tests BehaviorOS/tests -q                                  # 테스트 54개
.venv/bin/python scripts/check_clearance.py trees/b6_takeover.json --seeds 0,1,2,3,4    # 로봇–상자·선반 접촉 0 (오래 걸림)
.venv/bin/python scripts/check_reach.py --seeds 0,1,2,3,4                               # 모든 컵·칸에 팔이 닿는지
```

## 4. 구조

```
trees/                   행동 트리 (JSON) — 무엇을 어떤 순서로
  b2 → b3 → b4 → b5_recovery.json (1단계 완성) → b6_takeover.json (최종, 2단계 포함)
BehaviorOS/              트리 실행 엔진 (기존 라이브러리 + 이번 수정)
shelving/                트리 노드 (BehaviorOS 액션), 파일 하나 = 노드 하나 또는 짝
  sim_action.py          모든 노드의 틀: 단계(동작, Check, Do, Guarded) 실행, 실패 기록, 안전 정리
  go_to_box.py  pick_cup.py  go_to_shelf.py  place_cup.py  ledger.py  recover.py  takeover.py
  utils/                 노드가 쓰는 도우미
    base_action.py       여러 노드가 같이 쓰는 동작: BaseMove, ToolLine, Wait
    ik.py  planner.py  common.py  teleop.py
sim/                     시뮬레이션 세상
  world.py               제공 장면 + 로봇 조립, 흡착, 조회
  robot/youbot_lift.xml  로봇 모델
scripts/                 run_bt.py(실행기), faults.py(실패 주입), 검사·보기 도구
tests/                   shelving 테스트 (컵 상태 판정, 움직임, 2단계 조종자)
results/                 반복 실험 결과
presentation/            발표 자료 (PPT)
```

한 tick(20 ms)마다 BehaviorOS가 트리를 훑어 실행 중인 노드를 찾고, 그 노드가 모터 명령을 한 조각 쓴 뒤 RUNNING / SUCCESS / FAILURE를 보고합니다. 그다음 물리를 2 ms씩 10번 계산합니다([`scripts/run_bt.py`](scripts/run_bt.py)).

### 로봇 (임바디먼트)

로봇 모델이 지정되지 않아(회사 안내: 임바디먼트 자유) KUKA youBot을 바탕으로 과제에 맞게 개조했습니다.

- 위 칸(1 m)과 선반 깊이에 닿도록 **리프트 기둥**(어깨 높이 0.70~1.30 m)을 세우고 팔 링크 두 개를 늘림(0.155 → 0.23 m, 0.135 → 0.20 m)
- 컵 뚜껑을 위에서 잡도록 **흡착 툴**. MuJoCo에 진공 흡착이 없어서, 흡착판이 뚜껑에 제대로 닿았는지(중심 2.5 cm, 기울기 8° 이내) 검사한 뒤 weld 제약으로 고정
- 베이스는 바퀴 물리 대신 평면 3자유도(앞뒤·옆·회전)로 단순화, 겉모습은 youBot 메시

## 5. 설계 요약

### 행동 트리 (`trees/b6_takeover.json`)

```
init_episode                                  컵·칸 상태를 장면에서 확인
repeat_until(all_cups_done, 최대 20회)
└─ 선택: [ 컵 하나 처리 | recover ]
    컵 하나 처리:
      select_next_task                        들고 있는 컵 우선, 아니면 가장 가까운 컵 / 가장 낮은 빈칸
      선택: [ is_holding | retry×2 go_to_box → retry×3 pick_cup ]
      go_to_shelf
      선택: [ 로봇이 놓기:  check_place_ready → place_cup → verify_placed
              사람이 놓기:  operator_available → retry×3 [request_takeover → human_control
                                                         → verify_placed → back_away]
              조종자 없음:  place_cup → verify_placed ]
      mark_placed                             컵·칸 상태를 장면에서 다시 확인
    recover:  안전하게 정리 → 시도 횟수 +1 → 못 간 칸 기억 → 컵·칸 상태를 장면에서 다시 확인
```

- **실패 크기에 따라 되돌아가는 정도를 다르게**: 그 자리에서 다시 해도 될 실패는 `retry`, 나머지는 `recover`가 로봇을 안전하게 정리한 뒤 다음 회차가 처음부터 다시 계획합니다.
- **컵 상태는 계획에 맞춰 고치지 않고 매번 장면을 보고 판정**: 놓친 컵이 상자로 돌아갔는지 바닥에 떨어졌는지는 계획으로 알 수 없기 때문입니다. 상자로 돌아간 컵은 다시 집고, 바닥에 떨어진 컵은 길찾기 지도에 **장애물로 등록**해 피해 다니며, 그 때문에 선반 앞이 막히면 다른 주차 자리를 찾습니다.
- **모든 실패에 이유를 기록**(`seal_failed`, `cup_dropped`, `blocked` …): 복구 판단, 사람에게 보여 줄 요청 메시지, CSV의 실패·복구 열이 이 기록에서 나옵니다.

### 사람 개입 (2단계)

과제 문장("정확한 위치에 진열할 수 있도록, 진열의 마지막 단계에서")에 따라 **놓는 단계에서만** 제어권이 넘어갑니다.

- **넘어가는 때**: 사람이 `t`를 누를 때, 또는 로봇이 정확한 진열을 약속할 수 없을 때 로봇이 요청합니다. 컵이 흡착판에 4° 넘게 기울어 붙었거나 2 cm 넘게 비켜 붙었을 때, 놓을 팔 자세가 없을 때, 놓은 뒤 검사에서 벗어났을 때입니다.
- **제어권 상태**: 자동 → (요청) → 사람 → (돌려주기) → 자동. 사람이 조종하는 동안 트리에서는 `human_control` 노드가 실행 중으로 보입니다.
- **돌려받은 뒤**: 로봇이 직접 검사하고, 틀리면 이유를 말하며 다시 요청합니다(최대 3번). 조종자가 없으면 로봇이 직접 놓습니다.

## 6. 기존 코드와 새 코드

- **기존 코드**: `BehaviorOS/`. 수정 전 원본은 태그 [`behavioros-v0.1-legacy`](https://github.com/hayoung0Lee/cupramen/tree/behavioros-v0.1-legacy)이고, 이후 커밋이 이번 과제의 수정입니다. 바뀐 부분 보기:
  ```bash
  git diff behavioros-v0.1-legacy -- BehaviorOS/behavioros
  ```
- **새 코드**: 그 밖의 전부 (`trees/`, `shelving/`, `sim/`, `scripts/`, `tests/`, `tools/`, `results/`).

### BehaviorOS 수정 사항 (테스트: `BehaviorOS/tests/`)

|     | 수정                                                                     | 이유                                                         |
| --- | ------------------------------------------------------------------------ | ------------------------------------------------------------ |
| ①   | 등록되지 않은 노드 타입이면 트리를 만들 때 오류                          | 오타 난 노드가 조용히 "성공" 노드로 바뀌어 단계가 건너뛰어짐 |
| ②   | 노드가 (다시) 시작할 때 `reset()`                                        | 재시도한 노드가 지난 진행 상태에서 이어져 바로 끝남          |
| ③   | 데코레이터 `retry`, `repeat_until`(신규), `inverter`                     | JSON 트리로 재시도·반복을 표현할 수 없었음                   |
| ④   | `self.fail(reason, …)` → `last_failure`, `failure_log`, 예외도 기록      | FAILURE만으로는 왜 실패했는지 몰라 복구할 수 없음            |
| ⑤   | 시계 주입(시뮬레이션 시간), 0.0초 시작 처리                              | 실제 시계라 실행 방식에 따라 소요 시간이 달라짐              |
| ⑥   | 장치(`devices`)를 모든 노드에 `self.devices`로 전달                      | 로봇 같은 객체가 데이터용 블랙보드에 섞임                    |

그 밖에 ROS·Docker 관련 계획 문서를 정리하고 Python 3.11 설정을 맞췄습니다.

## 7. 한계

- **인식**: 컵·상자·선반 위치는 카메라 인식 대신 시뮬레이션의 정답 위치를 씁니다(과제: 인식 방법 자유). 컵 상태를 매번 관찰로 판정하는 구조라, 실제 로봇에서는 `shelving/ledger.py`의 `observe()`만 인식 결과로 바꾸면 됩니다.
- **바닥에 떨어진 컵**은 팔이 닿지 않아 포기합니다.
- 노드는 "컵 집기"처럼 굵은 단위이고, 그 안의 세부 동작은 노드 코드 안의 단계 목록입니다. 세부 동작을 트리 노드로 노출하면 LLM이 트리를 만들기에 더 자유로워집니다.

## 8. 출처와 라이선스

- youBot 메시: [youbot/youbot_description](https://github.com/youbot/youbot_description)(GPLv3)에서 변환. 출처와 변경 내용은 [`sim/robot/meshes/youbot/README.md`](sim/robot/meshes/youbot/README.md)
- BehaviorOS: MIT
- 컵라면 시뮬레이션 자산: 회사 제공, 과제 목적으로만 사용하며 이 저장소에 포함하지 않음
