"""Action이 '다시 시작'될 때 진행 상태를 초기화하는지(BaseAction.reset) 보는 테스트.

배경
----
실제 로봇 Action은 한 번에 끝나지 않는다. 여러 tick 동안 RUNNING을 돌려주며
"지금 몇 번째 단계인지" 같은 진행 상태를 자기 안에 들고 있다.

    tick 1: 1단계 → RUNNING
    tick 2: 2단계 → RUNNING
    tick 3: 3단계 → SUCCESS   (끝)

같은 노드가 끝난 뒤 다시 실행되는 경우가 있다.
  - 반복(6개 컵을 같은 서브트리로 처리)
  - 재시도(실패한 집기를 다시)
이때 진행 상태가 지난번 값(3단계)으로 남아 있으면, 아무 일도 안 하고 바로 끝난다.

py_trees는 노드가 '새로 시작'될 때(직전 상태가 RUNNING이 아닐 때 tick되면)
initialise()를 부른다. BaseAction은 여기서 reset()을 부른다.
→ Action은 실행마다 새로 시작해야 하는 상태를 reset()에서 정한다.
"""

import py_trees

from behavioros.core import TreeExecutor
from behavioros.core.actions import BaseAction, action

S = py_trees.common.Status


@action("test_three_steps", "3 tick 걸리는 Action. 진행 상태를 reset()에서 초기화한다")
class ThreeSteps(BaseAction):
    def reset(self):
        self.step = 0
        self.blackboard.setdefault("resets", 0)
        self.blackboard["resets"] += 1

    def execute(self):
        self.step += 1
        self.blackboard.setdefault("step_log", []).append(self.step)
        return S.SUCCESS if self.step >= 3 else S.RUNNING


@action("test_three_steps_v01", "v0.1 방식: 진행 상태를 __init__에서만 정한다")
class ThreeStepsV01(BaseAction):
    def __init__(self, name, blackboard=None):
        super().__init__(name, blackboard)
        self.step = 0

    def execute(self):
        self.step += 1
        self.blackboard.setdefault("step_log", []).append(self.step)
        return S.SUCCESS if self.step >= 3 else S.RUNNING


def run_until_done(tree):
    """끝날 때까지 tick하고, 그동안의 상태 목록을 돌려준다."""
    statuses = []
    while True:
        statuses.append(tree.tick().status)
        if statuses[-1] != "RUNNING":
            return statuses


def test_second_run_starts_from_the_first_step_again():
    tree = TreeExecutor({"type": "test_three_steps", "name": "move"}, {})

    # 1번째 실행: 1 → 2 → 3단계
    assert run_until_done(tree) == ["RUNNING", "RUNNING", "SUCCESS"]
    # 2번째 실행(반복·재시도가 하는 일): 다시 1단계부터
    assert run_until_done(tree) == ["RUNNING", "RUNNING", "SUCCESS"]

    assert tree.blackboard["step_log"] == [1, 2, 3, 1, 2, 3]


def test_reset_runs_once_per_start_not_on_every_tick():
    # RUNNING인 동안에는 reset되지 않는다. 그랬다면 영원히 1단계에 머문다.
    tree = TreeExecutor({"type": "test_three_steps", "name": "move"}, {})
    run_until_done(tree)
    assert tree.blackboard["resets"] == 1
    run_until_done(tree)
    assert tree.blackboard["resets"] == 2


def test_v01_action_without_reset_skips_its_work_on_the_second_run():
    # 이 테스트는 "왜 reset이 필요한가"를 기록해 둔 것이다.
    # reset()이 없는 v0.1 방식 Action은 동작이 바뀌지 않는다(호환). 그래서 두 번째 실행은
    # 진행 상태가 3에서 이어져 첫 tick에 바로 SUCCESS가 된다. 실제 로봇이면 움직이지 않고 '성공'.
    tree = TreeExecutor({"type": "test_three_steps_v01", "name": "move"}, {})
    assert run_until_done(tree) == ["RUNNING", "RUNNING", "SUCCESS"]
    assert run_until_done(tree) == ["SUCCESS"]
    assert tree.blackboard["step_log"] == [1, 2, 3, 4]
