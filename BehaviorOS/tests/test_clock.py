"""시계 주입: TreeExecutor(..., clock=함수).

시뮬레이션은 실제 시간과 다르게 흐른다. 화면 없이 돌리면 40초짜리 작업이 0.03초에
끝나고, 화면을 보며 돌리면 실제 시간과 비슷하게 걸린다. 결과(CSV의 duration_s)는
어느 쪽으로 돌려도 같아야 하므로 "시뮬레이션 시간"으로 재야 한다.

    tree = TreeExecutor(tree_json, blackboard, clock=lambda: d.time)   # MuJoCo 시뮬레이션 시간

- 소요 시간(get_execution_result().duration)을 이 시계로 잰다.
- Action도 같은 시계를 쓴다: self.now()
- 실패 기록에 시각이 남는다: last_failure["time"]
- clock을 넘기지 않으면 예전처럼 실제 시계(time.time)를 쓴다.
"""

import py_trees

from behavioros.core import TreeExecutor
from behavioros.core.actions import BaseAction, action

S = py_trees.common.Status


class FakeClock:
    """tick마다 시뮬레이션 시간이 20초씩 흐르는 척하는 시계."""

    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@action("test_three_ticks_then_fail", "3 tick 진행한 뒤 실패. 시작 시각을 기억한다")
class ThreeTicksThenFail(BaseAction):
    def reset(self):
        self.started_at = self.now()
        self.ticks = 0

    def execute(self):
        self.ticks += 1
        if self.ticks < 3:
            return S.RUNNING
        return self.fail("timeout", waited_s=self.now() - self.started_at)


def test_duration_and_failure_time_use_the_injected_clock():
    clock = FakeClock()
    tree = TreeExecutor({"type": "test_three_ticks_then_fail", "name": "wait"}, {}, clock=clock)

    while tree.tick().status == "RUNNING":
        clock.t += 20.0          # 시뮬레이션 시간이 20초 흐름

    # tick 3번: 100 → 120 → 140초에 tick. 시작 140-100 = 40초
    assert tree.get_execution_result().duration == 40.0
    failure = tree.blackboard["last_failure"]
    assert failure["time"] == 140.0
    assert failure["detail"]["waited_s"] == 40.0     # Action의 self.now()도 같은 시계


def test_without_a_clock_the_wall_clock_is_used():
    tree = TreeExecutor({"type": "test_three_ticks_then_fail"}, {})
    while tree.tick().status == "RUNNING":
        pass
    duration = tree.get_execution_result().duration
    assert 0.0 <= duration < 1.0                     # 실제 시간: 거의 0초


def test_a_clock_that_starts_at_zero_still_measures_duration():
    # MuJoCo 시뮬레이션 시간은 0.0초에서 시작한다. 0.0은 Python에서 '거짓'이라
    # "if self.start_time" 같은 검사는 시작 시각이 없는 것으로 오해한다.
    clock = FakeClock()
    clock.t = 0.0
    tree = TreeExecutor({"type": "test_three_ticks_then_fail"}, {}, clock=clock)
    while tree.tick().status == "RUNNING":
        clock.t += 20.0
    assert tree.get_execution_result().duration == 40.0


def test_tree_builder_passes_the_clock_through():
    from behavioros.core.builder import TreeBuilder

    clock = FakeClock()
    tree = TreeBuilder({"type": "test_three_ticks_then_fail"}, {}, devices={}, clock=clock).build()
    assert tree.clock is clock
    assert tree.py_tree.root.now() == clock.t
