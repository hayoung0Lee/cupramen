"""JSON 트리에서 쓰는 데코레이터 노드: retry, repeat_until, inverter.

데코레이터는 자식을 정확히 하나 가지는 노드다. 자식의 결과를 보고 자기 결과를 정한다.

    {"type": "retry", "num_failures": 3, "children": [ <자식 하나> ]}
        자식이 FAILURE면 다시 실행한다. 최대 3번 시도한다(= 3번째 실패에서 FAILURE).
        자식이 SUCCESS면 바로 SUCCESS.
        과제: 집기 실패 → 다시 집기.

    {"type": "repeat_until", "until": "all_done", "children": [ <자식 하나> ]}
        자식이 SUCCESS로 끝날 때마다 blackboard["all_done"]을 확인한다.
        참이면 SUCCESS, 아니면 자식을 처음부터 다시 실행한다.
        자식이 FAILURE면 FAILURE. max_iterations(선택)를 넘으면 FAILURE(무한 반복 방지).
        과제: 남은 컵이 없을 때까지 "컵 하나 처리" 반복.

    {"type": "inverter", "children": [ <자식 하나> ]}
        SUCCESS ↔ FAILURE를 뒤집는다. RUNNING은 그대로.
        과제: "~가 아니면" 조건.

재시도·반복은 같은 자식을 다시 시작하는 일이다. 그래서 자식 Action이 reset()으로
진행 상태를 초기화해야 제대로 동작한다(tests/test_action_reset.py).
"""

import py_trees
import pytest

from behavioros.core import TreeExecutor
from behavioros.core.actions import BaseAction, action

S = py_trees.common.Status


# ---------------------------------------------------------------------------
# 테스트용 Action
# ---------------------------------------------------------------------------
@action("test_flaky", "2 tick 걸리는 시도. 처음 fail_times번은 실패하고 그다음부터 성공")
class Flaky(BaseAction):
    """집기처럼 여러 tick 걸리고, 처음 몇 번은 실패하는 동작.

    blackboard["attempt_log"]에 시도마다 (시도 번호, 진행한 단계 목록)을 남긴다.
    """

    def reset(self):
        log = self.blackboard.setdefault("attempt_log", [])
        log.append([])            # 새 시도 시작
        self.step = 0

    def execute(self):
        self.step += 1
        self.blackboard["attempt_log"][-1].append(self.step)
        if self.step < 2:
            return S.RUNNING
        attempt = len(self.blackboard["attempt_log"])
        return S.FAILURE if attempt <= self.param("fail_times", 0) else S.SUCCESS


@action("test_handle_one_cup", "컵 하나 처리: done_count를 1 올리고, 목표에 닿으면 all_done=True")
class HandleOneCup(BaseAction):
    def execute(self):
        self.blackboard["done_count"] = self.blackboard.get("done_count", 0) + 1
        if self.blackboard["done_count"] >= self.param("total"):
            self.blackboard["all_done"] = True
        return S.SUCCESS


@action("test_always", "param 'result'에 적힌 상태를 그대로 돌려준다")
class Always(BaseAction):
    def execute(self):
        return {"SUCCESS": S.SUCCESS, "FAILURE": S.FAILURE}[self.param("result")]


def run_until_done(tree, max_ticks=100):
    for _ in range(max_ticks):
        status = tree.tick().status
        if status != "RUNNING":
            return status
    raise AssertionError("tree did not finish")


# ---------------------------------------------------------------------------
# retry
# ---------------------------------------------------------------------------
def test_retry_succeeds_when_an_attempt_succeeds_within_the_limit():
    # 2번 실패 후 3번째에 성공하는 동작을, 최대 3번 시도로 감싼다
    tree = TreeExecutor({"type": "retry", "num_failures": 3, "children": [
        {"type": "test_flaky", "name": "pick", "fail_times": 2}]}, {})

    assert run_until_done(tree) == "SUCCESS"
    # 시도마다 1단계부터 다시 진행했다(reset 덕분)
    assert tree.blackboard["attempt_log"] == [[1, 2], [1, 2], [1, 2]]


def test_retry_fails_when_every_allowed_attempt_fails():
    # 같은 동작을 최대 2번 시도로 감싸면, 2번 다 실패해서 FAILURE
    tree = TreeExecutor({"type": "retry", "num_failures": 2, "children": [
        {"type": "test_flaky", "name": "pick", "fail_times": 2}]}, {})

    assert run_until_done(tree) == "FAILURE"
    assert len(tree.blackboard["attempt_log"]) == 2


# ---------------------------------------------------------------------------
# repeat_until
# ---------------------------------------------------------------------------
def test_repeat_until_runs_the_child_until_the_key_becomes_true():
    tree = TreeExecutor({"type": "repeat_until", "until": "all_done", "children": [
        {"type": "test_handle_one_cup", "name": "one cup", "total": 6}]}, {})

    assert run_until_done(tree) == "SUCCESS"
    assert tree.blackboard["done_count"] == 6     # 정확히 6번 실행하고 멈췄다


def test_repeat_until_does_not_run_the_child_when_already_done():
    tree = TreeExecutor({"type": "repeat_until", "until": "all_done", "children": [
        {"type": "test_handle_one_cup", "name": "one cup", "total": 6}]}, {"all_done": True})

    assert run_until_done(tree) == "SUCCESS"
    assert "done_count" not in tree.blackboard    # 한 번도 실행하지 않았다


def test_repeat_until_stops_with_failure_after_max_iterations():
    # 끝나지 않는 반복을 막는 안전장치: 6번이 목표인데 최대 3번만 허용
    tree = TreeExecutor({"type": "repeat_until", "until": "all_done", "max_iterations": 3,
                         "children": [{"type": "test_handle_one_cup", "total": 6}]}, {})

    assert run_until_done(tree) == "FAILURE"
    assert tree.blackboard["done_count"] == 3


def test_repeat_until_fails_when_the_child_fails():
    tree = TreeExecutor({"type": "repeat_until", "until": "all_done", "children": [
        {"type": "test_always", "result": "FAILURE"}]}, {})

    assert run_until_done(tree) == "FAILURE"


# ---------------------------------------------------------------------------
# inverter
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("child, expected", [("SUCCESS", "FAILURE"), ("FAILURE", "SUCCESS")])
def test_inverter_flips_success_and_failure(child, expected):
    tree = TreeExecutor({"type": "inverter", "children": [
        {"type": "test_always", "result": child}]}, {})

    assert run_until_done(tree) == expected


# ---------------------------------------------------------------------------
# 잘못 쓴 JSON은 트리를 만들 때 오류
# ---------------------------------------------------------------------------
def test_a_decorator_needs_exactly_one_child():
    with pytest.raises(ValueError, match="exactly one child"):
        TreeExecutor({"type": "inverter", "children": [
            {"type": "test_always", "result": "SUCCESS"},
            {"type": "test_always", "result": "SUCCESS"}]}, {})


def test_required_decorator_settings_must_be_given():
    with pytest.raises(ValueError, match="num_failures"):
        TreeExecutor({"type": "retry", "children": [{"type": "test_always", "result": "SUCCESS"}]}, {})
    with pytest.raises(ValueError, match="until"):
        TreeExecutor({"type": "repeat_until", "children": [
            {"type": "test_always", "result": "SUCCESS"}]}, {})
