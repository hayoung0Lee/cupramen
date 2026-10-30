"""실패 이유 기록: BaseAction.fail(reason, **detail).

트리의 FAILURE는 "실패했다"만 위로 전한다. "왜"는 전하지 않는다.
그래서 Action이 실패할 때 이유를 blackboard에 표준 형식으로 남긴다.

    return self.fail("seal_failed", cup="cup_nugoori", offset_mm=31)

    blackboard["last_failure"] = {
        "node": "Pick cup",            # 노드 이름(JSON의 name)
        "action": "pick_cup",          # Action 종류(JSON의 type)
        "reason": "seal_failed",       # 실패 이유(짧은 코드)
        "detail": {"cup": "cup_nugoori", "offset_mm": 31},
    }
    blackboard["failure_log"] = [ ...지금까지의 실패가 순서대로... ]

- last_failure: 가장 최근 실패 하나. 실패 처리 노드가 읽고 이유별로 대응한다.
- failure_log: 전체 이력. CSV의 failures / notes 칸을 여기서 만든다.
- Action 안에서 예외가 나도 같은 형식으로 기록된다(reason="exception").
  예외를 잡는 곳이 BaseAction.update()라서 코어에서만 할 수 있는 일이다.
"""

import py_trees

from behavioros.core import TreeExecutor
from behavioros.core.actions import BaseAction, action

S = py_trees.common.Status


@action("test_seal_fails", "흡착 밀착 실패를 흉내 낸다")
class SealFails(BaseAction):
    def execute(self):
        return self.fail("seal_failed", cup=self.param("cup"), offset_mm=31)


@action("test_raises", "실행 중 예외가 난다(예: IK 계산 오류)")
class Raises(BaseAction):
    def execute(self):
        raise RuntimeError("IK did not converge")


@action("test_plain_failure", "v0.1 방식: 이유 없이 FAILURE만 돌려준다")
class PlainFailure(BaseAction):
    def execute(self):
        return S.FAILURE


def test_fail_returns_failure_and_records_why():
    tree = TreeExecutor({"type": "test_seal_fails", "name": "Pick cup", "cup": "cup_nugoori"}, {})

    assert tree.tick().status == "FAILURE"
    assert tree.blackboard["last_failure"] == {
        "node": "Pick cup",
        "action": "test_seal_fails",
        "reason": "seal_failed",
        "detail": {"cup": "cup_nugoori", "offset_mm": 31},
    }


def test_an_exception_inside_an_action_is_recorded_the_same_way():
    tree = TreeExecutor({"type": "test_raises", "name": "Solve IK"}, {})

    assert tree.tick().status == "FAILURE"
    failure = tree.blackboard["last_failure"]
    assert (failure["node"], failure["reason"]) == ("Solve IK", "exception")
    assert "IK did not converge" in failure["detail"]["error"]


def test_failure_log_keeps_every_failure_in_order():
    # 재시도하면 실패가 여러 번 쌓인다. last_failure는 마지막 것, failure_log는 전부.
    tree = TreeExecutor({"type": "retry", "num_failures": 3, "children": [
        {"type": "test_seal_fails", "name": "Pick cup", "cup": "cup_a"}]}, {})
    while tree.tick().status == "RUNNING":
        pass

    assert [f["reason"] for f in tree.blackboard["failure_log"]] == ["seal_failed"] * 3
    assert tree.blackboard["last_failure"] == tree.blackboard["failure_log"][-1]


def test_returning_failure_without_fail_still_works_but_records_nothing():
    # 호환: v0.1 Action처럼 FAILURE만 돌려줘도 동작은 같다. 다만 이유는 남지 않는다.
    tree = TreeExecutor({"type": "test_plain_failure"}, {})

    assert tree.tick().status == "FAILURE"
    assert "last_failure" not in tree.blackboard
