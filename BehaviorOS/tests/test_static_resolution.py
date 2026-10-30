"""Static Resolution과 '아직 정해지지 않은 값'.

트리를 만들 때 StaticProcessor가 "${이름}"을 먼저 처리한다.
  - 등록된 static resolver가 있거나, 그 순간 blackboard에 값이 있으면 → 값으로 바꾼다
  - 등록된 runtime resolver면 → 그대로 둔다(매 tick 갱신)
  - 둘 다 아니면? → 예전에는 None(또는 빈 문자열)으로 바꿨다.

예) "cup": "${current_cup}"
    current_cup은 트리가 실행되면서 '다음 컵 고르기' 노드가 정한다. 트리를 만드는
    순간에는 아직 없다. None으로 바뀌면 노드는 영원히 None을 읽는다.

그래서 지금 값을 찾을 수 없는 참조는 "${이름}" 그대로 남긴다. 노드는
self.param()으로 읽는 순간 blackboard에서 값을 찾는다(tests/test_action_params.py).
"""

import py_trees

from behavioros.core.actions import BaseAction, action
from behavioros.core.builder import TreeBuilder
from behavioros.resolution import StaticProcessor

S = py_trees.common.Status


@action("test_static_read_cup", "self.param('cup')로 읽은 값을 read_log에 남긴다")
class ReadCup(BaseAction):
    def execute(self):
        self.blackboard.setdefault("read_log", []).append(self.param("cup"))
        return S.SUCCESS


def test_reference_to_a_value_not_known_yet_is_kept():
    out = StaticProcessor(devices={}).process({"tree": {"type": "x", "cup": "${current_cup}"}})
    assert out["tree"]["cup"] == "${current_cup}"


def test_reference_inside_a_longer_string_is_kept_too():
    out = StaticProcessor(devices={}).process({"tree": {"type": "x", "note": "slot ${current_slot}"}})
    assert out["tree"]["note"] == "slot ${current_slot}"


def test_value_already_on_the_blackboard_is_still_filled_in_at_build_time():
    # 바뀌지 않는 동작: 그 순간 값이 있으면 static resolution이 채운다
    out = StaticProcessor(devices={}, blackboard={"speed": 0.4}).process(
        {"tree": {"type": "x", "v": "${speed}"}})
    assert out["tree"]["v"] == 0.4


def test_a_node_reads_a_value_chosen_while_the_tree_runs():
    bb = {"read_log": []}
    tree = TreeBuilder({"tree": {"type": "test_static_read_cup", "cup": "${current_cup}"}},
                       bb, devices={}).with_runtime().build()

    bb["current_cup"] = "cup_a"     # 실행 중에 정해진 값
    tree.tick()
    bb["current_cup"] = "cup_b"
    tree.tick()

    assert bb["read_log"] == ["cup_a", "cup_b"]
