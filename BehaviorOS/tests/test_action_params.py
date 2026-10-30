"""Action 파라미터(노드별)와 blackboard(공용)의 관계를 보여 주는 테스트.

용어
----
- 파라미터: JSON 노드에 적은 설정값. 예) {"type": "go_to", "target": "BOX"} 의 target.
  노드마다 자기 것을 따로 가진다 → action.params, 읽을 때는 self.param("target").
- blackboard: 트리의 모든 노드가 같이 쓰는 dict 하나("공용 메모판").
  쓰기 self.write_blackboard(k, v), 읽기 self.read_blackboard(k).

파라미터 값이 "${이름}" 이면 고정값이 아니라 "blackboard[이름]을 따라가라"는 뜻이다.
self.param()은 읽는 순간의 blackboard 값을 돌려준다. 그래서 다른 노드가 그 값을
덮어쓰면, 이 노드도 다음에 읽을 때 바뀐 값을 받는다.

    ┌──────────── blackboard (하나, 공용) ────────────┐
    │  current_cup: ...   read_log: [...]             │
    └──────▲──────────────────────────▲───────────────┘
           │                          │
      node A (params: 자기 것)     node B (params: 자기 것)

테스트용 Action은 모두 첫 tick에 바로 SUCCESS를 돌려준다. 그래서 sequence 하나가
tick 한 번에 끝까지 실행된다. (실제 로봇 Action은 여러 tick 동안 RUNNING이다.)
"""

import py_trees

from behavioros.core import TreeExecutor
from behavioros.core.actions import BaseAction, action

SUCCESS = py_trees.common.Status.SUCCESS


# ---------------------------------------------------------------------------
# 테스트용 Action
# ---------------------------------------------------------------------------
@action("test_read_target", "self.param('target')로 읽은 값을 read_log에 남긴다")
class ReadTarget(BaseAction):
    """자기 파라미터 target을 읽고, '누가 무엇을 읽었는지'를 기록한다.

    read_log는 테스트가 확인하려고 만든 기록장(목록)이다. 실행될 때마다 한 줄씩
    추가(append)된다. 값을 저장하는 곳이 아니라 이력이다.
    """

    def execute(self):
        value = self.param("target")
        self.blackboard.setdefault("read_log", []).append((self.name, value))
        return SUCCESS


@action("test_set_current_cup", "자기 파라미터 choice를 blackboard['current_cup']에 쓴다")
class SetCurrentCup(BaseAction):
    """공용 값을 덮어쓰는 노드. 실제 과제의 select_next_task가 하는 일과 같다."""

    def execute(self):
        self.write_blackboard("current_cup", self.param("choice"))
        return SUCCESS


@action("test_legacy_read_target", "v0.1 방식: 파라미터를 blackboard['target']에서 읽는다")
class LegacyReadTarget(BaseAction):
    """기존 Webots Action들이 쓰던 방식. 호환을 위해 계속 동작해야 한다."""

    def execute(self):
        self.blackboard.setdefault("read_log", []).append(
            (self.name, self.read_blackboard("target"))
        )
        return SUCCESS


def tick_once(tree_json):
    """트리를 만들고 tick을 한 번 한다(테스트용 Action은 한 번에 끝난다)."""
    tree = TreeExecutor(tree_json, {"read_log": []})
    tree.tick()
    return tree


# ---------------------------------------------------------------------------
# 1. 노드마다 자기 파라미터를 가진다
# ---------------------------------------------------------------------------
def test_each_node_reads_its_own_param():
    # 준비: 같은 Action 두 개, target만 다르다 (상자로 이동 / 선반으로 이동)
    tree_json = {"type": "sequence", "children": [
        {"type": "test_read_target", "name": "go to box", "target": "BOX"},
        {"type": "test_read_target", "name": "go to shelf", "target": "SHELF"},
    ]}

    # 실행
    tree = tick_once(tree_json)

    # 확인: 각자 자기 값을 읽었다
    assert tree.blackboard["read_log"] == [("go to box", "BOX"), ("go to shelf", "SHELF")]

    # 확인: 파라미터는 노드마다 따로, blackboard는 모두 같은 객체 하나
    box_node, shelf_node = tree.py_tree.root.children
    assert box_node.params == {"target": "BOX"}
    assert shelf_node.params == {"target": "SHELF"}
    assert box_node.blackboard is shelf_node.blackboard is tree.blackboard


def test_v01_copy_to_blackboard_is_overwritten_when_the_tree_is_built():
    # 이 테스트는 "왜 노드별 파라미터가 필요했나"를 기록해 둔 것이다.
    # v0.1은 파라미터를 blackboard에 같은 이름으로 복사한다(호환을 위해 지금도 남아 있다).
    # 복사는 트리를 만드는 순간 일어나므로, 실행하기도 전에 마지막 노드 값으로 덮인다.
    tree = TreeExecutor({"type": "sequence", "children": [
        {"type": "test_legacy_read_target", "name": "go to box", "target": "BOX"},
        {"type": "test_legacy_read_target", "name": "go to shelf", "target": "SHELF"},
    ]}, {"read_log": []})

    # 아직 tick 전인데 이미 SHELF
    assert tree.blackboard["target"] == "SHELF"

    # 그래서 blackboard에서 읽는 옛 방식은 둘 다 SHELF를 읽는다
    tree.tick()
    assert tree.blackboard["read_log"] == [("go to box", "SHELF"), ("go to shelf", "SHELF")]


# ---------------------------------------------------------------------------
# 2. "${이름}" 파라미터는 공용 값을 따라간다 → 덮어쓰기가 필요한 값은 이렇게 쓴다
# ---------------------------------------------------------------------------
def test_reference_param_follows_the_shared_value():
    # 준비: target이 고정값이 아니라 blackboard['current_cup']을 가리킨다
    tree = TreeExecutor(
        {"type": "test_read_target", "name": "pick", "target": "${current_cup}"},
        {"read_log": [], "current_cup": "cup_a"},
    )

    # 실행 1: 지금 공용 값은 cup_a
    tree.tick()
    # 바깥에서(또는 다른 노드가) 공용 값을 덮어쓴다
    tree.blackboard["current_cup"] = "cup_b"
    # 실행 2: 같은 노드인데 이번에는 cup_b를 읽는다
    tree.tick()

    assert tree.blackboard["read_log"] == [("pick", "cup_a"), ("pick", "cup_b")]


def test_a_node_overwrites_the_shared_value_and_later_nodes_see_it():
    # 실제 과제와 같은 구조: "다음 컵 고르기" 노드가 current_cup을 덮어쓰고,
    # "집기" 노드는 ${current_cup}을 따라간다.
    tree = tick_once({"type": "sequence", "children": [
        {"type": "test_set_current_cup", "name": "choose 1", "choice": "cup_a"},
        {"type": "test_read_target", "name": "pick 1", "target": "${current_cup}"},
        {"type": "test_set_current_cup", "name": "choose 2", "choice": "cup_b"},
        {"type": "test_read_target", "name": "pick 2", "target": "${current_cup}"},
    ]})

    # 각 집기 노드는 자기 바로 앞에서 정해진 컵을 읽었다
    assert tree.blackboard["read_log"] == [("pick 1", "cup_a"), ("pick 2", "cup_b")]
    # 공용 값은 하나다: 마지막으로 쓴 cup_b만 남는다 (두 개가 저장되는 게 아니다)
    assert tree.blackboard["current_cup"] == "cup_b"


# ---------------------------------------------------------------------------
# 3. 기본값과 호환
# ---------------------------------------------------------------------------
def test_missing_param_returns_the_default():
    node = ReadTarget(name="no params")
    assert node.param("target", "fallback") == "fallback"


def test_reference_to_a_missing_shared_value_returns_the_default():
    node = ReadTarget(name="n", blackboard={})
    node.params = {"target": "${not_set_yet}"}
    assert node.param("target", "fallback") == "fallback"


def test_v01_style_action_still_reads_its_param_from_the_blackboard():
    # 노드가 하나뿐이면 덮어쓸 상대가 없으니 옛 방식도 맞는 값을 읽는다
    tree = tick_once({"type": "test_legacy_read_target", "name": "legacy", "target": "BOX"})
    assert tree.blackboard["read_log"] == [("legacy", "BOX")]
