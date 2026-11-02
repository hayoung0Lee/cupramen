"""장치(devices)와 블랙보드(blackboard) 분리: TreeExecutor(..., devices={...}).

블랙보드는 노드끼리 주고받는 **데이터**를 위한 곳이다(지금 처리 중인 컵, 실패 기록 ...).
로봇·시뮬레이터·계산 도구 같은 **객체**는 데이터가 아니라 노드가 쓰는 장치다.
예전에는 Action이 받는 통로가 블랙보드뿐이라 객체까지 블랙보드에 넣어야 했다.

    tree = TreeExecutor(tree_json, blackboard={}, devices={"robot": robot})

- 모든 Action은 self.devices로 같은 장치 묶음을 받는다(시계 self.now()와 같은 방식).
- 장치는 블랙보드에 들어가지 않는다 → 블랙보드에는 데이터만 남는다.
- devices를 넘기지 않으면 빈 묶음 {} (예전 트리는 그대로 동작).
"""

import py_trees

from behavioros.core import TreeExecutor
from behavioros.core.actions import BaseAction, action

S = py_trees.common.Status


class FakeRobot:
    """장치 역할: 위치를 갖고 있고 move() 명령을 받는다."""

    def __init__(self):
        self.x = 0.0

    def move(self, dx):
        self.x += dx


@action("test_move_robot", "장치(robot)를 움직이고, 결과 위치를 블랙보드에 데이터로 적는다")
class MoveRobot(BaseAction):
    def execute(self):
        robot = self.devices["robot"]           # 장치는 self.devices에서
        robot.move(self.param("dx", 1.0))
        self.write_blackboard("robot_x", robot.x)   # 블랙보드에는 데이터만
        return S.SUCCESS


def test_action_gets_devices_not_from_blackboard():
    # 장치를 devices로 넘기면 Action이 self.devices로 받는다.
    robot = FakeRobot()
    tree = TreeExecutor({"tree": {"type": "test_move_robot", "dx": 2.0}},
                        blackboard={}, devices={"robot": robot})
    assert tree.tick().status == "SUCCESS"
    assert robot.x == 2.0                       # 장치가 실제로 움직였고
    assert "robot" not in tree.blackboard       # 장치는 블랙보드에 없다
    assert tree.blackboard["robot_x"] == 2.0    # 블랙보드에는 데이터(위치)가 있다
    # (참고: 노드 매개변수 dx도 블랙보드에 복사돼 있다. 예전 Webots 예제와의 호환 때문에
    #  남겨 둔 v0.1 동작이고, 새 액션은 self.param()으로 읽는다.)


def test_every_node_shares_the_same_devices():
    # 노드가 여러 개여도 같은 장치 묶음(같은 객체)을 본다.
    robot = FakeRobot()
    tree = TreeExecutor(
        {"tree": {"type": "sequence", "children": [
            {"type": "test_move_robot", "name": "first", "dx": 1.0},
            {"type": "test_move_robot", "name": "second", "dx": 1.0},
        ]}},
        blackboard={}, devices={"robot": robot},
    )
    tree.tick()
    assert robot.x == 2.0
    nodes = [n for n in tree.py_tree.root.iterate() if isinstance(n, BaseAction)]
    assert all(n.devices["robot"] is robot for n in nodes)


def test_without_devices_nodes_get_an_empty_dict():
    # devices를 안 넘기면 {}: 장치를 안 쓰는 예전 트리는 그대로 동작한다.
    tree = TreeExecutor({"tree": {"type": "sequence", "children": []}})
    assert tree.devices == {}
