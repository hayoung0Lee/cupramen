"""2단계 조종자(Operator) 테스트: 제어권이 언제, 어떻게 넘어가고 돌아오는지.

상태는 셋뿐이다.
  auto       로봇(행동 트리)이 움직인다
  requested  로봇이 도움을 요청하고 사람이 받아 주기를 기다린다
  human      사람이 키보드로 움직인다 (트리는 human_control 노드에서 기다린다)

    .venv/bin/python -m pytest tests/test_teleop.py -q
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from shelving.utils.teleop import NoOperator, Operator, ScriptedOperator  # noqa: E402


def test_robot_asks_then_person_takes_over_and_hands_back():
    # 로봇이 요청 -> 사람이 t -> 사람 조종 -> 사람이 h(돌려줌)를 누르면 human_control이 키로 받는다.
    op = Operator()
    op.request("cup is 20 mm off slot 1")
    assert op.mode == "requested"
    assert not op.accept()               # 사람이 아직 t를 안 누름 -> 계속 기다림
    op.press("t")
    assert op.accept()                   # 이제 넘겨받음
    assert op.mode == "human"
    op.press("f")
    assert op.next_key() == "f"
    op.press("h")
    assert [op.next_key(), op.next_key()] == ["h", None]
    op.hand_back()
    assert op.mode == "auto"


def test_person_presses_t_early_and_robot_hands_over_at_the_slot():
    # 로봇이 상자에 있을 때 t를 눌러도 그때 바로 넘기지 않는다: 표시만 해 두고(want_takeover)
    # 진열 마지막 단계(check_place_ready)에서 넘긴다.
    op = Operator()
    op.press("t")
    assert op.mode == "auto" and op.want_takeover
    assert op.accept()                   # 칸 앞에서 request_takeover가 확인하면 바로 넘어감
    assert op.mode == "human" and not op.want_takeover


def test_keys_are_ignored_while_the_robot_drives():
    # 로봇이 움직이는 동안 누른 조종 키는 쌓아 두지 않는다(넘겨받은 뒤 엉뚱하게 움직이지 않게).
    op = Operator()
    op.press("w")
    op.press("f")
    op.press("t")
    op.accept()
    assert op.next_key() is None


def test_nobody_at_the_keyboard():
    # 일괄 실행(화면 없음)에는 조종자가 없다 -> 트리는 사람 가지를 건너뛰고 로봇이 직접 놓는다.
    assert not NoOperator().available()
    assert Operator().available()


def test_scripted_operator_waits_for_the_request():
    # 시험용: "wait,t,..."는 로봇이 요청할 때까지 기다렸다가 넘겨받는다.
    op = ScriptedOperator("wait,t,f,h", every=1)
    for _ in range(3):
        op.tick()
    assert op.mode == "auto" and not op.want_takeover     # 요청이 없으니 그대로
    op.request("test")
    op.tick()                                             # wait 소비
    op.tick()                                             # t
    assert op.accept()
    op.tick()
    assert op.next_key() == "f"
    op.tick()
    assert op.next_key() == "h"


def test_keys_do_not_pile_up():
    # 키를 꾹 누르면(자동 반복) 또는 팔이 움직이는 동안 여러 번 누르면, 기다리는 키는
    # 마지막 하나만 남는다 -> 손을 떼면 곧 멈춘다.
    op = Operator()
    op.press("t")
    op.accept()
    for _ in range(10):
        op.press("r")
    op.press("h")
    assert [op.next_key(), op.next_key()] == ["h", None]
