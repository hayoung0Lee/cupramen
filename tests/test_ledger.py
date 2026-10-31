"""장부(ledger.observe) 테스트: 계획이 아니라 '시뮬레이션에 실제로 보이는 것'으로 장부를 만드는지.

실패 후에는 "컵을 떨어뜨렸다"는 사실만 알지, 컵이 상자로 다시 들어갔는지 바닥으로 갔는지는
모른다. 그래서 복구할 때마다 장부를 실제 위치로 다시 만든다. 여기서는 컵을 직접 옮겨 놓고
(순간이동) 장부가 그걸 맞게 분류하는지 본다.

    .venv/bin/python -m pytest tests/test_ledger.py -q
"""

import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from shelving import ledger  # noqa: E402
from sim.world import World  # noqa: E402


@pytest.fixture
def world():
    w = World.create()
    w.reset(0)
    return w


def put_cup(world, cup, xyz, tilt_deg=0.0):
    """컵(자유 관절)을 xyz에 세워 둔다. tilt_deg만큼 x축으로 기울일 수 있다."""
    j = world.m.body(cup).jntadr[0]
    adr = world.m.jnt_qposadr[j]
    a = np.radians(tilt_deg) / 2
    world.d.qpos[adr:adr + 7] = [*xyz, np.cos(a), np.sin(a), 0.0, 0.0]
    mujoco.mj_forward(world.m, world.d)


def slot_xyz(world, k):
    return world.d.site(f"slot_{k}").xpos.copy()


def test_start_every_cup_in_box(world):
    # 처음엔 6개 모두 상자 안, 선반 6칸 모두 비어 있음.
    cups, slots = ledger.observe(world)
    assert set(cups.values()) == {"in_box"}
    assert all(v is None for v in slots.values())
    assert not ledger.nothing_left(cups, slots)


def test_cup_on_slot_is_placed(world):
    # 컵을 3번 칸 정중앙에 세우면 placed, 3번 칸은 그 컵으로 채워짐.
    cup = world.cup_names()[0]
    put_cup(world, cup, slot_xyz(world, 3))
    cups, slots = ledger.observe(world)
    assert cups[cup] == "placed"
    assert slots[3] == cup


def test_cup_off_centre_blocks_the_slot(world):
    # 3 cm 빗나가게 놓이면(허용 1.5 cm 초과) 진열 실패지만 칸은 막힘 -> 다른 칸을 써야 함.
    cup = world.cup_names()[0]
    put_cup(world, cup, slot_xyz(world, 3) + [0.03, 0, 0])
    cups, slots = ledger.observe(world)
    assert cups[cup] == "misplaced"
    assert slots[3] == cup


def test_tilted_cup_on_slot_is_misplaced(world):
    # 정중앙이어도 10도 기울면(허용 5도 초과) misplaced.
    cup = world.cup_names()[0]
    put_cup(world, cup, slot_xyz(world, 2), tilt_deg=10)
    cups, _ = ledger.observe(world)
    assert cups[cup] == "misplaced"


def test_cup_on_floor_is_lost(world):
    # 상자 밖 바닥에 떨어진 컵: 팔이 바닥까지 못 내려가므로 lost(포기).
    cup = world.cup_names()[0]
    put_cup(world, cup, [0.0, 0.0, 0.0])
    cups, _ = ledger.observe(world)
    assert cups[cup] == "lost"


def test_too_many_attempts_skips_a_cup(world):
    # 같은 컵에서 복구를 MAX_ATTEMPTS번 했으면 상자 안에 있어도 skipped(더 시도 안 함).
    cup = world.cup_names()[0]
    cups, _ = ledger.observe(world, {cup: ledger.MAX_ATTEMPTS})
    assert cups[cup] == "skipped"
    cups, _ = ledger.observe(world, {cup: ledger.MAX_ATTEMPTS - 1})
    assert cups[cup] == "in_box"


def test_done_when_no_empty_slot_or_no_cup(world):
    # 끝나는 조건: 집을 컵(in_box/held)이 없거나, 빈 칸이 없을 때.
    cups = {"a": "placed", "b": "lost"}
    assert ledger.nothing_left(cups, {1: "a", 2: None})
    cups = {"a": "in_box"}
    assert ledger.nothing_left(cups, {1: "x"})
    assert not ledger.nothing_left({"a": "held"}, {1: None})
