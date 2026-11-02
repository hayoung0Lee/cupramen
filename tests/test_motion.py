"""움직임(motion) 테스트: 목표에 못 가도 끝나는지.

JointMove / ToolLine은 정해진 시간(duration) 동안 목표로 움직이고, 목표 근처(tol)에 오면
끝난다. 막혀서 끝내 못 가더라도 duration + settle 초가 지나면 "끝"으로 처리한다(무한히
기다리지 않음). 이 경우를 실제로 돌려 본다.

    .venv/bin/python -m pytest tests/test_motion.py -q
"""

import sys
from pathlib import Path

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from shelving.utils.motion import JointMove  # noqa: E402
from sim.world import World  # noqa: E402


def run(world, motion, max_s):
    """20 ms마다 step → 끝나면 걸린 시간(초), 안 끝나면 None."""
    m, d = world.m, world.d
    t0 = d.time
    while d.time - t0 < max_s:
        if motion.step(world):
            return d.time - t0
        for _ in range(10):
            mujoco.mj_step(m, d)
    return None


def test_joint_move_that_cannot_arrive_still_finishes():
    # arm2의 범위는 최대 1.57 rad. 3.0 rad을 목표로 주면 관절이 끝까지 못 간다(막힌 것과 같음).
    # → 오류 없이, duration(1초) + settle(0.5초) 뒤에 끝나야 한다.
    world = World.create()
    world.reset(0)
    took = run(world, JointMove({"arm2": 3.0}, duration=1.0, settle=0.5), max_s=5.0)
    assert took is not None and 1.4 < took < 1.7


def test_joint_move_that_arrives_finishes_early():
    # 갈 수 있는 목표면 duration 직후, 목표 근처에 도착하자마자 끝난다.
    world = World.create()
    world.reset(0)
    took = run(world, JointMove({"arm2": 0.3}, duration=1.0), max_s=5.0)
    assert took is not None and took < 1.5
