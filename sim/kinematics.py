"""Inverse kinematics for the youBot-lift arm (lift + arm1..arm4), tool pointing down.

Damped least squares on the tool_tip site Jacobian, solved on a scratch MjData so the
running simulation is untouched. arm5 (tool roll) does not affect a downward suction tool
and is left out.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from sim.world import PREFIX, r

IK_JOINTS = ["lift_z", "arm1", "arm2", "arm3", "arm4"]
DOWN = np.array([0.0, 0.0, -1.0])
SHOULDER_AT_ZERO_LIFT = 0.75  # arm2 height when lift_z = 0
TOOL_LENGTH = 0.153           # arm4 axis to tool tip


def level_reach_seed(target: np.ndarray) -> dict[str, float]:
    """IK start: shoulder level with the wrist, arm reaching straight out, tool down.

    Starting here keeps the elbow from popping up into the shelf board above.
    """
    lift = float(np.clip(target[2] + TOOL_LENGTH - SHOULDER_AT_ZERO_LIFT, 0.0, 0.6))
    return {"lift_z": lift, "arm2": 1.45, "arm3": 0.1, "arm4": 1.55}


@dataclass
class IKResult:
    q: dict[str, float]
    pos_err: float      # m
    axis_err: float     # rad between tool axis and straight down
    ok: bool


class ArmIK:
    def __init__(self, m: mujoco.MjModel):
        self.m = m
        self.d = mujoco.MjData(m)
        self.site = m.site(r("tool_tip")).id
        self.dofs = np.array([m.joint(r(j)).dofadr[0] for j in IK_JOINTS])
        self.qadr = np.array([m.joint(r(j)).qposadr[0] for j in IK_JOINTS])
        self.lo = np.array([m.joint(r(j)).range[0] for j in IK_JOINTS])
        self.hi = np.array([m.joint(r(j)).range[1] for j in IK_JOINTS])

    def _load(self, src: mujoco.MjData) -> None:
        """Copy the full configuration, including mocap box/shelf poses, from src."""
        self.d.qpos[:] = src.qpos
        self.d.mocap_pos[:] = src.mocap_pos
        self.d.mocap_quat[:] = src.mocap_quat

    def solve(
        self,
        src: mujoco.MjData,
        target: np.ndarray,
        seed: dict[str, float] | None = None,
        base: tuple[float, float, float] | None = None,
        iters: int = 200,
        damping: float = 0.05,
        pos_tol: float = 0.003,
        axis_tol: float = 0.03,
    ) -> IKResult:
        """src: state to start from. base: solve as if the base stood at (x, y, yaw) instead.
        target: tool tip, world frame."""
        m, d = self.m, self.d
        self._load(src)
        if base is not None:
            for j, v in zip(("base_x", "base_y", "base_yaw"), base):
                d.qpos[m.joint(r(j)).qposadr[0]] = v
        if seed:
            for j, v in seed.items():
                d.qpos[m.joint(r(j)).qposadr[0]] = v
        q = d.qpos[self.qadr].copy()
        jacp = np.zeros((3, m.nv))
        jacr = np.zeros((3, m.nv))
        for _ in range(iters):
            d.qpos[self.qadr] = q
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            tip = d.site_xpos[self.site]
            axis = d.site_xmat[self.site].reshape(3, 3)[:, 2]
            e_pos = target - tip
            e_rot = np.cross(axis, DOWN)  # rotate tool axis toward straight down
            if np.linalg.norm(e_pos) < pos_tol and np.linalg.norm(e_rot) < axis_tol:
                break
            mujoco.mj_jacSite(m, d, jacp, jacr, self.site)
            J = np.vstack([jacp[:, self.dofs], 0.3 * jacr[:, self.dofs]])
            e = np.concatenate([e_pos, 0.3 * e_rot])
            dq = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(6), e)
            q = np.clip(q + np.clip(dq, -0.2, 0.2), self.lo, self.hi)
        d.qpos[self.qadr] = q
        mujoco.mj_kinematics(m, d)
        pos_err = float(np.linalg.norm(target - d.site_xpos[self.site]))
        axis = d.site_xmat[self.site].reshape(3, 3)[:, 2]
        axis_err = float(np.arccos(np.clip(axis @ DOWN, -1.0, 1.0)))
        return IKResult(
            q={j: float(v) for j, v in zip(IK_JOINTS, q)},
            pos_err=pos_err,
            axis_err=axis_err,
            ok=pos_err < 0.005 and axis_err < 0.05,
        )

    def contacts_at(self, src: mujoco.MjData, q: dict[str, float],
                    base: tuple[float, float, float] | None = None) -> list[tuple[str, str]]:
        """Robot-vs-scene contacts if the robot in state src were moved to q (and base), as a
        pure collision check."""
        m, d = self.m, self.d
        self._load(src)
        if base is not None:
            for j, v in zip(("base_x", "base_y", "base_yaw"), base):
                d.qpos[m.joint(r(j)).qposadr[0]] = v
        for j, v in q.items():
            d.qpos[m.joint(r(j)).qposadr[0]] = v
        mujoco.mj_forward(m, d)
        out = []
        for c in d.contact[: d.ncon]:
            n1, n2 = m.geom(c.geom1).name, m.geom(c.geom2).name
            if n1.startswith(PREFIX) != n2.startswith(PREFIX):
                out.append((n1, n2))
        return out
