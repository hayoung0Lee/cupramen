"""Inverse kinematics for the youBot-lift arm (lift + arm1..arm4), tool pointing down.

Layer: shelving, robot capability (knows no cups or shelves) — arm math. Computes joint
angles and collision checks on a copy of the state; never moves the robot.

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
SHOULDER_AT_ZERO_LIFT = 0.70  # arm2 height when lift_z = 0
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
        self.d.eq_active[:] = src.eq_active      # which cup the pad holds

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

    def tool_tip_at(self, src: mujoco.MjData, q: dict[str, float],
                    base: tuple[float, float, float]) -> np.ndarray:
        """Where the tool tip would be with the base at `base` and the joints at q."""
        m, d = self.m, self.d
        self._load(src)
        for j, v in zip(("base_x", "base_y", "base_yaw"), base):
            d.qpos[m.joint(r(j)).qposadr[0]] = v
        for j, v in q.items():
            d.qpos[m.joint(r(j)).qposadr[0]] = v
        mujoco.mj_kinematics(m, d)
        return d.site_xpos[self.site].copy()

    def contacts_at(self, src: mujoco.MjData, q: dict[str, float],
                    base: tuple[float, float, float] | None = None,
                    margin: float = 0.0) -> list[tuple[str, str]]:
        """Robot-vs-scene contacts if the robot in state src were moved to q (and base), as a
        pure collision check. A cup held by suction moves with the pad and counts as part
        of the robot. margin > 0 also reports geoms closer than that (the robot's geom
        margins are raised for this one check and put back)."""
        m, d = self.m, self.d
        self._load(src)
        if base is not None:
            for j, v in zip(("base_x", "base_y", "base_yaw"), base):
                d.qpos[m.joint(r(j)).qposadr[0]] = v
        for j, v in q.items():
            d.qpos[m.joint(r(j)).qposadr[0]] = v
        held = self._carry_held_cup()
        mine = np.array([m.body(m.geom_bodyid[g]).name.startswith(PREFIX)
                         or m.geom_bodyid[g] == held for g in range(m.ngeom)])
        saved = m.geom_margin.copy()
        m.geom_margin[mine] += margin
        try:
            mujoco.mj_forward(m, d)
        finally:
            m.geom_margin[:] = saved
        out = []
        for c in d.contact[: d.ncon]:
            if mine[c.geom1] != mine[c.geom2] and c.dist < margin + 1e-9:
                n1, n2 = m.geom(c.geom1).name, m.geom(c.geom2).name
                out.append((n1, n2) if mine[c.geom1] else (n2, n1))
        return out

    def _carry_held_cup(self) -> int:
        """Move the suction-held cup (if any) to where its weld puts it relative to the pad
        at the current scratch joints. Returns its body id, or -1."""
        m, d = self.m, self.d
        for e in range(m.neq):
            if not d.eq_active[e] or not m.equality(e).name.startswith("suction_"):
                continue
            cup = m.eq_obj2id[e]
            mujoco.mj_kinematics(m, d)
            pad = m.eq_obj1id[e]
            pos, quat = np.zeros(3), np.zeros(4)
            mujoco.mju_rotVecQuat(pos, m.eq_data[e, 3:6], d.xquat[pad])
            mujoco.mju_mulQuat(quat, d.xquat[pad], m.eq_data[e, 6:10])
            adr = m.jnt_qposadr[m.body_jntadr[cup]]
            d.qpos[adr:adr + 3] = d.xpos[pad] + pos
            d.qpos[adr + 3:adr + 7] = quat
            return cup
        return -1
