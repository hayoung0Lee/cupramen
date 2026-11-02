"""Incremental motion primitives.

Layer: shelving, robot capability (knows no cups or shelves) — how to move over time.
Each step() writes motor commands (ctrl) for one tick; physics then moves the robot.
Doesn't decide what to do or when (the nodes in shelving/ do).

Each primitive is advanced by calling step(world) once per control tick and returns True when
finished. Nothing here loops over mj_step: the caller owns the simulation loop. This is the
shape a Behavior Tree action needs (RUNNING until done): the nodes run them as stages.
"""

from __future__ import annotations

import math

import numpy as np

from sim.world import POSITION_ACTUATOR, World, r


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class JointMove:
    """Move lift/arm joints to targets along a smooth (smoothstep) joint-space path."""

    def __init__(self, target: dict[str, float], duration: float = 2.0, tol: float = 0.01):
        self.target, self.duration, self.tol = target, duration, tol
        self.t0 = None
        self.q0: dict[str, float] = {}

    def step(self, world: World) -> bool:
        d = world.d
        if self.t0 is None:
            self.t0 = d.time
            self.q0 = {j: float(d.actuator(r(POSITION_ACTUATOR[j])).ctrl[0]) for j in self.target}
        s = min(1.0, (d.time - self.t0) / self.duration)
        s = s * s * (3 - 2 * s)
        for j, q1 in self.target.items():
            d.actuator(r(POSITION_ACTUATOR[j])).ctrl = self.q0[j] + s * (q1 - self.q0[j])
        if s < 1.0:
            return False
        err = max(abs(float(d.joint(r(j)).qpos[0]) - q1) for j, q1 in self.target.items())
        return err < self.tol or d.time - self.t0 > self.duration + self.settle


class BaseMove:
    """Drive the holonomic base through waypoints (x, y, yaw) with a capped P controller."""

    STUCK_S = 3.0
    STUCK_GAIN = 0.01

    def __init__(self, waypoints, vmax: float = 0.4, wmax: float = 0.8,
                 amax: float = 0.4, alpha_max: float = 1.0,
                 pos_tol: float = 0.01, yaw_tol: float = 0.01, pass_tol: float = 0.08):
        self.waypoints = [tuple(w) for w in waypoints]
        self.vmax, self.wmax = vmax, wmax
        self.amax, self.alpha_max = amax, alpha_max  # m/s^2, rad/s^2: no jerky starts/stops
        self.pos_tol, self.yaw_tol, self.pass_tol = pos_tol, yaw_tol, pass_tol
        self.i = 0
        self.t_last = None
        self.cmd = np.zeros(3)
        self.stuck = False            # set when the base stops getting closer (see step)
        self.best, self.t_best = math.inf, None

    def step(self, world: World) -> bool:
        d = world.d
        x, y, yaw = world.base_pose()
        gx, gy, gyaw = self.waypoints[self.i]
        last = self.i == len(self.waypoints) - 1
        e = np.array([gx - x, gy - y])
        dist = float(np.linalg.norm(e))
        eyaw = _wrap(gyaw - yaw)
        if not last and dist < self.pass_tol:
            self.i += 1
            return False
        if last and dist < self.pos_tol and abs(eyaw) < self.yaw_tol:
            self.stop(world)
            return True
        # Blocked? Remaining distance (turning counts too) has not shrunk by STUCK_GAIN in
        # STUCK_S. Without this a base pushing against something waits for the timeout.
        remaining = dist + 0.3 * abs(eyaw) + sum(
            math.dist(self.waypoints[k][:2], self.waypoints[k + 1][:2])
            for k in range(self.i, len(self.waypoints) - 1))
        if remaining < self.best - self.STUCK_GAIN or self.t_best is None:
            self.best, self.t_best = remaining, d.time
        elif d.time - self.t_best > self.STUCK_S:
            self.stuck = True
        # P control toward the waypoint; speed also capped so we can still brake within amax
        vcap = min(self.vmax, math.sqrt(2 * self.amax * dist)) if last else self.vmax
        v = e * min(1.5, vcap / max(dist, 1e-6))
        w = float(np.clip(2.0 * eyaw, -self.wmax, self.wmax))
        dt = 0.0 if self.t_last is None else d.time - self.t_last
        self.t_last = d.time
        dv = np.clip(np.array([v[0], v[1]]) - self.cmd[:2], -self.amax * dt, self.amax * dt)
        dw = float(np.clip(w - self.cmd[2], -self.alpha_max * dt, self.alpha_max * dt))
        self.cmd = self.cmd + np.array([dv[0], dv[1], dw])
        d.actuator(r("base_vx")).ctrl = self.cmd[0]
        d.actuator(r("base_vy")).ctrl = self.cmd[1]
        d.actuator(r("base_wz")).ctrl = self.cmd[2]
        return False

    @staticmethod
    def stop(world: World) -> None:
        for a in ("base_vx", "base_vy", "base_wz"):
            world.d.actuator(r(a)).ctrl = 0.0


class Wait:
    def __init__(self, seconds: float):
        self.seconds, self.t0 = seconds, None

    def step(self, world: World) -> bool:
        if self.t0 is None:
            self.t0 = world.d.time
        return world.d.time - self.t0 >= self.seconds


class Lazy:
    """Build the real primitive on first step, from the state at that moment.

    For motions whose target depends on what actually happened before (e.g. where the cup
    ended up on the pad), as opposed to targets planned up front.
    """

    def __init__(self, factory):
        self.factory, self.inner = factory, None

    def step(self, world: World) -> bool:
        if self.inner is None:
            self.inner = self.factory(world)
        return self.inner.step(world)


class ToolLine:
    """Move the tool tip along a straight line to `target` (world), tool pointing down.

    JointMove interpolates joint angles, so the tip travels on a curve between the two
    poses; under a shelf board that curve can rise into the board. Here the tip target
    moves along the line and IK is re-solved every step from the current joints.
    """

    def __init__(self, ik, target, duration: float = 1.5, tol: float = 0.005,
                 axis_tol: float | None = None, settle: float = 1.0):
        self.ik, self.target = ik, np.asarray(target, float)
        self.duration, self.tol, self.settle = duration, tol, settle
        # How straight down the tool must point. The IK default (0.03 rad) moves the bottom
        # of a held cup ~11 cm below the pad by ~3 mm; fine teleop steps need it tighter.
        self.solve_opts = {} if axis_tol is None else {"axis_tol": axis_tol}
        self.t0 = None
        self.p0 = None

    def step(self, world: World) -> bool:
        d = world.d
        if self.t0 is None:
            self.t0, self.p0 = d.time, world.tool_tip()
        s = min(1.0, (d.time - self.t0) / self.duration)
        s = s * s * (3 - 2 * s)
        res = self.ik.solve(d, self.p0 + s * (self.target - self.p0), seed=world.joint_positions(),
                            **self.solve_opts)
        for j, q in res.q.items():
            d.actuator(r(POSITION_ACTUATOR[j])).ctrl = q
        if s < 1.0:
            return False
        err = float(np.linalg.norm(world.tool_tip() - self.target))
        return err < self.tol or d.time - self.t0 > self.duration + self.settle
