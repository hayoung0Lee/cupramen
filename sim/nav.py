"""2D grid path planning for the base: A* over an occupancy grid of the arena.

Obstacles are the box stand and the shelf footprints (oriented rectangles), inflated by the
robot's footprint radius, so the base centre can be treated as a point. The robot drives with
the arm tucked, which keeps that radius small.
"""

from __future__ import annotations

import heapq
import math

import numpy as np

from sim.placement import BOX_STAND_HALF, SHELF_HALF_DEPTH, SHELF_HALF_WIDTH

RES = 0.05            # m per cell
ARENA = (-3.3, 3.3)   # m, square; layouts are sampled in (-3, 3)
ROBOT_RADIUS = 0.45   # chassis half-diagonal 0.35 + tucked cup + margin


class GridPlanner:
    def __init__(self, info: dict, radius: float = ROBOT_RADIUS):
        self.n = int(round((ARENA[1] - ARENA[0]) / RES))
        c = ARENA[0] + (np.arange(self.n) + 0.5) * RES
        self.xs, self.ys = np.meshgrid(c, c, indexing="ij")
        self.occ = np.zeros((self.n, self.n), bool)
        self._add_rect(info["box"]["pos"], info["box"]["yaw"], BOX_STAND_HALF, radius)
        self._add_rect(info["shelf"]["pos"], info["shelf"]["yaw"],
                       (SHELF_HALF_WIDTH, SHELF_HALF_DEPTH), radius)

    def _add_rect(self, pos, yaw, half, radius):
        dx, dy = self.xs - pos[0], self.ys - pos[1]
        c, s = math.cos(yaw), math.sin(yaw)
        lx, ly = c * dx + s * dy, -s * dx + c * dy           # cell centres in the rect frame
        ox = np.maximum(np.abs(lx) - half[0], 0.0)
        oy = np.maximum(np.abs(ly) - half[1], 0.0)
        self.occ |= np.hypot(ox, oy) <= radius               # rounded-rectangle inflation

    def _cell(self, p):
        return tuple(int(np.clip((v - ARENA[0]) // RES, 0, self.n - 1)) for v in p[:2])

    def _pt(self, ij):
        return np.array([ARENA[0] + (ij[0] + 0.5) * RES, ARENA[0] + (ij[1] + 0.5) * RES])

    def free(self, p) -> bool:
        return not self.occ[self._cell(p)]

    def plan(self, start, goal) -> list[np.ndarray] | None:
        """Shortest collision-free (x, y) path, shortcut to few waypoints. None if blocked."""
        s, g = self._cell(start), self._cell(goal)
        if self.occ[s] or self.occ[g]:
            return None
        steps = [(dx, dy, math.hypot(dx, dy)) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]
        h = lambda a: math.hypot(a[0] - g[0], a[1] - g[1])  # noqa: E731
        openq, came, cost = [(h(s), s)], {s: None}, {s: 0.0}
        while openq:
            _, cur = heapq.heappop(openq)
            if cur == g:
                break
            for dx, dy, w in steps:
                nb = (cur[0] + dx, cur[1] + dy)
                if not (0 <= nb[0] < self.n and 0 <= nb[1] < self.n) or self.occ[nb]:
                    continue
                nc = cost[cur] + w
                if nc < cost.get(nb, math.inf):
                    cost[nb], came[nb] = nc, cur
                    heapq.heappush(openq, (nc + h(nb), nb))
        if g not in came:
            return None
        cells = [g]
        while came[cells[-1]] is not None:
            cells.append(came[cells[-1]])
        pts = [self._pt(c) for c in reversed(cells)]
        pts[0], pts[-1] = np.asarray(start[:2], float), np.asarray(goal[:2], float)
        return self._shortcut(pts)

    def _line_free(self, a, b) -> bool:
        n = max(2, int(np.linalg.norm(b - a) / (RES / 2)))
        return all(self.free(a + t * (b - a)) for t in np.linspace(0, 1, n))

    def _shortcut(self, pts):
        out, i = [pts[0]], 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self._line_free(pts[i], pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        return out
