"""Where to park the base so the arm can work on a box cup or a shelf slot.

Layer: sim — geometry only. Computes candidate base poses; never moves anything.

The arm is mounted at the front of the base, so the base stops facing the target with its
front edge a small gap away from the box stand / shelf, laterally aligned with the target.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

BASE_HALF_LENGTH = 0.29   # chassis half size along robot x (youbot_lift.xml)
# Chassis front edge to the box stand / shelf front. The youBot arm base and first link
# stick out ~5 cm past the chassis front; with 3 cm they sat 2 cm inside the box stand and
# the shelf's upper board.
GAP = 0.06
BOX_STAND_HALF = (0.253, 0.198)   # box_stand half sizes (x, y), box.xml
BOX_INTERIOR_HALF = (0.225, 0.17)
SHELF_HALF_DEPTH = 0.20   # shelf 900 x 400 mm, front = -y
SHELF_HALF_WIDTH = 0.45


@dataclass
class BasePose:
    x: float
    y: float
    yaw: float
    side: str


def _rot(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def _to_world(origin_xy, yaw, local_xy) -> np.ndarray:
    return np.asarray(origin_xy) + _rot(yaw) @ np.asarray(local_xy)


def box_side_poses(box_pos, box_yaw, target_xy) -> list[BasePose]:
    """Candidate base poses on all four sides of the box stand, nearest side first."""
    local = _rot(box_yaw).T @ (np.asarray(target_xy) - np.asarray(box_pos[:2]))
    hx, hy = BOX_STAND_HALF
    ix, iy = BOX_INTERIOR_HALF
    stand = BASE_HALF_LENGTH + GAP
    # (name, outward normal in box frame, half depth, lateral axis index, lateral limit)
    sides = [
        ("-y", np.array([0.0, -1.0]), hy, 0, ix),
        ("+y", np.array([0.0, 1.0]), hy, 0, ix),
        ("-x", np.array([-1.0, 0.0]), hx, 1, iy),
        ("+x", np.array([1.0, 0.0]), hx, 1, iy),
    ]
    out = []
    for name, n, h, lat_i, lat_lim in sides:
        p = n * (h + stand)
        p[lat_i] = np.clip(local[lat_i], -lat_lim, lat_lim)
        depth = h - float(n @ local)  # how far the target is inside from this face
        heading_local = np.arctan2(-n[1], -n[0])  # robot faces the box
        w = _to_world(box_pos[:2], box_yaw, p)
        out.append((depth, BasePose(float(w[0]), float(w[1]), float(box_yaw + heading_local), name)))
    out.sort(key=lambda t: t[0])
    return [p for _, p in out]


# Standoffs to try in front of the shelf. Close in, the long arm has to fold and its elbow
# hits the board above; further out it reaches in nearly straight and level.
SHELF_GAPS = (0.03, 0.10, 0.15, 0.20, 0.25, 0.30)


def shelf_front_pose(shelf_pos, shelf_yaw, target_xy, gap: float = GAP) -> BasePose:
    """Base pose in front of the shelf, aligned with the slot, `gap` from the shelf front."""
    local = _rot(shelf_yaw).T @ (np.asarray(target_xy) - np.asarray(shelf_pos[:2]))
    p = np.array([local[0], -(SHELF_HALF_DEPTH + BASE_HALF_LENGTH + gap)])
    w = _to_world(shelf_pos[:2], shelf_yaw, p)
    return BasePose(float(w[0]), float(w[1]), float(shelf_yaw + np.pi / 2), f"front+{gap:.2f}")
