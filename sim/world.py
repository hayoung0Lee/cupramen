"""The simulated world: the provided cup-ramen scene with the youBot-lift robot attached.

Layer: sim — the scene and the robot's body, like a remote control with a status screen.
  does    build the scene, reset it for a seed, answer "where is ...?", and make *instant*
          changes: teleport (for setup and checks), suction on / off
  doesn't decide what to do or when (the nodes in shelving/), and doesn't move the robot over
          time (shelving/utils/base_action.py writes motor commands; physics moves the robot)

The provided assets are never modified; the robot is attached at load time with MjSpec.
Asset location comes from $CUPRAMEN_ASSETS (default: guide/cupramen_sim_assets).
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parent.parent
ASSETS = Path(os.environ.get("CUPRAMEN_ASSETS", REPO / "guide" / "cupramen_sim_assets"))
ROBOT_XML = REPO / "sim" / "robot" / "youbot_lift.xml"
PREFIX = "robot/"

if str(ASSETS) not in sys.path:
    sys.path.insert(0, str(ASSETS))
import randomize  # noqa: E402  (provided module, lives in ASSETS)

ARM_JOINTS = ["arm1", "arm2", "arm3", "arm4", "arm5"]
BASE_JOINTS = ["base_x", "base_y", "base_yaw"]
# Arm upright, lift at the bottom: compact pose for driving.
STOW = {"lift_z": 0.0, "arm1": 0.0, "arm2": 0.0, "arm3": 0.0, "arm4": 0.0, "arm5": 0.0}
# Position actuators share the joint name, except the lift.
POSITION_ACTUATOR = {"lift_z": "lift", **{j: j for j in ARM_JOINTS}}


def r(name: str) -> str:
    """Prefixed robot element name."""
    return PREFIX + name


# Suction seal. Switching suction on holds a cup only if the pad sits on its lid:
SEAL_MAX_OFFSET = 0.025   # m:   pad centre at most 2.5 cm (sideways) from the lid centre
SEAL_MAX_TILT = 8.0       # deg: cup axis at most 8 deg off the tool axis


def build_model() -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(str(ASSETS / "scene.xml"))
    robot = mujoco.MjSpec.from_file(str(ROBOT_XML))
    spec.attach(robot, prefix=PREFIX, frame=spec.worldbody.add_frame())
    # Suction is modelled as a vacuum seal: one inactive weld per cup, switched on with the
    # cup's current pose relative to the pad once a seal forms. MuJoCo's adhesion actuator
    # only pulls at contact points, so a lifted cup peeled to one point and swung ~50 deg.
    for cup in randomize.CUP_BODIES:
        eq = spec.add_equality()
        eq.name, eq.type, eq.objtype = f"suction_{cup}", mujoco.mjtEq.mjEQ_WELD, mujoco.mjtObj.mjOBJ_BODY
        eq.name1, eq.name2, eq.active = r("suction_pad"), cup, False
    return spec.compile()


@dataclass
class World:
    m: mujoco.MjModel
    d: mujoco.MjData
    info: dict | None = None

    @classmethod
    def create(cls) -> "World":
        m = build_model()
        return cls(m, mujoco.MjData(m))

    # ---- episode -------------------------------------------------------
    def reset(self, seed: int) -> dict:
        """Apply the benchmark layout for `seed`, then put the robot at the start pose.

        episode_for_seed() calls mj_resetData, so the robot pose must be set afterwards.
        """
        self.info = randomize.episode_for_seed(self.m, self.d, seed)
        start = self.info["mobility"]["robot_start"]
        self.set_base_pose(start["pos"][0], start["pos"][1], start["yaw"])
        self.set_joint_positions(STOW)
        mujoco.mj_forward(self.m, self.d)
        return self.info

    # ---- direct state access (teleport; for setup and checks) -----------
    def set_base_pose(self, x: float, y: float, yaw: float) -> None:
        for name, v in zip(BASE_JOINTS, (x, y, yaw)):
            self.d.joint(r(name)).qpos = v
            self.d.joint(r(name)).qvel = 0.0
        for a in ("base_vx", "base_vy", "base_wz"):
            self.d.actuator(r(a)).ctrl = 0.0

    def base_pose(self) -> tuple[float, float, float]:
        return tuple(float(self.d.joint(r(j)).qpos[0]) for j in BASE_JOINTS)

    def set_joint_positions(self, q: dict[str, float]) -> None:
        """Set lift/arm joints and their position targets so the pose is held."""
        for name, v in q.items():
            self.d.joint(r(name)).qpos = v
            self.d.joint(r(name)).qvel = 0.0
            self.d.actuator(r(POSITION_ACTUATOR[name])).ctrl = v

    def joint_positions(self) -> dict[str, float]:
        return {j: float(self.d.joint(r(j)).qpos[0]) for j in POSITION_ACTUATOR}

    # ---- suction ---------------------------------------------------------
    def suction_on(self) -> str | None:
        """Switch suction on. A cup is held only if all three are true:
          1. the pad touches the cup
          2. seal_offset(cup) <= SEAL_MAX_OFFSET (2.5 cm)
          3. seal_tilt(cup)   <= SEAL_MAX_TILT   (8 deg)
        Then the pad and the cup are welded together (see _weld).
        Returns the held cup, or None if no cup sealed."""
        for cup in self.cups_touching_pad():
            if self.seal_offset(cup) <= SEAL_MAX_OFFSET and self.seal_tilt(cup) <= SEAL_MAX_TILT:
                self._weld(cup)
                return cup
        return None

    def cups_touching_pad(self) -> list[str]:
        """Cups in contact with the suction pad right now."""
        m, d = self.m, self.d
        pad = m.geom(r("pad")).id
        cups = []
        for c in d.contact[: d.ncon]:
            if pad in (c.geom1, c.geom2):
                other = m.body(m.geom_bodyid[c.geom2 if c.geom1 == pad else c.geom1]).name
                if other in randomize.CUP_BODIES and other not in cups:
                    cups.append(other)
        return cups

    def seal_offset(self, cup: str) -> float:
        """Sideways distance (m) from the pad centre to the cup's lid centre."""
        return float(np.linalg.norm((self.tool_tip() - self.cup_top(cup))[:2]))

    def seal_tilt(self, cup: str) -> float:
        """Angle (deg) between the cup's axis and the tool axis (0 = pad flat on the lid)."""
        tool_down = self.d.site(r("tool_tip")).xmat.reshape(3, 3)[:, 2]
        cup_up = self.d.body(cup).xmat.reshape(3, 3)[:, 2]
        return float(np.degrees(np.arccos(np.clip(-tool_down @ cup_up, -1.0, 1.0))))

    def suction_off(self) -> None:
        for cup in randomize.CUP_BODIES:
            self.d.eq_active[self.m.equality(f"suction_{cup}").id] = 0

    def held_cup(self) -> str | None:
        for cup in randomize.CUP_BODIES:
            if self.d.eq_active[self.m.equality(f"suction_{cup}").id]:
                return cup
        return None

    def _weld(self, cup: str) -> None:
        """Glue the cup to the pad as it sits now. MuJoCo has no vacuum; a weld is an invisible
        rigid link between two bodies that can be switched on and off. One per cup is made
        (inactive) in build_model; this records where the cup sits relative to the pad, so
        an off-centre cup stays off-centre instead of jumping to the pad centre."""
        m, d = self.m, self.d
        pad, body = m.body(r("suction_pad")).id, m.body(cup).id
        inv = np.zeros(4)
        mujoco.mju_negQuat(inv, d.xquat[pad])
        relpos, relquat = np.zeros(3), np.zeros(4)
        mujoco.mju_rotVecQuat(relpos, d.xpos[body] - d.xpos[pad], inv)
        mujoco.mju_mulQuat(relquat, inv, d.xquat[body])
        e = m.equality(f"suction_{cup}").id
        # weld eq_data: anchor(3), relpos(3), relquat(4), torquescale
        m.eq_data[e, 0:3] = 0.0
        m.eq_data[e, 3:6] = relpos
        m.eq_data[e, 6:10] = relquat
        m.eq_data[e, 10] = 1.0
        d.eq_active[e] = 1

    # ---- queries ---------------------------------------------------------
    def tool_tip(self) -> np.ndarray:
        return self.d.site(r("tool_tip")).xpos.copy()

    def cup_names(self) -> list[str]:
        return list(randomize.CUP_BODIES)

    def cup_top(self, cup: str) -> np.ndarray:
        return self.d.site(f"{cup}_top").xpos.copy()

    def robot_scene_contacts(self) -> list[tuple[str, str]]:
        """Contacts between a robot geom and a non-robot geom (names)."""
        out = []
        for c in self.d.contact[: self.d.ncon]:
            n1, n2 = self.m.geom(c.geom1).name, self.m.geom(c.geom2).name
            if n1.startswith(PREFIX) != n2.startswith(PREFIX):
                out.append((n1, n2))
        return out
