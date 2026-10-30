"""
YouBot Actions with Resolver Integration
"""

# === 정리된 import (중복 제거) ===

import math  # noqa: F401

import py_trees

import controllers.youbot_controller.youbot_utils as YU  # noqa: F401
from behavioros.core import BaseAction, action

# youbot_actions.py (원래 방식으로 복원: init에서 home 캡처, home 스텝에서 self.home_q 사용)
# 유틸/IK: 반드시 "모델 프레임"으로 통일
from controllers.youbot_controller.youbot_utils import (
    arm_still,
    config_gripper,
    gripper_diag,
    read_joint_positions_model,
    read_world_pose_supervisor,
    set_arm_positions,  # q_model → 모터 변환 + 클램프 + setPosition()
    set_gripper_close,
    set_gripper_open,
    solve_youbot_ik_from_robot_frame,
)


def deref_bb(read_fn, val):
    # val이 "${key}"면 key를 블랙보드에서 읽어서 반환, 아니면 그대로 반환
    if isinstance(val, str) and val.startswith("${") and val.endswith("}"):
        key = val[2:-1].strip()
        return read_fn(key, None)
    return val


# youbot_actions.py
# Minimal: init → goto(IK) → home, with command latching


# youbot_actions.py
# ------------------------------------------------------------
# Minimal action: resolve → IK once → setPosition once → wait
#  - no gripper
#  - no waypoint / time-scaling
#  - no home return (just stops with SUCCESS when reached)
# ------------------------------------------------------------


# IK/변환/명령 헬퍼는 컨트롤러 유틸을 그대로 사용


# ======================================================
# Minimal Action: just go to the resolved IK once
#   - 읽기: devices, nearest_red_can
#   - 쓰기: pick_status (성공 시)
#   - 도달 판정: '터치 반경'으로 TCP-타깃 거리 체크
# ======================================================
@action(
    "pick_can",
    "Minimal: resolve → IK once → setPosition once → wait (no gripper/home)",
    reads=["devices", "nearest_can"],
    writes=["pick_status"],
)
class PickCanAction(BaseAction):
    def __init__(self, name: str, blackboard=None):
        super().__init__(name, blackboard)
        self.phase = "init"
        self.goal_q = None
        self._printed_once = False
        self._max_ticks = 500  # 최대 틱수 (약 10초 @10ms)
        self._tick_count = 0
        self.start_q = None

        self._grip_tick = 0
        self.GRIP_CLOSE_TICKS = 60  # ≈ 0.6s (주기 10ms 가정)
        self.GRIP_SETTLE_TICKS = 20  # ≈ 0.2s

    def execute(self) -> py_trees.common.Status:
        self._tick_count += 1
        if self._tick_count > self._max_ticks:
            print("[Pick][ERR] Timeout exceeded.")
            return py_trees.common.Status.FAILURE

        devs = self.read_blackboard("devices", {})
        raw = self.read_blackboard("nearest_can", None)
        tgt = deref_bb(self.read_blackboard, raw)

        if not devs or not isinstance(tgt, dict):
            print("[Pick][ERR] devices or nearest_can unavailable.")
            return py_trees.common.Status.FAILURE

        if self.phase == "init":
            # (1) 현재 관절 → IK 한 번만
            q_cur = read_joint_positions_model(devs)
            try:
                if not tgt.get("detected", False):
                    print("[Pick] No target detected → FAIL")
                    self.write_blackboard("pick_status", "no_target")
                    return py_trees.common.Status.FAILURE

                self.goal_q = solve_youbot_ik_from_robot_frame(
                    tgt, current_q=q_cur, debug=True
                )
                self.start_q = q_cur
            except Exception as e:
                print(f"[Pick][ERR] IK failed: {e}")
                return py_trees.common.Status.FAILURE

            try:
                config_gripper(devs, max_vel=0.03, avail_force=50.0)
            except Exception:
                pass
            set_gripper_open(devs)
            gripper_diag(devs, tag="init")  # 한 번 출력해 확인

            # (2) setPosition 한 번만
            set_arm_positions(devs, self.goal_q)
            print("[Pick] command sent once. Waiting to reach target...")
            self.phase = "approach_wait"
            self._printed_once = False
            return py_trees.common.Status.RUNNING

        if self.phase == "approach_wait":
            # 현재 관절 읽고, TCP-타깃 거리로 '닿음' 판정
            print(f"[Pick] waiting... (tick {self._tick_count}/{self._max_ticks})")
            q_now = read_joint_positions_model(devs)

            if not self._printed_once:
                print("[Pick] q_now(model): ", [f"{v:+.2f}" for v in q_now])
                print("[Pick] goal_q(model):", [f"{v:+.2f}" for v in self.goal_q])
                self._printed_once = True

            ok, max_delta = arm_still(devs)
            if ok:
                print("[Pick] TOUCH reached (distance ≤ radius) → SUCCESS")
                self.write_blackboard("pick_status", "success")

                self.phase = "grip"
                return py_trees.common.Status.RUNNING

            return py_trees.common.Status.RUNNING

        if self.phase == "grip":
            # 첫 진입 시 한 번만 닫기 명령
            if self._grip_tick == 0:
                set_gripper_close(devs)
                gripper_diag(devs, tag="after_close_cmd")
                print("[Pick][Grip] closing gripper...")
            self._grip_tick += 1

            if self._grip_tick < self.GRIP_CLOSE_TICKS:
                return py_trees.common.Status.RUNNING

            print("[Pick][Grip] close done → wait_grip")
            self._grip_tick = 0
            self.phase = "wait_grip"
            return py_trees.common.Status.RUNNING

        if self.phase == "wait_grip":
            # 닫은 뒤 잠깐 안정화
            self._grip_tick += 1
            if self._grip_tick < self.GRIP_SETTLE_TICKS:
                return py_trees.common.Status.RUNNING

            print("[Pick][Grip] settled → returning to start_q...")
            self.phase = "return"
            return py_trees.common.Status.RUNNING

        if self.phase == "return":
            # (미사용)
            set_arm_positions(devs, self.start_q)
            print("[Pick] returning to start_q...")
            self.phase = "return_wait"
            return py_trees.common.Status.RUNNING

        if self.phase == "return_wait":
            q_now = read_joint_positions_model(devs)
            ok, max_delta = arm_still(devs)
            if ok:
                print("[Pick] returned to start_q → done.")
                self.phase = "done"
                return py_trees.common.Status.RUNNING
            return py_trees.common.Status.RUNNING

        if self.phase == "done":
            print("[Pick] done.")
            return py_trees.common.Status.SUCCESS

        print(f"[Pick][WARN] Unknown phase: {self.phase}")
        return py_trees.common.Status.FAILURE


@action(
    "visit_can",
    "Visit a given colored can",
    reads=["devices", "nearest_can"],
    writes=["pick_status"],
)
class VisitCanAction(PickCanAction):
    def __init__(self, name: str, blackboard=None):
        super().__init__(name, blackboard)


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _wheel_max(devs: dict, fallback: float = 10.0) -> float:
    vals = []
    for name, m in devs.items():
        if isinstance(name, str) and name.startswith("wheel"):
            try:
                if hasattr(m, "getMaxVelocity"):
                    v = m.getMaxVelocity()
                    if v is not None:
                        vals.append(float(v))
                        continue
                if hasattr(m, "maxVelocity"):
                    v = m.maxVelocity
                    v = v() if callable(v) else v
                    if v is not None:
                        vals.append(float(v))
                        continue
            except Exception:
                pass
    return max(0.1, min(vals)) if vals else fallback


def _stop(devs: dict) -> None:
    for wid, m in devs.items():
        if (
            isinstance(wid, str)
            and wid.startswith("wheel")
            and hasattr(m, "setVelocity")
        ):
            m.setVelocity(0.0)


# =========================================================
# 축/회전 유틸 (XY 평면 투영)
# =========================================================
def _forward_left_axes_from_M_XY(M):
    """
    M(row-major 3x3)에서 로컬 X/Z축의 월드 표현 열벡터를 꺼내
    XY 평면으로 투영한 뒤 forward/left 선택.
      - 로컬 X = (m00, m10, m20)
      - 로컬 Z = (m02, m12, m22)
    forward는 XY 투영 노름 큰 쪽, left는 90° CCW 회전: (-fy, fx)
    반환: (fx, fy, lx, ly)  [정규화됨]
    """
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = M
    # 로컬 X/Z 열벡터의 XY성분
    fxX, fyX = m00, m10
    fxZ, fyZ = m02, m12

    if (fxZ * fxZ + fyZ * fyZ) > (fxX * fxX + fyX * fyX):
        fx, fy = fxZ, fyZ  # forward = local Z
    else:
        fx, fy = fxX, fyX  # forward = local X

    n = math.hypot(fx, fy)
    if n < 1e-9:
        fx, fy = 1.0, 0.0
    else:
        fx, fy = fx / n, fy / n

    lx, ly = -fy, fx
    return fx, fy, lx, ly


def _yaw_from_axis_angle_Z(
    ax: float, ay: float, az: float, ang: float, z_thresh: float = 0.9
):
    """
    목표 yaw(수평 회전) 추출. Up=Z 가정. 회전축이 (0,0,1)에 충분히 가까울 때만 사용.
    """
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-9:
        return None
    ax, ay, az = ax / n, ay / n, az / n
    if abs(az) < z_thresh:  # 수평 yaw로 보기 어려우면 무시
        return None
    # 회전행렬 일부로 로컬 X의 XY 성분에서 yaw 계산
    c, s, C = math.cos(ang), math.sin(ang), 1.0 - math.cos(ang)
    r00 = c + ax * ax * C
    r10 = ay * ax * C - az * s
    return math.atan2(r10, r00)


# =========================================================
# 타깃 포즈 읽기 (블랙보드)
# =========================================================
def _target_from_bb(read_fn, key, default_pose):
    pose = read_fn(key, None) or default_pose
    if not pose:
        raise ValueError(f"target '{key}' not set")
    t = pose.get("translation")
    r = pose.get("rotation")
    if not (isinstance(t, (list, tuple)) and len(t) == 3):
        raise ValueError("translation must be [x,y,z]")
    if not (isinstance(r, (list, tuple)) and len(r) == 4):
        raise ValueError("rotation must be [ax,ay,az,angle]")
    return (float(t[0]), float(t[1]), float(t[2])), (
        float(r[0]),
        float(r[1]),
        float(r[2]),
        float(r[3]),
    )


# =========================================================
# 메카넘 역기구학
# =========================================================
def _set_omni(
    devs,
    vx,
    vy,
    w,
    R=0.047,
    L=0.25,
    variant="B",
    wheel_order=("wheel3", "wheel4", "wheel1", "wheel2"),
    vy_flip=False,
    wheel_sign=(+1, +1, +1, +1),
):
    if vy_flip:
        vy = -vy
    if variant == "B":
        w1 = (vx + vy - L * w) / R
        w2 = (vx - vy + L * w) / R
        w3 = (vx - vy - L * w) / R
        w4 = (vx + vy + L * w) / R
    else:
        w1 = (vx - vy - L * w) / R
        w2 = (vx + vy + L * w) / R
        w3 = (vx + vy - L * w) / R
        w4 = (vx - vy + L * w) / R
    ws = [w1, w2, w3, w4]
    ws = [ws[i] * wheel_sign[i] for i in range(4)]
    wmax = _wheel_max(devs)
    for wid, val in zip(wheel_order, ws):
        m = devs.get(wid)
        if m and hasattr(m, "setVelocity"):
            m.setVelocity(max(-wmax, min(wmax, val)))


# =========================================================
# Omni Drive Base (XY 평면 전용)
# =========================================================
class OmniDriveToPoseBase(BaseAction):
    WHEEL_RADIUS = 0.047
    L_SUM = 0.25
    KX, KY = 0.8, 0.8
    KW = 2.8
    V_MAX = 0.60
    W_MAX = 2.50
    A_BRAKE = 0.9
    NEAR_XY = 0.06
    POS_TOL = 0.015
    YAW_TOL = math.radians(3.0)
    V_MIN = 0.03

    V_MAX_CREEP = 0.12
    V_MIN_CREEP = 0.0
    KX_CREEP, KY_CREEP = 1.2, 1.2
    KW_CREEP = 3.5
    BRAKE_MARGIN = 0.05
    V_EPS = 0.01
    SETTLE_TICKS = 15

    def __init__(self, name, pose_key, status_key, default_pose=None, blackboard=None):
        self.name = name
        self.blackboard = blackboard
        self.pose_key = pose_key
        self.status_key = status_key
        self.default_pose = default_pose or {}
        self._dbg_once = False
        super().__init__(name, blackboard)

        # 캘리 기본
        self.default_variant = "A"
        self.default_vy_flip = False
        self.default_order = ("wheel3", "wheel4", "wheel1", "wheel2")
        self.default_sign = (+1, +1, +1, +1)

        self._v_last = 0.0
        self._settle = 0

    # 블랙보드 헬퍼 (프로젝트의 BaseAction 대체용 최소구현)
    def read_blackboard(self, k, d=None):
        return self.blackboard.get(k, d) if self.blackboard else d

    def write_blackboard(self, k, v):
        if self.blackboard is not None:
            self.blackboard[k] = v

    def execute(self) -> py_trees.common.Status:
        self.write_blackboard(self.status_key, "running")

        devs = self.read_blackboard("devices", {})
        if not devs:
            print(f"[{self.name}] devices missing")
            return py_trees.common.Status.FAILURE

        # 1) 타깃 포즈 읽기
        try:
            (tx, ty, tz), (ax, ay, az, ang) = _target_from_bb(
                self.read_blackboard, self.pose_key, self.default_pose
            )
            if not self._dbg_once:
                print(
                    f"[{self.name}] target '{self.pose_key}': t=({tx:+.6f},{ty:+.6f},{tz:+.6f}), r=({ax:+.6f},{ay:+.6f},{az:+.6f},{ang:+.6f})"
                )
        except Exception as e:
            print(f"[{self.name}] {e}")
            return py_trees.common.Status.FAILURE

        # 2) 현재 포즈/행렬 (XY 평면)
        pose = read_world_pose_supervisor(devs)
        if pose is None:
            print(f"[{self.name}][ERR] supervisor could not provide pose")
            return py_trees.common.Status.FAILURE
        xw, yw, M = pose

        # 3) 전/좌 축 (XY 평면 투영)
        fx, fy, lx, ly = _forward_left_axes_from_M_XY(M)

        # 4) 월드 오차 → 전/좌 축으로 투영 (ex=앞/뒤, ey=좌/우)
        dx, dy = tx - xw, ty - yw
        ex = dx * fx + dy * fy
        ey = dx * lx + dy * ly

        if not self._dbg_once:
            self._dbg_once = True
            yaw_now_dbg = math.atan2(fy, fx)
            print(
                f"[{self.name}] axes f=({fx:+.3f},{fy:+.3f}), l=({lx:+.3f},{ly:+.3f}) | now=({xw:+.3f},{yw:+.3f}) goal=({tx:+.3f},{ty:+.3f}) world d=({dx:+.3f},{dy:+.3f}) proj ex/ey=({ex:+.3f},{ey:+.3f}) yaw_now={math.degrees(yaw_now_dbg):+.2f}°"
            )

        # 5) yaw: 기본 OFF, 근접 시에만 ON (Up=Z 기준)
        use_yaw = bool(self.read_blackboard("align_yaw", False))
        yaw_now = math.atan2(fy, fx)
        tyaw = _yaw_from_axis_angle_Z(ax, ay, az, ang) if use_yaw else None

        d_world = math.hypot(dx, dy)
        YAW_GATE_DIST = self.NEAR_XY
        if tyaw is not None and (
            d_world > YAW_GATE_DIST or abs(ang) < math.radians(3.0)
        ):
            tyaw = None
        eyaw = _wrap_pi((tyaw - yaw_now) if tyaw is not None else 0.0)

        # # 6) 완료 판정
        # d_proj = math.hypot(ex, ey)
        # if d_proj < self.POS_TOL and (tyaw is None or abs(eyaw) < self.YAW_TOL):
        #     _stop(devs)
        #     self.write_blackboard(self.status_key, "complete")
        #     return py_trees.common.Status.SUCCESS

        # 7) 속도 명령 (P + 제동거리 감속)
        # 7) velocity command (P + braking-based creep)
        # base gains/limits
        kx, ky, kw = self.KX, self.KY, self.KW
        v_max_cmd = self.V_MAX
        v_min_cmd = self.V_MIN

        # current error norms
        d_xy = math.hypot(ex, ey)

        # braking distance from previous commanded speed
        d_stop = (self._v_last * self._v_last) / (2.0 * self.A_BRAKE)

        # enter creep if we are within stop distance + margin (or simply very near)
        creep = (d_world <= d_stop + self.BRAKE_MARGIN) or (d_xy <= 0.25)

        if creep:
            kx, ky, kw = self.KX_CREEP, self.KY_CREEP, self.KW_CREEP
            v_max_cmd = self.V_MAX_CREEP
            v_min_cmd = self.V_MIN_CREEP  # crucial: allow true zero

        # translational P
        vx = kx * ex
        vy = ky * ey

        # yaw control gated by distance (kept from your logic)
        if tyaw is None or abs(eyaw) < math.radians(2.0):
            w = 0.0
        else:
            yaw_weight = max(0.0, min(1.0, 1.0 - (d_world / self.NEAR_XY)))
            w = kw * eyaw * yaw_weight
            w = max(-self.W_MAX, min(self.W_MAX, w))

        # speed limiting: braking envelope and absolute cap
        v_cmd = math.hypot(vx, vy)
        # desired max based on distance remaining (same envelope as before)
        v_brake_cap = math.sqrt(max(0.0, 2.0 * self.A_BRAKE * d_xy))
        if v_cmd > 1e-9:
            scale = min(1.0, v_brake_cap / v_cmd, v_max_cmd / v_cmd)
            vx *= scale
            vy *= scale

        # deadzone: only when not in creep
        if (not creep) and d_xy > self.POS_TOL and math.hypot(vx, vy) < self.V_MIN:
            if abs(ex) >= abs(ey):
                vx = math.copysign(self.V_MIN, vx if abs(vx) > 1e-6 else ex)
            else:
                vy = math.copysign(self.V_MIN, vy if abs(vy) > 1e-6 else ey)

        # 8) 구동 설정
        variant = self.read_blackboard("omni_variant", self.default_variant)
        vy_flip = self.read_blackboard("omni_vy_flip", self.default_vy_flip)
        wheel_order = self.read_blackboard("wheel_order", self.default_order)
        wheel_sign = self.read_blackboard("wheel_sign", self.default_sign)

        _set_omni(
            devs,
            vx,
            vy,
            w,
            R=self.WHEEL_RADIUS,
            L=self.L_SUM,
            variant=variant,
            wheel_order=wheel_order,
            wheel_sign=wheel_sign,
            vy_flip=vy_flip,
        )

        v_mag = math.hypot(vx, vy)
        self._v_last = v_mag
        d_proj = math.hypot(ex, ey)
        if d_proj < self.POS_TOL and (tyaw is None or abs(eyaw) < self.YAW_TOL):
            if v_mag <= self.V_EPS and abs(w) <= math.radians(1.0):
                self._settle += 1
                if self._settle >= self.SETTLE_TICKS:
                    _stop(devs)
                    self.write_blackboard(self.status_key, "complete")
                    return py_trees.common.Status.SUCCESS
            else:
                self._settle = 0
        else:
            self._settle = 0

        print(
            f"[{self.name}] cmd: vx={vx:+.3f}, vy={vy:+.3f}, w={w:+.3f}, d_xy={d_xy:+.3f}"
        )
        return py_trees.common.Status.RUNNING


# =========================================================
# Actions
# =========================================================
@action(
    "approach_work_area",
    "Holonomic drive to work area (world pose)",
    reads=[
        "devices",
        "work_area_pose",
        "omni_variant",
        "omni_vy_flip",
        "wheel_order",
        "wheel_sign",
        "align_yaw",
    ],
    writes=["approach_work_area_status"],
)
class ApproachWorkAreaAction(OmniDriveToPoseBase):
    def __init__(self, name="ApproachWorkAreaAction", blackboard=None):
        super().__init__(
            name=name,
            pose_key="work_area_pose",
            status_key="approach_work_area_status",
            default_pose={
                # 기본: world의 youbot 노드에 박힌 초기 포즈
                "translation": [
                    2.3392430145569888e-05,
                    3.9843097036594354e-05,
                    0.0988754668909782,
                ],
                "rotation": [
                    -0.34912126661372916,
                    0.9370315540803987,
                    -0.009284818560321423,
                    0.002605330968490995,
                ],
            },
            blackboard=blackboard,
        )


@action(
    "approach_box",
    "Holonomic drive to box (world pose)",
    reads=[
        "devices",
        "box_pose",
        "omni_variant",
        "omni_vy_flip",
        "wheel_order",
        "wheel_sign",
        "align_yaw",
    ],
    writes=["approach_box_status"],
)
class ApproachBoxAction(OmniDriveToPoseBase):
    def __init__(self, name="ApproachBoxAction", blackboard=None, default_pose=None):
        super().__init__(
            name=name,
            pose_key="box_pose",
            status_key="approach_box_status",
            default_pose={
                # 기본: world의 youbot 노드에 박힌 초기 포즈
                "translation": [
                    3.08732e-05,
                    0.530043,
                    0.0988755,
                ],
                "rotation": [
                    -0.3491036401243609,
                    0.9370446962544616,
                    -0.00905825,
                    0.00260536,
                ],
            },
            blackboard=blackboard,
        )


@action(
    "drop_can",
    "Given target: IK once → setPosition once → open → settle → close → return",
    reads=["devices", "drop_goal"],
    writes=["drop_status"],
)
class DropCanAction(BaseAction):
    def __init__(self, name: str, blackboard=None):
        super().__init__(name, blackboard)
        self.phase = "init"
        self._tick_count = 0
        self._max_ticks = 500

        self.start_q = None
        self.goal_q = None
        self._printed_once = False

        # timing (고정)
        self.OPEN_TICKS = 50  # ≈0.5s
        self.OPEN_SETTLE_TICKS = 20  # ≈0.2s
        self.CLOSE_TICKS = 40  # ≈0.4s
        self._phase_tick = 0

    def execute(self) -> py_trees.common.Status:
        self._tick_count += 1
        if self._tick_count > self._max_ticks:
            print("[Drop][ERR] Timeout exceeded.")
            return py_trees.common.Status.FAILURE

        devs = self.read_blackboard("devices", {})
        if not devs:
            print("[Drop][ERR] devices unavailable.")
            return py_trees.common.Status.FAILURE

        if self.phase == "init":
            # (0) 네가 준 타깃 그대로 읽기 (직접 dict 또는 "${key}" 간접참조 모두 허용)
            tgt = self.read_blackboard("drop_goal", None)

            if not tgt or not isinstance(tgt, dict):
                print("[Drop][ERR] drop_goal unavailable.")
                return py_trees.common.Status.FAILURE

            # (1) IK 1회 → 목표 명령 1회
            yaw_extra = float(tgt.get("yaw_extra_rad", 0.0))  # 없으면 0
            _ik_bias_deg_save = YU.IK_BIAS_DEG.copy()

            q_cur = read_joint_positions_model(devs)
            try:
                YU.IK_BIAS_DEG[0] = yaw_extra
                q_cur = read_joint_positions_model(devs)
                self.goal_q = solve_youbot_ik_from_robot_frame(
                    tgt, current_q=q_cur, debug=True
                )
                self.start_q = q_cur
            except Exception as e:
                print(f"[Drop][ERR] IK failed: {e}")
                return py_trees.common.Status.FAILURE
            finally:
                YU.IK_BIAS_DEG = _ik_bias_deg_save  # 반드시 복원

            # try:
            #     config_gripper(devs, max_vel=0.03, avail_force=50.0)
            # except Exception:
            #     pass

            set_arm_positions(devs, self.goal_q)
            print("[Drop] command sent once. Waiting to reach drop pose...")
            self._printed_once = False
            self.phase = "approach_wait"
            return py_trees.common.Status.RUNNING

        if self.phase == "approach_wait":
            if not self._printed_once:
                q_now = read_joint_positions_model(devs)
                print("[Drop] q_now(model): ", [f"{v:+.2f}" for v in q_now])
                print("[Drop] goal_q(model):", [f"{v:+.2f}" for v in self.goal_q])
                self._printed_once = True

            ok, _ = arm_still(devs)
            if ok:
                print("[Drop] Reached drop pose → release(open).")
                self._phase_tick = 0
                self.phase = "release_open"
                return py_trees.common.Status.RUNNING
            return py_trees.common.Status.RUNNING

        if self.phase == "release_open":
            if self._phase_tick == 0:
                set_gripper_open(devs)
                gripper_diag(devs, tag="after_open_cmd")
                print("[Drop][Release] opening gripper...")
            self._phase_tick += 1
            if self._phase_tick < self.OPEN_TICKS:
                return py_trees.common.Status.RUNNING

            print("[Drop][Release] open done → settle.")
            self._phase_tick = 0
            self.phase = "release_settle"
            return py_trees.common.Status.RUNNING

        if self.phase == "release_settle":
            self._phase_tick += 1
            if self._phase_tick < self.OPEN_SETTLE_TICKS:
                return py_trees.common.Status.RUNNING

            print("[Drop][Release] settled → close gripper.")
            self._phase_tick = 0
            self.phase = "return"
            return py_trees.common.Status.RUNNING

        if self.phase == "return":
            set_arm_positions(devs, self.start_q)
            print("[Drop] returning to start_q...")
            self.phase = "return_wait"
            return py_trees.common.Status.RUNNING

        if self.phase == "return_wait":
            ok, _ = arm_still(devs)
            if ok:
                print("[Drop] returned to start_q → done.")
                self.write_blackboard("drop_status", "success")
                self.phase = "done"
                return py_trees.common.Status.RUNNING
            return py_trees.common.Status.RUNNING

        if self.phase == "done":
            print("[Drop] done.")
            return py_trees.common.Status.SUCCESS

        print(f"[Drop][WARN] Unknown phase: {self.phase}")
        return py_trees.common.Status.FAILURE
