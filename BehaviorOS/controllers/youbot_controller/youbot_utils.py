# controllers/youbot_controller/youbot_utils.py
# ------------------------------------------------------------
# 모터 프레임 vs 모델 프레임, 변환 함수, 리미트, IK, 레디, 공용 헬퍼 (개선판)
#  - MIN_TIP_X: 기본 소프트 페널티, 필요시 하드 클램프 토글
#  - yaw(arm1) 스루 제한으로 큰 회전 튐 방지
#  - 연속성/손목/팁 페널티 밸런스 조정
#  - 디버그 출력 강화
#  - 정확 FK 기반 터치/리치 판정 (XY/Z 분리 임계 지원)
# ------------------------------------------------------------

import math


# -------------------- 유틸 --------------------
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def _wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _nearest_equivalent_angle(target, reference):
    """reference 근처의 동치각(target + k*2π)을 선택."""
    twopi = 2.0 * math.pi
    k = round((reference - target) / twopi)
    return target + k * twopi


# ============================================================
# 1) youBot 기구 파라미터(모델)
# ============================================================
YAW_BASE_FWD = 0.156
YAW_BASE_ZW = 0.120 + 0.077  # = 0.197

SHOULDER_DX = 0.033  # 현재 IK에서는 z만 사용
SHOULDER_DZ = 0.070

L1 = 0.155
L2 = 0.135
L3 = 0.081 + 0.090  # ≈ 0.171

JOINT_LIMITS = {
    "arm1": (-2.9496, 2.9496),
    "arm2": (-1.13446, 1.5708),
    "arm3": (-2.63545, 2.54818),
    "arm4": (-1.78024, 1.78024),
    "arm5": (-2.92343, 2.92343),
}

# ============================================================
# 2) 모델↔모터 변환(장치 보정)
# ============================================================
JOINT_SIGN = {
    "arm1": 1.0,
    "arm2": -1.0,  # youBot에선 arm2 반전일 때가 많음
    "arm3": +1.0,
    "arm4": +1.0,
    "arm5": +1.0,
}
JOINT_OFFSET = {
    "arm1": 0.0,
    "arm2": -0.2,  # -0.5,
    "arm3": -0.5,  # -0.5,
    "arm4": 0.0,
    "arm5": 0.0,
}

IK_BIAS_DEG = [
    0.0,  # arm1 (yaw)
    0.0,
    5.0 * math.pi / 180,  # arm3: -3°
    -20.0 * math.pi / 180,  # arm4: +1°
    0.0,  # arm5 (roll)
]


def deg2rad(d):
    return d * math.pi / 180.0


def apply_ik_bias_model(q_model):
    # q_model: 길이 5 리스트(또는 튜플)
    return [q + b for q, b in zip(q_model, IK_BIAS_DEG)]


def to_actuator(q_model):
    names = ["arm1", "arm2", "arm3", "arm4", "arm5"]
    return [JOINT_SIGN[n] * v + JOINT_OFFSET[n] for n, v in zip(names, q_model)]


def from_actuator(q_act):
    names = ["arm1", "arm2", "arm3", "arm4", "arm5"]
    return [(v - JOINT_OFFSET[n]) / JOINT_SIGN[n] for n, v in zip(names, q_act)]


def transformed_limits():
    out = {}
    for name, (lo, hi) in JOINT_LIMITS.items():
        a_lo = JOINT_SIGN[name] * lo + JOINT_OFFSET[name]
        a_hi = JOINT_SIGN[name] * hi + JOINT_OFFSET[name]
        out[name] = (min(a_lo, a_hi), max(a_lo, a_hi))
    return out


JOINT_LIMITS_ACT = transformed_limits()


# ============================================================
# 3) 튜너블 파라미터(중요)
# ============================================================
# (A) yaw 축 보정
Q1_AUTO_CALIB = False
BASE_Y_SIGN = -1.0  # 로봇 프레임 +y가 '왼쪽'이면 +1.0, 반대면 -1.0 (<< 유지)
# YAW_TRIM = 0  # math.radians(-30)  # 유지
# --- yaw 보정(우/좌 비대칭) ---
YAW_BIAS_L = -6 * math.pi / 180  # math.pi / 360  # 좌회전(+yaw) 쪽 고정 바이어스(예시)
YAW_GAIN_L = 1.00  # 좌 스케일

YAW_BIAS_R = (
    -11.0 * math.pi / 180
)  # 우회전(-yaw) 쪽 바이어스(오른쪽이 부족하니 절댓값 더 크게)
YAW_GAIN_R = 1.08  # 우 스케일(조금 과보정해서 부족함 상쇄)

LR_REACH_PULL_L = 0.00  # left일 때 x_plane에 더할 값(음수면 덜 뻗음)
LR_REACH_PULL_R = -0.04  # right일 때는 그대로


def apply_yaw_trim(yaw):
    """카메라 기준 yaw(라디안)를 q1에 반영하기 전 비대칭 보정"""
    if yaw >= 0.0:  # 좌회전(화면 기준 왼쪽 타깃)
        return YAW_BIAS_L + YAW_GAIN_L * yaw
    else:  # 우회전(화면 기준 오른쪽 타깃)
        return YAW_BIAS_R + YAW_GAIN_R * yaw


Q1_OFFSET_FIXED = math.radians(23)  # 유지

# yaw 스루 제한(틱당 최대 회전량) — 큰 점프 방지
Q1_SLEW_MAX_DEG = 45.0
Q1_SLEW_MAX = math.radians(Q1_SLEW_MAX_DEG)

# (B) 자세/연속성/팁 페널티 (유지)
WRIST_BIAS = -0.4
WRIST_MARGIN = 0.02
WRIST_COMFORT = 0.8

USE_TIP_HARD_CLAMP = False
MIN_TIP_X = L3 + 0.18
TIP_SOFT_WEIGHT = 120.0

FWD_CONE_DEG = 60.0
FWD_SOFT_PENALTY = 6.0
FWD_HARD_PENALTY = 200.0
CONT_W = 1.2

# (C) ★ Z 편의 오프셋 — 여기만 바꿔서 높이 맞춤
Z_TARGET_BIAS = 0  # yaw 기준 좌표의 z에 더함 (복구)
Z_PLANE_BIAS = 0  # 평면 IK 입력 z_plane에 더함 (복구)
Z_WORLD_OFFSET = 0  # 입력 z_world 전역 보정 (유지)


# ============================================================
# 4) 페널티 함수 (유지)
# ============================================================
def _yaw_cone_penalty(q1):
    a = abs(_wrap_pi(q1))
    cone = math.radians(FWD_CONE_DEG)
    return FWD_SOFT_PENALTY * a + FWD_HARD_PENALTY * max(0.0, a - cone)


def _tip_soft_penalty(x_plane):
    if x_plane >= MIN_TIP_X:
        return 0.0
    return TIP_SOFT_WEIGHT * (MIN_TIP_X - x_plane)


def _wrist_penalty(q4):
    d = abs(q4 - WRIST_BIAS)
    return 2.0 * d + 8.0 * max(0.0, d - WRIST_COMFORT)


def _continuity_penalty(q_candidate, q_current):
    if not q_current:
        return 0.0
    total = 0.0
    for qc, qm in zip(q_candidate[:4], q_current[:4]):  # arm1..4
        if qm is None:
            continue
        total += abs(qc - qm)
    return CONT_W * total


# ============================================================
# 5) 평면 3링크 IK(수평 접근 선호) — (유지)
# ============================================================
def _planar_3link_ik(x, z, elbow="down"):
    # TCP가 수평이라는 가정 → x에서 L3를 미리 빼 손목 위치 사용
    xw = x - L3
    zw = z
    r = math.hypot(xw, zw)
    r = clamp(r, abs(L1 - L2) + 1e-6, L1 + L2 - 1e-6)

    cos_q3 = clamp((r * r - L1 * L1 - L2 * L2) / (2.0 * L1 * L2), -1.0, 1.0)
    q3 = math.acos(cos_q3)
    if elbow == "down":
        q3 = -q3

    phi = math.atan2(zw, xw)
    k = math.atan2(L2 * math.sin(q3), L1 + L2 * math.cos(q3))
    q2 = phi - k

    # 손목은 bias 근처 선호
    lo4, hi4 = JOINT_LIMITS["arm4"]
    q4_des = (q2 + q3) - WRIST_BIAS
    q4 = clamp(q4_des, lo4 + WRIST_MARGIN, hi4 - WRIST_MARGIN)
    return q2, q3, q4


# ============================================================
# 6) 메인 IK: 로봇 프레임 목표 → q_model
#     - Z 보정 3단계 반영 (전역, yaw좌표, 평면 입력)
# ============================================================
def solve_youbot_ik_from_robot_frame(target_robot, current_q=None, debug=False):
    """
    target_robot: {"x_robot": xr, "y_robot": yr, "z_world": zw}
      - xr, yr: 로봇 베이스 평면 (정면 +x, 좌 +y) — 필요시 BASE_Y_SIGN로 좌우 반전
      - zw    : 월드 z(바닥 기준)
    current_q: 현재 모델 프레임 관절각(연속성/언랩 및 yaw 스루 제한에 사용)
    """
    xr = float(target_robot["x_robot"])
    yr = float(target_robot["y_robot"]) * BASE_Y_SIGN

    # (1) 전역 Z 보정
    zw_in = float(target_robot["z_world"])
    zw = zw_in + Z_WORLD_OFFSET  # 전역 오프셋

    # arm1(yaw) 기준 상대좌표(베이스 오프셋 보정)
    x_rel_yaw = xr - YAW_BASE_FWD
    y_rel_yaw = yr
    # (2) yaw 기준 좌표에 타깃 바이어스
    z_rel_yaw = (zw - YAW_BASE_ZW) + Z_TARGET_BIAS

    # q1 원시값(+ 전진축 기준 방위) + 트림/오프셋
    q1_raw = math.atan2(y_rel_yaw, x_rel_yaw)
    q1_raw = math.atan2(y_rel_yaw, x_rel_yaw)  # (그대로)
    yaw_corr = apply_yaw_trim(q1_raw)  # << 추가
    q1_des = _wrap_pi(yaw_corr + Q1_OFFSET_FIXED)  # << 변경

    # 현재각 주변으로 스루 제한(틱 당 최대 회전량)
    if current_q is not None and current_q[0] is not None:
        q1_ref = current_q[0]
        q1_des = _nearest_equivalent_angle(q1_des, q1_ref)
        dq = _wrap_pi(q1_des - q1_ref)
        dq = clamp(dq, -Q1_SLEW_MAX, Q1_SLEW_MAX)
        q1 = _wrap_pi(q1_ref + dq)
    else:
        q1 = q1_des

    # yaw 정렬 평면으로 투영 (q1만큼 역회전)
    c1, s1 = math.cos(-q1), math.sin(-q1)
    x_after_yaw = x_rel_yaw * c1 - y_rel_yaw * s1
    z_after_yaw = z_rel_yaw

    # 어깨 z 오프셋 보정
    # x_plane = x_after_yaw
    x_plane = x_after_yaw + (LR_REACH_PULL_L if q1 >= 0.0 else LR_REACH_PULL_R)
    # (3) 평면 IK 입력 z_plane에 바이어스
    z_plane = (z_after_yaw - SHOULDER_DZ) + Z_PLANE_BIAS

    # 팁 전방거리 하드 클램프(옵션)
    if USE_TIP_HARD_CLAMP and x_plane < MIN_TIP_X:
        x_plane = MIN_TIP_X

    # 뒤로 말림 방지(정면 margin)
    FRONT_MARGIN = 0.02
    rear_penalty = 0.0
    if x_rel_yaw < FRONT_MARGIN:
        rear_penalty = 100.0 * (FRONT_MARGIN - x_rel_yaw)

    best = None
    for elbow in ("down", "up"):
        q2, q3, q4 = _planar_3link_ik(x_plane, z_plane, elbow=elbow)
        q_model = [q1, q2, q3, q4, 0.0]

        # 모터 리미트 위반량(모터 프레임 변환 후 계산)
        q_act = to_actuator(q_model)
        over = 0.0
        for name, ang in zip(["arm1", "arm2", "arm3", "arm4", "arm5"], q_act):
            lo, hi = JOINT_LIMITS_ACT[name]
            if ang < lo:
                over += lo - ang
            elif ang > hi:
                over += ang - hi

        # 스코어 (유지)
        score = 0.0
        if q2 < 0:
            score += 0.8
        if q3 < 0:
            score += 0.8
        score += _wrist_penalty(q4)
        score += _yaw_cone_penalty(q1)
        score += _tip_soft_penalty(x_plane)
        score += _continuity_penalty(q_model, current_q)
        score += 10.0 * over
        score += rear_penalty

        if (best is None) or (score < best[0]):
            best = (score, q_model)

    q_best_model = best[1]

    q_best_model = apply_ik_bias_model(q_best_model)

    # DELTA_RAD = math.radians(2)  # 손맛으로 조절(1~3° 권장). ↓로 가려면 +로 두세요.
    # q_best_model[1] += DELTA_RAD  # arm2 (shoulder)
    # q_best_model[2] -= DELTA_RAD  # arm3 (elbow)  ← arm2와 반대로

    if debug:

        def deg(a):
            return math.degrees(a)

        print(
            f"[IK] target(xr,yr,zw)=({xr:+.3f},{yr:+.3f},{zw_in:+.3f})  BASE_Y_SIGN={BASE_Y_SIGN:+.1f}"
        )
        print(
            f"[IK] q1_raw(deg)={deg(q1_raw):+.1f}, fixed_off(deg)={deg(Q1_OFFSET_FIXED):+.1f} "
            f"→ q1_des(deg)={deg(q1_des):+.1f} → q1_used(deg)={deg(q_best_model[0]):+.1f}"
        )
        # 디버그: Z 일관성 체크
        z_world_est = YAW_BASE_ZW + (
            (z_plane - Z_PLANE_BIAS) + SHOULDER_DZ
        )  # 평면 입력 역보정해서 추정
        dz = z_world_est - zw_in
        print(
            f"[IK][Z-CHK] zw_in={zw_in:+.3f}, used zw={zw:+.3f} (WORLD_OFF {Z_WORLD_OFFSET:+.3f}, "
            f"T_BIAS {Z_TARGET_BIAS:+.3f}, P_BIAS {Z_PLANE_BIAS:+.3f}) → "
            f"est TCP z_world={z_world_est:+.3f}, Δz(est-target)={dz:+.3f}"
        )

    return q_best_model


# ============================================================
# 7) 장치 쓰기/읽기 헬퍼(단일 경로) — (유지)
# ============================================================
def set_arm_positions(devices, q_model):
    names = ["arm1", "arm2", "arm3", "arm4", "arm5"]
    applied = []
    for name, ang_model in zip(names, q_model):
        m = devices.get(name)
        if not m:
            applied.append(None)
            continue
        q_act = JOINT_SIGN[name] * ang_model + JOINT_OFFSET[name]
        lo, hi = JOINT_LIMITS_ACT[name]
        clamped = clamp(q_act, lo, hi)
        if abs(clamped - q_act) > 1e-6:
            print(
                f"[set] {name}: MODEL={ang_model:+.3f} → ACT={q_act:+.3f}  clamped→ {clamped:+.3f} (lim {lo:+.3f}..{hi:+.3f})"
            )
        m.setPosition(clamped)
        applied.append(clamped)
    return applied


# ============================================================
# 8) READY(모델 프레임) & 이동 — (유지)
# ============================================================
READY_MODEL = [0.1, 0.63, 1.60, 0.75, 0.0]


def move_to_ready(devs, robot, timestep, tol=0.02, hold_steps=10):
    for name in ["arm1", "arm2", "arm3", "arm4", "arm5"]:
        m = devs.get(name)
        if m:
            m.setVelocity(0.6)

    set_arm_positions(devs, READY_MODEL)

    sensors = [
        devs.get(n + "_sensor") for n in ["arm1", "arm2", "arm3", "arm4", "arm5"]
    ]
    stable = 0
    while robot.step(timestep) != -1:
        q_act = [s.getValue() if s else 0.0 for s in sensors]
        q_cur = from_actuator(q_act)  # 모델 프레임으로 환산
        err = [abs(a - b) for a, b in zip(q_cur, READY_MODEL)]
        ok = all(e < tol for e in err)
        stable = stable + 1 if ok else 0
        if stable >= hold_steps:
            print("[ready] reached. model_err=", ["%.3f" % e for e in err])
            break


# -------------------- 로컬 헬퍼 --------------------
def read_joint_positions_model(devs):
    """PositionSensor → actuator → model 변환."""
    q_act = []
    for name in [
        "arm1_sensor",
        "arm2_sensor",
        "arm3_sensor",
        "arm4_sensor",
        "arm5_sensor",
    ]:
        s = devs.get(name)
        q_act.append(0.0 if s is None else s.getValue())
    return from_actuator(q_act)


def fk_tip_robot(q_model):
    """
    수평 접근 가정 버전:
    L3는 항상 x로 뻗는다고 가정(= IK 가정과 일치).
    """
    q1, q2, q3, q4, _ = q_model  # q4는 방향에 반영하지 않음(그립 회전 전용으로 간주)

    # 평면 x,z (q2,q3만 사용) + L3는 x로 더하기
    x_plane = L1 * math.cos(q2) + L2 * math.cos(q2 + q3) + L3
    z_plane = SHOULDER_DZ + L1 * math.sin(q2) + L2 * math.sin(q2 + q3)

    # yaw 회전 + 베이스 오프셋 복구
    c1, s1 = math.cos(q1), math.sin(q1)
    xr = YAW_BASE_FWD + x_plane * c1
    yr = 0.0 + x_plane * s1
    zw = YAW_BASE_ZW + z_plane
    return xr, yr, zw


TOUCH_RADIUS = 0.02  # 2 cm (기본 3D 반경)


# ---- 움직임 감지용 상태 버퍼 (간단 캐시) ----
_ARM_LAST_Q = None


def read_arm_positions_model(devs):
    """
    관절 포지션센서로 현재 관절각(model frame) 읽기.
    (이미 비슷한게 있다면 그거 써도 됨)
    """
    qs = []
    for i in range(1, 6):
        s = devs.get(f"arm{i}_sensor")
        if s and hasattr(s, "getValue"):
            qs.append(float(s.getValue()))
        else:
            qs.append(0.0)
    return qs


def arm_still(devs, eps=0.003):
    """
    직전 틱 대비 관절각 최대 변화량으로 '정지' 여부 판단.
    eps: 라디안 허용치 (~0.17°). 노이즈에 맞춰 0.002~0.008에서 튜닝.
    반환: (is_still: bool, max_delta: float)
    """
    global _ARM_LAST_Q
    q_now = read_arm_positions_model(devs)
    if _ARM_LAST_Q is None or len(_ARM_LAST_Q) != len(q_now):
        _ARM_LAST_Q = list(q_now)
        return False, float("inf")

    max_delta = max(abs(a - b) for a, b in zip(q_now, _ARM_LAST_Q))
    _ARM_LAST_Q = list(q_now)
    return (max_delta < eps), max_delta


# ==== names & safe limits ====
GRIPPER_LEFT = "finger::left"
GRIPPER_RIGHT = "finger::right"
GRIP_OPEN_SAFE = 0.0240  # 0.025(하드리미트)보다 약간 안쪽
GRIP_CLOSE_SAFE = 0.0008
GRIP_EPS = 1e-4


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def config_gripper(devs, max_vel=0.03, avail_force=50.0):
    """LinearMotor을 position 모드 + 속도/가용힘 세팅"""
    for key in (GRIPPER_LEFT, GRIPPER_RIGHT):
        m = devs.get(key)
        if not m:
            print(f"[Grip][ERR] device missing: {key}")
            continue
        # ① 포지션 모드 확실히: inf 모드였다면 정상값으로 돌려놓기
        try:
            # getTargetPosition이 inf인 경우가 있음 → 현재 실제 위치로 모드 복귀
            cur_pos = m.getPosition() if hasattr(m, "getPosition") else None
            if hasattr(m, "setPosition"):
                m.setPosition(float(cur_pos) if cur_pos is not None else GRIP_OPEN_SAFE)
        except Exception as e:
            print(f"[Grip][WARN] setPosition(mode restore) {key}: {e}")
        # ② 속도/힘
        try:
            if hasattr(m, "setVelocity"):
                m.setVelocity(abs(float(max_vel)))
            if hasattr(m, "setAvailableForce"):
                m.setAvailableForce(abs(float(avail_force)))
        except Exception as e:
            print(f"[Grip][WARN] vel/force {key}: {e}")


def set_gripper_positions(devs, left, right):
    """목표를 안전 범위로 클램프해서 설정"""
    for key, tgt in ((GRIPPER_LEFT, left), (GRIPPER_RIGHT, right)):
        m = devs.get(key)
        if not m:
            print(f"[Grip][ERR] device missing: {key}")
            continue
        # 모터 리미트 조회 후 안전 클램프
        lo = 0.0
        hi = 0.025
        try:
            if hasattr(m, "getMinPosition"):
                lo = float(m.getMinPosition())
            if hasattr(m, "getMaxPosition"):
                hi = float(m.getMaxPosition())
        except Exception:
            pass
        safe_tgt = _clamp(
            float(tgt),
            max(lo + GRIP_EPS, GRIP_CLOSE_SAFE),
            min(hi - GRIP_EPS, GRIP_OPEN_SAFE),
        )
        try:
            m.setPosition(safe_tgt)
        except Exception as e:
            print(f"[Grip][ERR] setPosition {key}: {e}")


def set_gripper_open(devs):
    set_gripper_positions(devs, GRIP_OPEN_SAFE, GRIP_OPEN_SAFE)


def set_gripper_close(devs):
    set_gripper_positions(devs, GRIP_CLOSE_SAFE, GRIP_CLOSE_SAFE)


def gripper_diag(devs, tag=""):
    for key in (GRIPPER_LEFT, GRIPPER_RIGHT):
        m = devs.get(key)
        if not m:
            print(f"[Grip][{tag}] {key}: <missing>")
            continue
        try:
            pos = m.getPosition() if hasattr(m, "getPosition") else None
            tpos = m.getTargetPosition() if hasattr(m, "getTargetPosition") else None
            v = m.getVelocity() if hasattr(m, "getVelocity") else None
            lo = m.getMinPosition() if hasattr(m, "getMinPosition") else None
            hi = m.getMaxPosition() if hasattr(m, "getMaxPosition") else None
            print(
                f"[Grip][{tag}] {key}: pos={pos:.4f}  target={tpos}  vel={v}  lim=[{lo},{hi}]"
            )
        except Exception as e:
            print(f"[Grip][{tag}] {key}: diag err {e}")


# =========================================================
# Webots Supervisor pose (XY 평면 기준)
# =========================================================
def read_world_pose_supervisor(devs):
    """(xw, yw, M) 또는 None. M: row-major 3x3(flat, len=9)"""
    sup = devs.get("supervisor")
    if not sup or not hasattr(sup, "getSelf"):
        return None
    try:
        node = sup.getSelf()
        if node is None:
            return None
        pos = node.getPosition()  # (x, y, z)
        xw, yw = float(pos[0]), float(pos[1])  # XY만 사용
        M = node.getOrientation()  # row-major
        return xw, yw, M
    except Exception:
        return None
