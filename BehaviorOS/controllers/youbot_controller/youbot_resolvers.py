"""
YouBot Resolvers for Dynamic Values
- Colors: red, green (RGB 기준)
"""

import math

from behavioros.resolution import ResolutionContext, runtime_resolver, static_resolver

# youbot_actions.py (원래 방식으로 복원: init에서 home 캡처, home 스텝에서 self.home_q 사용)
# 유틸/IK: 반드시 "모델 프레임"으로 통일
from controllers.youbot_controller.youbot_utils import (
    read_world_pose_supervisor,  # noqa: F401
)

# ========== 튜닝(색상) ==========
# RED
T_R, T_G, T_B = 110, 100, 100
RED_DOM = 15
MIN_TOTAL_RED = 40

# GREEN (조금 관대)
T_Rg, T_Gg, T_Bg = 100, 105, 100
GREEN_DOM = 12
MIN_TOTAL_GREEN = 10


RB_DOM_OVER_G = 50  # (R-G), (B-G) 우위 ↑ (기존 45→50)


BRIGHT_MIN = 95  # 전체 밝기 하한 소폭 ↑
CHROMA_MIN = 45  # 채도 하한 소폭 ↑

# ========== 거리/좌표/깊이 ==========
CAMERA_OFFSET_X = 0.22
CAMERA_OFFSET_Y = 0.00
X_MIN, X_MAX = 0.05, 2.0
Y_MAX = 0.50
DEPTH_IS_RAY = True
FALLBACK_X = 0.30
FALLBACK_Z = 0.101

# 깊이 샘플링
PATCH = 4  # 반경 → (2*PATCH+1)²
LOW_TAIL_P = 0.4  # 하위 40% 평균(바닥·노이즈 완화)
CY_BIAS = 6  # 컴포넌트 중심보다 살짝 아래에서 깊이 샘플

# 로깅
DEBUG_VERBOSE = False  # True면 단계 로그 더 자세히


# -------------------- 공용 헬퍼 --------------------
def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _fallback(tag: str, debug=None):
    if DEBUG_VERBOSE:
        print(f"[Resolver] Fallback: {tag} debug={debug}")
    return {
        "detected": False,
        "x_robot": float(FALLBACK_X),
        "y_robot": 0.0,
        "z_world": float(FALLBACK_Z),
        "x_world": float(FALLBACK_X),
        "y_world": 0.0,
        "frame": f"fallback({tag})",
        "distance": abs(FALLBACK_X),
        "why": tag,
        "debug": debug or {},
    }


def _get_depth_at(rf, rng, w, x, y):
    """RangeFinder depth read with graceful fallback to flat array indexing."""
    try:
        return rf.rangeImageGetDepth(rng, w, int(x), int(y))
    except Exception:
        idx = int(y) * int(w) + int(x)
        return float(rng[idx])


# -------------------- 색 판정(RGB) --------------------
def _is_red_rgb(r, g, b):
    return (
        (r > T_R)
        and (g < T_G)
        and (b < T_B)
        and (r - g >= RED_DOM)
        and (r - b >= RED_DOM)
    )


def _is_green_rgb(r, g, b):
    return (
        (g > T_Gg)
        and (r < T_Rg)
        and (b < T_Bg)
        and (g - r >= GREEN_DOM)
        and (g - b >= GREEN_DOM)
    )


# -------------------- 스캔/컴포넌트/깊이 --------------------
def _scan_color_columns(cam, w, h, color="red"):
    """
    열별 픽셀 카운트. color in {'red','green'}
    """
    img = cam.getImage()
    counts = [0] * w
    sumy = [0] * w
    total = 0

    if color == "red":
        pred = _is_red_rgb
        min_total = MIN_TOTAL_RED
    elif color == "green":
        pred = _is_green_rgb
        min_total = MIN_TOTAL_GREEN
    else:
        raise ValueError("color must be 'red','green'")

    for y in range(h):
        for x in range(w):
            r = cam.imageGetRed(img, w, x, y)
            g = cam.imageGetGreen(img, w, x, y)
            b = cam.imageGetBlue(img, w, x, y)
            if pred(r, g, b):
                counts[x] += 1
                sumy[x] += y
                total += 1

    return counts, sumy, total, min_total


def _column_components(counts, sumy, h):
    """
    열별 카운트를 연결 구간으로 묶어 대표점(cx, cy) 생성
    """
    w = len(counts)
    per_col_thresh = max(4, int(0.012 * h))  # 열당 약 1.2% 이상일 때 유효
    comps = []
    run = None
    for x in range(w):
        ok = counts[x] >= per_col_thresh
        if ok and run is None:
            run = [x, x]
        elif ok and run is not None:
            run[1] = x
        elif (not ok) and run is not None:
            comps.append(tuple(run))
            run = None
    if run is not None:
        comps.append(tuple(run))

    out = []
    for xs, xe in comps:
        cx = (xs + xe) // 2
        pix = sum(counts[xs : xe + 1])
        sy = sum(sumy[xs : xe + 1])
        cy = (sy / pix) if pix > 0 else (h // 2)
        out.append(
            {
                "xs": xs,
                "xe": xe,
                "cx": float(cx),
                "cy": float(cy),
                "cols": xe - xs + 1,
                "pix": int(pix),
            }
        )
    return out


def _depth_patch_values_with_spiral_fallback(
    rf, rng, w, h, cx, cy, patch=PATCH, max_R=8
):
    """
    패치에서 유효 깊이가 없으면 주변 테두리를 확장하며 한 픽셀이라도 수집
    (rf 해상도 좌표계 기준)
    """
    vals = []
    try:
        d_min = rf.getMinRange()
        d_max = rf.getMaxRange()
    except Exception:
        d_min, d_max = 0.0, float("inf")

    # 1) 로컬 패치
    x0 = max(0, int(cx) - patch)
    x1 = min(w - 1, int(cx) + patch)
    y0 = max(0, int(cy) - patch)
    y1 = min(h - 1, int(cy) + patch)
    for yy in range(y0, y1 + 1):
        for xx in range(x0, x1 + 1):
            d = _get_depth_at(rf, rng, w, xx, yy)
            if math.isfinite(d) and (d_min < d < d_max):
                vals.append(d)
    if vals:
        return vals

    # 2) 주변 확장(사각 테두리)
    cx_i, cy_i = int(round(cx)), int(round(cy))
    for R in range(1, max_R + 1):
        found = False
        # 상/하 테두리
        for xx in range(max(0, cx_i - R), min(w - 1, cx_i + R) + 1):
            for yy in (max(0, cy_i - R), min(h - 1, cy_i + R)):
                d = _get_depth_at(rf, rng, w, xx, yy)
                if math.isfinite(d) and (d_min < d < d_max):
                    vals.append(d)
                    found = True
        # 좌/우 테두리
        for yy in range(max(0, cy_i - R + 1), min(h - 1, cy_i + R)):
            for xx in (max(0, cx_i - R), min(w - 1, cx_i + R)):
                d = _get_depth_at(rf, rng, w, xx, yy)
                if math.isfinite(d) and (d_min < d < d_max):
                    vals.append(d)
                    found = True
        if found:
            break
    return vals


def _mean_of_lowest(vals, p=LOW_TAIL_P):
    if not vals:
        return float("inf")
    k = max(1, int(len(vals) * p))
    vv = sorted(vals)[:k]
    return sum(vv) / float(len(vv))


RED_PULLIN_R = 0.04  # 4 cm (0.02~0.05에서 튜닝)
GREEN_PULLIN_R = 0.00


# -------------------- 메인 리졸버 --------------------
def _resolve_nearest_color_can_impl(context, color: str):
    """
    color ∈ {'red','green'}
    """
    try:
        devs = context.devices if isinstance(context.devices, dict) else {}
        if not devs and isinstance(context.blackboard, dict):
            devs = context.blackboard.get("devices", {})

        cam = devs.get("camera")
        rf = devs.get("depth")

        if cam is None:
            return _fallback(f"no_camera_{color}")
        if cam.getImage() is None:
            return _fallback(f"no_cam_image_{color}")

        w, h = cam.getWidth(), cam.getHeight()
        fov = cam.getFov()
        if DEBUG_VERBOSE:
            print(
                f"[Resolver/{color}] cam {w}x{h} fov={fov:.3f} depth={'on' if rf else 'off'}"
            )

        # 1) 컬럼 스캔
        counts, sumy, total_pix, min_total = _scan_color_columns(cam, w, h, color=color)
        if total_pix < min_total:
            return _fallback(f"too_few_{color}", {f"total_{color}": total_pix})

        comps = _column_components(counts, sumy, h)
        if not comps:
            return _fallback(f"no_{color}_columns", {f"total_{color}": total_pix})

        # 2) 대표점 + 깊이/각도
        if rf is None:
            # 깊이 없으면 화면 중앙과 가장 가까운 덩어리
            best = min(comps, key=lambda c: abs(c["cx"] - w / 2))
            cx, cy = best["cx"], best["cy"]
            dx_norm = (cx - (w / 2.0)) / (w / 2.0)
            yaw = dx_norm * (fov / 2.0)
            x_fwd = FALLBACK_X
            y_lat = math.tan(yaw) * FALLBACK_X
            picked = {"cx": cx, "cy": cy, "yaw": yaw}
            d_used = None
            nvals = 0
        else:
            rng = rf.getRangeImage()
            if rng is None:
                return _fallback(f"no_depth_image_{color}", {"has_depth_device": True})

            try:
                rf_w = rf.getWidth()
                rf_h = rf.getHeight()
            except Exception:
                rf_w, rf_h = w, h

            scored = []
            for c in comps:
                # 카메라 → RF 해상도 변환 + 약간 아래쪽(CY_BIAS)
                cx_cam = float(c["cx"])
                cy_cam = min(h - 1.0, max(0.0, float(c["cy"]) + CY_BIAS))
                cx_rf = cx_cam * (rf_w / float(w))
                cy_rf = cy_cam * (rf_h / float(h))
                vals = _depth_patch_values_with_spiral_fallback(
                    rf, rng, rf_w, rf_h, cx_rf, cy_rf
                )
                d = _mean_of_lowest(vals, LOW_TAIL_P)
                scored.append((d, c, len(vals)))

            scored = [t for t in scored if math.isfinite(t[0])]
            if not scored:
                return _fallback(f"no_valid_depth_{color}", {"comps": len(comps)})

            scored.sort(key=lambda t: t[0])
            d_used, best, nvals = scored[0]
            cx, cy = best["cx"], best["cy"]
            dx_norm = (cx - (w / 2.0)) / (w / 2.0)
            yaw = dx_norm * (fov / 2.0)

            if DEPTH_IS_RAY:
                x_fwd = d_used * math.cos(yaw)
                y_lat = d_used * math.sin(yaw)
            else:
                x_fwd = d_used
                y_lat = d_used * math.tan(yaw)

            picked = {"cx": cx, "cy": cy, "yaw": yaw, "d": d_used, "nvals": nvals}

        # 3) 카메라→로봇 변환 + 클램프
        if color == "red":
            # 레이 방향으로 r -= PULLIN_R (x,y를 함께 보정)
            x_fwd = x_fwd - RED_PULLIN_R * math.cos(yaw)
            y_lat = y_lat - RED_PULLIN_R * math.sin(yaw)
            # 안전 클램프
            x_fwd = max(X_MIN, x_fwd)

        x_robot = _clamp(x_fwd + CAMERA_OFFSET_X, X_MIN, X_MAX)
        y_robot = _clamp(y_lat + CAMERA_OFFSET_Y, -Y_MAX, +Y_MAX)

        out = {
            "detected": True,
            "x_robot": float(x_robot),
            "y_robot": float(y_robot),
            "z_world": float(FALLBACK_Z),
            "x_world": float(x_robot),
            "y_world": float(y_robot),
            "frame": f"vision/{color}(col_low{int(LOW_TAIL_P * 100)}%)",
            "distance": math.hypot(x_robot, y_robot),
            "why": "ok",
            "debug": {
                "w": w,
                "h": h,
                "fov": float(fov),
                f"total_{color}": int(total_pix),
                "picked": picked,
                "depth_used": rf is not None,
            },
        }

        # 로그(한 줄 요약)
        yaw_deg = math.degrees(yaw)
        if d_used is None:
            print(
                f"[RESOLVE/{color}] comps={len(comps)} pick=({cx:.1f},{cy:.1f}) "
                f"yaw={yaw_deg:+.1f}° → (x,y)=({x_robot:+.3f},{y_robot:+.3f})"
            )
        else:
            print(
                f"[RESOLVE/{color}] comps={len(comps)} pick=({cx:.1f},{cy:.1f}) "
                f"yaw={yaw_deg:+.1f}° d={d_used:.3f} n={nvals} → "
                f"(x,y)=({x_robot:+.3f},{y_robot:+.3f})"
            )

        return out

    except Exception as e:
        return _fallback(f"exception_{color}:{type(e).__name__}", {"msg": str(e)})


# -------------------- 래퍼(키 이름 유지) --------------------
@runtime_resolver("nearest_red_can")
def resolve_nearest_red_can(context: ResolutionContext):
    return _resolve_nearest_color_can_impl(context, "red")


@runtime_resolver("nearest_green_can")
def resolve_nearest_green_can(context: ResolutionContext):
    return _resolve_nearest_color_can_impl(context, "green")


# -------------------- 나머지(유지) --------------------
@runtime_resolver("all_red_cans")
def resolve_all_red_cans(context: ResolutionContext):
    """모든 빨간 캔 위치들"""
    robot_x = context.blackboard.get("robot_x", 0)
    robot_y = context.blackboard.get("robot_y", 0)

    red_cans = [{"x": 0.8, "y": 0.0, "z": 0.061}, {"x": 1.0, "y": -0.1, "z": 0.061}]
    for can in red_cans:
        can["distance"] = math.hypot(can["x"] - robot_x, can["y"] - robot_y)
        can["angle"] = math.degrees(math.atan2(can["y"] - robot_y, can["x"] - robot_x))
    red_cans.sort(key=lambda c: c["distance"])
    return red_cans


@runtime_resolver("gripper_status")
def resolve_gripper_status(context: ResolutionContext):
    """그리퍼 상태 확인"""
    holding = context.blackboard.get("holding_can", False)
    return {
        "is_open": not holding,
        "is_holding": holding,
        "held_object": "can" if holding else None,
    }


@static_resolver("pickup_height")
def resolve_pickup_height(context: ResolutionContext):
    return 0.061


@static_resolver("transport_speed")
def resolve_transport_speed(context: ResolutionContext):
    return 0.3


@static_resolver("approach_distance")
def resolve_approach_distance(context: ResolutionContext):
    return 0.25


# --- 타겟 (world frame) ---
TGT_POS_WORLD = (0.5298987744395782, 0.5398222100438806, 0.009607804517333198)

# --- Z: 박스 테두리 + 호버 ---
BOX_RIM_Z_OFFSET = 0.08  # 중심 z에서 테두리 윗면까지
HOVER_ABOVE_RIM = 0.02  # 테두리 위로 2 cm

# --- 딱 45°만 틀기 ---
YAW_DIR = 1  # +1이면 +45°, -1이면 -45°
EXTRA_YAW = YAW_DIR * (math.pi / 4.0)  # ← 고정 45°

DEBUG_VERBOSE = True

YAW_DIR = +1  # +1 => +45°, -1 => -45°
YAW_EXTRA = YAW_DIR * (math.pi / 4) + math.radians(5)  # 필요할 때만 사용


def _rot2d(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


@static_resolver("drop_goal")
def resolve_drop_goal(ctx: ResolutionContext):
    devs = ctx.devices if isinstance(ctx.devices, dict) else {}
    if not devs and isinstance(ctx.blackboard, dict):
        devs = ctx.blackboard.get("devices", {})

    sup_pose = read_world_pose_supervisor(devs)  # -> (xw_r, yw_r, M) or None
    xw_t, yw_t, zw_t = TGT_POS_WORLD

    # 1) 월드→로봇: Δr = R_wr^T · Δw  (row-major 행 기준)
    if sup_pose is not None:
        xw_r, yw_r, M = sup_pose
        dx, dy, dz = (xw_t - xw_r), (yw_t - yw_r), 0.0
        m00, m01, m02, m10, m11, m12, m20, m21, m22 = [float(v) for v in M]
        xr0 = m00 * dx + m01 * dy + m02 * dz
        yr0 = m10 * dx + m11 * dy + m12 * dz
        src = "supervisor_Rt_rowmajor"
        dbg_R = [m00, m01, m02, m10, m11, m12, m20, m21, m22]
    else:
        xw_r = float(ctx.blackboard.get("robot_x", 0.0))
        yw_r = float(ctx.blackboard.get("robot_y", 0.0))
        yaw = float(ctx.blackboard.get("robot_yaw", 0.0))
        dx, dy = (xw_t - xw_r), (yw_t - yw_r)
        c, s = math.cos(yaw), math.sin(yaw)
        xr0 = c * dx + s * dy
        yr0 = -s * dx + c * dy
        src = "bb_rotT_delta"
        dbg_R = None

    # 2) XY를 '정확히' ±45°만 회전
    xr, yr = _rot2d(xr0, yr0, EXTRA_YAW)

    # 3) Z: 테두리 위로 호버
    z_world = float(zw_t + BOX_RIM_Z_OFFSET + HOVER_ABOVE_RIM)

    out = {
        "detected": True,
        "x_robot": float(xr),
        "y_robot": float(yr),
        "z_world": float(z_world),
        "x_world": float(xw_t),
        "y_world": float(yw_t),
        "frame": "robot",  # xy=robot frame, z=world
        "distance": math.hypot(xr, yr),
        "why": "ok_rotate_xy_45deg_only",
        "yaw_extra_rad": float(YAW_EXTRA),
        "debug": {
            "source": src,
            "robot_world_xy": (float(xw_r), float(yw_r)),
            "delta_world_xy": (float(dx), float(dy)),
            "pre_rot_xy": (float(xr0), float(yr0)),
            "post_rot_xy": (float(xr), float(yr)),
            "EXTRA_YAW_deg": math.degrees(EXTRA_YAW),
            "YAW_DIR": YAW_DIR,
            "BOX_RIM_Z_OFFSET": BOX_RIM_Z_OFFSET,
            "HOVER_ABOVE_RIM": HOVER_ABOVE_RIM,
            "R_rowmajor": dbg_R,
            "target_world": TGT_POS_WORLD,
        },
    }

    if DEBUG_VERBOSE:
        print(
            f"[RESOLVE/DROP 45°] Δw=({dx:+.3f},{dy:+.3f}) "
            f"→ RtΔ=(x0,y0)=({xr0:+.3f},{yr0:+.3f}) "
            f"→ rot(±45° dir={YAW_DIR:+d})=(x,y)=({xr:+.3f},{yr:+.3f}) "
            f"| z_w={z_world:+.3f}"
        )

    return out
