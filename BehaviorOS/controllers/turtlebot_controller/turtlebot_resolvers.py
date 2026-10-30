"""
TurtleBot3 Resolvers - Improved Box Detection
"""

import time

from behavioros.resolution import ResolutionContext
from behavioros.resolution.decorators import runtime_resolver, static_resolver


@runtime_resolver(
    "nearest_object", description="Practical object detection with simple heuristics"
)
def resolve_nearest_object(context: ResolutionContext):
    """실용적인 물체 감지 - 간단한 휴리스틱 사용"""

    lidar = context.devices.get("LDS-01")
    if not lidar:
        return {"detected": False, "distance": float("inf")}

    try:
        ranges = lidar.getRangeImage()
        if not ranges:
            return {"detected": False, "distance": float("inf")}

        print(f"[DEBUG] Total LiDAR points: {len(ranges)}")

        # 1단계: 유효한 거리 범위 필터링 (매우 가까운 것과 적당한 것만)
        object_candidates = []

        for i, r in enumerate(ranges):
            if 0.15 < r < 1.0:  # 15cm~1m만 (벽은 보통 1m 이상)
                angle = (i / len(ranges)) * 360
                if angle > 180:
                    angle -= 360

                object_candidates.append((i, r, angle))

        print(f"[DEBUG] Objects in range (0.15-1.0m): {len(object_candidates)}")

        if not object_candidates:
            print("[DEBUG] No objects in close range")
            return {"detected": False, "distance": float("inf")}

        # 2단계: 거리별 우선순위 (0.3-0.7m를 선호)
        preferred_candidates = [x for x in object_candidates if 0.3 <= x[1] <= 0.7]

        if preferred_candidates:
            print(
                f"[DEBUG] Preferred range objects (0.3-0.7m): {len(preferred_candidates)}"
            )
            # 선호 범위에서 가장 가까운 것
            idx, min_distance, min_angle = min(preferred_candidates, key=lambda x: x[1])
        else:
            print("[DEBUG] Using any close object")
            # 선호 범위에 없으면 아무거나
            idx, min_distance, min_angle = min(object_candidates, key=lambda x: x[1])

        # 3단계: 간단한 벽 체크 (너무 많은 연속점이 있으면 의심)
        def is_likely_wall(ranges, center_idx, center_dist):
            n = len(ranges)
            continuous_count = 0

            # 좌우 5개씩만 체크 (간단하게)
            for offset in range(-5, 6):
                idx = (center_idx + offset) % n
                if abs(ranges[idx] - center_dist) < 0.3:  # 30cm 이내면 비슷
                    continuous_count += 1

            # 11개 중 8개 이상이 비슷하면 벽일 가능성
            return continuous_count >= 8

        if is_likely_wall(ranges, idx, min_distance):
            print(f"[DEBUG] Object at {min_distance:.2f}m looks like wall, skipping")

            # 벽 같은 것 제외하고 다시 선택
            non_wall_candidates = []
            for i, r, angle in object_candidates:
                if not is_likely_wall(ranges, i, r):
                    non_wall_candidates.append((i, r, angle))

            if non_wall_candidates:
                idx, min_distance, min_angle = min(
                    non_wall_candidates, key=lambda x: x[1]
                )
                print(
                    f"[DEBUG] Selected non-wall object: {min_distance:.2f}m @ {min_angle:.0f}°"
                )
            else:
                print("[DEBUG] All objects look like walls")
                return {"detected": False, "distance": float("inf")}
        else:
            print(f"[DEBUG] Selected object: {min_distance:.2f}m @ {min_angle:.0f}°")

        return {
            "detected": True,
            "distance": min_distance,
            "angle": min_angle,
            "timestamp": time.time(),
        }

    except Exception as e:
        print(f"[DEBUG] Error: {e}")
        return {"detected": False, "distance": float("inf")}


@runtime_resolver("obstacle", description="LiDAR obstacle detection")
def resolve_obstacle(context: ResolutionContext):
    """실시간 장애물 감지"""
    lidar = context.devices.get("LDS-01")

    if not lidar:
        return {"detected": False, "distance": float("inf"), "error": "No LiDAR"}

    try:
        ranges = lidar.getRangeImage()
        if not ranges or len(ranges) == 0:
            return {"detected": False, "distance": float("inf")}

        # 유효한 범위만 (0.12m ~ 3.5m는 TurtleBot3 LDS-01 스펙)
        valid_ranges = [r for r in ranges if 0.12 < r < 3.5]
        if not valid_ranges:
            return {"detected": False, "distance": float("inf")}

        min_dist = min(valid_ranges)
        min_idx = ranges.index(min_dist)

        # 각도 계산 (-180 ~ 180도)
        angle_deg = (min_idx / len(ranges)) * 360
        if angle_deg > 180:
            angle_deg -= 360

        return {
            "detected": min_dist < 1.0,
            "distance": min_dist,
            "angle": angle_deg,
            "timestamp": time.time(),
        }
    except Exception as e:
        print(f"[Obstacle Error] {e}")
        return {"detected": False, "distance": float("inf")}


@runtime_resolver("wall_distance", description="Wall distances for following")
def resolve_wall_distance(context: ResolutionContext):
    """실시간 벽 거리 측정"""
    lidar = context.devices.get("LDS-01")

    if not lidar:
        return {"error": "No LiDAR"}

    try:
        ranges = lidar.getRangeImage()
        if not ranges:
            return {"error": "No data"}

        n = len(ranges)

        # 각 방향별 최소 거리
        def get_min_in_sector(start_angle, end_angle):
            start_idx = int((start_angle + 180) * n / 360) % n
            end_idx = int((end_angle + 180) * n / 360) % n

            if start_idx < end_idx:
                sector_ranges = ranges[start_idx:end_idx]
            else:
                sector_ranges = ranges[start_idx:] + ranges[:end_idx]

            valid = [r for r in sector_ranges if 0.12 < r < 3.5]
            return min(valid) if valid else float("inf")

        return {
            "front": get_min_in_sector(-20, 20),
            "right": get_min_in_sector(70, 110),
            "left": get_min_in_sector(-110, -70),
            "has_wall_right": get_min_in_sector(70, 110) < 1.0,
            "has_wall_left": get_min_in_sector(-110, -70) < 1.0,
            "timestamp": time.time(),
        }

    except Exception as e:
        print(f"[Wall Distance Error] {e}")
        return {"error": str(e)}


@runtime_resolver("safe_speed", description="Adaptive speed based on obstacles")
def resolve_safe_speed(context: ResolutionContext):
    """장애물 거리에 따른 안전 속도"""
    obstacle = resolve_obstacle(context)
    distance = obstacle.get("distance", float("inf"))

    # TurtleBot3 Burger 최대: 0.22 m/s
    if distance < 0.3:
        return 0.05  # 매우 느림
    elif distance < 0.5:
        return 0.10  # 느림
    elif distance < 1.0:
        return 0.15  # 중간
    else:
        return 0.20  # 빠름


# ========== STATIC RESOLVERS (고정값) ==========


@static_resolver("robot_model")
def resolve_robot_model(context: ResolutionContext):
    """로봇 모델명"""
    return "TurtleBot3 Burger"


@static_resolver("robot_specs")
def resolve_robot_specs(context: ResolutionContext):
    """로봇 사양"""
    return {
        "model": "TurtleBot3 Burger",
        "max_linear_speed": 0.22,  # m/s
        "max_angular_speed": 2.84,  # rad/s
        "wheel_base": 0.160,  # m
        "wheel_radius": 0.033,  # m
        "lidar_samples": 360,
    }


@static_resolver("default_speeds")
def resolve_default_speeds(context: ResolutionContext):
    """기본 속도 설정"""
    return {
        "slow": 0.05,
        "normal": 0.15,
        "fast": 0.20,
        "turn_speed": 1.57,  # 90도/초
    }
