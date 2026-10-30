"""
TurtleBot3 Actions - Realistic Behaviors
Practical actions that work well with 2D LiDAR limitations
"""

import math
import random
import time

import py_trees

from behavioros.core import BaseAction, action

# ========== CORE MOVEMENT ACTIONS ==========


@action(
    "move_forward",
    "Move forward with obstacle avoidance",
    reads=["devices", "move_speed", "move_duration", "obstacle", "safe_speed"],
    writes=["movement_status", "distance_traveled"],
)
class MoveForwardAction(BaseAction):
    """적응적 전진 - 장애물에 따라 속도 조절"""

    def __init__(self, name: str, blackboard=None):
        super().__init__(name, blackboard)
        self.start_time = None

    def execute(self) -> py_trees.common.Status:
        devices = self.read_blackboard("devices", {})
        left_motor = devices.get("left wheel motor")
        right_motor = devices.get("right wheel motor")

        if not left_motor or not right_motor:
            self._reset_state()
            return py_trees.common.Status.FAILURE

        # 기본 속도 vs 적응적 속도
        base_speed = self.read_blackboard("move_speed", 0.15)
        safe_speed = self.read_blackboard("safe_speed", base_speed)
        actual_speed = min(base_speed, safe_speed)  # 더 느린 것 선택

        duration = self.read_blackboard("move_duration", 3.0)
        obstacle = self.read_blackboard("obstacle", {})

        if self.start_time is None:
            self.start_time = time.time()
            print(
                f"[{self.name}] Moving forward: {actual_speed:.2f}m/s for {duration}s"
            )

        elapsed = time.time() - self.start_time

        # 완료 체크
        if elapsed >= duration:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            distance = actual_speed * duration
            self.write_blackboard("distance_traveled", distance)
            self.write_blackboard("movement_status", "forward_complete")
            print(f"[{self.name}] Completed: {distance:.2f}m")
            self._reset_state()
            return py_trees.common.Status.SUCCESS

        # 급작스런 장애물 체크
        if obstacle.get("detected") and obstacle.get("distance", float("inf")) < 0.25:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            print(f"[{self.name}] Emergency stop: {obstacle['distance']:.2f}m")
            self._reset_state()
            return py_trees.common.Status.FAILURE

        # 실시간 속도 조절 전진
        wheel_radius = 0.033
        angular_speed = actual_speed / wheel_radius
        left_motor.setVelocity(angular_speed)
        right_motor.setVelocity(angular_speed)

        return py_trees.common.Status.RUNNING

    def _reset_state(self):
        self.start_time = None


@action(
    "adaptive_navigation",
    "Smart navigation with real-time obstacle avoidance",
    reads=["devices", "obstacle", "safe_speed", "target_direction"],
    writes=["movement_status", "navigation_complete"],
)
class AdaptiveNavigationAction(BaseAction):
    """실시간 장애물 회피 네비게이션"""

    def __init__(self, name: str, blackboard=None):
        super().__init__(name, blackboard)
        self.start_time = None
        self.stuck_count = 0
        self.last_position_check = 0
        self.last_obstacle_distance = float("inf")

    def execute(self) -> py_trees.common.Status:
        devices = self.read_blackboard("devices", {})
        left_motor = devices.get("left wheel motor")
        right_motor = devices.get("right wheel motor")

        if not left_motor or not right_motor:
            self._reset_state()
            return py_trees.common.Status.FAILURE

        if self.start_time is None:
            self.start_time = time.time()
            print(f"[{self.name}] Starting adaptive navigation...")

        current_time = time.time()

        # 타임아웃 (30초)
        if current_time - self.start_time > 30.0:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            print(f"[{self.name}] Navigation timeout")
            self._reset_state()
            return py_trees.common.Status.SUCCESS

        obstacle = self.read_blackboard("obstacle", {})
        safe_speed = self.read_blackboard("safe_speed", 0.15)

        obstacle_distance = obstacle.get("distance", float("inf"))
        obstacle_angle = obstacle.get("angle", 0)

        # 정체 감지 (장애물 거리가 계속 비슷하면)
        if abs(obstacle_distance - self.last_obstacle_distance) < 0.05:
            self.stuck_count += 1
        else:
            self.stuck_count = 0

        self.last_obstacle_distance = obstacle_distance

        # 정체 해결: 후진 + 회전
        if self.stuck_count > 20:  # 20틱 이상 정체
            print(f"[{self.name}] Stuck detected, backing up...")
            left_motor.setVelocity(-1.5)  # 후진
            right_motor.setVelocity(-1.5)
            self.stuck_count = 0
            return py_trees.common.Status.RUNNING

        wheel_radius = 0.033

        # 장애물 거리에 따른 행동 결정
        if obstacle_distance < 0.4:  # 가까운 장애물 - 회피
            # 장애물이 있는 반대 방향으로 회전
            avoidance_speed = 1.5
            if obstacle_angle > 0:  # 우측에 장애물 - 좌회전
                left_motor.setVelocity(-avoidance_speed)
                right_motor.setVelocity(avoidance_speed)
            else:  # 좌측에 장애물 - 우회전
                left_motor.setVelocity(avoidance_speed)
                right_motor.setVelocity(-avoidance_speed)

        elif obstacle_distance < 0.8:  # 중간 거리 - 조심스럽게 전진
            angular_speed = (safe_speed * 0.5) / wheel_radius  # 절반 속도
            left_motor.setVelocity(angular_speed)
            right_motor.setVelocity(angular_speed)

        else:  # 안전한 거리 - 정상 전진
            angular_speed = safe_speed / wheel_radius
            left_motor.setVelocity(angular_speed)
            right_motor.setVelocity(angular_speed)

        return py_trees.common.Status.RUNNING

    def _reset_state(self):
        self.start_time = None
        self.stuck_count = 0
        self.last_obstacle_distance = float("inf")


@action(
    "search_and_approach_object",
    "Spin search then approach detected objects",
    reads=["devices", "nearest_object", "obstacle", "approach_distance"],
    writes=["movement_status", "object_reached", "search_complete"],
)
class SearchAndApproachObjectAction(BaseAction):
    """360도 탐색 + 실시간 접근"""

    def __init__(self, name: str, blackboard=None):
        super().__init__(name, blackboard)
        self.state = "SEARCHING"  # SEARCHING, APPROACHING, COMPLETED
        self.start_time = None
        self.total_rotation = 0.0
        self.last_log_time = 0
        self.approach_start_time = None

    def execute(self) -> py_trees.common.Status:
        devices = self.read_blackboard("devices", {})
        left_motor = devices.get("left wheel motor")
        right_motor = devices.get("right wheel motor")

        if not left_motor or not right_motor:
            self._reset_state()
            return py_trees.common.Status.FAILURE

        current_time = time.time()

        if self.start_time is None:
            self.start_time = current_time
            print(f"[{self.name}] Starting object search...")

        # 전체 타임아웃
        if current_time - self.start_time > 30.0:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            print(f"[{self.name}] Total timeout")
            self._reset_state()
            return py_trees.common.Status.FAILURE

        if self.state == "SEARCHING":
            return self._handle_searching(left_motor, right_motor, current_time)
        elif self.state == "APPROACHING":
            return self._handle_approaching(left_motor, right_motor, current_time)
        else:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            return py_trees.common.Status.SUCCESS

    def _handle_searching(self, left_motor, right_motor, current_time):
        """360도 회전 탐색"""

        # 물체 발견 체크
        nearest_object = self.read_blackboard("nearest_object", {})

        if nearest_object and nearest_object.get("detected"):
            distance = nearest_object.get("distance", float("inf"))
            angle = nearest_object.get("angle", 0)

            if 0.3 <= distance <= 1.2:  # 유효한 물체
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)

                self.state = "APPROACHING"
                self.approach_start_time = current_time

                print(f"[{self.name}] Object found! {distance:.2f}m @ {angle:.0f}°")
                return py_trees.common.Status.RUNNING

        # 360도 완료 체크
        if self.total_rotation >= 2 * math.pi:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            print(f"[{self.name}] Search complete - no objects found")
            self.write_blackboard("search_complete", True)
            self._reset_state()
            return py_trees.common.Status.FAILURE

        # 계속 회전
        search_speed = 0.8
        left_motor.setVelocity(-search_speed)
        right_motor.setVelocity(search_speed)

        dt = 0.064  # TurtleBot3 timestep
        self.total_rotation += search_speed * dt

        if current_time - self.last_log_time > 3.0:
            progress = (self.total_rotation / (2 * math.pi)) * 100
            print(f"[{self.name}] Searching... {progress:.0f}%")
            self.last_log_time = current_time

        return py_trees.common.Status.RUNNING

    def _handle_approaching(self, left_motor, right_motor, current_time):
        """실시간 물체 접근"""

        approach_distance = self.read_blackboard("approach_distance", 0.3)

        # 접근 타임아웃
        if current_time - self.approach_start_time > 20.0:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            print(f"[{self.name}] Approach timeout")
            self._reset_state()
            return py_trees.common.Status.SUCCESS

        nearest_object = self.read_blackboard("nearest_object", {})
        obstacle = self.read_blackboard("obstacle", {})

        # 물체 사라짐
        if not nearest_object or not nearest_object.get("detected"):
            print(f"[{self.name}] Object lost, resuming search")
            self.state = "SEARCHING"
            return py_trees.common.Status.RUNNING

        current_distance = nearest_object.get("distance", float("inf"))
        current_angle = nearest_object.get("angle", 0)

        # 성공: 목표 거리 도달
        if current_distance <= approach_distance:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            self.write_blackboard("object_reached", True)
            print(f"[{self.name}] SUCCESS! Reached at {current_distance:.2f}m")
            self._reset_state()
            return py_trees.common.Status.SUCCESS

        # 안전: 너무 가까운 장애물
        if (
            obstacle.get("detected", False)
            and obstacle.get("distance", float("inf")) < 0.2
        ):
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            print(f"[{self.name}] Emergency stop - too close")
            self._reset_state()
            return py_trees.common.Status.SUCCESS

        # 각도 정규화
        target_angle = current_angle
        if target_angle > 180:
            target_angle -= 360
        elif target_angle < -180:
            target_angle += 360

        wheel_radius = 0.033

        # 회전 vs 직진
        if abs(target_angle) > 15:
            turn_speed = min(1.5, abs(target_angle) / 30.0)

            if target_angle > 0:
                left_motor.setVelocity(turn_speed)
                right_motor.setVelocity(-turn_speed)
            else:
                left_motor.setVelocity(-turn_speed)
                right_motor.setVelocity(turn_speed)

        else:
            # 거리 비례 속도
            if current_distance > 0.8:
                speed = 0.15
            elif current_distance > 0.5:
                speed = 0.10
            else:
                speed = 0.05

            angular_speed = speed / wheel_radius
            left_motor.setVelocity(angular_speed)
            right_motor.setVelocity(angular_speed)

        return py_trees.common.Status.RUNNING

    def _reset_state(self):
        self.state = "SEARCHING"
        self.start_time = None
        self.total_rotation = 0.0
        self.approach_start_time = None


@action(
    "proximity_wall_follow",
    "Follow nearby obstacles on the right side",
    reads=["devices", "wall_distance", "obstacle", "follow_duration"],
    writes=["movement_status"],
)
class ProximityWallFollowAction(BaseAction):
    """우측 장애물 근접 추종 (벽이든 뭐든)"""

    def __init__(self, name: str, blackboard=None):
        super().__init__(name, blackboard)
        self.start_time = None
        self.lost_wall_time = None

    def execute(self) -> py_trees.common.Status:
        devices = self.read_blackboard("devices", {})
        wall_info = self.read_blackboard("wall_distance", {})
        obstacle = self.read_blackboard("obstacle", {})
        duration = self.read_blackboard("follow_duration", 10.0)

        left_motor = devices.get("left wheel motor")
        right_motor = devices.get("right wheel motor")

        if not left_motor or not right_motor:
            self._reset_state()
            return py_trees.common.Status.FAILURE

        if self.start_time is None:
            self.start_time = time.time()
            print(f"[{self.name}] Following right-side obstacles for {duration}s")

        current_time = time.time()
        elapsed = current_time - self.start_time

        if elapsed >= duration:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            self.write_blackboard("movement_status", "follow_complete")
            self._reset_state()
            return py_trees.common.Status.SUCCESS

        if not wall_info or "error" in wall_info:
            self._reset_state()
            return py_trees.common.Status.FAILURE

        right_dist = wall_info.get("right", float("inf"))
        front_dist = wall_info.get("front", float("inf"))

        wheel_radius = 0.033
        target_distance = 0.5  # 50cm 목표

        # 우측에 추종할 대상이 없음
        if right_dist > 1.5:
            if self.lost_wall_time is None:
                self.lost_wall_time = current_time
                print(f"[{self.name}] Lost right obstacle")

            # 3초간 우측 대상 없으면 포기
            if current_time - self.lost_wall_time > 3.0:
                print(f"[{self.name}] No right obstacle found")
                self._reset_state()
                return py_trees.common.Status.FAILURE

            # 우측 탐색하며 천천히 회전
            left_motor.setVelocity(0.5)
            right_motor.setVelocity(-0.5)
            return py_trees.common.Status.RUNNING
        else:
            self.lost_wall_time = None  # 대상 재발견

        # 전방 장애물 - 좌회전으로 회피
        if front_dist < 0.4:
            left_motor.setVelocity(-1.2)
            right_motor.setVelocity(1.2)

        # 우측과의 거리 조절
        elif right_dist < target_distance - 0.1:  # 너무 가까움
            # 좌측으로 이동
            left_speed = 0.08 / wheel_radius
            right_speed = 0.12 / wheel_radius
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

        elif right_dist > target_distance + 0.1:  # 너무 멀음
            # 우측으로 접근
            left_speed = 0.12 / wheel_radius
            right_speed = 0.08 / wheel_radius
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

        else:  # 적절한 거리
            speed = 0.10 / wheel_radius
            left_motor.setVelocity(speed)
            right_motor.setVelocity(speed)

        return py_trees.common.Status.RUNNING

    def _reset_state(self):
        self.start_time = None
        self.lost_wall_time = None


# ========== BASIC MOVEMENTS ==========


@action(
    "move_backward",
    "Move backward safely",
    reads=["devices", "move_speed", "move_duration"],
    writes=["movement_status"],
)
class MoveBackwardAction(BaseAction):
    """안전한 후진"""

    def __init__(self, name: str, blackboard=None):
        super().__init__(name, blackboard)
        self.start_time = None

    def execute(self) -> py_trees.common.Status:
        devices = self.read_blackboard("devices", {})
        left_motor = devices.get("left wheel motor")
        right_motor = devices.get("right wheel motor")

        if not left_motor or not right_motor:
            self._reset_state()
            return py_trees.common.Status.FAILURE

        speed = self.read_blackboard("move_speed", 0.05)
        duration = self.read_blackboard("move_duration", 2.0)

        if self.start_time is None:
            self.start_time = time.time()
            print(f"[{self.name}] Backing up for {duration}s")

        elapsed = time.time() - self.start_time

        if elapsed >= duration:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            self.write_blackboard("movement_status", "backup_complete")
            self._reset_state()
            return py_trees.common.Status.SUCCESS

        wheel_radius = 0.033
        angular_speed = -speed / wheel_radius
        left_motor.setVelocity(angular_speed)
        right_motor.setVelocity(angular_speed)

        return py_trees.common.Status.RUNNING

    def _reset_state(self):
        self.start_time = None


@action(
    "turn_left",
    "Turn left in place",
    reads=["devices", "turn_speed", "turn_duration"],
    writes=["movement_status"],
)
class TurnLeftAction(BaseAction):
    """좌회전"""

    def __init__(self, name: str, blackboard=None):
        super().__init__(name, blackboard)
        self.start_time = None

    def execute(self) -> py_trees.common.Status:
        devices = self.read_blackboard("devices", {})
        left_motor = devices.get("left wheel motor")
        right_motor = devices.get("right wheel motor")

        if not left_motor or not right_motor:
            self._reset_state()
            return py_trees.common.Status.FAILURE

        turn_speed = self.read_blackboard("turn_speed", 1.57)
        duration = self.read_blackboard("turn_duration", 1.0)

        if self.start_time is None:
            self.start_time = time.time()
            print(f"[{self.name}] Turning left: {math.degrees(turn_speed):.0f}°/s")

        elapsed = time.time() - self.start_time

        if elapsed >= duration:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            self.write_blackboard("movement_status", "turn_complete")
            self._reset_state()
            return py_trees.common.Status.SUCCESS

        left_motor.setVelocity(-turn_speed)
        right_motor.setVelocity(turn_speed)

        return py_trees.common.Status.RUNNING

    def _reset_state(self):
        self.start_time = None


@action(
    "turn_right",
    "Turn right in place",
    reads=["devices", "turn_speed", "turn_duration"],
    writes=["movement_status"],
)
class TurnRightAction(BaseAction):
    """우회전"""

    def __init__(self, name: str, blackboard=None):
        super().__init__(name, blackboard)
        self.start_time = None

    def execute(self) -> py_trees.common.Status:
        devices = self.read_blackboard("devices", {})
        left_motor = devices.get("left wheel motor")
        right_motor = devices.get("right wheel motor")

        if not left_motor or not right_motor:
            self._reset_state()
            return py_trees.common.Status.FAILURE

        turn_speed = self.read_blackboard("turn_speed", 1.57)
        duration = self.read_blackboard("turn_duration", 1.0)

        if self.start_time is None:
            self.start_time = time.time()
            print(f"[{self.name}] Turning right: {math.degrees(turn_speed):.0f}°/s")

        elapsed = time.time() - self.start_time

        if elapsed >= duration:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            self.write_blackboard("movement_status", "turn_complete")
            self._reset_state()
            return py_trees.common.Status.SUCCESS

        left_motor.setVelocity(turn_speed)
        right_motor.setVelocity(-turn_speed)

        return py_trees.common.Status.RUNNING

    def _reset_state(self):
        self.start_time = None


@action("stop", "Stop all motors", reads=["devices"], writes=["movement_status"])
class StopAction(BaseAction):
    """즉시 정지"""

    def execute(self) -> py_trees.common.Status:
        devices = self.read_blackboard("devices", {})

        left_motor = devices.get("left wheel motor")
        right_motor = devices.get("right wheel motor")

        if left_motor and right_motor:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)

        self.write_blackboard("movement_status", "stopped")
        print(f"[{self.name}] Stopped")

        return py_trees.common.Status.SUCCESS


# ========== EXPLORATION ACTIONS ==========


@action(
    "smart_exploration",
    "Intelligent room exploration",
    reads=["devices", "obstacle", "safe_speed", "exploration_duration"],
    writes=["movement_status", "exploration_complete"],
)
class SmartExplorationAction(BaseAction):
    """지능적 환경 탐색"""

    def __init__(self, name: str, blackboard=None):
        super().__init__(name, blackboard)
        self.start_time = None
        self.last_turn_time = 0
        self.consecutive_obstacles = 0

    def execute(self) -> py_trees.common.Status:
        devices = self.read_blackboard("devices", {})
        left_motor = devices.get("left wheel motor")
        right_motor = devices.get("right wheel motor")

        if not left_motor or not right_motor:
            self._reset_state()
            return py_trees.common.Status.FAILURE

        if self.start_time is None:
            self.start_time = time.time()
            print(f"[{self.name}] Starting smart exploration...")

        current_time = time.time()
        duration = self.read_blackboard("exploration_duration", 20.0)

        if current_time - self.start_time > duration:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            self.write_blackboard("exploration_complete", True)
            print(f"[{self.name}] Exploration complete")
            self._reset_state()
            return py_trees.common.Status.SUCCESS

        obstacle = self.read_blackboard("obstacle", {})
        safe_speed = self.read_blackboard("safe_speed", 0.15)

        obstacle_distance = obstacle.get("distance", float("inf"))
        obstacle_angle = obstacle.get("angle", 0)

        wheel_radius = 0.033

        # 장애물 패턴 분석
        if obstacle_distance < 0.6:
            self.consecutive_obstacles += 1
        else:
            self.consecutive_obstacles = 0

        # 막다른 길 감지 (연속된 장애물)
        if self.consecutive_obstacles > 15:
            print(f"[{self.name}] Dead end detected, turning around")
            left_motor.setVelocity(-1.8)
            right_motor.setVelocity(1.8)
            self.consecutive_obstacles = 0
            self.last_turn_time = current_time
            return py_trees.common.Status.RUNNING

        # 최근에 회전했으면 잠시 직진
        if current_time - self.last_turn_time < 2.0:
            angular_speed = (safe_speed * 0.7) / wheel_radius
            left_motor.setVelocity(angular_speed)
            right_motor.setVelocity(angular_speed)
            return py_trees.common.Status.RUNNING

        # 장애물 거리에 따른 행동
        if obstacle_distance < 0.4:  # 가까운 장애물
            # 랜덤 방향으로 회피 (더 자연스러운 탐색)
            if random.random() < 0.7:  # 70% 확률로 왼쪽
                turn_speed = 1.5
                left_motor.setVelocity(-turn_speed)
                right_motor.setVelocity(turn_speed)
            else:  # 30% 확률로 오른쪽
                turn_speed = 1.5
                left_motor.setVelocity(turn_speed)
                right_motor.setVelocity(-turn_speed)
            self.last_turn_time = current_time

        elif obstacle_distance < 0.8:  # 중간 거리
            # 장애물 각도를 고려한 미세 조정
            base_speed = safe_speed * 0.8
            if abs(obstacle_angle) < 20:  # 정면
                angular_speed = base_speed / wheel_radius
                left_motor.setVelocity(angular_speed)
                right_motor.setVelocity(angular_speed)
            else:  # 측면 - 살짝 회피
                adjust = 0.3 if obstacle_angle > 0 else -0.3
                left_speed = (base_speed + adjust) / wheel_radius
                right_speed = (base_speed - adjust) / wheel_radius
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)

        else:  # 안전한 거리 - 정상 탐색
            angular_speed = safe_speed / wheel_radius
            left_motor.setVelocity(angular_speed)
            right_motor.setVelocity(angular_speed)

        return py_trees.common.Status.RUNNING

    def _reset_state(self):
        self.start_time = None
        self.consecutive_obstacles = 0
        self.last_turn_time = 0


# ========== CONDITION CHECKS ==========


@action(
    "check_for_object",
    "Check if object detected",
    reads=["nearest_object"],
    writes=["object_found"],
)
class CheckForObjectAction(BaseAction):
    """물체 감지 확인"""

    def execute(self) -> py_trees.common.Status:
        nearest_object = self.read_blackboard("nearest_object", {})

        if nearest_object and nearest_object.get("detected"):
            distance = nearest_object.get("distance", 0)
            self.write_blackboard("object_found", True)
            print(f"[{self.name}] Object found: {distance:.2f}m")
            return py_trees.common.Status.SUCCESS
        else:
            self.write_blackboard("object_found", False)
            return py_trees.common.Status.FAILURE


@action(
    "check_clear_path",
    "Check if path is clear ahead",
    reads=["obstacle", "min_safe_distance"],
    writes=["path_clear"],
)
class CheckClearPathAction(BaseAction):
    """전방 경로 안전 확인"""

    def execute(self) -> py_trees.common.Status:
        obstacle = self.read_blackboard("obstacle", {})
        min_safe = self.read_blackboard("min_safe_distance", 0.5)

        if not obstacle or "error" in obstacle:
            self.write_blackboard("path_clear", False)
            return py_trees.common.Status.FAILURE

        distance = obstacle.get("distance", 0)
        angle = obstacle.get("angle", 0)

        # 전방 ±30도 범위에서 안전 거리 체크
        if abs(angle) < 30 and distance < min_safe:
            self.write_blackboard("path_clear", False)
            print(f"[{self.name}] Path blocked: {distance:.2f}m")
            return py_trees.common.Status.FAILURE
        else:
            self.write_blackboard("path_clear", True)
            return py_trees.common.Status.SUCCESS


@action(
    "check_wall_proximity",
    "Check if there's a wall nearby for following",
    reads=["wall_distance", "proximity_threshold"],
    writes=["wall_nearby"],
)
class CheckWallProximityAction(BaseAction):
    """근처 벽/장애물 존재 확인"""

    def execute(self) -> py_trees.common.Status:
        wall_info = self.read_blackboard("wall_distance", {})
        threshold = self.read_blackboard("proximity_threshold", 1.2)

        if not wall_info or "error" in wall_info:
            self.write_blackboard("wall_nearby", False)
            return py_trees.common.Status.FAILURE

        right_dist = wall_info.get("right", float("inf"))
        left_dist = wall_info.get("left", float("inf"))

        # 좌우 어느 쪽이든 임계거리 내에 장애물 있으면
        if right_dist < threshold or left_dist < threshold:
            self.write_blackboard("wall_nearby", True)
            print(f"[{self.name}] Wall nearby - R:{right_dist:.2f}m L:{left_dist:.2f}m")
            return py_trees.common.Status.SUCCESS
        else:
            self.write_blackboard("wall_nearby", False)
            return py_trees.common.Status.FAILURE
