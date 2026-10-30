"""
Webots Robot Actions for BehaviorOS
Simple movement actions for robot control.
"""

import time

import py_trees

from behavioros.core import BaseAction, action


@action(
    "move_forward",
    "Move robot forward for specified duration",
    reads=["robot", "devices", "move_speed", "move_duration"],
    writes=["movement_status"],
)
class MoveForwardAction(BaseAction):
    """Move robot forward."""

    def __init__(self, name: str, blackboard=None):
        super().__init__(name, blackboard)
        self.start_time = None
        self.moving = False
        self.tick_count = 0  # 틱 카운터 추가

    def execute(self) -> py_trees.common.Status:
        robot = self.read_blackboard("robot")
        devices = self.read_blackboard("devices", {})
        self.tick_count += 1  # 틱 카운트 증가

        if not robot or not devices:
            print(f"[{self.name}] Robot or devices not available")
            return py_trees.common.Status.FAILURE

        # Get parameters
        speed = self.read_blackboard("move_speed", 1.0)
        duration = self.read_blackboard("move_duration", 3.0)

        # TIAGo robot has wheel_left_joint and wheel_right_joint
        left_motor = devices.get("wheel_left_joint")
        right_motor = devices.get("wheel_right_joint")

        if not left_motor or not right_motor:
            print(f"[{self.name}] Could not find wheel motors")
            return py_trees.common.Status.FAILURE

        # Initialize movement
        if not self.moving:
            self.start_time = time.time()
            self.moving = True
            print(f"[{self.name}] Moving forward: speed={speed}, duration={duration}s")

            # Set infinite position for continuous rotation
            left_motor.setPosition(float("inf"))
            right_motor.setPosition(float("inf"))

        # Set velocities (same speed for straight movement)
        left_motor.setVelocity(speed)
        right_motor.setVelocity(speed)

        # Check duration
        elapsed = time.time() - self.start_time

        # 디버그 출력 추가
        if self.tick_count % 10 == 0:  # 10틱마다 출력
            print(f"[DEBUG] Elapsed: {elapsed:.2f}s / Duration: {duration}s")

        if elapsed >= duration:
            # Stop motors
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)

            print(f"[{self.name}] STOPPING NOW!")  # 확실히 멈추는지 확인

            self.write_blackboard("movement_status", "forward_complete")

            # Reset for next execution
            self.moving = False
            self.start_time = None
            return py_trees.common.Status.SUCCESS

        # Still moving
        return py_trees.common.Status.RUNNING


@action(
    "turn_left",
    "Turn robot left for specified duration",
    reads=["robot", "devices", "turn_speed", "turn_duration"],
    writes=["movement_status"],
)
class TurnLeftAction(BaseAction):
    """Turn robot left."""

    def __init__(self, name: str, blackboard=None):
        super().__init__(name, blackboard)
        self.start_time = None
        self.turning = False

    def execute(self) -> py_trees.common.Status:
        robot = self.read_blackboard("robot")
        devices = self.read_blackboard("devices", {})

        if not robot or not devices:
            return py_trees.common.Status.FAILURE

        # Get parameters
        speed = self.read_blackboard("turn_speed", 3.0)
        duration = self.read_blackboard("turn_duration", 2.0)

        # TIAGo robot wheels
        left_motor = devices.get("wheel_left_joint")
        right_motor = devices.get("wheel_right_joint")

        if not left_motor or not right_motor:
            print(f"[{self.name}] Could not find wheel motors")
            return py_trees.common.Status.FAILURE

        # Initialize turn
        if not self.turning:
            self.start_time = time.time()
            self.turning = True
            print(f"[{self.name}] Turning left: speed={speed}, duration={duration}s")

            left_motor.setPosition(float("inf"))
            right_motor.setPosition(float("inf"))

        # Turn left: left wheel slower/backward, right wheel forward
        left_motor.setVelocity(-speed)
        right_motor.setVelocity(speed)

        # Check duration
        elapsed = time.time() - self.start_time

        if elapsed >= duration:
            # Stop
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)

            self.write_blackboard("movement_status", "turn_complete")
            print(f"[{self.name}] Turn completed")

            self.turning = False
            self.start_time = None
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


@action(
    "turn_right",
    "Turn robot right for specified duration",
    reads=["robot", "devices", "turn_speed", "turn_duration"],
    writes=["movement_status"],
)
class TurnRightAction(BaseAction):
    """Turn robot right."""

    def __init__(self, name: str, blackboard=None):
        super().__init__(name, blackboard)
        self.start_time = None
        self.turning = False

    def execute(self) -> py_trees.common.Status:
        robot = self.read_blackboard("robot")
        devices = self.read_blackboard("devices", {})

        if not robot or not devices:
            return py_trees.common.Status.FAILURE

        # Get parameters
        speed = self.read_blackboard("turn_speed", 3.0)
        duration = self.read_blackboard("turn_duration", 2.0)

        # TIAGo robot wheels
        left_motor = devices.get("wheel_left_joint")
        right_motor = devices.get("wheel_right_joint")

        if not left_motor or not right_motor:
            print(f"[{self.name}] Could not find wheel motors")
            return py_trees.common.Status.FAILURE

        # Initialize turn
        if not self.turning:
            self.start_time = time.time()
            self.turning = True
            print(f"[{self.name}] Turning right: speed={speed}, duration={duration}s")

            left_motor.setPosition(float("inf"))
            right_motor.setPosition(float("inf"))

        # Turn right: left wheel forward, right wheel slower/backward
        left_motor.setVelocity(speed)
        right_motor.setVelocity(-speed)

        # Check duration
        elapsed = time.time() - self.start_time

        if elapsed >= duration:
            # Stop
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)

            self.write_blackboard("movement_status", "turn_complete")
            print(f"[{self.name}] Turn completed")

            self.turning = False
            self.start_time = None
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


@action(
    "stop",
    "Stop all robot motors",
    reads=["robot", "devices"],
    writes=["movement_status"],
)
class StopAction(BaseAction):
    """Stop robot."""

    def execute(self) -> py_trees.common.Status:
        devices = self.read_blackboard("devices", {})

        if not devices:
            return py_trees.common.Status.FAILURE

        print(f"[{self.name}] Stopping robot...")

        # Stop wheel motors specifically
        wheel_motors = ["wheel_left_joint", "wheel_right_joint"]
        stopped = 0

        for motor_name in wheel_motors:
            if motor_name in devices:
                try:
                    devices[motor_name].setVelocity(0)
                    stopped += 1
                except:
                    pass

        self.write_blackboard("movement_status", "stopped")
        print(f"[{self.name}] Stopped {stopped} wheel motors")

        return py_trees.common.Status.SUCCESS
