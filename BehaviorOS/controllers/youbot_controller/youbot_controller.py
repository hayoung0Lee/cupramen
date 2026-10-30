#!/usr/bin/env python3
"""
YouBot Controller with TCP Server and BehaviorOS Integration
"""

import asyncio
import json
import os
import sys

# ---------- paste-from-here ----------
import time

# from controller import Robot
from controller import Supervisor

# Add BehaviorOS to path
venv_path = "/home/hayoung/hayoung/behaviorOS/venv"
site_packages = os.path.join(venv_path, "lib", "python3.13", "site-packages")
sys.path.insert(0, site_packages)
sys.path.insert(0, "/home/hayoung/hayoung/behaviorOS")
# Import YouBot components
import controllers.youbot_controller.youbot_actions as youbot_actions  # noqa: F401
import controllers.youbot_controller.youbot_resolvers as youbot_resolvers  # noqa: F401
from behavioros.core import get_registry
from behavioros.core.builder import TreeBuilder
from controllers.youbot_controller.youbot_utils import move_to_ready

print("[YouBot] Registered actions:")
for action_type in get_registry().list_actions():
    print(f"  - {action_type}")


def _wheel_max(devices, default=10.0):
    vmax = []
    for wid in ("wheel1", "wheel2", "wheel3", "wheel4"):
        m = devices.get(wid)
        if m and hasattr(m, "getMaxVelocity"):
            try:
                vmax.append(m.getMaxVelocity())
            except:
                pass
    return min(vmax) if vmax else default


def _set_omni(
    devs,
    vx,
    vy,
    w,
    R=0.047,
    L=0.25,
    variant="A",
    wheel_order=("wheel1", "wheel2", "wheel3", "wheel4"),
    vy_flip=False,
    wheel_sign=(+1, +1, +1, +1),
):
    # vy_flip: 좌/우 방향 뒤집기
    if vy_flip:
        vy = -vy
    # 메카넘 역기구학 2가지 버전
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
    # 모터 부호 적용
    ws = [ws[i] * wheel_sign[i] for i in range(4)]
    # 속도 클램프 + 출력
    wmax = _wheel_max(devs)
    for wid, val in zip(wheel_order, ws):
        m = devs.get(wid)
        if m:
            m.setVelocity(max(-wmax, min(wmax, val)))


def _stop(devs):
    for wid in ("wheel1", "wheel2", "wheel3", "wheel4"):
        if wid in devs:
            devs[wid].setVelocity(0.0)


def _read_world_pose(devs):
    """Supervisor로 월드 포즈/방향축 읽기. (x,z,m00,m02) 반환. 없으면 None"""
    sup = devs.get("supervisor")
    if not sup or not hasattr(sup, "getSelf"):
        return None
    node = sup.getSelf()
    if node is None:
        return None
    pos = node.getPosition()  # [x,y,z]
    xw, zw = float(pos[0]), float(pos[2])
    M = node.getOrientation()  # row-major 3x3
    m00, m01, m02, *_ = M
    # forward_world = (m00, m02), left_world = (-m02, m00)
    return xw, zw, m00, m02


def _step(robot, timestep_ms, seconds):
    """Webots에서 wall-clock 대신 시뮬레이션 스텝"""
    if robot is None or timestep_ms is None:
        time.sleep(seconds)
        return
    n = max(1, int(seconds * 1000.0 / timestep_ms))
    for _ in range(n):
        robot.step(int(timestep_ms))


def run_quick_strafe_calibration(devices, robot=None, timestep_ms=32):
    """
    ▶ 실행하면 10~15초 내에 '옆으로 제일 잘 가는' 조합을 자동으로 찾아서
      콘솔에 initial_values JSON 추천을 출력합니다.
    - devices: 네 컨트롤러에서 만든 devices dict (wheel1..4, supervisor 포함 권장)
    - robot, timestep_ms: Webots 컨트롤러 객체/타임스텝 (없으면 None/None 가능)
    """
    pose0 = _read_world_pose(devices)
    if pose0 is None:
        print(
            "[STRAFE-CAL] Supervisor가 없어서 자동 보정 불가. (수동: omni_variant A/B, omni_vy_flip True/False 토글)"
        )
        return

    print("\n[STRAFE-CAL] 좌우 스트레이프 자동 보정 시작...")
    best = None
    # 간단 조합: variant 2 × vy_flip 2 × order 2 (원본/앞뒤스왑) = 8가지
    orders = [
        ("wheel1", "wheel2", "wheel3", "wheel4"),  # 원본
        ("wheel3", "wheel4", "wheel1", "wheel2"),  # front<->rear swap
    ]
    combos = [(v, f, o) for v in ("A", "B") for f in (False, True) for o in orders]

    for variant, vy_flip, order in combos:
        # 시작 포즈/기저축 저장
        x0, z0, m00, m02 = _read_world_pose(devices)
        lx, lz = -m02, m00  # left_world
        fx, fz = m00, m02  # forward_world

        # 왼쪽으로 0.5초
        _set_omni(
            devices,
            vx=0.0,
            vy=+0.20,
            w=0.0,
            variant=variant,
            wheel_order=order,
            vy_flip=vy_flip,
        )
        _step(robot, timestep_ms, 0.50)
        _stop(devices)
        _step(robot, timestep_ms, 0.20)

        # 끝 포즈
        x1, z1, _, _ = _read_world_pose(devices)
        dx, dz = (x1 - x0), (z1 - z0)
        lateral = dx * lx + dz * lz
        forward = dx * fx + dz * fz
        score = lateral - 0.3 * abs(forward)  # 옆으로 잘 가되 앞/뒤 새는 건 패널티

        print(
            f"[STRAFE-CAL] variant={variant}, vy_flip={vy_flip}, order={order} "
            f"→ lateral={lateral:+.3f}m, forward={forward:+.3f}m, score={score:+.3f}"
        )

        if (best is None) or (score > best[0]):
            best = (score, variant, vy_flip, order)

        # 원위치로 살짝 복귀(오버런 누적 방지; 대충 반대 펄스)
        _set_omni(
            devices,
            vx=0.0,
            vy=-0.15,
            w=0.0,
            variant=variant,
            wheel_order=order,
            vy_flip=vy_flip,
        )
        _step(robot, timestep_ms, 0.35)
        _stop(devices)
        _step(robot, timestep_ms, 0.15)

    if best is None:
        print("[STRAFE-CAL] 측정 실패")
        return

    _, v, f, o = best
    print("\n[STRAFE-CAL] ✅ 추천 설정 (initial_values에 그대로 복사하세요):")
    print("{")
    print(f'  "omni_variant": "{v}",')
    print(f'  "omni_vy_flip": {"true" if f else "false"},')
    print(f'  "wheel_order": {list(o)},')
    print('  "wheel_sign": [1,1,1,1]')
    print("}")
    print(
        "\n[STRAFE-CAL] 적용 후에도 미세하게 대각선으로 새면 V_MAX↓(0.45), A_BRAKE↑(1.0)로 안정화하세요."
    )


# ---------- paste-until-here ----------


class YouBotController:
    def __init__(self):
        # self.robot = Robot()
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Initialize devices
        self.devices = self._initialize_devices()

        # Blackboard for BehaviorOS
        self.blackboard_static = {
            "robot": self.robot,
            "devices": self.devices,
            "timestep": self.timestep,
            "gripper_open": True,
        }
        self.blackboard = {}

        # TCP Server setup
        self.server_port = 9999
        self.server = None
        self.reader = None
        self.writer = None

        # Tree execution
        self.current_tree = None
        self.tree_queue = asyncio.Queue()

        print("[YouBot] Controller initialized")
        print(f"[YouBot] TCP server will listen on port {self.server_port}")

    def _initialize_devices(self):
        devices = {}
        timestep = int(self.robot.getBasicTimeStep())
        # --- Wheels ---

        for i in range(1, 5):
            name = f"wheel{i}"
            m = self.robot.getDevice(name)
            if m:
                m.setPosition(float("inf"))
                m.setVelocity(0.0)
                devices[name] = m

                ps = m.getPositionSensor()
                if ps:
                    ps.enable(timestep)
                    devices[f"{name}_sensor"] = ps  # ★ 오도메트리에서 쓰는 키
                else:
                    print(f"[YouBot][WARN] {name} has no PositionSensor")
        devices["wheels"] = {
            "left": ["wheel1", "wheel3"],
            "right": ["wheel2", "wheel4"],
        }

        # --- Arms (motors) + PositionSensors ---
        timestep = int(self.robot.getBasicTimeStep())
        for i in range(1, 6):
            arm_name = f"arm{i}"
            motor = self.robot.getDevice(arm_name)
            if motor:
                motor.setVelocity(1.2)
                devices[arm_name] = motor
                ps = motor.getPositionSensor()
                if ps:
                    ps.enable(timestep)
                    devices[f"{arm_name}_sensor"] = ps

        # --- Camera (필수) ---
        # 주 카메라 이름이 'camera'가 아닐 수도 있으니 필요하면 바꾸세요.
        cam = None
        for cand in ["camera", "front_camera", "Camera", "CAMERA"]:
            print("[YouBot] Trying to find camera device:", cand)
            try:
                c = self.robot.getDevice(cand)
                if c:
                    cam = c
                    print("[YouBot] Camera found:", cand)
                    break
            except Exception:
                pass

        if cam:
            cam.enable(timestep)  # 프레임 출력 시작(중요)
            # 인식 기능을 쓴다면: if hasattr(cam, "recognitionEnable"): cam.recognitionEnable(timestep)
            devices["camera"] = cam
            print(
                "[YouBot] Camera initialized:",
                cam.getName(),
                cam.getWidth(),
                "x",
                cam.getHeight(),
            )
            # 카메라는 enable 직후엔 프레임이 None일 수 있어요 → 워밍업 몇 틱
            for _ in range(3):
                self.robot.step(timestep)
        else:
            print("[YouBot] Camera NOT found. Vision resolver will fallback to static.")

        depth = None
        for cand in ["range-finder", "depth", "Depth", "range", "rangefinder"]:
            try:
                d = self.robot.getDevice(cand)
                if d:
                    depth = d
                    depth.enable(timestep)  # RangeFinder도 enable
                    devices["depth"] = depth
                    print(
                        "[YouBot] Depth initialized:",
                        depth.getName(),
                        depth.getWidth(),
                        "x",
                        depth.getHeight(),
                    )
                    break
            except Exception:
                pass

        # --- Gripper (linear motors) ---
        def _get(dev_name_list):
            for nm in dev_name_list:
                d = self.robot.getDevice(nm)
                if d:
                    return nm, d
            return None, None

        left_key, left_finger = _get(["finger::left", "finger_left", "left_finger"])
        right_key, right_finger = _get(
            ["finger::right", "finger_right", "right_finger"]
        )

        if left_finger and right_finger:
            left_finger.setVelocity(0.03)
            right_finger.setVelocity(0.03)
            left_finger.setPosition(0.025)
            right_finger.setPosition(0.025)
            devices[left_key] = left_finger
            devices[right_key] = right_finger
            print("[YouBot] Gripper devices initialized:", left_key, right_key)
        else:
            print("[YouBot] Gripper NOT found (left/right). Skipping.")

        print("[YouBot] devices keys:", sorted(list(devices.keys())))

        for _ in range(3):
            self.robot.step(timestep)

        move_to_ready(devices, self.robot, self.timestep)

        devices["supervisor"] = self.robot

        # run_quick_strafe_calibration(devices, robot=self.robot, timestep_ms=timestep)
        return devices

    async def handle_client(self, reader, writer):
        """Handle TCP client connection"""
        addr = writer.get_extra_info("peername")
        print(f"\n[YouBot] Client connected from {addr}")

        self.reader = reader
        self.writer = writer

        try:
            while True:
                data = await reader.readline()
                if not data:
                    break

                try:
                    message = json.loads(data.decode())
                    await self.process_message(message)
                except json.JSONDecodeError as e:
                    print(f"[YouBot] Invalid JSON: {e}")
                    await self.send_error("Invalid JSON format")

        except Exception as e:
            print(f"[YouBot] Client error: {e}")

        finally:
            writer.close()
            await writer.wait_closed()
            self.reader = None
            self.writer = None
            print("[YouBot] Client disconnected")

    async def process_message(self, message):
        """Process message from client"""
        msg_type = message.get("type")

        if msg_type == "tree":
            tree_json = message.get("data")
            print("\n[YouBot] Received behavior tree")

            # Queue tree for execution
            await self.tree_queue.put(tree_json)

        elif msg_type == "stop":
            print("[YouBot] Stop requested")
            self.current_tree = None
            # Stop all motors
            for i in range(1, 5):
                wheel = f"wheel{i}"
                if wheel in self.devices:
                    self.devices[wheel].setVelocity(0)

        else:
            await self.send_error(f"Unknown message type: {msg_type}")

    async def send_status(self, tick, status, blackboard):
        """Send status update to client"""
        if not self.writer:
            return

        # Filter out non-serializable items
        clean_blackboard = {}
        for key, value in blackboard.items():
            if key not in ["robot", "devices"]:
                clean_blackboard[key] = value

        message = {
            "type": "status",
            "tick": tick,
            "status": status,
            "blackboard": clean_blackboard,
        }

        try:
            self.writer.write((json.dumps(message) + "\n").encode())
            await self.writer.drain()
        except Exception as e:
            print(f"[YouBot] Failed to send status: {e}")

    async def send_error(self, error_msg):
        """Send error message to client"""
        if not self.writer:
            return

        message = {"type": "error", "message": error_msg}

        try:
            self.writer.write((json.dumps(message) + "\n").encode())
            await self.writer.drain()
        except:
            pass

    def execute_tree(self, tree_json):
        """Start executing a behavior tree using TreeBuilder"""
        try:
            print("\n[YouBot] Starting tree execution with TreeBuilder...")

            print("[YouBot] tree_json:", json.dumps(tree_json, indent=2))

            # Use TreeBuilder instead of TreeExecutor directly
            self.blackboard = {}
            self.blackboard.update(self.blackboard_static)
            self.current_tree = (
                TreeBuilder(tree_json, self.blackboard, self.devices)
                .with_runtime()  # Add runtime resolution
                .with_logging()  # Add logging
                .build()
            )

            print(f"[YouBot] self.current_tree: {self.current_tree}")

            print("[YouBot] Tree created successfully with TreeBuilder")

        except Exception as e:
            print(f"[YouBot] Failed to start tree: {e}")
            import traceback

            traceback.print_exc()
            self.current_tree = None

    async def start_server(self):
        """Start TCP server"""
        self.server = await asyncio.start_server(
            self.handle_client, "localhost", self.server_port
        )
        print(f"[YouBot] TCP server listening on port {self.server_port}")

    async def execution_loop(self):
        """Main execution loop"""
        tick_count = 0

        print("[YouBot] Execution loop started")
        print("[YouBot] Waiting for commands...")
        print("-" * 40)
        print("Run youbot_client.py to send commands")
        print("-" * 40)

        try:
            while True:
                # Check for new tree only if not executing
                if self.current_tree is None:
                    try:
                        tree_json = self.tree_queue.get_nowait()
                        self.execute_tree(tree_json)
                        tick_count = 0
                    except asyncio.QueueEmpty:
                        pass

                # Execute current tree if running
                if self.current_tree:
                    # Tick the tree
                    result = self.current_tree.tick()
                    tick_count += 1

                    # Send status update
                    if tick_count % 10 == 0:
                        await self.send_status(
                            tick_count, result.status, self.current_tree.blackboard
                        )
                        print(f"[YouBot] Tick {tick_count}: {result.status}")

                    # Check completion
                    if result.status in ["SUCCESS", "FAILURE"]:
                        final = self.current_tree.get_execution_result()
                        print(f"\n[YouBot] Tree completed: {final.status}")
                        print(f"  Total ticks: {final.total_ticks}")
                        print(f"  Duration: {final.duration:.2f}s")

                        # Send final status
                        await self.send_status(
                            final.total_ticks, final.status, final.blackboard
                        )

                        # Clear tree
                        self.current_tree = None
                        tick_count = 0

                # Webots simulation step
                if self.robot.step(self.timestep) == -1:
                    print("[YouBot] Simulation ended")
                    break

                # Allow other async tasks
                await asyncio.sleep(0)

        except Exception as e:
            print(f"[ERROR] Exception in execution_loop: {e}")
            import traceback

            traceback.print_exc()
            raise

    async def run(self):
        """Main run method"""
        # Start TCP server
        await self.start_server()

        # Create server task
        server_task = asyncio.create_task(self.server.serve_forever())

        # Run execution loop
        try:
            await self.execution_loop()
        except Exception as e:
            print(f"[YouBot] Execution error: {e}")
        finally:
            # Cancel server
            server_task.cancel()

            # Close server
            if self.server:
                self.server.close()
                await self.server.wait_closed()

            print("[YouBot] Shutdown complete")


# Main entry point
if __name__ == "__main__":
    controller = YouBotController()

    # Run async controller
    try:
        asyncio.run(controller.run())
    except KeyboardInterrupt:
        print("\n[YouBot] Interrupted by user")
    except Exception as e:
        print(f"[YouBot] Fatal error: {e}")
        import traceback

        traceback.print_exc()
