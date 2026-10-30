#!/usr/bin/env python3
"""
TurtleBot3 Burger Controller with TCP Server
Receives behavior trees from client and executes them
"""

import asyncio
import json
import os
import sys

from controller import Robot

# Add BehaviorOS to path
venv_path = "/home/hayoung/hayoung/behaviorOS/venv"
site_packages = os.path.join(venv_path, "lib", "python3.13", "site-packages")
sys.path.insert(0, site_packages)
sys.path.insert(0, "/home/hayoung/hayoung/behaviorOS")

import controllers.turtlebot_controller.turtlebot_actions as turtlebot_actions  # noqa: F401
import controllers.turtlebot_controller.turtlebot_resolvers as turtlebot_resolvers  # noqa: F401
from behavioros.core.builder import TreeBuilder


class TurtleBotController:
    """
    TurtleBot3 Controller with TCP server for receiving commands
    """

    def __init__(self):
        # Initialize Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Initialize devices
        self.devices = self._initialize_devices()

        # TCP server setup
        self.server_port = 9999
        self.server = None
        self.reader = None
        self.writer = None

        # Tree execution
        self.current_tree = None
        self.tree_queue = asyncio.Queue()

        # Blackboard
        self.blackboard = {
            "robot": self.robot,
            "devices": self.devices,
            "timestep": self.timestep,
            "position": {"x": 0, "y": 0},
        }

        print("[TurtleBot3] Controller initialized")
        print("  Model: TurtleBot3 Burger")
        print(f"  Timestep: {self.timestep}ms")
        print(f"  Devices: {list(self.devices.keys())}")
        print(f"  Waiting for client on port {self.server_port}...")

    def _initialize_devices(self):
        """Initialize TurtleBot3 devices"""
        devices = {}

        # Motors
        devices["left wheel motor"] = self.robot.getDevice("left wheel motor")
        devices["right wheel motor"] = self.robot.getDevice("right wheel motor")

        # LiDAR
        devices["LDS-01"] = self.robot.getDevice("LDS-01")
        if devices["LDS-01"]:
            devices["LDS-01"].enable(self.timestep)

        # Initialize motors
        for motor in ["left wheel motor", "right wheel motor"]:
            if devices[motor]:
                devices[motor].setPosition(float("inf"))
                devices[motor].setVelocity(0)

        return devices

    async def handle_client(self, reader, writer):
        """Handle client connection"""
        addr = writer.get_extra_info("peername")
        print(f"[TurtleBot3] Client connected from {addr}")

        self.reader = reader
        self.writer = writer

        try:
            while True:
                # Read message from client
                data = await reader.readline()
                if not data:
                    break

                # Parse message
                try:
                    message = json.loads(data.decode())
                    await self.process_message(message)
                except json.JSONDecodeError as e:
                    print(f"[TurtleBot3] Invalid JSON: {e}")
                    await self.send_error("Invalid JSON format")

        except Exception as e:
            print(f"[TurtleBot3] Client error: {e}")

        finally:
            writer.close()
            await writer.wait_closed()
            self.reader = None
            self.writer = None
            print("[TurtleBot3] Client disconnected")

    async def process_message(self, message):
        """Process message from client"""
        msg_type = message.get("type")

        if msg_type == "tree":
            # Received new tree
            tree_json = message.get("data")
            print("[TurtleBot3] Received tree")

            # Queue for execution
            await self.tree_queue.put(tree_json)

        elif msg_type == "stop":
            # Stop current execution
            print("[TurtleBot3] Stop requested")
            self.current_tree = None

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
            print(f"[TurtleBot3] Failed to send status: {e}")

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

    def _test_lidar_data(self):
        """LiDAR 데이터 테스트"""
        lidar = self.devices.get("LDS-01")
        if lidar:
            ranges = lidar.getRangeImage()
            close_objects = [r for r in ranges if r < 2.0]  # 2m 이내 물체
            print(f"🔍 LiDAR test: {len(close_objects)} objects within 2m")
            print(f"   Range data sample: {ranges[:10]}")

    def execute_tree(self, tree_json):
        """Start executing a behavior tree"""
        try:
            self._test_lidar_data()

            self.current_tree = (
                TreeBuilder(tree_json, self.blackboard, self.devices)
                .with_runtime()
                .with_logging()
                .build()
            )

        except Exception as e:
            print(f"[TurtleBot3] Failed to start tree: {e}")
            self.current_tree = None

    async def start_server(self):
        """Start TCP server"""
        self.server = await asyncio.start_server(
            self.handle_client, "localhost", self.server_port
        )
        print(f"[TurtleBot3] Server listening on port {self.server_port}")

    async def execution_loop(self):
        """Main execution loop for trees"""
        tick_count = 0

        print("[TurtleBot3] Execution loop started")

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

                    # Send status update every 10 ticks
                    if tick_count % 10 == 0:
                        await self.send_status(
                            tick_count, result.status, self.current_tree.blackboard
                        )
                        print(f"[TurtleBot3] Tick {tick_count}: {result.status}")

                    # Check completion
                    if result.status in ["SUCCESS", "FAILURE"]:
                        # Final status
                        final = self.current_tree.get_execution_result()
                        print(f"[Controller] Execution completed: {final.status}")
                        print(f"  Total ticks: {final.total_ticks}")
                        print(f"  Duration: {final.duration:.2f}s")

                        # Send final status
                        await self.send_status(
                            final.total_ticks, final.status, final.blackboard
                        )

                        # Clear tree to allow next one
                        self.current_tree = None
                        tick_count = 0

                # Webots simulation step
                if self.robot.step(self.timestep) == -1:
                    print("[TurtleBot3] Simulation ended")
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
        # Start server
        await self.start_server()

        # Create server task
        server_task = asyncio.create_task(self.server.serve_forever())

        # Run execution loop
        try:
            await self.execution_loop()
        except Exception as e:
            print(f"[TurtleBot3] Execution loop error: {e}")
        finally:
            # Cancel server when execution ends
            server_task.cancel()

            # Close server
            if self.server:
                self.server.close()
                await self.server.wait_closed()

            print("[TurtleBot3] Shutdown complete")


# Main entry point for Webots
if __name__ == "__main__":
    controller = TurtleBotController()

    # Run async controller
    try:
        asyncio.run(controller.run())
    except KeyboardInterrupt:
        print("\n[TurtleBot3] Interrupted by user")
    except Exception as e:
        print(f"[TurtleBot3] Fatal error: {e}")
        import traceback

        traceback.print_exc()
