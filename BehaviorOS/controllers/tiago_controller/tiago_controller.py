#!/usr/bin/env python3
"""
BehaviorOS Webots Controller
Receives behavior trees from client and executes them in Webots.

This file should be set as the robot controller in Webots.
"""

import asyncio
import json
import os
import sys

from controller import Robot

# Add behavioros to path
venv_path = "/home/hayoung/hayoung/behaviorOS/venv"
site_packages = os.path.join(venv_path, "lib", "python3.13", "site-packages")
sys.path.insert(0, site_packages)
sys.path.insert(0, "/home/hayoung/hayoung/behaviorOS")

# Import Webots actions (auto-registers them)

import controllers.tiago_controller.tiago_actions as tiago_actions  # noqa: F401
from behavioros.core.helpers import create_tree


class WebotsController:
    """
    Controller that runs in Webots simulation.
    Receives trees from client and executes them.
    """

    def __init__(self):
        # Initialize Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Initialize devices
        self.devices = self._initialize_devices()

        # TCP server for receiving trees
        self.server_port = 9999
        self.server = None
        self.reader = None
        self.writer = None

        # Tree execution
        self.current_tree = None
        self.tree_queue = asyncio.Queue()

        print("[Controller] Initialized")
        # print(f"  Timestep: {self.timestep}ms")
        # print(f"  Devices: {list(self.devices.keys())}")
        print(f"  Waiting for client on port {self.server_port}...")

    def _initialize_devices(self):
        """Initialize all robot devices."""
        devices = {}

        # Get all devices
        n_devices = self.robot.getNumberOfDevices()

        for i in range(n_devices):
            device = self.robot.getDeviceByIndex(i)
            name = device.getName()
            devices[name] = device

            # Enable sensors
            if hasattr(device, "enable"):
                try:
                    device.enable(self.timestep)
                    # print(f"  Enabled sensor: {name}")
                except:
                    pass

        return devices

    async def handle_client(self, reader, writer):
        """Handle client connection."""
        addr = writer.get_extra_info("peername")
        print(f"[Controller] Client connected from {addr}")

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
                    print(f"[Controller] Invalid JSON: {e}")
                    await self.send_error("Invalid JSON format")

        except Exception as e:
            print(f"[Controller] Client error: {e}")

        finally:
            writer.close()
            await writer.wait_closed()
            self.reader = None
            self.writer = None
            print("[Controller] Client disconnected")

    async def process_message(self, message):
        """Process message from client."""
        msg_type = message.get("type")

        if msg_type == "tree":
            # Received new tree
            tree_json = message.get("data")
            print("[Controller] Received tree")

            # Queue tree for execution
            await self.tree_queue.put(tree_json)

        elif msg_type == "stop":
            # Stop current execution
            print("[Controller] Stop requested")
            self.current_tree = None

        else:
            await self.send_error(f"Unknown message type: {msg_type}")

    async def send_status(self, tick, status, blackboard):
        """Send status update to client."""
        if not self.writer:
            return

        # Filter out non-serializable items from blackboard
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
            print(f"[Controller] Failed to send status: {e}")

    async def send_error(self, error_msg):
        """Send error message to client."""
        if not self.writer:
            return

        message = {"type": "error", "message": error_msg}

        try:
            self.writer.write((json.dumps(message) + "\n").encode())
            await self.writer.drain()
        except:
            pass

    def execute_tree(self, tree_json):
        """Start executing a behavior tree."""
        try:
            # Setup blackboard with robot and devices
            blackboard = {
                "robot": self.robot,
                "devices": self.devices,
                "timestep": self.timestep,
            }

            self.current_tree = create_tree(tree_json, blackboard, self.devices)

        except Exception as e:
            print(f"[Controller] Failed to start tree: {e}")
            self.current_tree = None

    async def start_server(self):
        """Start TCP server."""
        self.server = await asyncio.start_server(
            self.handle_client, "localhost", self.server_port
        )
        print(f"[Controller] Server listening on port {self.server_port}")

    async def execution_loop(self):
        """Main execution loop for trees."""
        tick_count = 0
        print("[DEBUG] Execution loop started!")

        try:  # 전체를 try-except로 감싸기
            while True:
                # Check for new tree ONLY if not currently executing
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
                        print(f"[Controller] Tick {tick_count}: {result.status}")

                    # Check completion
                    if result.status in ["SUCCESS", "FAILURE"]:
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
                    print("[Controller] Simulation ended")
                    break

                # Allow other async tasks to run
                await asyncio.sleep(0)

        except Exception:
            print("[ERROR] Exception in execution_loop at line:")
            import traceback

            traceback.print_exc()
            raise  # 에러를 다시 발생시켜서 상위에서 처리

    async def run(self):
        """Main run method."""
        # Start server
        await self.start_server()

        # Create server task
        server_task = asyncio.create_task(self.server.serve_forever())

        # Run execution loop
        try:
            await self.execution_loop()
        except Exception as e:
            print(f"[Controller] Execution loop error: {e}")
        finally:
            # Cancel server when execution ends
            server_task.cancel()

            # Close server
            if self.server:
                self.server.close()
                await self.server.wait_closed()

            print("[Controller] Shutdown complete")


# Main entry point for Webots
if __name__ == "__main__":
    controller = WebotsController()

    # Run async controller
    try:
        asyncio.run(controller.run())
    except KeyboardInterrupt:
        print("\n[Controller] Interrupted by user")
    except Exception as e:
        print(f"[Controller] Fatal error: {e}")
        import traceback

        traceback.print_exc()
