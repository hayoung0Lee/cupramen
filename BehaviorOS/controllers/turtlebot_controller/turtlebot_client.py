"""
TurtleBot3 Client
Connects CLI commands to TurtleBot controller via LLM-generated behavior trees.

Usage:
    # Interactive mode
    python turtlebot_client.py

    # Single command
    python turtlebot_client.py -c "go to the nearest box"

    # Batch commands
    python turtlebot_client.py -b "move forward" "follow wall" "stop"
"""

import asyncio
import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add behavioros to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

# Import TurtleBot examples to register them
import controllers.turtlebot_controller.turtlebot_prompts  # noqa: F401
from behavioros.llm import OpenAIClient
from behavioros.prompts import build_prompt


class TurtleBotClient:
    """
    Client that generates behavior trees from natural language commands
    and sends them to TurtleBot controller for execution.
    """

    def __init__(self, controller_host: str = "localhost", controller_port: int = 9999):
        """
        Initialize the TurtleBot client.

        Args:
            controller_host: Host address of the TurtleBot controller
            controller_port: Port number of the TurtleBot controller
        """
        self.controller_host = controller_host
        self.controller_port = controller_port
        self.llm_client = OpenAIClient()
        self.reader: Optional[asyncio.StreamReader] = None
        self.writer: Optional[asyncio.StreamWriter] = None
        self.connected = False

    async def connect_to_controller(self) -> bool:
        """
        Establish TCP connection to the TurtleBot controller.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.reader, self.writer = await asyncio.open_connection(
                self.controller_host, self.controller_port
            )
            self.connected = True
            print(
                f"[Connected] TurtleBot controller at {self.controller_host}:{self.controller_port}"
            )
            return True

        except ConnectionRefusedError:
            print(
                f"[Error] Cannot connect to controller at {self.controller_host}:{self.controller_port}"
            )
            print("Make sure the TurtleBot controller is running first in Webots.")
            return False

        except Exception as e:
            print(f"[Error] Connection failed: {e}")
            return False

    async def disconnect(self):
        """Close connection to the controller."""
        if self.writer:
            self.writer.close()
            await self.writer.wait_closed()
            self.connected = False
            print("[Disconnected] Connection closed")

    async def generate_behavior_tree(self, command: str) -> Optional[Dict[str, Any]]:
        """
        Generate a behavior tree from natural language command using LLM.

        Args:
            command: Natural language command for the robot

        Returns:
            Generated behavior tree as JSON dict, or None if generation failed
        """
        print(f"\n[LLM] Generating tree for: '{command}'")

        try:
            # Build prompt with TurtleBot-specific examples
            prompt = build_prompt(
                command,
                example_tags=["turtlebot"],  # Use TurtleBot examples
                max_examples=3,
            )

            # Add TurtleBot-specific context
            turtlebot_context = """

IMPORTANT - TurtleBot3 Burger specific actions:
- move_forward: Move forward with adaptive speed (uses move_speed, move_duration, ${safe_speed}, ${obstacle})
- search_and_approach_object: 360° search then approach detected objects (uses ${nearest_object}, ${obstacle})
- adaptive_navigation: Smart navigation with real-time obstacle avoidance (uses ${obstacle}, ${safe_speed})
- proximity_wall_follow: Follow nearby obstacles on right side (uses ${wall_distance}, ${obstacle})
- smart_exploration: Intelligent room exploration (uses ${obstacle}, ${safe_speed})
- move_backward: Move backward safely (uses move_speed, move_duration)
- turn_left/turn_right: Turn in place (uses turn_speed, turn_duration)
- stop: Stop all motors
- check_for_object: Check if object detected (uses ${nearest_object})
- check_clear_path: Check if path is clear ahead (uses ${obstacle})
- check_wall_proximity: Check for nearby walls (uses ${wall_distance})

Resolution templates available:
- ${obstacle}: Real-time obstacle detection with distance and angle
- ${nearest_object}: Detect nearest non-wall objects with wall filtering
- ${wall_distance}: Wall distances in front/left/right sectors
- ${safe_speed}: Adaptive speed based on obstacle proximity (0.05-0.20 m/s)

TurtleBot3 Burger specs:
- Max linear speed: 0.22 m/s
- Max angular speed: 2.84 rad/s  
- Wheel radius: 0.033 m
- Wheel base: 0.160 m
- LiDAR: 360 degree scanning, 0.12-3.5m range

Key behaviors:
- Object search: Spin 360° then approach detected objects
- Adaptive movement: Real-time speed adjustment based on obstacles
- Wall following: Track nearby obstacles/walls on right side
- Smart exploration: Intelligent room navigation with dead-end detection
- Safety: All actions include obstacle avoidance and emergency stops

Return ONLY valid JSON with templates for dynamic values.
"""

            # Combine prompt with TurtleBot context
            full_prompt = prompt + turtlebot_context

            # Call LLM
            print("[LLM] Calling OpenAI API...")
            response = await self.llm_client.generate(full_prompt)
            tree_json = json.loads(response)

            print("[LLM] Tree generated successfully")

            # Display tree structure (compact view)
            self._display_tree(tree_json)

            return tree_json

        except json.JSONDecodeError as e:
            print(f"[Error] Invalid JSON from LLM: {e}")
            return None

        except Exception as e:
            print(f"[Error] Failed to generate tree: {e}")
            return None

    def _display_tree(self, tree_json: Dict[str, Any]):
        """Display tree structure in readable format."""
        tree_str = json.dumps(tree_json, indent=2)
        if len(tree_str) > 500:
            # Show first 500 chars for large trees
            print("Generated tree (truncated):")
            print(tree_str[:500] + "\n...")
        else:
            print("Generated tree:")
            print(tree_str)

    async def send_tree_to_controller(self, tree_json: Dict[str, Any]) -> bool:
        """
        Send generated behavior tree to the TurtleBot controller.

        Args:
            tree_json: Behavior tree in JSON format

        Returns:
            True if sent successfully, False otherwise
        """
        if not self.connected or not self.writer:
            print("[Error] Not connected to controller")
            return False

        try:
            message = {"type": "tree", "data": tree_json}

            # Send as JSON with newline delimiter
            json_str = json.dumps(message) + "\n"
            self.writer.write(json_str.encode())
            await self.writer.drain()

            print("[Sent] Tree sent to TurtleBot controller")
            return True

        except Exception as e:
            print(f"[Error] Failed to send tree: {e}")
            return False

    async def receive_status_updates(self, timeout: float = 60.0):
        """
        Receive and display execution status updates from controller.

        Args:
            timeout: Maximum time to wait for completion (seconds)
        """
        if not self.connected or not self.reader:
            print("[Error] Not connected to controller")
            return

        print("\n[Execution] Waiting for status updates...")
        start_time = asyncio.get_event_loop().time()
        last_tick = 0

        try:
            while True:
                # Check overall timeout
                elapsed = asyncio.get_event_loop().time() - start_time
                if elapsed > timeout:
                    print(f"[Timeout] No completion after {timeout}s")
                    break

                # Read with short timeout
                try:
                    data = await asyncio.wait_for(self.reader.readline(), timeout=1.0)
                except asyncio.TimeoutError:
                    # No data for 1 second, continue waiting
                    continue

                if not data:
                    print("[Connection] Controller disconnected")
                    break

                # Parse and handle message
                try:
                    message = json.loads(data.decode())

                    # Handle different message types
                    if message.get("type") == "status":
                        tick = message.get("tick", 0)
                        status = message.get("status", "UNKNOWN")

                        # Show progress every 10 ticks
                        if tick >= last_tick + 10:
                            print(f"  [Tick {tick:3d}] Status: {status}")
                            last_tick = tick

                        # Check if execution completed
                        if status in ["SUCCESS", "FAILURE"]:
                            self._display_completion(message)
                            break

                    elif message.get("type") == "error":
                        error_msg = message.get("message", "Unknown error")
                        print(f"[Controller Error] {error_msg}")
                        break

                    elif message.get("type") == "log":
                        log_msg = message.get("message", "")
                        print(f"[TurtleBot] {log_msg}")

                except json.JSONDecodeError as e:
                    print(f"[Error] Invalid message from controller: {e}")

        except Exception as e:
            print(f"[Error] Connection error: {e}")

    def _display_completion(self, message: Dict[str, Any]):
        """Display execution completion details."""
        status = message.get("status", "UNKNOWN")
        tick = message.get("tick", 0)

        print("\n[Completed] Execution finished")
        print(f"  Final Status: {status}")
        print(f"  Total Ticks: {tick}")

        # Display final blackboard state
        blackboard = message.get("blackboard", {})
        if blackboard:
            print("  Final State:")
            for key, value in blackboard.items():
                if not key.startswith("_"):  # Skip internal keys
                    print(f"    - {key}: {value}")

    async def execute_command(self, command: str) -> bool:
        """
        Execute a single command: generate tree and send to controller.

        Args:
            command: Natural language command

        Returns:
            True if execution successful, False otherwise
        """
        # Generate behavior tree from command
        tree_json = await self.generate_behavior_tree(command)
        if not tree_json:
            return False

        # Send tree to controller
        if not await self.send_tree_to_controller(tree_json):
            return False

        # Wait for execution to complete
        await self.receive_status_updates()

        return True

    async def interactive_mode(self):
        """Run interactive command-line interface."""
        self._print_banner()

        # Connect to controller
        if not await self.connect_to_controller():
            print("\nPlease start the TurtleBot controller in Webots first.")
            return

        print("\nType commands for TurtleBot3, or 'help' for examples.\n")

        try:
            while True:
                # Get user input
                try:
                    command = input("> ").strip()
                except EOFError:
                    break

                # Handle special commands
                if command.lower() in ["quit", "exit", "q"]:
                    print("Goodbye!")
                    break

                elif command.lower() == "help":
                    self._print_help()
                    continue

                elif command.lower() == "clear":
                    os.system("cls" if os.name == "nt" else "clear")
                    continue

                elif not command:
                    continue

                # Execute robot command
                await self.execute_command(command)

        except KeyboardInterrupt:
            print("\n\nInterrupted by user")

        finally:
            await self.disconnect()

    async def batch_mode(self, commands: List[str]):
        """
        Execute multiple commands in sequence.

        Args:
            commands: List of natural language commands
        """
        print(f"\n[Batch Mode] {len(commands)} commands to execute\n")

        if not await self.connect_to_controller():
            print("Failed to connect to controller")
            return

        try:
            for i, command in enumerate(commands, 1):
                print(f"\n[Command {i}/{len(commands)}] {command}")
                print("-" * 50)

                success = await self.execute_command(command)

                if not success:
                    print(f"[Error] Failed to execute: {command}")
                    continue

                # Brief pause between commands
                if i < len(commands):
                    print("\nWaiting 2 seconds before next command...")
                    await asyncio.sleep(2.0)

        finally:
            await self.disconnect()

    def _print_banner(self):
        """Print welcome banner."""
        print("\n" + "=" * 60)
        print(" TurtleBot3 Burger Controller v1.0")
        print("=" * 60)
        print("\nNatural language control for TurtleBot3 in Webots")
        print("Using Resolution System for dynamic sensor integration")

    def _print_help(self):
        """Print help information."""
        print("\n" + "-" * 40)
        print("TURTLEBOT3 COMMANDS:")
        print("-" * 40)
        print("  go to the nearest box")
        print("  follow the wall for 5 seconds")
        print("  circle around the box")
        print("  move forward slowly")
        print("  explore and find boxes")
        print("  avoid obstacles while moving")
        print("\n" + "-" * 40)
        print("RESOLUTION TEMPLATES USED:")
        print("-" * 40)
        print("  ${obstacle} - LiDAR obstacle detection")
        print("  ${nearest_object} - Object detection")
        print("  ${wall_distance} - Wall following")
        print("  ${safe_speed} - Adaptive speed")
        print("\n" + "-" * 40)
        print("SPECIAL COMMANDS:")
        print("-" * 40)
        print("  help   - Show this help")
        print("  clear  - Clear screen")
        print("  quit   - Exit program")
        print("-" * 40 + "\n")


async def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(
        description="TurtleBot3 Client - Natural language robot control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python turtlebot_client.py                          # Interactive mode
  python turtlebot_client.py -c "go to nearest box"   # Single command
  python turtlebot_client.py -b "move" "turn" "stop"  # Batch mode
        """,
    )

    parser.add_argument(
        "--host",
        default="localhost",
        help="TurtleBot controller host (default: localhost)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9999,
        help="TurtleBot controller port (default: 9999)",
    )
    parser.add_argument("--command", "-c", help="Execute a single command")
    parser.add_argument(
        "--batch", "-b", nargs="+", help="Execute multiple commands in sequence"
    )

    args = parser.parse_args()

    try:
        # Create client
        client = TurtleBotClient(args.host, args.port)

        # Run in appropriate mode
        if args.batch:
            await client.batch_mode(args.batch)
        elif args.command:
            if await client.connect_to_controller():
                await client.execute_command(args.command)
                await client.disconnect()
        else:
            await client.interactive_mode()

    except ValueError as e:
        print(f"\n[Setup Error] {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n[Error] {e}")
        sys.exit(1)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\nShutdown requested")
        sys.exit(0)
