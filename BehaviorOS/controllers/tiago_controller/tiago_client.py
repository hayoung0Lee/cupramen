"""
BehaviorOS Webots Client
Connects CLI commands to Webots controller via LLM-generated behavior trees.

Usage:
    # Interactive mode
    python tiago_client.py

    # Single command
    python tiago_client.py -c "move forward 5 meters"

    python tiago_client.py -c "move forward 2 seconds slowly"

    # Batch commands
    python tiago_client.py -b "move forward" "turn left" "stop"
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

# Import Webots examples to register them

import controllers.tiago_controller.tiago_prompt as tiago_prompt  # noqa: F401
from behavioros.llm import OpenAIClient
from behavioros.prompts import build_prompt


class WebotsClient:
    """
    Client that generates behavior trees from natural language commands
    and sends them to Webots controller for execution.
    """

    def __init__(self, controller_host: str = "localhost", controller_port: int = 9999):
        """
        Initialize the Webots client.

        Args:
            controller_host: Host address of the Webots controller
            controller_port: Port number of the Webots controller
        """
        self.controller_host = controller_host
        self.controller_port = controller_port
        self.llm_client = OpenAIClient()
        self.reader: Optional[asyncio.StreamReader] = None
        self.writer: Optional[asyncio.StreamWriter] = None
        self.connected = False

    async def connect_to_controller(self) -> bool:
        """
        Establish TCP connection to the Webots controller.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.reader, self.writer = await asyncio.open_connection(
                self.controller_host, self.controller_port
            )
            self.connected = True
            print(
                f"[Connected] Controller at {self.controller_host}:{self.controller_port}"
            )
            return True

        except ConnectionRefusedError:
            print(
                f"[Error] Cannot connect to controller at {self.controller_host}:{self.controller_port}"
            )
            print("Make sure the Webots controller is running first.")
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
            # Build prompt with Webots-specific examples
            prompt = build_prompt(
                command,
                example_tags=["turtlebot"],  # Prioritize turtlebot-tagged examples
                max_examples=3,
            )

            # Add TurtleBot-specific context
            turtlebot_context = """

IMPORTANT - Use ONLY these TurtleBot robot actions:
- move_forward: Move robot forward (uses move_speed, move_duration from initial_values)
- turn_left: Turn robot left (uses turn_speed, turn_duration from initial_values)  
- turn_right: Turn robot right (uses turn_speed, turn_duration from initial_values)
- stop: Stop all motors

Parameter defaults:
- move_speed: 5.0 rad/s
- move_duration: 3.0 seconds
- turn_speed: 3.0 rad/s  
- turn_duration: 2.0 seconds

Return ONLY valid JSON with initial_values and tree structure.
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
        Send generated behavior tree to the Webots controller.

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

            print("[Sent] Tree sent to controller")
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
                        print(f"[Controller] {log_msg}")

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
            print("\nPlease start the Webots controller first.")
            return

        print("\nType commands for the robot, or 'help' for examples.\n")

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
        print(" BehaviorOS Webots Client v1.0")
        print("=" * 60)
        print("\nNatural language to behavior tree converter for Webots")
        print("Connected to OpenAI for LLM generation")

    def _print_help(self):
        """Print help information."""
        print("\n" + "-" * 40)
        print("EXAMPLE COMMANDS:")
        print("-" * 40)
        print("  move forward for 3 seconds")
        print("  rotate 90 degrees clockwise")
        print("  check distance sensor and avoid obstacles")
        print("  move to position (10, 5)")
        print("  patrol between waypoints")
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
        description="BehaviorOS Webots Client - Natural language robot control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python webots_client.py                        # Interactive mode
  python webots_client.py -c "move forward"      # Single command
  python webots_client.py -b "move" "turn" "stop"  # Batch mode
        """,
    )

    parser.add_argument(
        "--host",
        default="localhost",
        help="Webots controller host (default: localhost)",
    )
    parser.add_argument(
        "--port", type=int, default=9999, help="Webots controller port (default: 9999)"
    )
    parser.add_argument("--command", "-c", help="Execute a single command")
    parser.add_argument(
        "--batch", "-b", nargs="+", help="Execute multiple commands in sequence"
    )

    args = parser.parse_args()

    try:
        # Create client
        client = WebotsClient(args.host, args.port)

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
