"""
YouBot Client
Natural language commands to YouBot via LLM-generated behavior trees.

Usage:
    python youbot_client.py -c "pick up the red can"
"""

import asyncio
import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, Optional

from dotenv import load_dotenv

load_dotenv()

# Add behavioros to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

# Import YouBot examples to register them
import controllers.youbot_controller.youbot_prompts as youbot_prompts  # noqa: F401
from behavioros.llm import OpenAIClient
from behavioros.prompts import build_prompt


class YouBotClient:
    """
    Client that generates behavior trees from natural language commands
    and sends them to YouBot controller for execution.
    """

    def __init__(self, controller_host: str = "localhost", controller_port: int = 9999):
        self.controller_host = controller_host
        self.controller_port = controller_port
        self.llm_client = OpenAIClient()
        self.reader: Optional[asyncio.StreamReader] = None
        self.writer: Optional[asyncio.StreamWriter] = None
        self.connected = False

    async def connect_to_controller(self) -> bool:
        """Establish TCP connection to YouBot controller."""
        try:
            self.reader, self.writer = await asyncio.open_connection(
                self.controller_host, self.controller_port
            )
            self.connected = True
            print(
                f"[Connected] YouBot controller at {self.controller_host}:{self.controller_port}"
            )
            return True

        except ConnectionRefusedError:
            print(
                f"[Error] Cannot connect to controller at {self.controller_host}:{self.controller_port}"
            )
            print("Make sure the YouBot controller is running first in Webots.")
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
        """Generate a behavior tree from natural language command using LLM."""
        print(f"\n[LLM] Generating tree for: '{command}'")

        try:
            # Build prompt with YouBot-specific examples
            prompt = build_prompt(
                command,
                example_tags=["youbot"],  # Use YouBot examples
                max_examples=3,
            )

            # Add YouBot-specific context
            youbot_context = """

You are producing a behavior tree for a KUKA YouBot controlled by BehaviorOS.

HARD CONSTRAINTS — DO NOT VIOLATE:
1) Allowed node types ONLY:
   Leaf actions:
     - "pick_can"            (REQUIRES: "nearest_can")
     - "visit_can"           (REQUIRES: "nearest_can")
     - "approach_work_area"  (no extra keys)
     - "approach_box"        (no extra keys)
     - "drop_can"            (REQUIRES: "drop_goal")
   Composites:
     - "sequence"
     - "fallback"
     - "parallel"

2) Resolver templates:
   - "nearest_can" MUST be a template of the form: "${nearest_red_can}" or "${nearest_green_can}"
   - Do NOT inline numeric poses into the tree. Poses are provided via resolvers/blackboard elsewhere.
   - Never output raw object references, code, comments, or prose—only the JSON.

3) OUTPUT FORMAT (STRICT JSON, no extra keys anywhere):
{
  "initial_values": { ... optional ... },
  "tree": {
    "type": "<one of: pick_can | visit_can | approach_work_area | approach_box | drop_can | sequence | fallback>",
    "name": "<non-empty string>",
    "children": [ <tree objects> ]            // required only for composites
    // additional fields by action (and ONLY these):
    // pick_can:        "nearest_can": "${nearest_red_can}" or "${nearest_green_can}"
    // visit_can:       "nearest_can": "${nearest_red_can}" or "${nearest_green_can}"
    // approach_work_area:  (no extra keys)
    // approach_box:        (no extra keys)
    // drop_can:        "drop_goal": "${drop_goal}"
  }
}

4) Mapping for unsupported requests:
   - If the user asks for something outside the allowed set, map it to the CLOSEST allowed actions
     (e.g., visiting a target can -> "visit_can"; picking a colored can -> "pick_can").
   - If no reasonable mapping exists, return a minimal "stop" plan.

5) VALIDATION RULES (must pass):
   - Every node must include "type" and "name".
   - If "type" is "sequence" or "fallback", it MUST include non-empty "children".
   - If "type" is "pick_can" or "visit_can", it MUST include "nearest_can" and its value MUST be a resolver template:
       exactly one of "${nearest_red_can}" or "${nearest_green_can}".
   - Leaf nodes MUST NOT have any keys other than the ones listed above for their action.
   - No comments, trailing text, markdown, or explanations.

Return ONLY the JSON. Do not include any prose.

"""

            # Combine prompt with YouBot context
            full_prompt = prompt + youbot_context

            # Call LLM
            print("[LLM] Calling OpenAI API...")
            response = await self.llm_client.generate(full_prompt)
            tree_json = json.loads(response)

            print("[LLM] Tree generated successfully")

            # Display tree structure
            print("\nGenerated tree:")
            print(json.dumps(tree_json, indent=2))

            return tree_json

        except json.JSONDecodeError as e:
            print(f"[Error] Invalid JSON from LLM: {e}")
            return None

        except Exception as e:
            print(f"[Error] Failed to generate tree: {e}")
            return None

    async def send_tree_to_controller(self, tree_json: Dict[str, Any]) -> bool:
        """Send generated behavior tree to the YouBot controller."""
        if not self.connected or not self.writer:
            print("[Error] Not connected to controller")
            return False

        try:
            message = {"type": "tree", "data": tree_json}

            # Send as JSON with newline delimiter
            json_str = json.dumps(message) + "\n"
            self.writer.write(json_str.encode())
            await self.writer.drain()

            print("[Sent] Tree sent to YouBot controller")
            return True

        except Exception as e:
            print(f"[Error] Failed to send tree: {e}")
            return False

    async def receive_status_updates(self, timeout: float = 120.0):
        """Receive and display execution status updates from controller."""
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
                    continue

                if not data:
                    print("[Connection] Controller disconnected")
                    break

                # Parse and handle message
                try:
                    message = json.loads(data.decode())

                    if message.get("type") == "status":
                        tick = message.get("tick", 0)
                        status = message.get("status", "UNKNOWN")

                        # Show progress
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
                        print(f"[YouBot] {log_msg}")

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
                if not key.startswith("_") and key not in ["robot", "devices"]:
                    print(f"    - {key}: {value}")

    async def execute_command(self, command: str) -> bool:
        """Execute a single command: generate tree and send to controller."""
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
            print("\nPlease start the YouBot controller in Webots first.")
            return

        print("\nType commands for YouBot, or 'help' for examples.\n")

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

    def _print_banner(self):
        """Print welcome banner."""
        print("\n" + "=" * 60)
        print(" YouBot Controller v1.0")
        print("=" * 60)
        print("\nNatural language control for KUKA YouBot")
        print("Using BehaviorOS Resolution System")

    def _print_help(self):
        """Print help information."""
        print("\n" + "-" * 40)
        print("EXAMPLE COMMANDS:")
        print("-" * 40)
        print("  pick up the red can")
        print("  grab the blue can")
        print("  move the can to the left")
        print("  sort all cans by color")
        print("  collect red cans")
        print("\n" + "-" * 40)
        print("RESOLUTION TEMPLATES:")
        print("-" * 40)
        print("  ${nearest_red_can} - Finds red can")
        print("  ${nearest_blue_can} - Finds blue can")
        print("  ${gripper_status} - Gripper state")
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
        description="YouBot Client - Natural language robot control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python youbot_client.py                        # Interactive mode
  python youbot_client.py -c "pick up red can"   # Single command
        """,
    )

    parser.add_argument(
        "--host",
        default="localhost",
        help="YouBot controller host (default: localhost)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9999,
        help="YouBot controller port (default: 9999)",
    )
    parser.add_argument("--command", "-c", help="Execute a single command")

    args = parser.parse_args()

    try:
        # Create client
        client = YouBotClient(args.host, args.port)

        # Run in appropriate mode
        if args.command:
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
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\nShutdown requested")
        sys.exit(0)
