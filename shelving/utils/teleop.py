"""Stage 2: the human operator and the keyboard.

The robot and the operator share one Operator object (blackboard["operator"]):

    mode     "auto"       the Behavior Tree drives the robot
             "requested"  the tree asked for help (reason) and waits for the operator
             "human"      the operator drives; the tree waits in human_control
    want_takeover   the operator pressed T. Honoured at the last step of shelving (in
                    front of the slot): the robot stops there and hands over.
    keys     key presses for human_control, oldest first

Keys come from the terminal (raw mode): the MuJoCo viewer already uses almost every letter
as a rendering shortcut. Keys (tool moves are in the robot's frame: forward = where the
base faces):

    t           take over (any time; the robot hands over in front of the slot)
    w / s       tool forward / back        a / d   tool left / right
    r / f       tool up / down             (lowercase 1 cm, uppercase 2 mm)
    i / k       base forward / back        j / l   base left / right   (5 cm)
    u / o       base turn left / right (5 deg)
    g           grip: suction on (the pad must sit flat on a lid)
    space       release: suction off
    h  (Enter)  hand back to the robot
    ?           help
"""

from __future__ import annotations

import collections
import os
import sys
import threading

HELP = __doc__[__doc__.index("    t           take over"):]
STEP_COARSE, STEP_FINE = 0.01, 0.002
BASE_STEP, TURN_STEP = 0.05, 0.0873
MEANING = {
    "w": "tool forward 1 cm", "s": "tool back 1 cm", "a": "tool left 1 cm", "d": "tool right 1 cm",
    "r": "tool up 1 cm", "f": "tool down 1 cm",
    "W": "tool forward 2 mm", "S": "tool back 2 mm", "A": "tool left 2 mm", "D": "tool right 2 mm",
    "R": "tool up 2 mm", "F": "tool down 2 mm",
    "i": "base forward 5 cm", "k": "base back 5 cm", "j": "base left 5 cm", "l": "base right 5 cm",
    "u": "base turn left 5 deg", "o": "base turn right 5 deg",
    "g": "grip (suction on)", " ": "release (suction off)", "h": "hand back",
}


class Operator:
    """Shared by the tree and the key source; see the module docstring."""

    def __init__(self):
        self.mode = "auto"
        self.reason = ""
        self.want_takeover = False
        self.gave_up = False
        self.keys: collections.deque[str] = collections.deque()
        self.lock = threading.Lock()
        self.status = ""            # one line for the viewer overlay

    # -- called by the key source ------------------------------------------
    def press(self, key: str) -> None:
        with self.lock:
            if key == "t":
                if self.mode == "auto" and not self.want_takeover:
                    self.want_takeover = True
                    print("\n  >> takeover requested: the robot hands over in front of the slot")
                elif self.mode == "requested":
                    self.want_takeover = True
            elif key == "?":
                print(HELP)
            elif self.mode == "human":
                if key in MEANING:
                    # Keep one key waiting at most: a held key (auto-repeat) or keys typed
                    # while the arm moves must not pile up and keep it moving afterwards.
                    if self.keys:
                        self.keys.clear()
                    print(f"  [{'space' if key == ' ' else key}] {MEANING[key]}")
                    self.keys.append(key)
                else:
                    print(f"  [{key!r}] unknown key (? for keys)")
            else:
                print(f"  [{key!r}] ignored: the robot is in control (t to take over)")

    # -- called by the tree ------------------------------------------------
    def request(self, reason: str) -> None:
        if self.mode != "requested":
            self.mode, self.reason = "requested", reason
            print(f"\n  >> HELP REQUESTED: {reason}\n  >> press t (in this terminal) to take over")

    def accept(self) -> bool:
        """True once the operator has taken over (pressed t)."""
        with self.lock:
            if self.want_takeover:
                self.mode, self.want_takeover, self.gave_up = "human", False, False
                self.keys.clear()
                return True
            return False

    def next_key(self) -> str | None:
        with self.lock:
            return self.keys.popleft() if self.keys else None

    def hand_back(self) -> None:
        self.mode, self.reason = "auto", ""

    def available(self) -> bool:
        return True

    def overlay(self) -> tuple[str, str]:
        """(title, detail) for the viewer."""
        if self.mode == "human":
            return "HUMAN CONTROL", self.status or "h: hand back   ?: keys (in the terminal)"
        if self.mode == "requested":
            return "HELP REQUESTED", f"{self.reason}   (press t in the terminal)"
        if self.want_takeover:
            return "AUTO", "takeover pending: robot stops in front of the slot"
        return "AUTO", "t: take over (in the terminal)"


class NoOperator(Operator):
    """Nobody at the keyboard (batch runs): a request for help cannot be answered."""

    def available(self) -> bool:
        return False


class ScriptedOperator(Operator):
    """Replays keys for tests and reproducible runs, one every `every` ticks.

        ScriptedOperator("t,f,f,space,h")       take over at once (pending until the slot)
        ScriptedOperator("wait,t,f,space,h")    take over only when the robot asks

    In human mode the keys go to human_control; "t" (and "wait,t") are pressed in auto or
    requested mode."""

    def __init__(self, script: str, every: int = 40):
        super().__init__()
        names = {"space": " ", "enter": "h"}
        self.script = collections.deque(names.get(k, k) for k in script.split(",") if k)
        self.every, self.n = every, 0

    def tick(self) -> None:
        self.n += 1
        if not self.script or self.n % self.every:
            return
        key = self.script[0]
        if key == "wait":
            if self.mode == "requested":
                self.script.popleft()
        elif key == "t" or self.mode == "human":
            if key != "t" or self.mode != "human":
                self.press(self.script.popleft())


def start_terminal_keys(op: Operator) -> None:
    """Read single key presses from the terminal in a background thread."""
    if not sys.stdin.isatty():
        print("  (stdin is not a terminal: no keyboard operator)")
        return
    import atexit
    import termios
    import tty

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    atexit.register(termios.tcsetattr, fd, termios.TCSADRAIN, saved)

    def loop():
        while True:
            ch = os.read(fd, 1).decode(errors="ignore")
            if ch in ("\n", "\r"):
                ch = "h"
            if ch:
                op.press(ch)

    threading.Thread(target=loop, daemon=True).start()
    print("  keyboard: press t in this terminal to take over, ? for keys")
