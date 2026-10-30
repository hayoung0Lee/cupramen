"""
Decorators that py_trees does not provide.
"""

from __future__ import annotations

import typing
from typing import Any, Dict, Optional

import py_trees
from py_trees.common import Status


class RepeatUntil(py_trees.decorators.Decorator):
    """Run the child again and again until a blackboard key becomes truthy.

    - The key is checked whenever the child is not mid-run: before the first run and
      after each SUCCESS. Truthy -> SUCCESS (the child is not started again).
    - Child FAILURE -> FAILURE.
    - max_iterations (optional) caps the number of child successes; reaching it while
      the key is still false -> FAILURE, so a key that never flips cannot loop forever.
    """

    def __init__(
        self,
        name: str,
        child: py_trees.behaviour.Behaviour,
        blackboard: Dict[str, Any],
        until: str,
        max_iterations: Optional[int] = None,
    ):
        super().__init__(name=name, child=child)
        self.blackboard = blackboard
        self.until = until
        self.max_iterations = max_iterations
        self.iterations = 0

    def initialise(self) -> None:
        self.iterations = 0

    def tick(self) -> typing.Iterator[py_trees.behaviour.Behaviour]:
        # Same skeleton as Decorator.tick(), except that the child is not ticked at all
        # when the loop is already done.
        if self.status != Status.RUNNING:
            self.initialise()
        if self.decorated.status != Status.RUNNING and self.blackboard.get(self.until):
            self.feedback_message = f"'{self.until}' is true after {self.iterations} runs"
            new_status = Status.SUCCESS
        else:
            yield from self.decorated.tick()
            new_status = self.update()
        if new_status != Status.RUNNING:
            self.stop(new_status)
        self.status = new_status
        yield self

    def update(self) -> Status:
        child = self.decorated.status
        if child == Status.FAILURE:
            self.feedback_message = f"child failed on run {self.iterations + 1}"
            return Status.FAILURE
        if child == Status.RUNNING:
            return Status.RUNNING
        self.iterations += 1
        if self.blackboard.get(self.until):
            self.feedback_message = f"'{self.until}' is true after {self.iterations} runs"
            return Status.SUCCESS
        if self.max_iterations is not None and self.iterations >= self.max_iterations:
            self.feedback_message = (
                f"'{self.until}' still false after max_iterations={self.max_iterations}"
            )
            return Status.FAILURE
        # Not done: RUNNING. Next tick the child (now SUCCESS) is started afresh.
        return Status.RUNNING
