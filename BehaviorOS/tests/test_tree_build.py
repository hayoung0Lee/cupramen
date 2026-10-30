"""Building a py_trees tree from JSON (core/tree.py)."""

import py_trees
import pytest

from behavioros.core import TreeExecutor
from behavioros.core.actions import BaseAction, action
from behavioros.core.tree import UnknownNodeTypeError


@action("test_build_ok", "always succeeds")
class AlwaysOk(BaseAction):
    def execute(self):
        return py_trees.common.Status.SUCCESS


def test_known_action_builds_and_runs():
    tree = TreeExecutor({"type": "sequence", "children": [{"type": "test_build_ok"}]}, {})
    assert tree.tick().status == "SUCCESS"


def test_unknown_action_type_is_an_error():
    # A typo used to be replaced by an always-succeeding node, so the tree "succeeded"
    # without ever running the step.
    tree_json = {
        "type": "sequence",
        "children": [
            {"type": "pick_cupp", "name": "Pick cup"},
            {"type": "test_build_ok"},
        ],
    }
    with pytest.raises(UnknownNodeTypeError, match="pick_cupp"):
        TreeExecutor(tree_json, {})


def test_the_blackboard_passed_in_is_the_one_the_tree_uses():
    # An empty dict is falsy, so `blackboard or {}` used to swap it for a new dict and the
    # caller could not see what the tree wrote.
    bb = {}
    tree = TreeExecutor({"type": "test_build_ok"}, bb)
    assert tree.blackboard is bb
    node = AlwaysOk(name="n", blackboard=bb)
    assert node.blackboard is bb
