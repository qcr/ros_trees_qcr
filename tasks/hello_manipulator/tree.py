from py_trees.composites import Sequence
from qut_trees.trees import BehaviourTree

from qut_tasks.leaves.console import Print


def tree():
    return BehaviourTree(
        "Hello Manipulator",
        Sequence("Hello manipulator", [Print(load_value="Hello manipulator")]))
