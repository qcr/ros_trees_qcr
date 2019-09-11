from __future__ import print_function
from py_trees.composites import Sequence
from qut_trees.leaves import Leaf
from qut_trees.trees import BehaviourTree
import sys
import time

from qut_tasks.leaves.console import Print, SelectItem
from qut_tasks.leaves.manipulation import GetNamedArmPoses, MoveToNamedArmPose



def tree():
    return BehaviourTree(
        "Hello Manipulator",
        Sequence("Hello manipulator", [
            GetNamedArmPoses(),
            SelectItem(select_text="Pick a pose"),
            MoveToNamedGripperPose()
        ]))
