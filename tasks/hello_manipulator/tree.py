from __future__ import print_function
from py_trees.composites import Sequence
from ros_trees.leaves import Leaf
from ros_trees.trees import BehaviourTree
import sys
import time
import rospy

from ros_leaves.leaves.console import Print, SelectItem
from ros_leaves.leaves.manipulation import GetNamedGripperPoses, MoveToNamedGripperPose


def tree():
    BehaviourTree(
        "Hello Manipulator",
        Sequence("Hello manipulator", [
            GetNamedGripperPoses(),
            Print(),
            Leaf("List from Message",
                 result_fn=lambda x: x.loaded_data.names_list,
                 save=True),
            SelectItem(select_text="Pick a pose"),
            MoveToNamedGripperPose()
        ])).run(hz=30, push_to_start=True, log_level='WARN')

if __name__ == '__main__':
    rospy.init_node("tree_hello_manipulator")
    tree()