from __future__ import print_function
from py_trees.composites import Sequence
from rv_trees.leaves import Leaf
from rv_trees.trees import BehaviourTree
import sys
import time
from rv_trees.leaves_ros import ActionLeaf

from rv_tasks.leaves.console import Print, SelectItem
from  rv_msgs.msg import ListenGoal
import rospy


#Lets make a listen leaf
listen_leaf = ActionLeaf("Listen",
                               action_namespace='/action/listen',
                               save=True,
                               load_value=ListenGoal(timeout_seconds=20.0,  wait_for_wake=True) )


#Get a list of objects from Cloudvis 

#Ok lets make an inference service leaf

def tree():
    BehaviourTree(
        "speech_move_manipulator",
        Sequence("Listen", [
        listen_leaf,
        Print(), 

        ])).run(hz=30, push_to_start=True, log_level='WARN')


if __name__ == '__main__':
    rospy.init_node("listener")
    tree()