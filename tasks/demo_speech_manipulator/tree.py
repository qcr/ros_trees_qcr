from __future__ import print_function
from py_trees.composites import Sequence
from rv_trees.leaves import Leaf
from rv_trees.trees import BehaviourTree
import sys
import time
from rv_trees.leaves_ros import ActionLeaf, SubscriberLeaf
from sensor_msgs.msg import 

from rv_tasks.leaves.console import Print, SelectItem
from  rv_msgs.msg import ListenGoal
import rospy


#Lets make a listen leaf
listen_leaf = ActionLeaf("Listen",
                               action_namespace='/action/listen',
                               save=True,
                               load_value=ListenGoal(timeout_seconds=20.0,  wait_for_wake=True) )


#Lets declare a subscriber leaf to grab an image
get_image = SubscriberLeaf("Get Image",
                                topic_name='/unavailable',
                                 topic_class=None,
                                 debug=debugging.DebugMode.INSTANT_FAILURE)

#Ok lets make an inference service leaf

def tree():
    BehaviourTree(
        "speech_move_manipulator",
        Sequence("Listen", [
        listen_leaf,            #Get some speech
        Print(),                #Print what was said
                                #

        ])).run(hz=30, push_to_start=True, log_level='WARN')


if __name__ == '__main__':
    rospy.init_node("listener")
    tree()