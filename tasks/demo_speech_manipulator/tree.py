#!/usr/bin/env python

from __future__ import print_function
from py_trees.composites import Sequence
from rv_trees.leaves import Leaf
from rv_trees.trees import BehaviourTree
import rv_trees.debugging as debugging
import sys
import time
from rv_trees.leaves_ros import ActionLeaf, SubscriberLeaf, ServiceLeaf
from sensor_msgs.msg import Image

from rv_tasks.leaves.console import Print, SelectItem
from  rv_msgs.msg import ListenGoal, ListenResult
from  rv_msgs.srv import ParseIntentRequest
import rospy


def default_intent(leaf):
    #ret = leaf._default_load_fn()
    ret = ParseIntentRequest(input_text='pickup the red ball')
    ret.intent_type = 'panda'
    print(ret)
    return ret

#Lets make a listen leaf
listen_leaf = ActionLeaf("Listen",
                               action_namespace='/action/listen',
                               save=True,
                               load_value=ListenGoal(timeout_seconds=120.0,  wait_for_wake=True), 
                               )



#Lets declare a subscriber leaf to grab an image
get_image = SubscriberLeaf("Get Image",
                                topic_name='/image',
                                topic_class=Image,
                                save = True)

#Ok lets make an inference service leaf
get_inference = ServiceLeaf("Get inference from string",
                            service_name= '/service/parse_intent',
                            save = True,
                            load_fn = default_intent
                            )

def tree():
    BehaviourTree(
        "speech_move_manipulator",
        Sequence("Listen", [
        #listen_leaf,            #Get some speech
        get_inference,
        Print(),                #Print what was said
                                #

        ])).run(hz=30, push_to_start=True, log_level='WARN')


if __name__ == '__main__':
    rospy.init_node("SpeechMover")
    tree()