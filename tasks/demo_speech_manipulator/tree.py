#!/usr/bin/env python

from __future__ import print_function
from py_trees.composites import Sequence, Selector
from rv_trees.leaves import Leaf
from rv_trees.trees import BehaviourTree
import rv_trees.debugging as debugging
import rv_trees.data_management as data_management
import sys
import time
from rv_trees.leaves_ros import ActionLeaf, SubscriberLeaf, ServiceLeaf, PublisherLeaf
from sensor_msgs.msg import Image, CameraInfo

from rv_tasks.leaves.console import Print, SelectItem
from  rv_msgs.msg import ListenGoal, ListenResult, GraspObjectGoal, GraspObjectResult, Objects
from  rv_msgs.srv import ParseIntentRequest, FindObjectsRequest
from panda_speech.srv import DoActionRequest
import rospy



def default_intent(leaf):
    print("using default intent")
    ret = ParseIntentRequest(input_text='can you see the bear') #Comment out once debugging done
    #ret = leaf._default_load_fn()   #Uncoment once debbugging done
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
                                topic_name='/camera/color/image_raw',
                                topic_class=Image,
                                save = True,
                                expiry_time = 10.0 #TODO this wasn't working without exp time. Confirm fixed
                                )

get_rgb_info = SubscriberLeaf("Get RGB Info",
                                topic_name='/camera/color/camera_info',
                                topic_class=CameraInfo,
                                save = True,
                                save_key = 'rgb_info',
                                expiry_time = 10.0 
                                )

#
get_depth_info = SubscriberLeaf("Get Depth Info",
                                topic_name='/camera/depth/camera_info',
                                topic_class=CameraInfo,
                                save = True,
                                save_key = 'depth_info',
                                expiry_time = 10.0 
                                )            

get_depth_image = SubscriberLeaf("Get Depth Image",
                                topic_name='/camera/depth/image_rect_raw',
                                topic_class=Image,
                                save = True,
                                save_key = 'depth_image',
                                expiry_time = 10.0
                                )     

#Ok lets make an inference service leaf
get_inference = ServiceLeaf("Get inference from string",
                            service_name= '/service/parse_intent',
                            save = True,
                            save_key = 'intent_json',
                            load_fn = default_intent
                            )

#Publish the image you are sending to the detector
send_image = PublisherLeaf("Publish Detection image",
                            topic_name= '/tree/detection_in',
                            topic_class=Image
                            )
def setObject(leaf):
  ret=GraspObjectGoal()
  objects=Objects()
  obejcts.depth_image=data_management.get_value('depth_image').depth_image
  objects.depth_image=data_management.get_value('depth_info').depth_info
  ret.objects=objects
  return ret

grasp_leaf = ActionLeaf("Grasp",
                               action_namespace='/grasp_object',
                               save=True,
                               load_fn=setObject
                              # load_value=GraspObjectGoal()
                               )

def rgb_findObjects_load(leaf):
    ret = FindObjectsRequest()
    ret.input_rgb_image = data_management.get_last_value() #TODO this isnt handled @Ben
    #print(ret)
    return ret

#Call yolo leaf to get list of objects
get_objects =  ServiceLeaf("Get list of objects from image",
                            service_name= '/cloudvis/yolo',
                            save = True,
                            load_fn = rgb_findObjects_load
                            )

def doThings_load(leaf):
    print("doing things")
    #ret = leaf._default_load_fn()   #TODO this isn't handled @Ben
    ret = DoActionRequest()
    ret.parsed_json = data_management.get_value('intent_json').intent_json
    #print(data_management.get_last_value().objects)
    ret.visible_objects = data_management.get_last_value().objects #TODO Why does this need .objects but the one above for RGB doesn't
    return ret

#Call the do thing service passing list of objects and json
do_things =  ServiceLeaf("Get instructions on what to do",
                            service_name= '/service/panda_do_things',
                            save = True,
                            load_fn = doThings_load
                            )

say_string = ActionLeaf("Say some text",
                            action_namespace='/action/say_string',
                            save = False
                            )


def tree():
    BehaviourTree(
        "speech_move_manipulator",
        Sequence("Listen", [
       # listen_leaf,            #Get some speech
        get_inference,
        get_image,
        get_rgb_info,
        get_depth_info,
        get_depth_image,
        send_image,
        get_objects,
        #Print(),
        do_things,
        say_string,
        grasp_leaf,
        #Print(),                    
        ])).run(hz=30, push_to_start=True, log_level='WARN')


if __name__ == '__main__':
    rospy.init_node("SpeechMover")
    tree()