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
from  rv_msgs.msg import ListenGoal, ListenResult, GraspObjectGoal, GraspObjectResult, Objects, Detection, ActuateGripperGoal
from  rv_msgs.srv import ParseIntentRequest, FindObjectsRequest
from rv_tasks.leaves.manipulation import GetNamedGripperPoses, MoveToNamedGripperPose
from panda_speech.srv import DoActionRequest, SayStringRequest
import rospy
import cv2
import cv_bridge
import numpy as np
from std_srvs.srv import Empty



def default_intent(leaf):
    # print("using default intent")
    # ret = ParseIntentRequest(input_text='can you move the bear') #Comment out once debugging done
    ret = leaf._default_load_fn()   #Uncoment once debbugging done
    ret.input_text= data_management.get_value('listen').text_heard
    ret.intent_type = 'panda'
    print(ret)
    return ret

#Lets make a listen leaf
listen_leaf = ActionLeaf("Listen",
                               action_namespace='/action/listen',
                               save_key='listen',
                               load_value=ListenGoal(timeout_seconds=120.0,  wait_for_wake=True), 
                               )



#Lets declare a subscriber leaf to grab an image
get_image = SubscriberLeaf("Get Image",
                                topic_name='/camera/color/image_rect_color',
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
                                topic_name='/camera/aligned_depth_to_color/camera_info',
                                topic_class=CameraInfo,
                                save = True,
                                save_key = 'depth_info',
                                expiry_time = 10.0 
                                )            

get_depth_image = SubscriberLeaf("Get Depth Image",
                                topic_name="/camera/depth/image_meters_aligned",
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

recovery= ServiceLeaf("Recover arm",
                      service_name= '/arm/recover'
)

def graspLoad(leaf):
  print("loading grasp variables")
  ret=GraspObjectGoal()
  objects=Objects()
  objects.depth_image=data_management.get_value('depth_image')
  objects.depth_info=data_management.get_value('depth_info')
  detection=data_management.get_value('doThingsret').object
  if not (detection.x_left==0 and detection.y_top==0):
    rospy.ServiceProxy('/arm/recover', Empty)
    word_string=detection.class_label+" located at "+ str(detection.x_left) + ", "+str(detection.y_top)
    print(word_string)
    bridge = cv_bridge.CvBridge()
    detection.cropped_mask = bridge.cv2_to_imgmsg(np.ones(shape=(objects.depth_image.height, objects.depth_image.width)))
    objects.detections.append(detection)
    ret.objects=objects
    return ret

grasp_leaf = ActionLeaf("Grasp",
                               action_namespace='/grasp_object',
                               save=True,
                               load_fn=graspLoad
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
                            save_key = 'doThingsret',
                            load_fn = doThings_load
                            )

def sayString_load(leaf):
    print("saying things")
    ret = SayStringRequest()
    ret.output_text = data_management.get_value('doThingsret').say
    print(ret.output_text)
    return ret

say_string = ActionLeaf("Say some text",
                            action_namespace='/action/say_string',
                            save = False,
                            # load_value="yes i can see"
                            load_fn=sayString_load
                            )

open_gripper = ActionLeaf("Open gripper",
                            action_namespace='/arm/gripper',
                            save = False,
                            load_value=ActuateGripperGoal(mode=0, width=0.07)
                            )

def tree():
    BehaviourTree(
        "speech_move_manipulator",
        Sequence("Listen", [
        recovery,
        open_gripper,
        listen_leaf,            #Get some speech
        Print(load_value="look_down",save=True),
        MoveToNamedGripperPose(), #@Ben how do i fix this
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
        Print(load_value="look_down",save=True),
        MoveToNamedGripperPose(),
        Print(load_value="bin",save=True),
        MoveToNamedGripperPose(),
        open_gripper,
        #Print(),                    
        Print(load_value="look_down",save=True),
        MoveToNamedGripperPose(), #@Ben how do i fix this
        ])).run(hz=30, push_to_start=True, log_level='WARN')


if __name__ == '__main__':
    rospy.init_node("SpeechMover")
    tree()