import ros_numpy
import numpy as np
import ros_trees.data_management as dm
import rospy
import yaml

from py_trees.composites import Sequence
from ros_trees.leaves import Leaf
from ros_trees.leaves_ros import SubscriberLeaf, PublisherLeaf

from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool, String

# 20203-06-27 Deprecated message type - need to revisit to include into armer (if needed)
# from rv_msgs.msg import Click
# from rv_msgs.msg import Detection


# class GetClick(SubscriberLeaf):

#     def __init__(self,
#                  name='Checking user input',
#                  topic_name='/click',
#                  topic_class=Click,
#                  result_fn=None,
#                  expiry_time=None,
#                  *args,
#                  **kwargs):
#         super(GetClick, self).__init__(name=name,
#                                        topic_name=topic_name,
#                                        topic_class=topic_class,
#                                        expiry_time=expiry_time,
#                                        *args,
#                                        **kwargs)

# # NOTE: this should be moved to branches (as a sequence)
# class GetMaskFromClick(Sequence):

#     def __init__(self,
#                  shape=None,
#                  offset=None,
#                  expiry_time=None,
#                  *args,
#                  **kwargs):

#         def load_fn(leaf):
#             observation = leaf._default_load_fn()
#             click = dm.get_value('_mask_click')

#             _shape = shape if shape and len(shape) == 2 else (100, 100)
#             _offset = offset if offset and len(offset) == 2 else (0, 0)

#             x = click.x + _offset[0]
#             y = click.y + _offset[1]

#             if observation.detections:
#                 valid = []
#                 for obj in observation.detections:
#                     if obj.x_left <= x <= obj.x_left + obj.width and \
#                             obj.y_top <= y <= obj.y_top + obj.height:
#                         valid.append(obj)

#                 observation.detections = valid
#                 print('Valid:', len(valid))

#             else:
#                 image = ros_numpy.numpify(observation.rgb_image)
#                 depth = ros_numpy.numpify(observation.depth_image)

#                 obj = Detection()

#                 obj.class_label = 'misc'

#                 obj.x_left = min(observation.depth_info.width - 1,
#                                  max(0, x - _shape[0] / 2))
#                 obj.y_top = min(observation.depth_info.height - 1,
#                                 max(0, y - _shape[1] / 2))

#                 obj.width = min(_shape[0] / 2, x) + max(
#                     0, min(_shape[0] / 2, observation.depth_info.width - x))
#                 obj.height = min(_shape[1] / 2, y) + max(
#                     0, min(_shape[1] / 2, observation.depth_info.height - y))

#                 obj.cropped_rgb = ros_numpy.msgify(
#                     Image,
#                     image[obj.y_top:int(obj.y_top + obj.height),
#                           obj.x_left:int(obj.x_left + obj.width)],
#                     encoding='rgb8')
#                 obj.cropped_depth = ros_numpy.msgify(
#                     Image, depth[obj.y_top:int(obj.y_top + obj.height),
#                                  obj.x_left:int(obj.x_left + obj.width)],
#                     '32FC1')
#                 obj.cropped_mask = ros_numpy.msgify(Image,
#                                                     np.ones(shape=(obj.height,
#                                                                    obj.width),
#                                                             dtype=np.uint8),
#                                                     encoding='8UC1')

#                 observation.detections.append(obj)

#             return observation

#         super(GetMaskFromClick, self).__init__(
#             'Get mask from click',
#             children=[
#                 GetClick(save_key='_mask_click',
#                          timeout=0.001,
#                          expiry_time=expiry_time if expiry_time else None),
#                 Leaf(name='Generate mask',
#                      load_fn=load_fn,
#                      save=True,
#                      *args,
#                      **kwargs)
#             ])

######### ADDED BY DG [TESTING]
### --------------------------------  General Leafs ----------------------------------- ###
class CheckAppState(Leaf):
    """
    Gets the application state from a config and confirms if current state is allowed (based on required states)
    Input
        - req_states: a list of states (i.e., 0, 1, 2) defined as acceptable
        - param_name: name of ROS param (if applicable) to config file (defaults to path otherwise)
        - config_path: path to a config with defined states
    """
    def __init__(self,
                save=False,
                req_states=None,
                param_name=None,
                config_path=None,
                *args,**kwargs):
        super().__init__(
            name="Get App State from Config",
            save=save,
            result_fn=self.result_fn,
            eval_fn=self.eval_fn,
            *args, **kwargs
        )
        self.req_states = req_states
        self.param_name = param_name
        self.config_path = config_path

    def result_fn(self):
        # Attempt to get param from server or from provided path
        path = rospy.get_param(self.param_name, self.config_path)
        with open(path, 'r') as handle:
            config = yaml.load(handle, Loader=yaml.SafeLoader)

        app_config = config['app'] if 'app' in config else None

        if app_config == None:
            rospy.logerr(f"[{self.name}] STATE NOT DEFINED! Please check the config/app.yaml file")

        state = app_config[0]['state']

        return state

    def eval_fn(self, value):
        if self.req_states and np.any([value == i for i in self.req_states]):
            return True
        else:
            return False
        
class Wait(Leaf):
    """
    General Wait Leaf (Waits in Seconds)
    Input:
        - duration: in seconds to wait (defaults to 1 second)
    """
    def __init__(self, name='Wait', duration=1):
        super().__init__(
            name=name,
            load_fn=self.load_fn,
            eval_fn=self.eval_fn,
            save=False
        )
        self.duration = duration

    def load_fn(self):
        self.start = rospy.get_time()
        return None

    def eval_fn(self, value):
        return True

    def _is_leaf_done(self):
        return rospy.get_time() - self.start > self.duration
    
### --------------------------------  Publisher Leafs ----------------------------------- ###
class UpdateUIStatus(PublisherLeaf):
    def __init__(self,
                 name=None,
                 topic_name='/ui/status/default',
                 topic_class=String,
                 *args,
                 **kwargs):
        super(UpdateUIStatus, self).__init__(name=name if name else 'Update UI Status',
                                   topic_name=topic_name,
                                   topic_class=topic_class,
                                   *args,
                                   **kwargs)
        
class SetUIReady(PublisherLeaf):
    def __init__(self,
                 name=None,
                 topic_name='/ui/status/ready',
                 topic_class=Bool,
                 *args,
                 **kwargs):
        super(SetUIReady, self).__init__(name=name if name else 'Set UI Ready',
                                   topic_name=topic_name,
                                   topic_class=topic_class,
                                   *args,
                                   **kwargs)

### --------------------------------  Subscriber Leafs ----------------------------------- ###
class GetUICommand(SubscriberLeaf):
    """
    This leaf will query if a UI command is available
    """
    def __init__(self, 
        name="Get UI Start Command", 
        topic_name='/ui/command', 
        topic_class=Int32, 
        expiry_time=1, 
        timeout=0.1,
        value=None,
        *args, **kwargs):

        super().__init__(name=name, 
            topic_name=topic_name, 
            topic_class=topic_class, 
            expiry_time=expiry_time, 
            timeout=timeout, 
            eval_fn=self.eval_fn,
            *args, **kwargs)
        
        if value is None:
            raise Exception('Put a value in!!!')

        self.value = value

    def eval_fn(self, value):
        return value and value.data == self.value