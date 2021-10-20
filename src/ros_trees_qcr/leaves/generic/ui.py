import ros_numpy
import numpy as np
import ros_trees.data_management as dm

from py_trees.composites import Sequence
from ros_trees.leaves import Leaf
from ros_trees.leaves_ros import SubscriberLeaf

from sensor_msgs.msg import Image

from rv_msgs.msg import Click
from rv_msgs.msg import Detection


class GetClick(SubscriberLeaf):

    def __init__(self,
                 name='Checking user input',
                 topic_name='/click',
                 topic_class=Click,
                 result_fn=None,
                 expiry_time=None,
                 *args,
                 **kwargs):
        super(GetClick, self).__init__(name=name,
                                       topic_name=topic_name,
                                       topic_class=topic_class,
                                       expiry_time=expiry_time,
                                       *args,
                                       **kwargs)


class GetMaskFromClick(Sequence):

    def __init__(self,
                 shape=None,
                 offset=None,
                 expiry_time=None,
                 *args,
                 **kwargs):

        def load_fn(leaf):
            observation = leaf._default_load_fn()
            click = dm.get_value('_mask_click')

            _shape = shape if shape and len(shape) == 2 else (100, 100)
            _offset = offset if offset and len(offset) == 2 else (0, 0)

            x = click.x + _offset[0]
            y = click.y + _offset[1]

            if observation.detections:
                valid = []
                for obj in observation.detections:
                    if obj.x_left <= x <= obj.x_left + obj.width and \
                            obj.y_top <= y <= obj.y_top + obj.height:
                        valid.append(obj)

                observation.detections = valid
                print('Valid:', len(valid))

            else:
                image = ros_numpy.numpify(observation.rgb_image)
                depth = ros_numpy.numpify(observation.depth_image)

                obj = Detection()

                obj.class_label = 'misc'

                obj.x_left = min(observation.depth_info.width - 1,
                                 max(0, x - _shape[0] / 2))
                obj.y_top = min(observation.depth_info.height - 1,
                                max(0, y - _shape[1] / 2))

                obj.width = min(_shape[0] / 2, x) + max(
                    0, min(_shape[0] / 2, observation.depth_info.width - x))
                obj.height = min(_shape[1] / 2, y) + max(
                    0, min(_shape[1] / 2, observation.depth_info.height - y))

                obj.cropped_rgb = ros_numpy.msgify(
                    Image,
                    image[obj.y_top:int(obj.y_top + obj.height),
                          obj.x_left:int(obj.x_left + obj.width)],
                    encoding='rgb8')
                obj.cropped_depth = ros_numpy.msgify(
                    Image, depth[obj.y_top:int(obj.y_top + obj.height),
                                 obj.x_left:int(obj.x_left + obj.width)],
                    '32FC1')
                obj.cropped_mask = ros_numpy.msgify(Image,
                                                    np.ones(shape=(obj.height,
                                                                   obj.width),
                                                            dtype=np.uint8),
                                                    encoding='8UC1')

                observation.detections.append(obj)

            return observation

        super(GetMaskFromClick, self).__init__(
            'Get mask from click',
            children=[
                GetClick(save_key='_mask_click',
                         timeout=0.001,
                         expiry_time=expiry_time if expiry_time else None),
                Leaf(name='Generate mask',
                     load_fn=load_fn,
                     save=True,
                     *args,
                     **kwargs)
            ])

