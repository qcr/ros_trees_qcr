from ros_trees.leaves_ros import SyncedSubscriberLeaf
from ros_trees.leaves import Leaf

# 2023-06-27 Deprecated message type - need to revisit inclusion into armer (if applicable)
# from rv_msgs.msg import Observation
from sensor_msgs.msg import Image, CameraInfo


# class GetObservation(SyncedSubscriberLeaf):

#     def __init__(self,
#                  name=None,
#                  topic_names=None,
#                  topic_classes=None,
#                  result_fn=None,
#                  *args,
#                  **kwargs):
#         super(GetObservation, self).__init__(
#             name=name if name else 'Get Observation',
#             topic_names=topic_names if topic_names else [
#                 '/camera/aligned_depth_to_color/camera_info',
#                 '/camera/depth/image_meters_aligned',
#                 '/camera/color/camera_info', '/camera/color/image_rect_color'
#             ],
#             topic_classes=topic_classes
#             if topic_classes else [CameraInfo, Image, CameraInfo, Image],
#             result_fn=result_fn if result_fn else
#             lambda leaf: Observation(depth_info=leaf._cached_data[0],
#                                      depth_image=leaf._cached_data[1],
#                                      rgb_info=leaf._cached_data[2],
#                                      rgb_image=leaf._cached_data[3]),
#             eval_fn=lambda leaf, value: value.depth_info.width and value.
#             rgb_info.width,
#             *args,
#             **kwargs)


class SortObservation(Leaf):

    def __init__(self, name=None, sort_fn=None, *args, **kwargs):
        super(SortObservation, self).__init__(name if name else 'Sort',
                                              result_fn=self.result_fn,
                                              *args,
                                              **kwargs)
        self.sort_fn = (sort_fn if sort_fn else
                        (lambda a, b: 1
                         if a.class_label > b.class_label else -1))

    def result_fn(self):
        self.loaded_data.detections = sorted(self.loaded_data.detections,
                                             self.sort_fn)
        return self.loaded_data
