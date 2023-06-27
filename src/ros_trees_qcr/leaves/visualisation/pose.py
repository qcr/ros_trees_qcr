import rospy
import tf2_ros

from ros_trees.leaves import Leaf
from geometry_msgs.msg import TransformStamped

# 2023-06-27 Deprecated message type - need to revisit inclusion into armer (if applicable)
# from rv_msgs.msg import Observation


# class VisualisePose(Leaf):

#     def __init__(self,
#                  name=None,
#                  transform_name='visualised',
#                  *args,
#                  **kwargs):
#         super(VisualisePose,
#               self).__init__(name if name else 'Visualise Poses',
#                              result_fn=self.result_fn,
#                              *args,
#                              **kwargs)
#         self.transform_name = transform_name
#         self.broadcaster = None

#     def _extra_setup(self, timeout):
#         self.broadcaster = tf2_ros.TransformBroadcaster()
#         return True

#     def result_fn(self):
#         result = self._default_result_fn()

#         if isinstance(result, Observation):
#             count = {}
#             for idx, detection in enumerate(result.detections):
#                 if detection.class_label not in count:
#                     count[detection.class_label] = 0

#                 t = TransformStamped()

#                 # Prepare broadcast message
#                 t.header.frame_id = detection.grasp_pose.header.frame_id
#                 t.child_frame_id = '{}_{}_{}'.format(
#                     self.transform_name, detection.class_label,
#                     count[detection.class_label])
#                 count[detection.class_label] += 1

#                 # Copy in pose values to transform
#                 t.transform.translation = detection.grasp_pose.pose.position
#                 t.transform.rotation = detection.grasp_pose.pose.orientation

#                 t.header.stamp = rospy.Time.now()

#                 self.broadcaster.sendTransform(t)

#         else:
#             t = TransformStamped()

#             # Prepare broadcast message
#             t.header.frame_id = result.header.frame_id
#             t.child_frame_id = self.transform_name

#             # Copy in pose values to transform
#             t.transform.translation = result.pose.position
#             t.transform.rotation = result.pose.orientation

#             t.header.stamp = rospy.Time.now()

#             self.broadcaster.sendTransform(t)

#         return result

