import rospy
import tf2_ros
import tf2_geometry_msgs

from tf.transformations import *
from rv_trees.leaves import Leaf

from geometry_msgs.msg import Quaternion

class TranslatePose(Leaf):
  def __init__(self, name="Translate Pose", x=0, y=0, z=0, rx=0, ry=0, rz=0, *args, **kwargs):
    super(TranslatePose, self).__init__(name, result_fn=self.result_fn, *args, **kwargs)
    self.x = x
    self.y = y
    self.z = z
    
    self.rx = rx
    self.ry = ry
    self.rz = rz

  def result_fn(self):
    result = self._default_result_fn()
    
    result.pose.position.x += self.x
    result.pose.position.y += self.y
    result.pose.position.z += self.z

    current = self.quaternion_to_list(result.pose.orientation)
    rotation = quaternion_from_euler(self.rx, self.ry, self.rz)
    
    rotated = quaternion_multiply(current, rotation)

    result.pose.orientation = self.list_to_quaternion(rotated)
    
    return result

  def quaternion_to_list(self, quaternion):
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

  def list_to_quaternion(self, l):
      q = Quaternion()
      q.x = l[0]
      q.y = l[1]
      q.z = l[2]
      q.w = l[3]
      return q

class TransformPose(Leaf):
  def __init__(self, name="Transform Pose", target_frame='base_link',save=True,  *args, **kwargs):
    super(TransformPose, self).__init__(name, result_fn=self.result_fn, save=save, *args, **kwargs)
    self.target_frame = target_frame

    self._tf_buffer = tf2_ros.Buffer()
    self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    rospy.sleep(1)
    
  def result_fn(self):
    tf = self._tf_buffer.lookup_transform(self.target_frame, self.loaded_data.header.frame_id, rospy.Duration(0.0))
    return tf2_geometry_msgs.do_transform_pose(self.loaded_data, tf)