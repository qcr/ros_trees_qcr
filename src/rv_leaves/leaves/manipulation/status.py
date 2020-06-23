import rospy
import math

from rv_trees.leaves_ros import SubscriberLeaf, ServiceLeaf

from rv_msgs.msg import ManipulatorState
from franka_msgs.msg import FrankaState

class IsErrored(SubscriberLeaf):
  def __init__(self, name='Checking arm for errors', timeout=2, topic_name='/arm/state', topic_class=ManipulatorState, result_fn=None, *args, **kwargs):
    super(IsErrored, self).__init__(
      name=name,
      topic_name=topic_name,
      topic_class=topic_class,
      result_fn=result_fn if result_fn else self.result_fn,
      *args,
      **kwargs
    )
    self.timeout = timeout
    self.ts = None

  def result_fn(self):
    if self._cached_data.errors > 0:
      if self.ts == None:
        self.ts = rospy.get_time()
      
      return rospy.get_time() - self.ts > self.timeout
    
    self.ts = None
    return False

class IsContacting(SubscriberLeaf):
  def __init__(self, name='Checking arm for errors', topic_name='/franka_state_controller/franka_states', topic_class=FrankaState, result_fn=None, *args, **kwargs):
    super(IsContacting, self).__init__(
      name=name,
      topic_name=topic_name,
      topic_class=topic_class,
      result_fn=self.result_fn,
      *args,
      **kwargs
    )
    self.tau_J = None

  def result_fn(self):
    if not self._cached_data:
      return None
    
    if not self.tau_J:
      self.tau_J = self._cached_data.tau_J
      return None
    
    error = math.fabs(self.tau_J[2] - self._cached_data.tau_J[2])
    
    if error > 0.2:
      self.tau_J = None
      return True
    
    return False

class GetEEPose(SubscriberLeaf):
  def __init__(self, name='Get EE Pose', topic_name='/arm/state', topic_class=ManipulatorState, result_fn=None, *args, **kwargs):
    super(GetEEPose, self).__init__(
      name=name,
      topic_name=topic_name,
      topic_class=topic_class,
      result_fn=lambda leaf: leaf._cached_data.ee_pose,
      *args,
      **kwargs
    )

class GetJointPoses(SubscriberLeaf):
  def __init__(self, name='Get EE Pose', topic_name='/arm/state', topic_class=ManipulatorState, result_fn=None, *args, **kwargs):
    super(GetJointPoses, self).__init__(
      name=name,
      topic_name=topic_name,
      topic_class=topic_class,
      result_fn=lambda leaf: leaf._cached_data.joint_poses,
      *args,
      **kwargs
    )

class SetCartesianImpedance(ServiceLeaf):
    def __init__(self, name='Set Impedance', *args, **kwargs):
        super(SetCartesianImpedance,
              self).__init__(name=name,
                             service_name='/arm/set_cartesian_impedance',
                             *args,
                             **kwargs)