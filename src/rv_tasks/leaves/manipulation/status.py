
from rv_trees.leaves_ros import SubscriberLeaf

from rv_msgs.msg import ManipulatorState
from franka_msgs.msg import FrankaState

class IsErrored(SubscriberLeaf):
  def __init__(self, name='Checking arm for errors', topic_name='/arm/state', topic_class=ManipulatorState, result_fn=None, *args, **kwargs):
    super(IsErrored, self).__init__(
      name=name,
      topic_name=topic_name,
      topic_class=topic_class,
      result_fn=result_fn if result_fn else lambda leaf: leaf._cached_data.errors > 0,
      *args,
      **kwargs
    )

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
    
    error = math.fabs(self.tau_J[2] - self._cached_data.tau_J[2]) #[math.fabs(self.tau_J[idx] - self._cached_data.tau_J[idx]) for idx in range(len(self.tau_J))]
    
    if error > 0.2:
      self.tau_J = None
      return True
    
    return False