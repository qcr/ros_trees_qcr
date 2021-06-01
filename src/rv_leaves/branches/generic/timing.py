import rospy
import uuid

from py_trees.composites import Sequence
from ros_trees.leaves import Leaf

class TimedRelease(Sequence):
  def __init__(self, name='Timed Release', children=[], expiry_time=1):
    key = 'time-release-{}'.format(uuid.uuid4())

    super(TimedRelease, self).__init__(
      name=name,
      children=[CheckTimer(expiry_time=expiry_time, load_key=key, default_value=True)] + children + [SetTimer(save_key=key)]
    )

class SetTimer(Leaf):
  def __init__(self, name='Set Timer', *args, **kwargs):
    super(SetTimer, self).__init__(
      name=name,
      result_fn=lambda leaf: rospy.get_time(),
      save=True,
      *args,
      **kwargs
    )

class CheckTimer(Leaf):
  def __init__(self, name='Check Timer', expiry_time=1, default_value=False, *args, **kwargs):
    super(CheckTimer, self).__init__(
      name=name, 
      eval_fn=self.eval_fn, 
      *args, 
      **kwargs
    )
    self.default_value = default_value
    self.expiry_time = expiry_time
  
  def eval_fn(self, value):
    if not value:
      return self.default_value
    return rospy.get_time() - value > self.expiry_time