
import rospy
from rv_trees.leaves import Leaf

class Noop(Leaf):
  def __init__(self, name="NOOP", *args, **kwargs):
    super(Noop, self).__init__(name, eval_fn=lambda leaf, value: True, *args, **kwargs)

class Wait(Leaf):
  def __init__(self, name='Wait', *args, **kwargs):
    super(Wait, self).__init__(name, *args, **kwargs)

  def _extra_initialise(self):
    self.ts = None

  def _extra_update(self):
    if not self.ts:
      self.ts = rospy.get_time()

  def _is_leaf_done(self):
    if rospy.get_time() - self.ts > self.loaded_data:
      return True

    return False
