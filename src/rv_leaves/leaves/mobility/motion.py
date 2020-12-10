from rv_trees.leaves_ros import ActionLeaf, PublisherLeaf, ServiceLeaf, SubscriberLeaf, SyncedSubscriberLeaf
from geometry_msgs.msg import Twist

class MoveToPose(ActionLeaf):
  def __init__(self, name=None, action_namespace="/move_base", *args, **kargs):
    super(MoveToPose, self).__init__(name if name else "Drive to pose", action_namespace=action_namespace, load_fn=self.load_fn, *args, **kargs)

  def load_fn(self):
    value = self._default_load_fn()
    return value

class Move(PublisherLeaf):
  def __init__(self, name=None, topic_name='/cmd_vel', topic_class=Twist, *args, **kwargs):
    super(Move, self).__init__(name=name if name else 'Move', topic_name=topic_name, topic_class=topic_class, *args, **kwargs)

class Dock(ServiceLeaf):
  def __init__(self,  name=None, *args, **kargs):
    super(Dock, self).__init__(name=name if name else "Dock Robot", service_name="/dock", *args, **kargs)  
