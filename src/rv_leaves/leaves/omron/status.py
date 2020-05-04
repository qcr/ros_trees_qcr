from rv_trees.leaves_ros import SubscriberLeaf
from rv_trees.leaves import Leaf

from rv_omron_driver.msg import Omron

class GetRobotStatus(SubscriberLeaf):
  def __init__(self, topic_name="/robot_status", topic_class=Omron, *args, **kargs):
      super(GetRobotStatus, self).__init__(
      name="GetStatus",
      save=True,
      topic_name=topic_name,
      topic_class=topic_class,
      *args,
      **kargs
    )

class IsVoltageLow(Leaf):
  def __init__(self, topic_name="/robot_status", low_voltage=90, topic_class=Omron, *args, **kargs):
    super(IsVoltageLow, self).__init__(name="IsVoltageLow", save=True, result_fn=self.result_fn,*args, **kargs) 
    self.low_voltage = low_voltage,

  def result_fn(self):
    if self.loaded_data.batteryPercentage > self.low_voltage[0]:
      return False
    else:
      return True