
from py_trees.composites import Selector
from rv_trees.leaves_ros import SubscriberLeaf

from std_msgs.msg import String

class Task(Selector):
  def __init__(self, task_name='', child=None):
    super(Task, self).__init__(
      name=task_name if task_name else 'fallback',
      children=[
        Inverter(IsTaskSelected(task_name=task_name)),
        child
      ]
    )

class IsTaskSelected(SubscriberLeaf):
  def __init__(self, task_name, topic_name='/task', topic_class=String, *args, **kwargs):
    super(IsTaskSelected,
          self).__init__('Task Selected'.format(task_name),
                          topic_name=topic_name, 
                          topic_class=topic_class,
                          eval_fn=self.eval_fn, 
                          *args, 
                          **kwargs)
    self.task_name = task_name

  def eval_fn(self, value):
    return (not value and self.task_name == '') or (value and value.data == self.task_name)
    