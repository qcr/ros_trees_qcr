from py_trees.composites import Selector
from py_trees.decorators import Inverter

class Task(Selector):
  def __init__(self, task_name='', child=None):
    super(Task, self).__init__(
      name=task_name if task_name else 'fallback',
      children=[
        Inverter(IsTaskSelected(task_name=task_name)),
        child
      ]
    )
