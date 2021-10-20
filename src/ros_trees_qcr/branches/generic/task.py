from py_trees.composites import Selector, Sequence
from py_trees.decorators import Inverter

from ros_trees.leaves import Leaf
from ros_trees.leaves_common import PeekItem, PopItem, PushItem

from ...leaves.generic.task import IsTaskSelected


class Task(Selector):

    def __init__(self, task_name='', child=None, before=[]):
        super(Task, self).__init__(
            name=task_name if task_name else 'fallback',
            children=[
                Inverter(IsTaskSelected(task_name=task_name)),
                Inverter(
                    Sequence(
                        'Before',
                        children=[
                            PeekItem(key='_last_task',
                                     eval_fn=lambda leaf, value: True),
                            Selector(children=[
                                Leaf(name='Check last task',
                                     eval_fn=lambda leaf, value: value ==
                                     task_name),
                                Sequence(children=[
                                    PopItem(key='_last_task',
                                            eval_fn=lambda leaf, value: True)
                                ] + ([before] if before else []) + [
                                    PushItem(key='_last_task',
                                             load_value=task_name)
                                ])
                            ])
                        ])), child
            ])


class ResetTask(PopItem):

    def __init__(self):
        super(ResetTask, self).__init__(name='Reset Task',
                                        key='_last_task',
                                        save=False)
