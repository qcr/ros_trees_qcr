from __future__ import print_function
from py_trees.composites import Sequence
from rv_trees.leaves import Leaf
from rv_trees.trees import BehaviourTree
import sys
import time

from rv_tasks.leaves.console import Print, SelectItem


class GetNamedGripperPoses(Leaf):
    # TODO replace with proper service call (& move into leaves library)
    NAMED_POSES = ["look_up", "look_down", "look_left", "look_right", "home"]

    def __init__(self, *args, **kwargs):
        super(GetNamedGripperPoses,
              self).__init__("Get named gripper poses",
                             load_value=GetNamedGripperPoses.NAMED_POSES,
                             save=True,
                             *args,
                             **kwargs)


class MoveToNamedGripperPose(Leaf):
    # TODO replace with proper action server call (& move into leaves library)

    def __init__(self, *args, **kwargs):
        super(MoveToNamedGripperPose,
              self).__init__("Move gripper to named pose",
                             result_fn=self._result_fn,
                             *args,
                             **kwargs)

    def _result_fn(self):
        # NOTE this is a dirty blocking hack to show some sort of execution
        # delay. When doing this properly you would be calling an action server
        print("Moving to named gripper pose '%s' ... " % self.loaded_data,
              end='')
        sys.stdout.flush()
        time.sleep(5)
        print("Done.")
        return True


def tree():
    return BehaviourTree(
        "Hello Manipulator",
        Sequence("Hello manipulator", [
            GetNamedGripperPoses(),
            SelectItem(select_text="Pick a pose"),
            MoveToNamedGripperPose()
        ]))
