from ros_trees.leaves_ros import ActionLeaf, SubscriberLeaf
from rv_msgs.msg import ActuateGripperGoal

from sensor_msgs.msg import JointState


class Grasp(ActionLeaf):

    def __init__(self,
                 name=None,
                 action_namespace='/arm/gripper',
                 speed=0,
                 force=0,
                 *args,
                 **kwargs):
        super(Grasp, self).__init__(name if name else 'Grasp object',
                                    action_namespace=action_namespace,
                                    load_fn=self.load_fn,
                                    *args,
                                    **kwargs)
        self.speed = speed
        self.force = force

    def load_fn(self):
        grasp_goal = self._default_load_fn(auto_generate=False)

        if type(grasp_goal) != ActuateGripperGoal:
            return ActuateGripperGoal(
                mode=ActuateGripperGoal.MODE_GRASP,
                width=grasp_goal if type(grasp_goal) == float else 0.0,
                speed=self.speed,
                force=self.force)

        return grasp_goal


class ActuateGripper(ActionLeaf):

    def __init__(self,
                 name=None,
                 action_namespace='/arm/gripper',
                 *args,
                 **kwargs):
        super(ActuateGripper, self).__init__(name if name else 'Open Gripper',
                                             action_namespace=action_namespace,
                                             load_fn=self.load_fn,
                                             *args,
                                             **kwargs)

    def load_fn(self):
        grasp_goal = self._default_load_fn()
        grasp_goal.width = grasp_goal.width if grasp_goal.width else 0.08
        grasp_goal.force = 0.0
        grasp_goal.speed = 0.0
        grasp_goal.e_inner = 0.0
        grasp_goal.e_outer = 0.0

        return grasp_goal


class IsGripperOpen(SubscriberLeaf):

    def __init__(self,
                 name=None,
                 topic_name='/arm/gripper/state',
                 topic_class=JointState,
                 save=False,
                 eval_fn=None,
                 *args,
                 **kwargs):
        super(IsGripperOpen,
              self).__init__(name=name if name else 'IsGripperOpen',
                             topic_name=topic_name,
                             topic_class=topic_class,
                             save=save,
                             eval_fn=self.eval_fn if eval_fn is None else
                             self._ensure_bound(eval_fn),
                             *args,
                             **kwargs)

    def eval_fn(self, value):
        return value.position[0] > 0.079 and value.position[1] > 0.079


class IsGripperClosed(IsGripperOpen):

    def __init__(self, name='Is Gripper Closed', *args, **kwargs):
        super(IsGripperClosed, self).__init__(name=name,
                                              eval_fn=self.eval_fn,
                                              *args,
                                              **kwargs)

    def eval_fn(self, value):
        return value.position[0] < 0.01 and value.position[1] < 0.01
