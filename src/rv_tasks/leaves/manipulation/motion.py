
from rv_trees.leaves_ros import ActionLeaf, ServiceLeaf
from rv_msgs.msg import MoveToJointPoseGoal

class GetNamedGripperPoses(ServiceLeaf):
    def __init__(self, *args, **kwargs):
        super(GetNamedGripperPoses,
              self).__init__("Get named gripper poses",
                             service_name='/arm/get_named_poses',
                             save=True,
                             *args,
                             **kwargs)


class MoveToNamedGripperPose(ActionLeaf):
    def __init__(self, action_namespace='/arm/cartesian/named_pose', *args, **kwargs):
        super(MoveToNamedGripperPose,
              self).__init__("Move gripper to named pose",
                             action_namespace=action_namespace,
                             *args,
                             **kwargs)

class MoveGripperToPose(ActionLeaf):
    # TODO This should go away if the magic is setup properly
    def __init__(self, action_namespace='/arm/cartesian/pose', speed=0.3, *args, **kwargs):
        super(MoveGripperToPose,
              self).__init__("Move gripper to pose",
                             action_namespace=action_namespace,
                             load_fn=self.load_fn,
                             *args,
                             **kwargs)
        self.speed = speed

    def load_fn(self):
      goal = self._default_load_fn()
      goal.speed = self.speed
      return goal

class Servo(ActionLeaf):
  def __init__(self, action_namespace='/arm/cartesian/servo_pose', *args, **kwargs):
      super(Servo,
            self).__init__("Servo",
                            action_namespace=action_namespace,
                            *args,
                            **kwargs)

class MoveJointsToPose(ActionLeaf):
    # TODO This should go away if the magic is setup properly
    def __init__(self, action_namespace='/arm/joint/pose', *args, **kwargs):
        super(MoveJointsToPose,
              self).__init__("Move joints to pose",
                             action_namespace=action_namespace,
                             load_fn=self.load_fn,
                             *args,
                             **kwargs)

    def load_fn(self):
      data = self._default_load_fn(auto_generate=False)
      if type(data) == MoveToJointPoseGoal:
        return data
      return MoveToJointPoseGoal(joints=data)