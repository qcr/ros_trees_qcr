
from rv_trees.leaves_ros import ActionLeaf, ServiceLeaf

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
    def __init__(self, action_namespace='/arm/cartesian/pose', *args, **kwargs):
        super(MoveGripperToPose,
              self).__init__("Move gripper to pose",
                             action_namespace=action_namespace,
                             *args,
                             **kwargs)

class Servo(ActionLeaf):
  def __init__(self, action_namespace='/arm/cartesian/servo_pose', *args, **kwargs):
      super(Servo,
            self).__init__("Servo",
                            action_namespace=action_namespace,
                            *args,
                            **kwargs)
