
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

class GetTaggedLocations(ServiceLeaf):
    def __init__(self, *args, **kwargs):
        super(GetTaggedLocations,
              self).__init__("Get Tagged Locations",
                             service_name='/location_server/get_locations',
                             save=True,
                             *args,
                             **kwargs)

class GetTaggedLocationPose(ServiceLeaf):
    def __init__(self, *args, **kwargs):
        super(GetTaggedLocationPose,
              self).__init__("Get Tagged Location Pose",
                             service_name='/location_server/get_location',
                             save=True,
                             result_fn=self.result_fn,
                             *args,
                             **kwargs)

    def result_fn(self):
      try:
        result = self._default_result_fn()
        return result.pose_stamped
      except:
        return False

class MoveToNamedGripperPose(ActionLeaf):
    def __init__(self, action_namespace='/arm/cartesian/named_pose', *args, **kwargs):
        super(MoveToNamedGripperPose,
              self).__init__("Move gripper to named pose",
                             action_namespace=action_namespace,
                             *args,
                             **kwargs)

class MoveGripperToPose(ActionLeaf):
    # TODO This should go away if the magic is setup properly
    def __init__(self, name=None, action_namespace='/arm/cartesian/pose', speed=0.3, *args, **kwargs):
        super(MoveGripperToPose,
              self).__init__(name if name else 'Move gripper to pose',
                             action_namespace=action_namespace,
                             load_fn=self.load_fn,
                             *args,
                             **kwargs)
        self.speed = speed

    def load_fn(self):
      goal = self._default_load_fn()
      goal.speed = self.speed
      return goal

class ServoGripperToPose(ActionLeaf):
  def __init__(self, action_namespace='/arm/cartesian/servo_pose', *args, **kwargs):
      super(ServoGripperToPose,
            self).__init__("Servo gripper to pose",
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
      