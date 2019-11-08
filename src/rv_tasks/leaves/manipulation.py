from rv_trees.leaves import Leaf
from rv_trees.leaves_ros import (ActionLeaf, PublisherLeaf, ServiceLeaf,
                                  SubscriberLeaf)

from  rv_msgs.msg import MoveToNamedPoseGoal, MoveToNamedPoseResult
from  rv_msgs.srv import GetNamesListRequest, GetNamesListResponse

# TODO
class GetNamedGripperPoses(ServiceLeaf):
    def __init__(self, *args, **kwargs):
        super(GetNamedGripperPoses,
              self).__init__("Get named gripper poses",
                             service_name='/arm/get_named_poses',
                             save=True,
                             *args,
                             **kwargs)

class MoveToNamedGripperPose(ActionLeaf):
    # TODO replace with proper action server call (& move into leaves library)

    def __init__(self, *args, **kwargs):
        super(MoveToNamedGripperPose,
              self).__init__("Move gripper to named pose",
                             action_namespace='/arm/cartesian/named_pose',
                             *args,
                             **kwargs)
