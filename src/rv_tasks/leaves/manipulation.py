import rv_trees.data_management as data_management


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
    # TODO This should go away if the magic is setup properly
    def __init__(self, action_namespace='/arm/cartesian/named_pose', *args, **kwargs):
        super(MoveToNamedGripperPose,
              self).__init__("Move gripper to named pose",
                             action_namespace=action_namespace,
                             *args,
                             **kwargs)

