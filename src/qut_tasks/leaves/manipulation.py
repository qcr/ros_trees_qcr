from qut_trees.leaves import Leaf
from qut_trees.leaves_ros import (ActionLeaf, PublisherLeaf, ServiceLeaf,
                                  SubscriberLeaf)

#Gets a list of the name poses from MoveIt
class GetNamedArmPoses(ServiceLeaf):
     def __init__(self, *args, **kwargs):
        super(GetNamedArmPoses,
              self).__init__("Get named arm poses",
                             service_name='/get_named_arm_poses',
                             save=True,
                             *args,
                             **kwargs)

#Moves arm to named pose using MoveIt
class MoveToNamedArmPose(ActionLeaf):
    def __init__(self, *args, **kwargs):
        super(MoveToNamedArmPose,
              self).__init__("Move arm to named pose",
                             action_namespace = '/cartesian/named_pose',
                             *args,
                             **kwargs)

