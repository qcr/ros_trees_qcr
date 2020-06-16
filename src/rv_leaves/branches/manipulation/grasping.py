import copy

import rv_trees.data_management as dm
from py_trees.composites import Sequence
from rv_trees.leaves import Leaf
from rv_leaves.leaves.generic.console import Print
from rv_leaves.leaves.generic.pose import TranslatePose
from rv_leaves.leaves.manipulation.grasping import ActuateGripper, Grasp
from rv_leaves.leaves.manipulation.motion import MoveToNamedGripperPose, MoveGripperToPose, ServoGripperToPose
from rv_leaves.leaves.manipulation.status import GetEEPose

class GraspFromObservation(Sequence):
  def __init__(self, gripper_width=None, *args, **kwargs):
    super(GraspFromObservation, self).__init__(
      'Grasp Object from Observation', children=[
        Leaf(
            name="Extractor Grasp Width",
            save=True,
            save_key='grasp_width',
            result_fn=lambda leaf: gripper_width if gripper_width else leaf.loaded_data.detections[0].grasp_width
        ),
        Leaf(
            name="Extract Grasp Pose",
            save=True,
            save_key='grasp_pose',
            result_fn=lambda leaf: leaf.loaded_data.detections[0].grasp_pose if len(leaf.loaded_data.detections) else None, 
        ),
        GetEEPose('Get EE Pose', save_key='ee_pose'),
        Sequence(name="Execute Grasp", children=[
          ActuateGripper(load_key="grasp_width"),
          TranslatePose(z=0.1, load_key='grasp_pose'),
          MoveGripperToPose(load_key='grasp_pose'),
          TranslatePose(z=-0.1, load_key='grasp_pose'),
          Print(load_key='grasp_pose'),
          ServoGripperToPose(load_key='grasp_pose'),
          Grasp(),
          MoveToNamedGripperPose(load_value='ready')
        ])
      ]
    )