import copy
import py_trees
import rv_trees.data_management as dm
from py_trees.composites import Sequence, Selector, Parallel
from py_trees.decorators import FailureIsRunning, Inverter, FailureIsSuccess
from rv_trees.leaves_ros import ServiceLeaf
from rv_trees.leaves import Leaf
from rv_leaves.leaves.generic.console import Print
from rv_leaves.leaves.generic.pose import TranslatePose
from rv_leaves.leaves.manipulation.grasping import ActuateGripper, Grasp, IsGripperClosed
from rv_leaves.leaves.manipulation.motion import MoveToNamedGripperPose, MoveGripperToPose, ServoGripperToPose, SetCartesianPlanningEnabled
from rv_leaves.leaves.manipulation.status import GetEEPose, IsContacting
from rv_leaves.leaves.visualisation.pose import VisualisePose

class GraspFromObservation(Sequence):
  def __init__(self, gripper_width=None, speed=0.3, *args, **kwargs):
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
        VisualisePose(load_key='grasp_pose'),
        GetEEPose('Get EE Pose', save_key='ee_pose'),
        Sequence(name="Execute Grasp", children=[
          ActuateGripper(load_key="grasp_width"),
          TranslatePose(z=0.1, load_key='grasp_pose'),
          VisualisePose(load_key='grasp_pose'),
          MoveGripperToPose(load_key='grasp_pose', speed=speed),
          TranslatePose(z=-0.09, load_key='grasp_pose'),
          VisualisePose(load_key='grasp_pose'),
          MoveGripperToPose(load_key='grasp_pose', speed=speed),
          FailureIsSuccess(Grasp()),
          TranslatePose(z=0.4, load_key='grasp_pose'),
          MoveGripperToPose(load_key='grasp_pose', speed=speed),
          # Inverter(IsGripperClosed())
        ])
      ]
    )