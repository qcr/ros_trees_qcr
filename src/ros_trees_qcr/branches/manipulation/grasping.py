from py_trees.composites import Sequence
from py_trees.decorators import FailureIsSuccess

from ros_trees.leaves import Leaf
from ros_trees.leaves_common import TranslatePose

from ...leaves.manipulation.grasping import ActuateGripper, Grasp
from ...leaves.manipulation.motion import MoveGripperToPose
from ...leaves.manipulation.status import GetEEPose
from ...leaves.visualisation.pose import VisualisePose


class GraspFromObservation(Sequence):

    def __init__(self, gripper_width=None, speed=0.3, *args, **kwargs):
        super(GraspFromObservation, self).__init__(
            'Grasp Object from Observation',
            children=[
                Leaf(name="Extractor Grasp Width",
                     save=True,
                     save_key='grasp_width',
                     result_fn=lambda leaf: gripper_width if gripper_width else
                     leaf.loaded_data.detections[0].grasp_width),
                Leaf(
                    name="Extract Grasp Pose",
                    save=True,
                    save_key='grasp_pose',
                    result_fn=lambda leaf: leaf.loaded_data.detections[0].
                    grasp_pose if len(leaf.loaded_data.detections) else None,
                ),
                VisualisePose(load_key='grasp_pose'),
                GetEEPose('Get EE Pose', save_key='ee_pose'),
                Sequence(
                    name="Execute Grasp",
                    children=[
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
            ],
            *args,
            **kwargs)

