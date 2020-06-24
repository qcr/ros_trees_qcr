from py_trees.composites import Sequence

from rv_leaves.leaves.manipulation.motion import *
from rv_leaves.leaves.generic.pose import *
from rv_leaves.leaves.generic.console import *
from rv_leaves.leaves.generic.queue import *
from rv_leaves.leaves.visualisation.pose import VisualisePose

class MoveAboveTaggedLocation(Sequence):
  def __init__(self, name='Move Above', location=None, dx=0, dy=0, dz=0.4, speed=0.3, load_key=None, height_fn=None):
    super(MoveAboveTaggedLocation, self).__init__(children=[
      GetTaggedLocationPose(load_value=location if location else None, load_key=load_key if load_key else None, save_key='tag_pose'),
      TransformPose(target_frame='panda_link0', load_key='tag_pose', save_key='tag_pose'),
      TranslatePose(rx=math.pi, rz=math.pi/2, load_key='tag_pose', save_key='tag_pose'),
      TranslatePose(x=dx, y=dy, z=dz, height_fn=height_fn, load_key='tag_pose', save_key='tag_pose'),
      MoveGripperToPose(speed=speed, load_key='tag_pose'),
      GetTaggedLocationPose(load_value=location if location else None, load_key=load_key if load_key else None, save_key='tag_pose'),
      VisualisePose(load_key='tag_pose'),
    ])