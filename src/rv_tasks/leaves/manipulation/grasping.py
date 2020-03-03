
from rv_trees.leaves_ros import ActionLeaf
from rv_msgs.msg import ActuateGripperGoal

class Grasp(ActionLeaf):
  def __init__(self, action_namespace='/arm/gripper', *args, **kwargs):
      super(Grasp,
            self).__init__("Grasp object",
                            action_namespace=action_namespace,
                            load_fn=self.load_fn,
                            *args,
                            **kwargs)

  def load_fn(self):
    return ActuateGripperGoal(mode=ActuateGripperGoal.MODE_GRASP, width=0)

class ActuateGripper(ActionLeaf):
  def __init__(self, action_namespace='/arm/gripper', *args, **kwargs):
      super(ActuateGripper,
            self).__init__("Open Gripper",
                            action_namespace=action_namespace,
                            load_fn=self.load_fn,
                            *args,
                            **kwargs)
  
  def load_fn(self):
    grasp_goal = self._default_load_fn()
    grasp_goal.width = grasp_goal.width if grasp_goal.width else 0.08
    grasp_goal.force = 0.0
    grasp_goal.speed = 0.0
    grasp_goal.e_inner = 0.0
    grasp_goal.e_outer = 0.0

    return grasp_goal