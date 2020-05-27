
from rv_trees.leaves_ros import ActionLeaf
from rv_msgs.msg import ActuateGripperGoal

class Grasp(ActionLeaf):
  def __init__(self, name=None, action_namespace='/arm/gripper', speed=0, force=0, *args, **kwargs):
      super(Grasp,
            self).__init__(name if name else 'Grasp object',
                            action_namespace=action_namespace,
                            load_fn=self.load_fn,
                            *args,
                            **kwargs)
      self.speed = speed
      self.force = force

  def load_fn(self):
    grasp_goal = self._default_load_fn(auto_generate=False)

    if type(grasp_goal) != ActuateGripperGoal:
      return ActuateGripperGoal(
        mode=ActuateGripperGoal.MODE_GRASP,
        width=grasp_goal if type(grasp_goal) == float else 0.0,
        speed=self.speed,
        force=self.force
      )
    
    return grasp_goal

class ActuateGripper(ActionLeaf):
  def __init__(self, name=None, action_namespace='/arm/gripper', *args, **kwargs):
      super(ActuateGripper,
            self).__init__(name if name else 'Open Gripper',
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