from rv_trees.leaves import Leaf

class TranslatePose(Leaf):
  def __init__(self, name="Translate Pose", x=0, y=0, z=0, *args, **kwargs):
    super(TranslatePose, self).__init__(name, result_fn=self.result_fn, *args, **kwargs)
    self.x = x
    self.y = y
    self.z = z

  def result_fn(self):
    result = self._default_result_fn()
    
    result.pose.position.x += self.x
    result.pose.position.y += self.y
    result.pose.position.z += self.z

    return result