
from rv_trees.leaves import Leaf

class Pass(Leaf):
  def __init__(self, name="NOOP", *args, **kwargs):
    super(Pass, self).__init__(name, eval_fn=lambda leaf, value: True, *args, **kwargs)
