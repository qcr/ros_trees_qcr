
from rv_trees.leaves_ros import ActionLeaf
from rv_leaves.leaves.generic.queue import ChooseRandom
from py_trees.composites import Sequence

class PlayAudio(ActionLeaf):
  def __init__(self, name='Say Audio', action_namespace='/play_audio', *args, **kwargs):
    super(PlayAudio, self).__init__(
      name=name,
      action_namespace=action_namespace,
      load_fn=self.load_fn,
      *args,
      **kwargs
    )

  def load_fn(self):
    value = self._default_load_fn()
    print(value)
    return value


class SayPrompt(Sequence):
  def __init__(self, filenames=[]):
    super(SayPrompt, self).__init__(
      name='Say prompt',
      children=[
        ChooseRandom(items=filenames, save_key='filename'),
        PlayAudio(load_key='filename')
      ]
    )

