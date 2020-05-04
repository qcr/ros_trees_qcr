
from rv_trees.leaves_ros import ActionLeaf

class PlayAudio(ActionLeaf):
  def __init__(self, name='Say Audio', action_namespace='/play_audio', *args, **kwargs):
    super(PlayAudio, self).__init__(
      name=name,
      action_namespace=action_namespace,
      *args,
      **kwargs
    )

