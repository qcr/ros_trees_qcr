from rv_trees.leaves_ros import SubscriberLeaf
from manipulation_capstone.msg import Click

class GetClick(SubscriberLeaf):
  def __init__(self, name='Checking user input', topic_name='/click', topic_class=Click, result_fn=None, expiry_time=None, *args, **kwargs):
    super(GetClick, self).__init__(
      name=name,
      topic_name=topic_name,
      topic_class=topic_class,
      expiry_time=expiry_time,
      *args,
      **kwargs
    )