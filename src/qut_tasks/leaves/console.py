from qut_trees.leaves import Leaf


class Print(Leaf):

    def __init__(self, *args, **kwargs):
        super(Print, self).__init__("Print",
                                    result_fn=self._result_fn,
                                    *args,
                                    **kwargs)

    def _result_fn(self):
        print(self.loaded_data)
        return self.loaded_data
