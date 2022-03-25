class SrRobotCommander(object):

    """
    Base class for hand and arm commanders.
    """

    def __init__(self, name):
        pass

    def set_max_velocity_scaling_factor(self, value):
        """Does some stuff

        Parameters
        ----------
        foo : int, float, str, or tf.Tensor
            The foo to bar, which has a really really, reeeeeeeeeeeeeeeeally
            unnecessarily long multiline description.
        bar : str
            Bar to use on foo
        baz : float
            Baz to frobnicate

        Returns
        -------
        float
            The frobnicated baz
        """
        self._move_group_commander.set_max_velocity_scaling_factor(value)
