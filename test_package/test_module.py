class SrRobotCommander(object):

    """
    Base class for hand and arm commanders.
    """

    def __init__(self, name):
        pass

    def set_max_velocity_scaling_factor(self, value):
        """
        Sets max velocity scaling factor

        Parameters
        ----------
        value: int
            Value of the scaling factor to be set within range (0,1)
        """
        self._move_group_commander.set_max_velocity_scaling_factor(value)
