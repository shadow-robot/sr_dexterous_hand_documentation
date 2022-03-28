
class SrRobotCommander(object):

    """
    Base class for hand and arm commanders.
    """

    def __init__(self, name):
        pass

    def set_max_velocity_scaling_factor(self, value):
        """
        Set a scaling factor for optionally reducing the maximum joint velocity.
        Parameters
        ----------
        value : float
            Allowed values are in (0,1]
        """
        self._move_group_commander.set_max_velocity_scaling_factor(value)