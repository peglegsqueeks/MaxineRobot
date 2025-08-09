from .Condition import Condition
from ...types.RobotModes import RobotMode


class RobotInModeCondition(Condition):
    """
    Condition based on the robot's current mode
    Will be true when the robot is in a specific mode
    """

    def __init__(self, mode: RobotMode):
        """
        Initialises the condition

        arguments:
            - mode: the mode to target
        """
        super().__init__(f"In {mode.name} mode")
        self.target_mode = mode

    def condition_met(self) -> bool:
        # condition is met if robots current mode is the target mode
        current_mode = self.get_robot().current_mode
        return current_mode == self.target_mode
