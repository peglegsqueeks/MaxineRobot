import py_trees
from ..conditions.Condition import Condition


import depthai as dai


class TargetOffCenterCondition(Condition):
    """
    Condition checking if the currently tracked target is within a threshold in the x axis
    """

    def __init__(self, min_threshold: float, max_threshold: float):
        """
        Initialises the condition

        argument:
            - min_threshold: the minimum value in the x axis for the condition to pass
            - max_threshold: the maximum value in the x axis for the condition to pass
        """
        super().__init__(f"{min_threshold} <= Target person offset <= {max_threshold}")
        self.min_threshold = min_threshold
        self.max_threshold = max_threshold

        # register use of the TARGET_PERSON on the blackboard
        self.blackboard.register_key(
            "TARGET_PERSON", access=py_trees.common.Access.WRITE
        )

    def get_target_x_center(self) -> float:
        """
        Returns the middle of the current targets x axis coordinate
        """
        target_person: dai.SpatialImgDetection = self.blackboard.get("TARGET_PERSON")
        x_diff = target_person.xmax - target_person.xmin
        return target_person.xmin + (x_diff / 2)

    def condition_met(self) -> bool:
        target_offset = self.get_target_x_center()

        # if target is below threshold -> condition not met
        if target_offset < self.min_threshold:
            return False

        # if target is above threshold -> condition not met
        if target_offset > self.max_threshold:
            return False

        # otherwise the target is within the threshold
        return True
