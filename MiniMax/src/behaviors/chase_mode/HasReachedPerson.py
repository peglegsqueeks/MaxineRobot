from ...behaviors.conditions.Condition import Condition


import py_trees


class HasReachedPerson(Condition):
    """
    Condition that determines if the robot is in range of a person
    """

    def __init__(self, threshold: float):
        """
        Initialises the condition

        arguments:
            - threshold: the threshold distance to concider the target reached
        """
        super().__init__("Has Reached Person")
        self.blackboard.register_key(
            "TARGET_PERSON", access=py_trees.common.Access.WRITE
        )
        self.threshold = threshold

    def condition_met(self) -> bool:
        if not self.blackboard.exists("TARGET_PERSON"):
            return False
        
        target_object = self.blackboard.get("TARGET_PERSON")

        # when target is first aquired, all values will be 0 for a few ticks, need to ignor this
        if target_object.spatialCoordinates.z== 0 and target_object.spatialCoordinates.x== 0 and target_object.spatialCoordinates.y ==  0:
            return  False

        # reached target if is within threshold
        return target_object.spatialCoordinates.z <= self.threshold
