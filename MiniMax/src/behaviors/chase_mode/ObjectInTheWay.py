from ...behaviors.conditions.Condition import Condition
from ...types.DirectionSensorLocation import DirectionSensorLocation


class ObjectInTheWay(Condition):
    """
    Determines if an object is in the way in a given direction.
    Uses distance sensor to check this
    """

    def __init__(self, direction: DirectionSensorLocation, threshold: float):
        """
        Initialises the condition

        arguments:
            - direction: the sensor direction to check for
            - threshold: the threashold to concider an object being in the way
        """
        super().__init__(f"Is object within {threshold} of {direction.value}")
        self.direction = direction
        self.threshold = threshold

    def condition_met(self) -> bool:
        distance_sensor = self.get_robot().distance_sensor

        # check sensor in the direction is below threshold
        distances = distance_sensor.get_reading()
        distance = distances[self.direction]

        return distance >= self.threshold
