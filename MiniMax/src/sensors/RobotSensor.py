from abc import ABC, abstractmethod
from typing import Generic, TypeVar

# the type of readings from this sensor
TReading = TypeVar("TReading")


class RobotSensor(Generic[TReading], ABC):
    """
    Base Class for all Sensors attached to the robot.
    This is an abstract class that cannot be instantiated
    """

    def __init__(self, name) -> None:
        self.name = name

    @abstractmethod
    def get_reading(self) -> TReading:
        """
        Returns a reading for the sensor.
        All behaviour trees nodes will call this function when they want the status of the sensor
        """
        pass
