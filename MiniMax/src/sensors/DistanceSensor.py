from typing import Dict
from ..sensors.RobotSensor import RobotSensor
from ..types.DirectionSensorLocation import DirectionSensorLocation


class DistanceReading:
    """
    Represents a reading from multiple distance sensors in different directions
    NOTE: This is now a placeholder since ultrasonic sensors have been removed
    """

    def __init__(self, readings: Dict[DirectionSensorLocation, float] = None) -> None:
        # Default to empty readings since ultrasonic sensors are removed
        self.readings = readings or {}

    def __str__(self):
        if not self.readings:
            return "No ultrasonic sensors available"
        return ", ".join(
            [
                f"{direction.value}: {round(value, 3) if value is not None else 'none'}"
                for direction, value in self.readings.items()
            ]
        )

    def __get_item__(self, direction):
        if direction not in self.readings:
            return None  # Return None instead of raising error
        return self.readings[direction]


class DistanceSensor(RobotSensor[DistanceReading]):
    """
    The distance sensor - simplified since ultrasonic sensors have been removed
    This class is kept for compatibility but returns empty readings
    """

    def __init__(self, sensors: Dict[DirectionSensorLocation, any] = None) -> None:
        """
        Initializes the sensor (now simplified since ultrasonic sensors removed)

        arguments:
            - sensors: Kept for compatibility but not used (can be None)
        """
        super().__init__("Distance Sensor (Disabled - No Ultrasonic Sensors)")
        # No sensors or polling threads since ultrasonic sensors are removed

    def get_reading(self) -> DistanceReading:
        """
        Returns empty reading since ultrasonic sensors have been removed
        """
        # Return empty reading - no ultrasonic sensors available
        return DistanceReading({})