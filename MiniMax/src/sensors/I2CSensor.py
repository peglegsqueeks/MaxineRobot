from typing import Generic, TypeVar


# the type of readings from this sensor
TReading = TypeVar("TReading")


class I2CSensor(Generic[TReading]):
    """
    Represents a single I2C sensor
    The I2C update thread will update these continuouly using the set_latest_reading method
    """

    def __init__(self, i2c_adress: int, port_index: int) -> None:
        """
        Initialises the sensor

        arguments:
            - i2c_adress: the i2c adress of the sensor
            - port_index: the index on the ultraborg board this sensor is on
        """
        self.i2c_adress = i2c_adress
        self.port_index = port_index
        self.latest_reading: TReading = None

    def get_latest_reading(self) -> TReading:
        """
        returns the latest reading of the sensor
        """
        return self.latest_reading

    def set_latest_reading(self, reading: TReading):
        """
        Sets the latest reading
        """
        self.latest_reading = reading
