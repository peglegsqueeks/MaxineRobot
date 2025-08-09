from .Condition import Condition
from ...types.KeyboardKey import KeyboardKey


class KeyPressedCondition(Condition):
    """
    Condition based on a Key being pressed.
    The condition is true if a certain key is currently being pressed down.
    """

    def __init__(self, key: KeyboardKey):
        """
        Initialises the Condition.

        arguments:
            - key: the key to condition on
            (if key is none condition is true if no keys are being pressed)
        """
        super().__init__(name=f"{key.name} pressed")
        self.target_key = key

    def condition_met(self) -> bool:
        # find keys currently pressed down
        sensor = self.get_robot().keyboard_sensor
        keys_pressed = sensor.get_reading()

        # condition met if key is being pressed
        return self.target_key in keys_pressed
