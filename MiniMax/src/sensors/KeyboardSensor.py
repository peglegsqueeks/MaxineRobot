from typing import List
import pygame

from ..types.KeyboardKey import KeyboardKey
from ..sensors.RobotSensor import RobotSensor


class KeyboardReading:
    """
    Snapshot of all the keys currently pressed down.
    We can obtain this from the KeyboardSensor's get_reading() function
    """

    # stores the keys pressed down
    keys_pressed_down: List[KeyboardKey]

    def __init__(self, keys_pressed_down: List[KeyboardKey] = []) -> None:
        """
        Initialises the Keyboard reading class

        arguments:
            - keys_pressed_down: the list of keys pressed down
        """
        self.keys_pressed_down = keys_pressed_down

    def __contains__(self, key: KeyboardKey):
        """
        Checks if a key is currently pressed down.

        This function allows us to use the 'in' keyword in python
        Ie:
            if KeyboardKey.A in keyboard_readings:
        """
        if type(key) != KeyboardKey:
            return False

        return key.value in self.keys_pressed_down

    def __len__(self):
        """
        Returns the number of keys being pressed down
        """
        return len(self.keys_pressed_down)


class KeyboardSensor(RobotSensor[KeyboardReading]):
    """
    The sensor for the Keyboard connected to the robot.

    Note that due to limitiations with pygame, this class CANNOT be run in a separate thread
    """

    def __init__(self) -> None:
        """
        Initialises the Keyboard Sensor
        """
        super().__init__("Keyboard Sensor")
        # initialise readings to empty reading
        self.current_reading = KeyboardReading()

    def update_reading(self):
        """
        Will update the current readings by:
            - removing any keys that have been 'keyed-up' since we last checked
            - adding keys that have been 'keyed-down' since we last checked
        """
        # copy previous keys
        pressed_down_keys = self.current_reading.keys_pressed_down.copy()

        # remove keys from list when KEYUP event happens
        key_up_events = pygame.event.get(eventtype=[pygame.KEYUP])
        for event in key_up_events:
            if event.key in pressed_down_keys:
                pressed_down_keys.remove(event.key)

        # add keys in list when KEYDOWN event happens
        key_down_events = pygame.event.get(eventtype=[pygame.KEYDOWN])
        for event in key_down_events:
            if event.key not in pressed_down_keys:
                pressed_down_keys.append(event.key)

        # set current reading to be new reading
        self.current_reading = KeyboardReading(pressed_down_keys)

    def get_reading(self) -> KeyboardReading:
        """
        Returns the current state of the keyboard readings.
        Returns a KeyboardReading instance which contains all the keys currently being pressed down.
        """
        self.update_reading()
        return self.current_reading

    def flush_readings(self) -> None:
        """
        Resets the keyboard reading to have no keys pressed.
        This seems to be necesarry after switching modes because of pygame limitations.
        """
        self.current_reading = KeyboardReading()
