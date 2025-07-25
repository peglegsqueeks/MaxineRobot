from enum import Enum


class RobotMode(Enum):
    """
    Represents the different robot modes
    """

    # Exit state is used to exit the program
    # once set to exit, the program will exit on the next tick
    EXIT = 0

    # Idle state is the default state of the robot
    # It will randomly run animations and sayings
    IDLE = 1

    # keyboard control mode allows movement using the keyboard
    KEYBOARD_CONTROL = 2

    # will travel towards a person
    CHASE = 3 

    # lidar chase mode 
    LIDARCHASE = 4

    # Discovery mode 
    DISCOVERY = 5

    # Head Turn Mode
    HEADTURN = 6

    # Diagnostic Mode
    DIAGNOSTIC = 7

    # Game Mode
    PLAYGAME = 8

    # Test Lidar Mode
    LIDAR_TEST = 9

    # Head Align Mode
    HEAD_ALIGN = 10