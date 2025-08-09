from enum import Enum


class HeadMovementDirection(Enum):
    """
    Represents the different directions the robot can move in
    """
    LEFT = "left"
    RIGHT = "right"
    NONE = "none"
