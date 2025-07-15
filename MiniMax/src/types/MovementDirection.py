from enum import Enum


class MovementDirection(Enum):
    """
    Represents the different directions the robot can move in
    """

    FORWARDS = "forwards"
    BACKWARDS = "backwards"

    LEFT = "left"
    RIGHT = "right"

    FORWARDS_LEFT = "forwards_left"
    FORWARDS_RIGHT = "forwards_right"

    BACKWARDS_LEFT = "backwards_left"
    BACKWARDS_RIGHT = "backwards_right"

    NONE = "none"
