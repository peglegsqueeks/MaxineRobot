import py_trees

from src.behaviors.HeadTurnBehavior import HeadTurn
from src.types.HeadMovementDirection import HeadMovementDirection
from ...behaviors.MoveBehavior import DecreaseSpeedBehavior, IncreaseSpeedBehavior, MoveBehavior
from ...behaviors.conditions.KeyPressedCondition import KeyPressedCondition
from ...types.KeyboardKey import KeyboardKey
from ...types.MovementDirection import MovementDirection
from ...behaviors.utils import (
    conditional_behavior,
    make_press_esc_to_exit_behavior,
)
from ...types.RobotModes import RobotMode


def make_stop_moving_behavior():
    """
    builds the sub tree for stop moving condition
    """
    movement_keys = [
        KeyboardKey.Q,
        KeyboardKey.W,
        KeyboardKey.E,
        KeyboardKey.A,
        KeyboardKey.S,
        KeyboardKey.D,
        KeyboardKey.Z,
        KeyboardKey.X,
    ]

    # conditions that each key is NOT being pressed
    keys_not_pressed = [
        py_trees.decorators.Inverter(
            f"{key.name} not pressed", child=KeyPressedCondition(key)
        )
        for key in movement_keys
    ]

    stop_moving_behavior = MoveBehavior(MovementDirection.NONE)

    # stop moving if each key is not being pressed
    return py_trees.composites.Sequence(
        "No Movement Keys Pressed",
        memory=True,
        children=[*keys_not_pressed, stop_moving_behavior],
    )


def make_move_in_direction_behaviors():
    """
    Builds the behavior to move in each of the directions.
    Returns a list of behaviors - 1 for each direction
    """
    # mapping between keys and directions
    key_to_direction = {
        KeyboardKey.Q: MovementDirection.FORWARDS_LEFT,
        KeyboardKey.W: MovementDirection.FORWARDS,
        KeyboardKey.E: MovementDirection.FORWARDS_RIGHT,
        KeyboardKey.A: MovementDirection.LEFT,
        KeyboardKey.S: MovementDirection.BACKWARDS,
        KeyboardKey.D: MovementDirection.RIGHT,
        KeyboardKey.Z: MovementDirection.BACKWARDS_LEFT,
        KeyboardKey.X: MovementDirection.BACKWARDS_RIGHT,

    }

    # make 1 behavior per direction
    move_in_direction_behaviors = []
    for key, direction in key_to_direction.items():
        # behavior = move if key is being pressed
        move_behavior = conditional_behavior(
            MoveBehavior(direction), KeyPressedCondition(key)
        )
        move_in_direction_behaviors.append(move_behavior)

    return move_in_direction_behaviors

def make_head_move_behaviours():
    """
    builds behaviour to turn head left right
    """
    
    move_left= conditional_behavior(HeadTurn(HeadMovementDirection.LEFT), KeyPressedCondition(KeyboardKey.O))
    move_right= conditional_behavior(HeadTurn(HeadMovementDirection.RIGHT), KeyPressedCondition(KeyboardKey.P))

    return move_left, move_right



def make_speed_change_behaviors():
    increase = IncreaseSpeedBehavior()
    decrease = DecreaseSpeedBehavior()

    return [conditional_behavior(
           increase, KeyPressedCondition(KeyboardKey.Plus)
        ),conditional_behavior(
            decrease, KeyPressedCondition(KeyboardKey.Minus)
        )]

def make_keyboard_mode_sub_tree():
    """
    Returns the keyboard mode subtree
    """
    # exit to idle behavior
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)

    # behavior for each of the directions
    move_in_direction_behaviors = make_move_in_direction_behaviors()

    # also add a behavior for not moving
    stop_moving_behavior = make_stop_moving_behavior()

    change_speed_bahviors = make_speed_change_behaviors()

    move_behaviours= make_head_move_behaviours()

    # sub tree is a selector (will perform the first behavior that passes)
    # ie will either
    # - exit to idle mode if esc pressed
    # - move in the first direction that is matching a key being pressed
    # - or stop moving if no key is being pressed
    return py_trees.composites.Selector(
        "keyboard mode behavior",
        memory=True,
        children=[
            exit_mode_behavior,
            *change_speed_bahviors,
            *move_in_direction_behaviors,
            *move_behaviours,
            stop_moving_behavior,
        ]
    )
