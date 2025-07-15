from typing import Tuple
import py_trees
from py_trees.composites import Sequence, Selector
from ...behaviors.MoveBehavior import MoveBehavior
from ...behaviors.SetRobotMode import SetRobotMode
from ...behaviors.chase_mode.AnnounceFoundPerson import AnnounceFoundPerson
from ...behaviors.chase_mode.AnnounceObstacleDetection import (AnnounceObstacleDetection,)
from ...behaviors.chase_mode.HasReachedPerson import HasReachedPerson
from ...behaviors.chase_mode.ObjectInTheWay import ObjectInTheWay
from .TargetOffCenterCondition import (TargetOffCenterCondition,)
from ...behaviors.chase_mode.SelectPerson import SelectPerson
from ...types.DirectionSensorLocation import DirectionSensorLocation
from ...types.MovementDirection import MovementDirection
from ...behaviors.ShowCameraFrame import ShowCameraFrame
from ...behaviors.utils import make_press_esc_to_exit_behavior
from ...types.RobotModes import RobotMode


# def make_avoid_obsticle_sub_tree():
#     def make_avoid(sensor_dir, turn_dir, revert_turn_dir):
#         condition = ObjectInTheWay(sensor_dir, 0.1)
#         stop_moving = MoveBehavior(MovementDirection.NONE)
#         announcement = AnnounceObstacleDetection(sensor_dir)
#         turn = MoveBehavior(turn_dir)
#         move_forward = MoveBehavior(MovementDirection.FORWARDS)
#         revert_turn = MoveBehavior(revert_turn_dir)

#         return Sequence(
#             f"Avoid in {sensor_dir.value} direction",
#             memory=True,
#             children=[
#                 condition,
#                 stop_moving,
#                 announcement,
#                 repeated_turn,
#                 repeated_move,
#                 repeated_revert_turn,
#             ],
#         )

#     sensor_to_turns = {
#         DirectionSensorLocation.LEFT: (MovementDirection.RIGHT, MovementDirection.LEFT),
#         DirectionSensorLocation.RIGHT: (
#             MovementDirection.LEFT,
#             MovementDirection.RIGHT,
#         ),
#     }
#     return Selector(
#         "Avoid sub tree",
#         memory=True,
#         children=[
#             make_avoid(sensor_dir, turn_dir, revert_dir)
#             for sensor_dir, (turn_dir, revert_dir) in sensor_to_turns.items()
#         ],
#     )


def make_target_reached_sub_tree():
    """
    Makes the sub tree for the target being reached
    """

    # conditions to enter this sub tree: target needs to be in the center and close to robot
    target_is_infront = TargetOffCenterCondition(0.4, 0.6)
    target_is_close = HasReachedPerson(600)

    # actions to perform if conditions are met
    stop_moving = MoveBehavior(MovementDirection.NONE)
    announce_found_person = AnnounceFoundPerson()
    exit_to_idle = SetRobotMode(RobotMode.IDLE)

    # target reached sub tree is sequence (performs all the following actions unless one of them fails):
    # - checks target is infront of robot
    # - checks target is close enough
    # - stops moving
    # - announces its found a person
    # - exits to idle mode
    return Sequence(
        "Target Reached sub tree",
        memory=True,
        children=[
            target_is_infront,
            target_is_close,
            stop_moving,
            announce_found_person,
            exit_to_idle,
        ],
    )


def make_move_towards_person_sub_tree():
    """
    Builds the sub tree to move towards a taget
    """

    # makes the move towards target in a direction for an x axis threshold
    def make_behavior(thresholds, direction):
        correct_direction = TargetOffCenterCondition(*thresholds)
        move = MoveBehavior(direction)


        # will move repeatedly in direction if the target is within the x axis threshold
        return Sequence(
            f"Move {direction.value} if target there",
            memory=True,
            children=[correct_direction, move],
        )

    # threshold directions in x axis and corresponding directions to move in
    threshold_directions = {
        MovementDirection.LEFT: (0.0, 0.25),
        MovementDirection.FORWARDS_LEFT: (0.25, 0.4),
        MovementDirection.FORWARDS: (0.4, 0.6),
        MovementDirection.FORWARDS_RIGHT: (0.6, 0.75),
        MovementDirection.RIGHT: (0.75, 1.0),
    }

    # make a move behaviour for each direction
    movement_behaviors = [
        make_behavior(threshold, direction)
        for direction, threshold in threshold_directions.items()
    ]

    # move towards target sub tree is a selector (will run the first condition that passes)
    # Will check if target is within each threshold and move in that direction if it is
    return Selector("Move towards target", memory=True, children=movement_behaviors)


def make_chase_sub_tree():
    """
    Builds the chase target sub tree
    """

    # build sub trees for the chase mode
    target_reached = make_target_reached_sub_tree()
    # avoid_obsticle = make_avoid_obsticle_sub_tree()
    move_towards_target = make_move_towards_person_sub_tree()

    # chase sub tree will run either:
    # - target reached
    # - avoid an obstacle
    # - move towards the target
    chase_action = Selector(
        "Chase sub tree actions",
        memory=True,
        children=[target_reached, move_towards_target],
    )

    # build behaviors that run every time
    select_target = SelectPerson()
    show_frame = ShowCameraFrame()

    # chase sub tree will:
    # - show the current frame
    # - select a target
    # - then run on of the selector actions
    return Sequence(
        "Chase sub tree",
        memory=True,
        children=[show_frame, select_target, chase_action],
    )


def make_chase_mode_sub_tree():
    """
    Returns the chase mode subtree
    """
    # exit to idle behavior
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)

    # the chase sub tree
    chase_sub_tree = make_chase_sub_tree()

    # chase mode behaviors is a Selector either:
    # - go to idle if esc pressed
    # - run the chase sub tree
    return py_trees.composites.Selector(
        "chase mode behavior",
        memory=True,
        children=[exit_mode_behavior, chase_sub_tree],
    )
