import py_trees
from py_trees.behaviour import Behaviour
from py_trees.composites import Sequence

from ..behaviors.CountBehavior import CountBehavior

from ..behaviors.conditions.RobotInModeCondition import RobotInModeCondition
from ..behaviors.SetRobotMode import SetRobotMode
from ..behaviors.conditions.KeyPressedCondition import KeyPressedCondition
from ..types.KeyboardKey import KeyboardKey
from ..types.RobotModes import RobotMode
from .conditions.Condition import Condition


def conditional_behavior(behavior: Behaviour, condition: Condition):
    """
    Builds a conditional behavior branch.
    This is a Sequence of Condition, Behavior that will run the behavior if a condition is met
    Or return FAILURE if the condition is not met
    """
    name = f"{behavior.name} if {condition.name}"
    return Sequence(name, memory=True, children=[condition, behavior])


def make_run_every_n_ticks(n: int, *behaviors: Behaviour) -> Sequence:
    count_behavior = CountBehavior(n)

    return Sequence(
        f"Run {behaviors[0].name} every {n} ticks",
        memory=True,
        children=[count_behavior, *behaviors],
    )


def make_press_esc_to_exit_behavior(target_mode: RobotMode):
    """
    A behavior that sets the robot mode if ESC is pressed.
    Used throughout all modes.

    arguments:
        - target_mode: the mode to switch to when ESC is pressed
    """
    # set robot mode if esc is pressed
    return conditional_behavior(
        SetRobotMode(target_mode), KeyPressedCondition(KeyboardKey.ESC)
    )


def make_mode_sub_tree(mode: RobotMode, mode_sub_tree: py_trees.behaviour.Behaviour):
    """
    Builds a sub tree for a particular mode.
    Adds a check to see if robot is in that particular mode.
    If it is in that mode, then follow that sub tree.

    arguments:
        - mode: the mode to be in
        - mode_sub_tree: the sub tree of the mode
    """
    # follow sub tree if robot is in this mode
    return conditional_behavior(mode_sub_tree, RobotInModeCondition(mode))
