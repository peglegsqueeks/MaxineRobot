import py_trees

from ...behaviors.SetRobotMode import SetRobotMode
from ...behaviors.conditions.KeyPressedCondition import KeyPressedCondition
from ...types.KeyboardKey import KeyboardKey
from ...behaviors.conditions.RandomProbCondition import RandomProbabilityCondition
from ...behaviors.idle_mode.RandomSayingBehavior import RandomSayingBehavior
from ...behaviors.utils import conditional_behavior
from ...types.RobotModes import RobotMode


def make_idle_mode_sub_tree():
    """
    Returns the idle mode subtree
    """
    keys_to_mode = {
        KeyboardKey.ESC: RobotMode.EXIT,
        KeyboardKey.K: RobotMode.KEYBOARD_CONTROL,
        KeyboardKey.C: RobotMode.CHASE,
        KeyboardKey.L: RobotMode.LIDARCHASE,
        KeyboardKey.R: RobotMode.DISCOVERY,
        KeyboardKey.H: RobotMode.HEADTURN,
        KeyboardKey.D: RobotMode.DIAGNOSTIC,
        KeyboardKey.G: RobotMode.PLAYGAME,
        KeyboardKey.T: RobotMode.LIDAR_TEST,
        KeyboardKey.U: RobotMode.HEAD_ALIGN,
    }
    
    # build behavior to switch to each mode
    switch_to_mode_behaviors = []
    for key, mode in keys_to_mode.items():
        # switch to mode if key is pressed
        switch_mode = conditional_behavior(SetRobotMode(mode), KeyPressedCondition(key))
        switch_to_mode_behaviors.append(switch_mode)

    # random saying behavior
    random_saying_behavior = conditional_behavior(
        RandomSayingBehavior(), RandomProbabilityCondition(0.01)
    )

    # sub tree is a selector (will perform the first behavior that passes)
    # either switch mode because a correct key is pressed
    # or perform a random saying 0.1 % of the time
    return py_trees.composites.Selector(
        "idle mode behavior",
        memory=True,
        children=[*switch_to_mode_behaviors, random_saying_behavior],
    )