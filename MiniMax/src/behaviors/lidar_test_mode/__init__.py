import py_trees
from src.behaviors.SetRobotMode import SetRobotMode
from src.behaviors.lidar_test_mode.LidarTestBehavior import LidarTestBehavior
from src.behaviors.utils import make_press_esc_to_exit_behavior
from src.types.RobotModes import RobotMode
from src.behaviors.CountBehavior import CountBehavior
from src.behaviors.chase_mode.SelectPerson import SelectPerson
from typing import Tuple
from py_trees.composites import Sequence, Selector

def make_lidar_test_sub_tree():
    """
    Creates the behavior tree for Lidar Test Mode.
    This mode performs lidar scanning and obstacle plotting without threading.
    """
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)

    lidar_test_behavior = LidarTestBehavior()

    # Lidar test sequence: run the lidar test behavior
    lidar_test_sequence = Sequence("Lidar Test sequence", 
                                   memory=True,
                                   children=[lidar_test_behavior, ]
                                   )

    # Lidar Test mode:
    #   Exit if esc pressed 
    #   OR 
    #   run lidar test mode
    #       - Perform lidar scanning and plotting
    return Selector("Lidar Test Mode", memory=True, children=[exit_mode_behavior, lidar_test_sequence])