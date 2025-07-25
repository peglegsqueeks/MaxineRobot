"""
LiDAR Chase Behavior Tree Integration - Isolated Version
"""

import py_trees
from py_trees.common import Status
from py_trees.composites import Selector, Sequence

from src.behaviors.MaxineBehavior import MaxineBehavior
from src.behaviors.SetRobotMode import SetRobotMode
from src.behaviors.lidarchase.UpdateLidarPlot import IsolatedLidarChase
from src.types.RobotModes import RobotMode


def create_lidar_chase_behavior_tree():
    """
    Create the isolated LiDAR chase behavior tree
    """
    return IsolatedLidarChase()


def create_simple_lidar_chase_behavior():
    """
    Create a simple isolated LiDAR chase behavior
    """
    return IsolatedLidarChase()


def make_lidar_chase_sub_tree():
    """
    Main factory function for creating isolated LiDAR chase behavior tree
    """
    return create_lidar_chase_behavior_tree()


# Export the main functions
__all__ = [
    "create_lidar_chase_behavior_tree",
    "create_simple_lidar_chase_behavior",
    "make_lidar_chase_sub_tree",
    "IsolatedLidarChase"
]