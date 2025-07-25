"""
Main behavior tree for LIDARCHASE mode - Ultra-Stable Version
Enhanced with ultra-stable LiDAR system from Working-STABLE-Lidar1.py template
Optimized for maximum stability during robot movement and turning
"""

from src.behaviors.lidarchase.UpdateLidarPlot import UltraStableLidarChase


def make_lidarchase_behavior_tree():
    """
    Create the behavior tree for LIDARCHASE mode with ultra-stable LiDAR system
    Uses proven ultra-stable parameters from Working-STABLE-Lidar1.py for maximum reliability
    """
    return UltraStableLidarChase()


def make_lidar_chase_sub_tree():
    """Alternative name for compatibility - uses ultra-stable LiDAR system"""
    return make_lidarchase_behavior_tree()


def make_lidar_chase_behavior_tree():
    """Alternative name for compatibility - uses ultra-stable LiDAR system"""
    return make_lidarchase_behavior_tree()


def create_lidar_chase_behavior_tree():
    """Alternative name for compatibility - uses ultra-stable LiDAR system"""
    return make_lidarchase_behavior_tree()


def create_simple_lidar_chase_behavior():
    """Alternative name for compatibility - uses ultra-stable LiDAR system"""
    return make_lidarchase_behavior_tree()


# Export the main functions
__all__ = [
    "make_lidarchase_behavior_tree",
    "make_lidar_chase_sub_tree", 
    "make_lidar_chase_behavior_tree",
    "create_lidar_chase_behavior_tree",
    "create_simple_lidar_chase_behavior"
]