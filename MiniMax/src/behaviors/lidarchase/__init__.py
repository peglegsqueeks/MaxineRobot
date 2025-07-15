"""
LiDAR Chase Module - Isolated Version
"""

from .UpdateLidarPlot import IsolatedLidarChase

def make_lidar_chase_sub_tree():
    """Create isolated LiDAR chase behavior"""
    return IsolatedLidarChase()

def make_working_lidar_chase_sub_tree():
    """Alternative factory function"""
    return make_lidar_chase_sub_tree()

def make_lidar_chase_behavior_tree():
    """Alternative factory function"""
    return make_lidar_chase_sub_tree()

def make_lidarchase_behavior_tree():
    """Factory function for LIDARCHASE mode"""
    return make_lidar_chase_sub_tree()

__all__ = [
    'make_lidar_chase_sub_tree',
    'make_working_lidar_chase_sub_tree',
    'make_lidar_chase_behavior_tree',
    'make_lidarchase_behavior_tree',
    'IsolatedLidarChase'
]