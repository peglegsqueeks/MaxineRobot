"""
LiDAR Test Module - Following Working LidarChase Pattern
"""

from .LidarTestBehavior import StableLidarTest

def make_lidar_test_sub_tree():
    """Create stable LiDAR test behavior - EXACT SAME PATTERN AS WORKING LIDARCHASE"""
    return StableLidarTest()

def make_working_lidar_test_sub_tree():
    """Alternative factory function"""
    return make_lidar_test_sub_tree()

def make_lidar_test_behavior_tree():
    """Alternative factory function"""
    return make_lidar_test_sub_tree()

def make_lidartest_behavior_tree():
    """Factory function for LIDAR_TEST mode"""
    return make_lidar_test_sub_tree()

__all__ = [
    'make_lidar_test_sub_tree',
    'make_working_lidar_test_sub_tree',
    'make_lidar_test_behavior_tree',
    'make_lidartest_behavior_tree',
    'StableLidarTest'
]