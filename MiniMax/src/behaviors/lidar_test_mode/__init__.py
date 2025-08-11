#!/usr/bin/env python3
"""
LIDAR Test Mode using WORKING servo control from LidarTestBehavior.py
"""

import py_trees
from src.behaviors.utils import make_press_esc_to_exit_behavior
from src.types.RobotModes import RobotMode

# Try to import the improved version with working servo control
try:
    from .ImprovedLidarTest import ImprovedLidarTest
    IMPROVED_AVAILABLE = True
    print("✅ Using ImprovedLidarTest with WORKING servo control from LidarTestBehavior")
except ImportError as e:
    print(f"⚠️ Failed to import ImprovedLidarTest: {e}")
    print("⚠️ Falling back to LidarTestBehavior")
    IMPROVED_AVAILABLE = False

# Fallback to original LidarTestBehavior if needed
if not IMPROVED_AVAILABLE:
    try:
        from .LidarTestBehavior import LidarTestBehavior as ImprovedLidarTest
        print("✅ Using original LidarTestBehavior")
    except ImportError:
        # Last resort - use basic test
        from .StandaloneLidarTest import ExactStandaloneLidarTest as ImprovedLidarTest
        print("⚠️ Using basic StandaloneLidarTest")


def make_lidar_test_sub_tree():
    """
    Create LIDAR Test Mode behavior tree with WORKING servo control
    Uses the PID controller and servo methods from LidarTestBehavior.py
    """
    
    # Create exit behavior - goes to IDLE mode when ESC pressed
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)
    
    # Use the improved test with working servo control
    lidar_behavior = ImprovedLidarTest()
    
    # Create sequence
    lidar_test_sub_tree = py_trees.composites.Sequence(
        name="LIDAR Test with Working Servo",
        memory=False,  # Allow proper exit behavior
        children=[lidar_behavior]
    )
    
    # Create selector for ESC handling
    return py_trees.composites.Selector(
        "LIDAR test mode with working servo control",
        memory=False,  # Allow exit behavior to work
        children=[exit_mode_behavior, lidar_test_sub_tree],
    )


# Export the main function
__all__ = ['make_lidar_test_sub_tree']