#!/usr/bin/env python3
"""
LIDAR Test Mode - Using FIXED LidarTestBehavior with variance tracking and CSV logging
"""

import py_trees
from src.behaviors.utils import make_press_esc_to_exit_behavior
from src.types.RobotModes import RobotMode

# CRITICAL: Use our FIXED LidarTestBehavior with variance tracking, NOT ImprovedLidarTest
try:
    from .LidarTestBehavior import LidarTestBehavior
    print("✅ Using LidarTestBehavior with VARIANCE TRACKING and CSV LOGGING")
    LIDAR_TEST_AVAILABLE = True
except ImportError as e:
    print(f"❌ Failed to import LidarTestBehavior: {e}")
    LIDAR_TEST_AVAILABLE = False
    
    # Fallback to ImprovedLidarTest if needed (but this doesn't have our fixes)
    try:
        from .ImprovedLidarTest import ImprovedLidarTest as LidarTestBehavior
        print("⚠️ WARNING: Using ImprovedLidarTest - NO variance tracking or CSV logging!")
        LIDAR_TEST_AVAILABLE = True
    except ImportError:
        print("❌ Failed to import ImprovedLidarTest")
        
        # Last resort - basic test
        try:
            from .StandaloneLidarTest import ExactStandaloneLidarTest as LidarTestBehavior
            print("⚠️ WARNING: Using basic StandaloneLidarTest - NO variance tracking!")
            LIDAR_TEST_AVAILABLE = True
        except ImportError:
            print("❌ No LiDAR test behaviors available!")
            LIDAR_TEST_AVAILABLE = False


def make_lidar_test_sub_tree():
    """
    Create LIDAR Test Mode behavior tree with VARIANCE TRACKING and CSV LOGGING
    Uses the FIXED LidarTestBehavior with all our improvements
    """
    
    if not LIDAR_TEST_AVAILABLE:
        print("❌ ERROR: No LiDAR test behavior available!")
        # Return a dummy behavior that immediately exits
        return make_press_esc_to_exit_behavior(RobotMode.IDLE)
    
    # Create exit behavior - goes to IDLE mode when ESC pressed
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)
    
    # Use our FIXED LidarTestBehavior with variance tracking
    lidar_behavior = LidarTestBehavior()
    
    # Create sequence
    lidar_test_sub_tree = py_trees.composites.Sequence(
        name="LIDAR Test with Variance Tracking",
        memory=False,  # Allow proper exit behavior
        children=[lidar_behavior]
    )
    
    # Create selector for ESC handling
    return py_trees.composites.Selector(
        "LIDAR test mode with variance tracking and CSV logging",
        memory=False,  # Allow exit behavior to work
        children=[exit_mode_behavior, lidar_test_sub_tree],
    )


# Export the main function
__all__ = ['make_lidar_test_sub_tree']