#!/usr/bin/env python3
"""
LiDAR Test Mode Package - Using Optimized Detection System
Located at: src/behaviors/lidar_test_mode/__init__.py
"""

import py_trees
from src.behaviors.utils import make_press_esc_to_exit_behavior
from src.types.RobotModes import RobotMode

# Try to import the optimized version first
LIDAR_TEST_AVAILABLE = False
LidarTestBehavior = None

try:
    # Try optimized version with <37px variance detection
    from .OptimizedLidarTestMode import LidarTestBehavior
    print("✅ Using OptimizedLidarTestMode with <37px variance detection")
    LIDAR_TEST_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ OptimizedLidarTestMode not available: {e}")
    
    # Try standard version with variance tracking
    try:
        from .LidarTestBehavior import StableLidarTest as LidarTestBehavior
        print("✅ Using standard LidarTestBehavior with variance tracking")
        LIDAR_TEST_AVAILABLE = True
    except ImportError as e:
        print(f"❌ Failed to import LidarTestBehavior: {e}")
        
        # Try ImprovedLidarTest as fallback
        try:
            from .ImprovedLidarTest import ImprovedLidarTest as LidarTestBehavior
            print("⚠️ Using ImprovedLidarTest (fallback mode)")
            LIDAR_TEST_AVAILABLE = True
        except ImportError:
            print("❌ Failed to import ImprovedLidarTest")
            
            # Last resort - standalone test
            try:
                from .StandaloneLidarTest import ExactStandaloneLidarTest as LidarTestBehavior
                print("⚠️ Using StandaloneLidarTest (basic mode)")
                LIDAR_TEST_AVAILABLE = True
            except ImportError:
                print("❌ No LiDAR test behaviors available!")
                LIDAR_TEST_AVAILABLE = False


def make_lidar_test_sub_tree():
    """
    Create LiDAR Test Mode behavior tree
    
    CRITICAL py_trees structure (per project instruction #9):
    - Uses Selector with exit_mode_behavior FIRST
    - memory=False on BOTH Selector and Sequence (CRITICAL!)
    - The behavior itself does NOT handle ESC
    - Exit behavior checks for ESC and returns to IDLE
    
    Priority order for behaviors:
    1. OptimizedLidarTestMode with <37px variance (best)
    2. Standard LidarTestBehavior with variance tracking
    3. ImprovedLidarTest (fallback)
    4. StandaloneLidarTest (basic)
    """
    
    if not LIDAR_TEST_AVAILABLE or LidarTestBehavior is None:
        print("❌ ERROR: No LiDAR test behavior available!")
        # Return a dummy behavior that immediately exits
        return make_press_esc_to_exit_behavior(RobotMode.IDLE)
    
    # Create exit behavior for ESC handling - this MUST be first in Selector
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)
    
    # Create the LiDAR test behavior
    try:
        lidar_behavior = LidarTestBehavior()
    except Exception as e:
        print(f"❌ Failed to create LiDAR test behavior: {e}")
        return make_press_esc_to_exit_behavior(RobotMode.IDLE)
    
    # Create sequence with memory=False (CRITICAL - per instruction #9)
    lidar_test_sequence = py_trees.composites.Sequence(
        name="LiDAR Test Sequence",
        memory=False,  # CRITICAL: memory=False allows exit behavior to be checked
        children=[lidar_behavior]
    )
    
    # Create selector with exit behavior FIRST and memory=False
    return py_trees.composites.Selector(
        "lidar test mode behavior",
        memory=False,  # CRITICAL: memory=False allows ESC checking
        children=[exit_mode_behavior, lidar_test_sequence],  # exit FIRST!
    )


# Export the main function
__all__ = ['make_lidar_test_sub_tree', 'LidarTestBehavior', 'LIDAR_TEST_AVAILABLE']