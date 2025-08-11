#!/usr/bin/env python3
"""
IMPROVED LiDAR Test Mode with better ESC key handling
"""

import py_trees
from src.behaviors.utils import make_press_esc_to_exit_behavior
from src.types.RobotModes import RobotMode

# Import the working base implementation
from .StandaloneLidarTest import ExactStandaloneLidarTest

# Import the corrected fixed settlement version
try:
    from .FixedSettlementLidarTest import FixedSettlementLidarTest
    FIXED_SETTLEMENT_AVAILABLE = True
    print("✅ CORRECTED FIX: FixedSettlementLidarTest available with proper head tracking")
except ImportError as e:
    print(f"❌ CORRECTED FIX: Failed to import FixedSettlementLidarTest: {e}")
    print("❌ Falling back to original (head tracking will not work)")
    FixedSettlementLidarTest = ExactStandaloneLidarTest
    FIXED_SETTLEMENT_AVAILABLE = False


def make_lidar_test_sub_tree():
    """
    IMPROVED LiDAR Test Mode with proper ESC handling
    
    ESC Behavior:
    - First ESC: Goes from LIDAR_TEST to IDLE mode (correct)
    - Second ESC: From IDLE mode, goes to EXIT mode (exits program)
    
    This is the intended behavior for all modes.
    """
    
    # Create exit behavior - goes to IDLE mode when ESC pressed
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)
    
    # Use corrected FixedSettlementLidarTest with proper thresholds
    if FIXED_SETTLEMENT_AVAILABLE:
        print("🔧 CORRECTED FIX: Using FixedSettlementLidarTest with 6° start threshold")
        lidar_behavior = FixedSettlementLidarTest()
        sequence_name = "CORRECTED LiDAR Test Sequence"
    else:
        print("❌ CORRECTED FIX: FixedSettlementLidarTest not available")
        lidar_behavior = ExactStandaloneLidarTest()
        sequence_name = "Original LiDAR Test Sequence"
    
    # Create sequence with memory=False for proper exit behavior
    lidar_test_sub_tree = py_trees.composites.Sequence(
        name=sequence_name,
        memory=False,  # CRITICAL: memory=False allows proper exit
        children=[lidar_behavior]
    )
    
    # Create selector with memory=False for proper exit behavior
    return py_trees.composites.Selector(
        "CORRECTED lidar test mode behavior",
        memory=False,  # CRITICAL: memory=False allows exit_behavior to work
        children=[exit_mode_behavior, lidar_test_sub_tree],
    )


# Export the main function
__all__ = ['make_lidar_test_sub_tree']