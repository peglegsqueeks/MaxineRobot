#!/usr/bin/env python3
"""
LiDAR Test Mode Package
Default mode now uses OptimizedLidarTestMode with <37px variance detection
"""

import py_trees
from src.behaviors.utils import make_press_esc_to_exit_behavior
from src.types.RobotModes import RobotMode

# Import existing behaviors for backwards compatibility
from .LidarTestBehavior import (
    StableLidarTest,
    LidarTestBehavior,
    UltraStableLidarSystem,
    EnhancedFacialAnimationRestorer
)

# Import OPTIMIZED behavior with standalone detection integration
try:
    from .OptimizedLidarTestMode import OptimizedLidarTestBehavior
    optimized_available = True
except ImportError:
    optimized_available = False
    print("⚠️ OptimizedLidarTestMode not available, using standard mode")


def make_lidar_test_sub_tree():
    """
    Create OPTIMIZED LiDAR test mode with <37px variance detection
    Uses the same detection system as detection_consistency_test.py
    
    Features:
    - Detection system from standalone test with <37px variance
    - Full LiDAR obstacle detection and display
    - Head tracking with conservative settings
    - CSV logging for consistency analysis
    - Proper py_trees structure with exit behavior
    """
    # Create exit behavior for ESC handling
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)
    
    # Use OPTIMIZED behavior if available, otherwise fallback
    if optimized_available:
        lidar_test_behavior = OptimizedLidarTestBehavior()
    else:
        lidar_test_behavior = StableLidarTest()
    
    # Create sequence with proper memory=False for exit checking
    lidar_test_sequence = py_trees.composites.Sequence(
        name="Optimized LiDAR Test Sequence",
        memory=False,  # CRITICAL: Allows exit behavior to be checked
        children=[lidar_test_behavior]
    )
    
    # Create selector with exit behavior first
    return py_trees.composites.Selector(
        "optimized lidar test mode behavior",
        memory=False,  # CRITICAL: Allows ESC checking
        children=[exit_mode_behavior, lidar_test_sequence],
    )


def make_standard_lidar_test_sub_tree():
    """
    Create standard LiDAR test mode (fallback if optimized not available)
    """
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)
    lidar_test_behavior = StableLidarTest()
    
    standard_sequence = py_trees.composites.Sequence(
        name="Standard LiDAR Test Sequence",
        memory=False,
        children=[lidar_test_behavior]
    )
    
    return py_trees.composites.Selector(
        "standard lidar test mode behavior",
        memory=False,
        children=[exit_mode_behavior, standard_sequence],
    )


# Export everything for easy import
__all__ = [
    # Optimized behavior with <37px variance
    'OptimizedLidarTestBehavior',
    
    # Standard behaviors for backwards compatibility
    'StableLidarTest',
    'LidarTestBehavior',
    'UltraStableLidarSystem',
    'EnhancedFacialAnimationRestorer',
    
    # Factory functions
    'make_lidar_test_sub_tree',           # DEFAULT: Uses optimized mode
    'make_standard_lidar_test_sub_tree',  # Fallback to standard mode
]