#!/usr/bin/env python3
"""
LiDAR Test Mode Package - RESTORED Original Functionality with FIXED Exit Behavior
Fixed to use original ExactStandaloneLidarTest but with proper py_trees exit structure
The issue was memory=True causing exit problems - now uses memory=False throughout
"""

import py_trees
from src.behaviors.utils import make_press_esc_to_exit_behavior
from src.types.RobotModes import RobotMode

# Import existing behaviors for backwards compatibility
from .LidarTestBehavior import (
    StableLidarTest,
    OptimizedStableLidarTest, 
    OptimizedLidarTestWithHybridDistance,
    LidarTestBehavior,
    FixedStableLidarTest,
    HybridDistanceCalculator,
    FixedUltraStablePyRPLidarA3,
    FixedUltraStableLidarSystem,
    EnhancedFacialAnimationRestorer
)

# Import potential field behavior
from .PotentialFieldLidarTest import PotentialFieldLidarTest

# Import direct detection behavior (camera sensor bypass)
from .DirectDetectionLidarTest import DirectDetectionLidarTest

# Import EXACT COPY standalone behavior (true copy of detection_consistency_test.py)
from .StandaloneLidarTest import ExactStandaloneLidarTest

# Import the FIXED version with working settlement detection (with fallback)
try:
    from .FixedSettlementLidarTest import FixedSettlementLidarTest
    FIXED_SETTLEMENT_AVAILABLE = True
    print("✅ Fixed settlement detection available")
except ImportError:
    print("⚠️ FixedSettlementLidarTest not found - using original (settlement issues will persist)")
    FixedSettlementLidarTest = ExactStandaloneLidarTest  # Fallback to original
    FIXED_SETTLEMENT_AVAILABLE = False


def make_lidar_test_sub_tree():
    """
    LiDAR test mode with settlement detection fix (if available)
    
    Uses FixedSettlementLidarTest if available, otherwise falls back to original.
    The fixed version has:
    - Movement threshold: 25 pixels (was 100 - too high!)
    - Settlement time: 1.0 seconds (was 2.0 - too long!)  
    - Position smoothing to reduce jitter
    - Proper initialization of timing
    
    This fixes the head tracking oscillation issue where person was never detected as settled.
    """
    # Add the exit behavior (this was missing from original)
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)
    
    # Use the FIXED version if available, otherwise original
    if FIXED_SETTLEMENT_AVAILABLE:
        print("🔧 Using FIXED settlement detection")
        standalone_behavior = FixedSettlementLidarTest()
        sequence_name = "LiDAR Test Fixed Settlement Sequence"
    else:
        print("⚠️ Using ORIGINAL settlement detection (oscillation issues may persist)")
        standalone_behavior = ExactStandaloneLidarTest()
        sequence_name = "LiDAR Test Original Sequence"
    
    # CRITICAL FIX: Use memory=False throughout (not memory=True)
    # This allows exit_behavior to be checked properly
    lidar_working_sub_tree = py_trees.composites.Sequence(
        name=sequence_name,
        memory=False,  # CRITICAL: memory=False allows proper exit
        children=[standalone_behavior]
    )
    
    # FIXED: Proper Selector structure with memory=False 
    return py_trees.composites.Selector(
        "lidar test mode behavior",
        memory=False,  # CRITICAL: memory=False allows exit_behavior to work
        children=[exit_mode_behavior, lidar_working_sub_tree],
    )


def make_original_lidar_test_sub_tree():
    """
    Original ExactStandaloneLidarTest (for comparison/debugging)
    Has the settlement detection issues but everything else works
    """
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)
    original_behavior = ExactStandaloneLidarTest()
    
    original_sub_tree = py_trees.composites.Sequence(
        name="LiDAR Test Original Sequence",
        memory=False,
        children=[original_behavior]
    )
    
    return py_trees.composites.Selector(
        "original lidar test mode behavior",
        memory=False,
        children=[exit_mode_behavior, original_sub_tree],
    )


def make_potential_field_lidar_test_sub_tree():
    """
    Create LiDAR test mode sub-tree with Potential Field Grid Overlay
    FIXED: Now includes proper exit behavior with memory=False
    """
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)
    potential_field_behavior = PotentialFieldLidarTest()
    
    potential_field_sub_tree = py_trees.composites.Sequence(
        name="LiDAR Test Potential Field Sequence",
        memory=False,  # CRITICAL: memory=False
        children=[potential_field_behavior]
    )
    
    return py_trees.composites.Selector(
        "potential field lidar test mode behavior",
        memory=False,  # CRITICAL: memory=False
        children=[exit_mode_behavior, potential_field_sub_tree],
    )


def make_legacy_lidar_test_sub_tree():
    """
    Create ORIGINAL legacy LiDAR test mode without any grid overlay
    FIXED: Now includes proper exit behavior with memory=False
    """
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)
    lidar_test_behavior = StableLidarTest()
    
    legacy_sub_tree = py_trees.composites.Sequence(
        name="LiDAR Test Legacy Sequence",
        memory=False,  # CRITICAL: memory=False
        children=[lidar_test_behavior]
    )
    
    return py_trees.composites.Selector(
        "legacy lidar test mode behavior",
        memory=False,  # CRITICAL: memory=False
        children=[exit_mode_behavior, legacy_sub_tree],
    )


def make_direct_detection_lidar_test_sub_tree():
    """
    Create Direct Detection LiDAR test mode (ObjectDetectionReading bypass)
    FIXED: Now includes proper exit behavior with memory=False
    """
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)
    direct_detection_behavior = DirectDetectionLidarTest()
    
    direct_detection_sub_tree = py_trees.composites.Sequence(
        name="LiDAR Test Direct Detection Sequence",
        memory=False,  # CRITICAL: memory=False
        children=[direct_detection_behavior]
    )
    
    return py_trees.composites.Selector(
        "direct detection lidar test mode behavior",
        memory=False,  # CRITICAL: memory=False
        children=[exit_mode_behavior, direct_detection_sub_tree],
    )


def make_optimized_lidar_test_sub_tree():
    """
    Create OPTIMIZED LiDAR test mode - ONLY use when specifically requested
    FIXED: Now includes proper exit behavior with memory=False
    """
    try:
        from .OptimizedLidarTestMode import OptimizedLidarTestBehavior
        
        exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)
        lidar_test_behavior = OptimizedLidarTestBehavior()
        
        optimized_sub_tree = py_trees.composites.Sequence(
            name="LiDAR Test Optimized Sequence",
            memory=False,  # CRITICAL: memory=False
            children=[lidar_test_behavior]
        )
        
        return py_trees.composites.Selector(
            "optimized lidar test mode behavior",
            memory=False,  # CRITICAL: memory=False
            children=[exit_mode_behavior, optimized_sub_tree],
        )
        
    except ImportError:
        # Fall back to exact copy if optimized version fails
        return make_lidar_test_sub_tree()


# Export everything for easy import - DEFAULT to FIXED version with working settlement detection
__all__ = [
    # FIXED working behavior - DEFAULT
    'FixedSettlementLidarTest',
    
    # ORIGINAL behavior (for comparison/debugging)
    'ExactStandaloneLidarTest',
    
    # Direct detection behavior for camerasensor.py bypass
    'DirectDetectionLidarTest',
    
    # Potential field behavior
    'PotentialFieldLidarTest',
    
    # Original classes for backwards compatibility
    'StableLidarTest',
    'OptimizedStableLidarTest',
    'OptimizedLidarTestWithHybridDistance', 
    'LidarTestBehavior',
    'FixedStableLidarTest',
    'HybridDistanceCalculator',
    'FixedUltraStablePyRPLidarA3',
    'FixedUltraStableLidarSystem',
    'EnhancedFacialAnimationRestorer',
    
    # FIXED Factory functions - All use memory=False and include exit behavior
    'make_lidar_test_sub_tree',                    # FIXED: Uses FixedSettlementLidarTest with working settlement detection (DEFAULT)
    'make_original_lidar_test_sub_tree',           # Original ExactStandaloneLidarTest (for comparison)
    'make_potential_field_lidar_test_sub_tree',    # Uses potential field with fixed exit
    'make_legacy_lidar_test_sub_tree',             # Uses legacy with fixed exit
    'make_direct_detection_lidar_test_sub_tree',   # Uses direct detection with fixed exit
    'make_optimized_lidar_test_sub_tree'           # Uses optimized with fixed exit
]