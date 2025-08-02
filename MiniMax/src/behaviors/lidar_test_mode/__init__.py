#!/usr/bin/env python3
"""
LiDAR Test Mode Package
OPTIMIZED with Enhanced Camera Integration and Multi-Processing
Integrates existing depthai camera sensor with multi-processing optimizations
"""

# Import the original LiDAR test behavior classes for backwards compatibility
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

# DO NOT import OptimizedLidarTestBehavior at module level to prevent early initialization

import py_trees


def make_lidar_test_sub_tree():
    """
    Create LiDAR test mode sub-tree - USE EXISTING WORKING BEHAVIOR INSTEAD
    
    REVERT TO WORKING CODE - use the existing StableLidarTest that was working
    before I broke it with the optimized version.
    """
    print("🔧 Creating LiDAR test behavior (using WORKING legacy version)")
    
    # Use the existing working behavior instead of the problematic optimized one
    lidar_test_behavior = StableLidarTest()
    
    # Create a simple subtree with the working behavior
    lidar_test_subtree = py_trees.composites.Sequence(
        name="LiDAR Test Mode (Working Legacy Version)",
        memory=False,
        children=[lidar_test_behavior]
    )
    
    print("✅ Created working LiDAR test behavior (no early initialization)")
    return lidar_test_subtree


def make_optimized_lidar_test_sub_tree():
    """
    Create OPTIMIZED LiDAR test mode - ONLY use this when specifically requested
    This version has the camera/LiDAR optimizations but may interfere with avatar
    """
    try:
        # Import the optimized behavior ONLY when this specific function is called
        from .OptimizedLidarTestMode import OptimizedLidarTestBehavior
        
        print("🚀 Creating OPTIMIZED LiDAR test behavior (may affect avatar)")
        
        # Create the optimized LiDAR test behavior
        lidar_test_behavior = OptimizedLidarTestBehavior()
        
        # Create a simple subtree with the optimized behavior
        lidar_test_subtree = py_trees.composites.Sequence(
            name="LiDAR Test Mode (Optimized Camera Integration)",
            memory=False,
            children=[lidar_test_behavior]
        )
        
        return lidar_test_subtree
        
    except ImportError as e:
        print(f"⚠️ Could not load optimized LiDAR test behavior: {e}")
        print("🔄 Falling back to legacy LiDAR test behavior")
        
        # Fall back to legacy behavior if optimized version fails
        return make_lidar_test_sub_tree()


def make_legacy_lidar_test_sub_tree():
    """
    Create legacy LiDAR test mode sub-tree for backwards compatibility
    Uses the original hybrid distance method implementation
    """
    # Create the legacy LiDAR test behavior
    lidar_test_behavior = StableLidarTest()
    
    # Create a simple subtree with the legacy behavior
    lidar_test_subtree = py_trees.composites.Sequence(
        name="LiDAR Test Mode (Legacy Hybrid Distance)",
        memory=False,
        children=[lidar_test_behavior]
    )
    
    return lidar_test_subtree


# Export everything for easy import - DEFAULT to working legacy version
__all__ = [
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
    
    # Factory functions - DEFAULT uses working legacy version
    'make_lidar_test_sub_tree',           # Uses WORKING legacy version
    'make_optimized_lidar_test_sub_tree', # Uses optimized version (may break avatar)
    'make_legacy_lidar_test_sub_tree'     # Explicitly legacy version
]