#!/usr/bin/env python3
"""
MINIMAL Test LiDAR - Does absolutely nothing except prove py_trees works
This is the simplest possible test to verify the behavior tree structure
"""

import time
import py_trees
from py_trees.common import Status

from src.behaviors.MaxineBehavior import MaxineBehavior


class MinimalTestLidar(MaxineBehavior):
    """
    The simplest possible LiDAR test that just counts and waits for ESC
    """
    
    def __init__(self):
        super().__init__("Minimal Test LiDAR")
        
        # py_trees Lifecycle: __init__ - ONLY variables
        self.counter = 0
        self.start_time = 0
        
        # NO imports, NO hardware, NO complex objects
    
    def setup(self, **kwargs):
        """py_trees setup - no hardware"""
        return True
    
    def initialise(self):
        """py_trees initialise - minimal setup"""
        print("🧪 MINIMAL LiDAR Test starting...")
        print("🧪 This test does nothing except count and wait for ESC")
        print("🧪 If this test works, the py_trees structure is correct")
        self.start_time = time.time()
        
        # Get robot and stop movement safely
        try:
            robot = self.get_robot()
            if hasattr(robot, 'direct_velocity_manager') and robot.direct_velocity_manager:
                from src.action_managers.VelocityManager import VelocityConfig
                from src.types.MovementDirection import MovementDirection
                stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                robot.direct_velocity_manager.perform_action(stop_config)
                print("✅ Robot movement stopped")
        except Exception as e:
            print(f"⚠️ Could not stop robot: {e}")
    
    def update(self):
        """py_trees update - minimal functionality"""
        self.counter += 1
        
        # Print status every 5 seconds
        if self.counter % 250 == 0:  # At 50 FPS this is ~5 seconds
            elapsed = time.time() - self.start_time
            print(f"🧪 Minimal test running: {elapsed:.1f}s, {self.counter} updates")
            print("🧪 Press ESC to exit to IDLE mode")
        
        # Handle ESC key - the ONLY functionality
        try:
            import pygame
            if pygame.get_init():
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            print("🧪 ESC pressed in minimal test - exiting to IDLE")
                            return Status.SUCCESS  # Exit to IDLE via exit behavior
                    elif event.type == pygame.QUIT:
                        print("🧪 Window closed in minimal test - exiting to IDLE")
                        return Status.SUCCESS
        except Exception:
            pass  # Even pygame might fail, but we continue
        
        # ALWAYS return RUNNING unless ESC pressed
        return Status.RUNNING
    
    def terminate(self, new_status):
        """py_trees terminate - minimal cleanup"""
        elapsed = time.time() - self.start_time
        print(f"🧪 MINIMAL LiDAR Test completed successfully after {elapsed:.1f}s")
        print(f"🧪 Total updates: {self.counter}")
        print("🧪 py_trees structure is working correctly!")
        
        # Try to restore facial animation
        try:
            robot = self.get_robot()
            if robot and hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                robot.facial_animation_manager.bring_to_front()
                print("✅ Facial animation restored")
        except Exception:
            pass
        
        super().terminate(new_status)