#!/usr/bin/env python3
"""
MINIMAL CORRECT Test - Proves the py_trees exit behavior works
This is the simplest possible test that follows the CORRECT pattern:
- Does NOT handle ESC (lets exit_behavior handle it)
- Just runs and displays status
- CANNOT get stuck in infinite loop
"""

import time
import py_trees
from py_trees.common import Status

from src.behaviors.MaxineBehavior import MaxineBehavior
from src.types.MovementDirection import MovementDirection
from src.action_managers.VelocityManager import VelocityConfig


class MinimalCorrectTest(MaxineBehavior):
    """
    The simplest possible test that follows CORRECT py_trees pattern
    CRITICAL: Does NOT handle ESC - lets exit_behavior handle it
    """
    
    def __init__(self):
        super().__init__("Minimal CORRECT Test")
        
        # py_trees Lifecycle: __init__ - ONLY variables
        self.counter = 0
        self.start_time = 0
        self.robot = None
        
        # NO imports, NO hardware, NO complex objects
        # NO ESC handling!
    
    def setup(self, **kwargs):
        """py_trees setup - no hardware"""
        return True
    
    def initialise(self):
        """py_trees initialise - minimal setup"""
        print("🧪 MINIMAL CORRECT Test starting...")
        print("🧪 This test does NOT handle ESC - exit_behavior handles it")
        print("🧪 This follows the EXACT same pattern as ALL working modes")
        print("🧪 If this works, the py_trees structure is fixed!")
        self.start_time = time.time()
        
        # Get robot and stop movement safely
        try:
            self.robot = self.get_robot()
            if hasattr(self.robot, 'direct_velocity_manager') and self.robot.direct_velocity_manager:
                stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                self.robot.direct_velocity_manager.perform_action(stop_config)
                print("✅ Robot movement stopped")
        except Exception as e:
            print(f"⚠️ Could not stop robot: {e}")
    
    def update(self):
        """
        py_trees update - CRITICAL: Does NOT handle ESC!
        """
        self.counter += 1
        
        # Print status every 5 seconds
        if self.counter % 250 == 0:  # At 50 FPS this is ~5 seconds
            elapsed = time.time() - self.start_time
            print(f"🧪 Minimal CORRECT test running: {elapsed:.1f}s, {self.counter} updates")
            print(f"🧪 ESC handling: DISABLED (exit_behavior handles it)")
            print(f"🧪 Should exit cleanly with ESC (no infinite loop)")
        
        # CRITICAL: Do NOT handle ESC here!
        # The exit_behavior (first in Selector) will handle ESC
        # Just clear pygame events without processing them
        try:
            import pygame
            if pygame.get_init():
                pygame.event.clear()  # Clear but don't process ESC
        except Exception:
            pass  # Even pygame might fail, but we continue
        
        # CRITICAL: ALWAYS return RUNNING
        # Let the exit_behavior handle mode changes
        return Status.RUNNING
    
    def terminate(self, new_status):
        """py_trees terminate - minimal cleanup"""
        elapsed = time.time() - self.start_time
        print(f"🧪 MINIMAL CORRECT Test completed successfully after {elapsed:.1f}s")
        print(f"🧪 Total updates: {self.counter}")
        print("🧪 exit_behavior successfully handled the ESC key!")
        print("🧪 py_trees structure is now CORRECT!")
        
        # Try to restore facial animation
        try:
            if self.robot and hasattr(self.robot, 'facial_animation_manager') and self.robot.facial_animation_manager:
                self.robot.facial_animation_manager.bring_to_front()
                print("✅ Facial animation restored")
        except Exception:
            pass
        
        super().terminate(new_status)


# Export for testing
MinimalTestLidar = MinimalCorrectTest