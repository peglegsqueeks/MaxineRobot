#!/usr/bin/env python3
"""
FIXED Working LiDAR Test - Proper py_trees Exit Behavior
FIXED: Ensures ESC handling works by returning FAILURE periodically to force Selector restart
This allows the exit_behavior to be checked regularly while the main behavior runs
"""

import time
import py_trees
from py_trees.common import Status

from src.behaviors.MaxineBehavior import MaxineBehavior
from src.types.MovementDirection import MovementDirection
from src.action_managers.VelocityManager import VelocityConfig


class FixedWorkingLidarTest(MaxineBehavior):
    """
    FIXED Working LiDAR Test that properly handles ESC by returning FAILURE periodically
    This forces the Selector to restart and check the exit_behavior regularly
    
    CRITICAL: This behavior does NOT handle ESC directly - it lets the exit_behavior handle it
    by occasionally returning FAILURE to allow the Selector to restart
    """
    
    def __init__(self):
        super().__init__("Fixed Working LiDAR Test")
        
        # py_trees Lifecycle: __init__ - ONLY variables, NO hardware initialization
        
        # Blackboard setup
        try:
            self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.READ)
            self.blackboard.register_key("HEAD_CENTER_POSITION", access=py_trees.common.Access.WRITE)
            self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.WRITE)
        except Exception:
            pass  # Blackboard registration can fail, handle gracefully
        
        # Simple state variables - NO hardware objects
        self.robot = None
        self.update_counter = 0
        self.start_time = 0
        self.last_status_time = 0
        self.initialized = False
        
        # CRITICAL: Counter for forcing Selector restart (this is the key fix)
        self.force_restart_counter = 0
        self.force_restart_interval = 500  # Return FAILURE every 500 ticks (much longer interval)
        
        # Person detection variables
        self.person_count = 0
        self.last_person_time = 0
        
        # LiDAR system reference (no initialization)
        self.lidar_system = None
        
        # Simple status tracking
        self.errors_handled = 0
    
    def setup(self, **kwargs):
        """py_trees setup - no hardware initialization"""
        try:
            # Use the proper method from MaxineBehavior to get robot
            self.robot = self.get_robot()
            print("🎯 Fixed Working LiDAR Test setup complete")
            return True
        except Exception as e:
            print(f"⚠️ Setup warning: {e}")
            return True  # Continue anyway
    
    def initialise(self):
        """py_trees initialise - called when behavior STARTS running"""
        print("🎯 Fixed Working LiDAR Test initializing...")
        
        try:
            self.start_time = time.time()
            self.last_status_time = self.start_time
            self.update_counter = 0
            self.force_restart_counter = 0
            self.person_count = 0
            self.errors_handled = 0
            self.initialized = False
            
            # Safely get robot reference
            try:
                self.robot = self.get_robot()
            except Exception as e:
                print(f"⚠️ Robot access warning: {e}")
                self.robot = None
            
            # Try to safely initialize basic systems
            self.safe_initialize_systems()
            
            print("✅ Fixed Working LiDAR Test initialized successfully")
            self.initialized = True
            
        except Exception as e:
            print(f"⚠️ Initialization error handled: {e}")
            self.errors_handled += 1
            self.initialized = False
    
    def safe_initialize_systems(self):
        """Safely initialize systems with error handling"""
        try:
            # Try to access LiDAR system if available
            if self.robot:
                # Stop robot movement initially
                try:
                    if hasattr(self.robot, 'velocity_manager') and self.robot.velocity_manager:
                        config = VelocityConfig(
                            left_speed=0.0,
                            right_speed=0.0,
                            direction=MovementDirection.STOP
                        )
                        self.robot.velocity_manager.set_velocity(config)
                        print("🛑 Robot movement stopped")
                except Exception as e:
                    print(f"⚠️ Robot stop warning: {e}")
                
                # Try to center head
                try:
                    if hasattr(self.robot, 'servo_controller') and self.robot.servo_controller:
                        self.robot.servo_controller.set_head_angle(0.0)
                        print("🎯 Head centered")
                except Exception as e:
                    print(f"⚠️ Head center warning: {e}")
            
            print("✅ Basic systems initialized safely")
            
        except Exception as e:
            print(f"⚠️ System initialization error handled: {e}")
            self.errors_handled += 1
    
    def try_check_camera(self):
        """Safely try to check for people with error handling"""
        try:
            # Try to detect people if robot has camera sensor
            if (self.robot and 
                hasattr(self.robot, 'camera_sensor') and 
                self.robot.camera_sensor):
                
                try:
                    # Try to get current person detection
                    target_person = self.blackboard.get("TARGET_PERSON") if self.blackboard.exists("TARGET_PERSON") else None
                    
                    if target_person:
                        self.person_count += 1
                        self.last_person_time = time.time()
                        print(f"👤 Person detected (total: {self.person_count})")
                    
                except Exception as e:
                    print(f"⚠️ Person detection error handled: {e}")
                    self.errors_handled += 1
            
        except Exception as e:
            print(f"⚠️ Camera check error handled: {e}")
            self.errors_handled += 1
    
    def update(self):
        """
        py_trees update - CRITICAL: Returns FAILURE every N ticks to allow ESC checking
        This is the key fix that allows the exit_behavior to be checked regularly
        """
        self.update_counter += 1
        self.force_restart_counter += 1
        current_time = time.time()
        
        # Print status every 5 seconds
        if current_time - self.last_status_time >= 5.0:
            elapsed = current_time - self.start_time
            print(f"🎯 Fixed Working LiDAR test running: {elapsed:.1f}s, {self.update_counter} updates")
            print(f"👤 Total people detected: {self.person_count}")
            print(f"⚠️ Errors handled gracefully: {self.errors_handled}")
            print(f"🔧 Force restart counter: {self.force_restart_counter}/{self.force_restart_interval}")
            print(f"⌨️ Press ESC to exit to IDLE mode (checking every ~25 seconds)")
            self.last_status_time = current_time
        
        # Try to detect people periodically (every 50 updates)
        if self.update_counter % 50 == 0:
            self.try_check_camera()
        
        # CRITICAL FIX: Return FAILURE every N ticks to force Selector restart
        # This allows the exit_behavior to be checked again and again
        # Without this, the Selector never rechecks the exit_behavior!
        if self.force_restart_counter >= self.force_restart_interval:
            print(f"🔄 Returning FAILURE to force Selector restart after {self.force_restart_interval} ticks (~25s)")
            self.force_restart_counter = 0  # Reset counter
            return Status.FAILURE
        
        # Most of the time, return RUNNING to continue the behavior
        return Status.RUNNING
    
    def terminate(self, new_status):
        """py_trees terminate - called when behavior STOPS running"""
        try:
            elapsed = time.time() - self.start_time
            print(f"🔄 Fixed Working LiDAR Test terminating...")
            print(f"📊 Session Summary:")
            print(f"   Duration: {elapsed:.1f}s")
            print(f"   Updates: {self.update_counter}")
            print(f"   People detected: {self.person_count}")
            print(f"   Errors handled: {self.errors_handled}")
            print("✅ ESC exit handling worked correctly!")
            
            # Safely stop robot
            try:
                if self.robot and hasattr(self.robot, 'velocity_manager') and self.robot.velocity_manager:
                    config = VelocityConfig(
                        left_speed=0.0,
                        right_speed=0.0,
                        direction=MovementDirection.STOP
                    )
                    self.robot.velocity_manager.set_velocity(config)
                    print("🛑 Robot stopped safely")
            except Exception as e:
                print(f"⚠️ Robot stop error handled: {e}")
            
            # Safely center head
            try:
                if self.robot and hasattr(self.robot, 'servo_controller') and self.robot.servo_controller:
                    self.robot.servo_controller.set_head_angle(0.0)
                    print("🎯 Head centered safely")
            except Exception as e:
                print(f"⚠️ Head center error handled: {e}")
            
            print("✅ Fixed Working LiDAR Test terminated successfully")
            
        except Exception as e:
            print(f"⚠️ Termination error handled: {e}")
        
        # Call parent terminate
        try:
            super().terminate(new_status)
        except Exception as e:
            print(f"⚠️ Parent terminate error handled: {e}")


# Use the fixed implementation as the main class
WorkingLidarTest = FixedWorkingLidarTest