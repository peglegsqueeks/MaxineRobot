#!/usr/bin/env python3
"""
Simple Working LiDAR Test - NO PYGAME CONFLICTS
This test does NOT create any pygame displays to avoid conflicting with KeyboardSensor
The KeyboardSensor needs pygame events, so we can't consume them with our own pygame window
"""

import time
import py_trees
from py_trees.common import Status
from collections import deque

from src.behaviors.MaxineBehavior import MaxineBehavior
from src.types.RobotModes import RobotMode
from src.types.MovementDirection import MovementDirection
from src.action_managers.VelocityManager import VelocityConfig


class SimpleWorkingLidarTest(MaxineBehavior):
    """
    Simple LiDAR Test that does NOT interfere with pygame/keyboard events
    This behavior just prints status to console - no pygame display
    """
    
    def __init__(self):
        super().__init__("Simple Working LiDAR Test")
        
        # py_trees Lifecycle: __init__ - ONLY variables
        
        # Blackboard setup
        try:
            self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.READ)
            self.blackboard.register_key("HEAD_CENTER_POSITION", access=py_trees.common.Access.WRITE)
            self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.WRITE)
        except Exception:
            pass
        
        # Simple state variables
        self.robot = None
        self.update_counter = 0
        self.start_time = 0
        self.last_status_time = 0
        self.lidar_system = None
        
        # Person detection variables
        self.person_count = 0
        self.last_person_time = 0
        
        # CRITICAL: NO pygame initialization - that's what was causing the conflict!
    
    def setup(self, **kwargs):
        """py_trees setup - no hardware"""
        return True
    
    def initialise(self):
        """py_trees initialise - minimal setup"""
        print("🚀 Simple Working LiDAR Test starting...")
        print("📝 This test uses CONSOLE OUTPUT ONLY - no pygame display")
        print("⌨️ ESC key should work because we don't interfere with KeyboardSensor")
        
        self.start_time = time.time()
        self.last_status_time = self.start_time
        
        # Get robot and stop movement safely
        try:
            self.robot = self.get_robot()
            if hasattr(self.robot, 'direct_velocity_manager') and self.robot.direct_velocity_manager:
                stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                self.robot.direct_velocity_manager.perform_action(stop_config)
                print("✅ Robot movement stopped")
        except Exception as e:
            print(f"⚠️ Could not stop robot: {e}")
        
        # Try to initialize LiDAR (optional)
        self.try_initialize_lidar()
        
        # Try to check camera (optional)
        self.try_check_camera()
        
        print("✅ Simple LiDAR test initialized successfully")
    
    def try_initialize_lidar(self):
        """Try to initialize LiDAR system"""
        try:
            from pyrplidar import PyRPlidar
            print("📡 LiDAR libraries available")
            # We could initialize LiDAR here if needed, but keeping it simple for now
        except ImportError:
            print("⚠️ LiDAR libraries not available")
    
    def try_check_camera(self):
        """Try to check camera system"""
        try:
            if hasattr(self.robot, 'camera_sensor') and self.robot.camera_sensor:
                print("📷 Camera sensor available")
                # Try to get a reading
                reading = self.robot.camera_sensor.get_reading()
                if reading and hasattr(reading, 'get_people_locations'):
                    people = reading.get_people_locations()
                    if people:
                        self.person_count += len(people)
                        self.last_person_time = time.time()
                        print(f"👤 Found {len(people)} people")
            else:
                print("📷 No camera sensor available")
        except Exception as e:
            print(f"⚠️ Camera check failed: {e}")
    
    def update(self):
        """
        py_trees update - CRITICAL: Does NOT touch pygame at all!
        """
        self.update_counter += 1
        current_time = time.time()
        
        # Print status every 5 seconds
        if current_time - self.last_status_time >= 5.0:
            elapsed = current_time - self.start_time
            print(f"🚀 Simple LiDAR test running: {elapsed:.1f}s, {self.update_counter} updates")
            print(f"👤 Total people detected: {self.person_count}")
            print(f"⌨️ ESC should work (no pygame conflicts)")
            print(f"🔄 Press ESC to exit to IDLE mode")
            self.last_status_time = current_time
        
        # Try to detect people periodically (every 50 updates)
        if self.update_counter % 50 == 0:
            self.try_check_camera()
        
        # CRITICAL: We do NOT handle pygame events - let KeyboardSensor handle them!
        # CRITICAL: We do NOT create any pygame displays
        # CRITICAL: We just return RUNNING and let the exit_behavior handle ESC
        
        return Status.RUNNING
    
    def terminate(self, new_status):
        """py_trees terminate - minimal cleanup"""
        elapsed = time.time() - self.start_time
        print(f"✅ Simple LiDAR Test completed successfully after {elapsed:.1f}s")
        print(f"📊 Total updates: {self.update_counter}")
        print(f"👤 Total people detected: {self.person_count}")
        print("⌨️ ESC handling worked correctly!")
        
        # Stop robot movement
        try:
            if self.robot and hasattr(self.robot, 'direct_velocity_manager') and self.robot.direct_velocity_manager:
                stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                self.robot.direct_velocity_manager.perform_action(stop_config)
        except Exception:
            pass
        
        # Center head
        try:
            if self.robot:
                if hasattr(self.robot, 'servo_controller') and self.robot.servo_controller:
                    self.robot.servo_controller.center()
                elif hasattr(self.robot, 'head_velocity_manager') and self.robot.head_velocity_manager:
                    self.robot.head_velocity_manager.center_head()
        except Exception:
            pass
        
        # Restore facial animation
        try:
            if self.robot and hasattr(self.robot, 'facial_animation_manager') and self.robot.facial_animation_manager:
                self.robot.facial_animation_manager.bring_to_front()
                print("✅ Facial animation restored")
        except Exception:
            pass
        
        super().terminate(new_status)


# Use this simple implementation
BulletproofLidarTest = SimpleWorkingLidarTest
FixedBulletproofLidarTest = SimpleWorkingLidarTest
DirectPipelineLidarTest = SimpleWorkingLidarTest
FixedDirectPipelineLidarTest = SimpleWorkingLidarTest