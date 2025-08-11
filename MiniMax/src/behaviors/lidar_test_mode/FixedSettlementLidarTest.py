#!/usr/bin/env python3
"""
COMPREHENSIVE HEAD TRACKING FIX
This addresses ALL identified issues and includes extensive diagnostics
"""

import time
import math
import csv
import statistics
import os
from collections import deque
from py_trees.common import Status
from .StandaloneLidarTest import ExactStandaloneLidarTest


class FixedSettlementLidarTest(ExactStandaloneLidarTest):
    """
    COMPREHENSIVE FIX with full diagnostics and coordinate system verification
    """
    
    def __init__(self):
        super().__init__()
        
        # Basic parameters
        self.deadband_pixels = 50               # REDUCED deadband for more tracking
        self.screen_center_x = None             
        
        # Filter parameters
        self.filtered_x_position = None
        self.filter_alpha = 0.2
        self.filter_initialized = False
        self.position_smoothing_window = deque(maxlen=5)

        # Movement parameters
        self.person_movement_threshold = 80     
        self.person_settled_time = 2.0          # REDUCED settlement time
        self.head_command_cooldown = 0.8        # REDUCED cooldown
        
        # CONSERVATIVE tracking thresholds
        self.head_move_threshold = math.radians(4)      # 4° to START (very low)
        self.head_stop_threshold = math.radians(2)      # 2° to STOP
        self.movement_scale_factor = 1.0                # NO scaling - use full calculated angles
        self.max_single_movement_deg = 45.0             # Allow larger movements
        
        # State tracking
        self.head_angle_target = 0.0           
        self.head_angle_current = 0.0          
        self.last_head_command_time = 0
        self.last_commanded_angle_deg = 0.0
        self.min_angle_change_deg = 1.0        # Very low minimum change
        
        # Settlement tracking
        self.last_significant_movement = time.time()
        self.person_is_settled = False
        self.last_smoothed_x = None
        
        # CSV setup
        self.csv_log_filename = "/home/jetson/maxine/MiniMax/LIDARTEST.csv"
        self.csv_file_created = False
        
        # Tracking variables
        self.detection_count = 0
        self.last_x_midpoint = None
        
        # Debug counters
        self.deadband_blocks = 0
        self.frequency_blocks = 0
        self.angle_blocks = 0
        self.movements_sent = 0
        
        # COORDINATE SYSTEM VERIFICATION
        self.coordinate_system_verified = False
        
        print("🔧 COMPREHENSIVE HEAD TRACKING FIX:")
        print(f"   Deadband: {self.deadband_pixels}px")
        print(f"   Movement threshold: {self.person_movement_threshold}px")
        print(f"   Settlement time: {self.person_settled_time}s")
        print(f"   Head start threshold: {math.degrees(self.head_move_threshold):.1f}°")
        print(f"   Head stop threshold: {math.degrees(self.head_stop_threshold):.1f}°")
        print(f"   Movement scale: {self.movement_scale_factor} (no scaling)")
        print(f"   Max movement: {self.max_single_movement_deg}°")
        print(f"   Min change: {self.min_angle_change_deg}°")
        print(f"   Cooldown: {self.head_command_cooldown}s")
    
    def verify_coordinate_system(self):
        """VERIFY the servo coordinate system with diagnostic movements"""
        if self.coordinate_system_verified:
            return
            
        try:
            robot = self.get_robot()
            if not (hasattr(robot, 'servo_controller') and robot.servo_controller):
                print("❌ DIAGNOSTIC: No servo controller available")
                return
                
            print("🔍 DIAGNOSTIC: Verifying servo coordinate system...")
            
            # Get initial position
            initial_pos = robot.servo_controller.get_position()
            initial_angle = robot.servo_controller.get_angle_degrees()
            print(f"🔍 INITIAL: position={initial_pos:.3f}, angle={initial_angle:.1f}°")
            
            # Test movement to +10 degrees
            print("🔍 TESTING: move_to_angle(+10°) - should move RIGHT")
            robot.servo_controller.move_to_angle(10.0)
            time.sleep(1.0)
            pos_10 = robot.servo_controller.get_position()
            angle_10 = robot.servo_controller.get_angle_degrees()
            print(f"🔍 RESULT +10°: position={pos_10:.3f}, angle={angle_10:.1f}°")
            
            # Test movement to -10 degrees  
            print("🔍 TESTING: move_to_angle(-10°) - should move LEFT")
            robot.servo_controller.move_to_angle(-10.0)
            time.sleep(1.0)
            pos_neg10 = robot.servo_controller.get_position()
            angle_neg10 = robot.servo_controller.get_angle_degrees()
            print(f"🔍 RESULT -10°: position={pos_neg10:.3f}, angle={angle_neg10:.1f}°")
            
            # Return to center
            print("🔍 TESTING: move_to_angle(0°) - should return to center")
            robot.servo_controller.move_to_angle(0.0)
            time.sleep(1.0)
            final_pos = robot.servo_controller.get_position()
            final_angle = robot.servo_controller.get_angle_degrees()
            print(f"🔍 FINAL: position={final_pos:.3f}, angle={final_angle:.1f}°")
            
            # Analyze results
            if pos_10 > pos_neg10:
                print("✅ COORDINATE SYSTEM: +angle = RIGHT (higher position), -angle = LEFT (lower position)")
            else:
                print("⚠️ COORDINATE SYSTEM: INVERTED - +angle = LEFT, -angle = RIGHT")
                
            self.coordinate_system_verified = True
            
        except Exception as e:
            print(f"❌ COORDINATE SYSTEM TEST FAILED: {e}")
    
    def force_create_new_csv_file(self):
        """Create new CSV file"""
        try:
            if os.path.exists(self.csv_log_filename):
                os.remove(self.csv_log_filename)
                print(f"🗑️ DELETED: {self.csv_log_filename}")
            
            with open(self.csv_log_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'mode_time_elapsed', 'timestamp', 'frame_number',
                    'x_midpoint_pixels', 'x_midpoint_normalized', 'x_camera_mm', 'z_depth_mm',
                    'raw_z_depth_mm', 'confidence', 'bbox_xmin', 'bbox_ymin', 'bbox_xmax', 'bbox_ymax',
                    'bbox_width', 'bbox_height', 'x_jump_from_previous', 'is_large_jump',
                    'rolling_variance_pixels', 'rolling_std_dev_pixels', 'rolling_mean_pixels',
                    'short_term_variance', 'medium_term_variance', 'long_term_variance',
                    'detection_count', 'consistent_detections', 'large_jumps_count',
                    'stability_classification', 'head_angle_deg', 'head_tracking_active',
                    'person_is_settled', 'detection_method',
                    'filtered_x_position', 'deadband_blocked', 'movement_scaled'
                ])
            
            print(f"✅ CREATED: {self.csv_log_filename}")
            self.csv_file_created = True
            
        except Exception as e:
            print(f"❌ CSV creation error: {e}")
            self.csv_file_created = False
    
    def get_screen_center_x(self):
        """Get screen center safely"""
        if self.screen_center_x is None:
            try:
                if hasattr(self, 'screen') and self.screen:
                    self.screen_center_x = self.screen.get_width() // 2
                else:
                    self.screen_center_x = 960  
                print(f"🎯 Screen center X set to: {self.screen_center_x}px")
            except Exception:
                self.screen_center_x = 960  
        return self.screen_center_x
    
    def apply_low_pass_filter(self, x_pixels):
        """Apply low-pass filter"""
        if not self.filter_initialized:
            self.filtered_x_position = x_pixels
            self.filter_initialized = True
        else:
            self.filtered_x_position = (self.filter_alpha * x_pixels + 
                                      (1.0 - self.filter_alpha) * self.filtered_x_position)
        
        self.position_smoothing_window.append(self.filtered_x_position)
        
        if len(self.position_smoothing_window) >= 3:
            smoothed_x = sum(self.position_smoothing_window) / len(self.position_smoothing_window)
        else:
            smoothed_x = self.filtered_x_position
        
        return smoothed_x
    
    def is_in_deadband(self, x_pixels):
        """Check deadband"""
        center_x = self.get_screen_center_x()
        distance_from_center = abs(x_pixels - center_x)
        return distance_from_center < self.deadband_pixels
    
    def update_settlement_detection(self, x_pixels):
        """Settlement detection"""
        current_time = time.time()
        
        smoothed_x = self.apply_low_pass_filter(x_pixels)
        
        if self.last_smoothed_x is not None:
            movement = abs(smoothed_x - self.last_smoothed_x)
            
            if movement > self.person_movement_threshold:
                self.last_significant_movement = current_time
                if self.person_is_settled:
                    print(f"🏃 Movement {movement:.1f}px - UNSETTLED")
                self.person_is_settled = False
            elif current_time - self.last_significant_movement > self.person_settled_time:
                if not self.person_is_settled:
                    print(f"✅ SETTLED after {current_time - self.last_significant_movement:.1f}s")
                self.person_is_settled = True
        
        self.last_smoothed_x = smoothed_x
        return smoothed_x
    
    def update_head_tracking_with_hysteresis(self, x_pixels):
        """COMPREHENSIVE head tracking with full diagnostics"""
        if not self.head_tracker:
            return
            
        if not self.person_is_settled:
            return
        
        current_time = time.time()
        
        # Startup delay
        if not hasattr(self, 'initialization_complete'):
            self.initialization_complete = False
            self.initialization_time = current_time
            self.startup_delay = 3.0
        
        if not self.initialization_complete:
            if current_time - self.initialization_time < self.startup_delay:
                return
            else:
                self.initialization_complete = True
                print("✅ Head tracking startup delay complete")
                # VERIFY COORDINATE SYSTEM AFTER STARTUP
                self.verify_coordinate_system()
        
        # Input validation
        if not isinstance(x_pixels, (int, float)) or x_pixels < 0 or x_pixels > 1920:
            print(f"⚠️ Invalid x_pixels: {x_pixels}")
            return
        
        # Deadband check
        center_x = self.get_screen_center_x()
        distance_from_center = abs(x_pixels - center_x)
        
        if self.is_in_deadband(x_pixels):
            self.deadband_blocks += 1
            if self.deadband_blocks <= 3:
                print(f"🔍 DEADBAND #{self.deadband_blocks}: x={x_pixels}px, dist={distance_from_center:.1f}px < {self.deadband_pixels}px")
            return
        
        # Frequency control
        if current_time - self.last_head_command_time < self.head_command_cooldown:
            self.frequency_blocks += 1
            return
        
        # Apply filtering
        smoothed_x = self.apply_low_pass_filter(x_pixels)
        
        # Calculate SIMPLE, DIRECT angle (no complex transformations)
        pixel_offset = smoothed_x - center_x
        pixel_offset_normalized = pixel_offset / center_x
        pixel_offset_normalized = max(-0.8, min(0.8, pixel_offset_normalized))
        
        # CAMERA FOV: 108 degrees, so ±54 degrees
        camera_half_fov_deg = 54.0
        
        # DIRECT calculation: person right of center = positive angle (head turns right)
        target_angle_deg = pixel_offset_normalized * camera_half_fov_deg
        
        # Apply minimal scaling
        scaled_target_angle_deg = target_angle_deg * self.movement_scale_factor
        
        # Safety limit
        if abs(scaled_target_angle_deg) > self.max_single_movement_deg:
            scaled_target_angle_deg = (self.max_single_movement_deg if scaled_target_angle_deg > 0 
                                     else -self.max_single_movement_deg)
        
        # Get ACTUAL current angle
        try:
            robot = self.get_robot()
            if hasattr(robot, 'servo_controller') and robot.servo_controller:
                actual_current_angle_deg = robot.servo_controller.get_angle_degrees()
                actual_current_pos = robot.servo_controller.get_position()
            else:
                actual_current_angle_deg = math.degrees(self.head_angle_current)
                actual_current_pos = 0.0
        except Exception:
            actual_current_angle_deg = math.degrees(self.head_angle_current)
            actual_current_pos = 0.0
        
        # Calculate movement needed
        angle_diff = abs(scaled_target_angle_deg - actual_current_angle_deg)
        movement_needed_deg = scaled_target_angle_deg - actual_current_angle_deg
        
        # Check minimum change
        if abs(movement_needed_deg) < self.min_angle_change_deg:
            self.angle_blocks += 1
            if self.angle_blocks <= 3:
                print(f"🔍 ANGLE BLOCK #{self.angle_blocks}: need {abs(movement_needed_deg):.1f}° < {self.min_angle_change_deg}°")
            return
        
        # Hysteresis check
        should_move = angle_diff > math.degrees(self.head_move_threshold)
        
        if should_move:
            try:
                # Safety check
                if abs(scaled_target_angle_deg) > 40:
                    print(f"⚠️ SAFETY: Refusing {scaled_target_angle_deg:.1f}°")
                    return
                
                # COMPREHENSIVE MOVEMENT with all methods
                robot = self.get_robot()
                movement_success = False
                method_used = "none"
                
                # Method 1: Direct servo controller
                if hasattr(robot, 'servo_controller') and robot.servo_controller:
                    try:
                        # FORCE CENTER POSITION CHECK
                        center_pos = robot.servo_controller.get_center_position()
                        if center_pos != 0.0:
                            print(f"⚠️ CRITICAL: servo center_position = {center_pos}, should be 0.0!")
                        
                        success = robot.servo_controller.move_to_angle(scaled_target_angle_deg)
                        if success:
                            movement_success = True
                            method_used = "servo_controller.move_to_angle"
                            
                            # Wait briefly and verify
                            time.sleep(0.1)
                            new_angle = robot.servo_controller.get_angle_degrees()
                            new_pos = robot.servo_controller.get_position()
                            print(f"📍 SERVO VERIFICATION: commanded={scaled_target_angle_deg:.1f}°, "
                                  f"actual={new_angle:.1f}°, pos={new_pos:.3f}")
                    except Exception as e:
                        print(f"⚠️ Servo move_to_angle failed: {e}")
                
                # Method 2: Head velocity manager
                if not movement_success and hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager:
                    try:
                        success = robot.head_velocity_manager.set_head_angle_degrees(scaled_target_angle_deg)
                        if success:
                            movement_success = True
                            method_used = "head_velocity_manager.set_head_angle_degrees"
                    except Exception as e:
                        print(f"⚠️ Head velocity manager failed: {e}")
                
                # Method 3: Direct position control
                if not movement_success and hasattr(robot, 'servo_controller') and robot.servo_controller:
                    try:
                        # Calculate direct position
                        position_offset = (scaled_target_angle_deg / 90.0) * 0.98
                        target_position = 0.0 + position_offset  # Force center = 0.0
                        
                        success = robot.servo_controller.set_position(target_position)
                        if success:
                            movement_success = True
                            method_used = "servo_controller.set_position(direct)"
                            print(f"📍 DIRECT POSITION: angle={scaled_target_angle_deg:.1f}° → pos={target_position:.3f}")
                    except Exception as e:
                        print(f"⚠️ Direct position control failed: {e}")
                
                if movement_success:
                    self.head_angle_current = math.radians(scaled_target_angle_deg)
                    self.last_head_command_time = current_time
                    self.last_commanded_angle_deg = scaled_target_angle_deg
                    self.movements_sent += 1
                    
                    print(f"🎯 MOVEMENT #{self.movements_sent}: Target={scaled_target_angle_deg:.1f}° "
                          f"(person_x:{x_pixels:.0f}, smoothed:{smoothed_x:.0f}, "
                          f"current:{actual_current_angle_deg:.1f}°, method:{method_used})")
                else:
                    print(f"❌ ALL MOVEMENT METHODS FAILED for {scaled_target_angle_deg:.1f}°")
                      
            except Exception as e:
                print(f"❌ Movement error: {e}")
    
    def independent_csv_logging(self, person_data):
        """CSV logging with diagnostics"""
        try:
            if not self.csv_file_created:
                self.force_create_new_csv_file()
            
            bbox_center = person_data['bbox_center']
            x_midpoint_pixels = bbox_center['x_pixels']
            x_midpoint_normalized = bbox_center['x_normalized']
            
            # Basic metrics
            x_jump = 0
            is_large_jump = False
            if self.last_x_midpoint is not None:
                x_jump = abs(x_midpoint_pixels - self.last_x_midpoint)
                is_large_jump = x_jump > 50
            
            self.last_x_midpoint = x_midpoint_pixels
            self.detection_count += 1
            
            # Get ACTUAL head angle
            head_angle_deg = 0.0
            try:
                robot = self.get_robot()
                if hasattr(robot, 'servo_controller') and robot.servo_controller:
                    head_angle_deg = robot.servo_controller.get_angle_degrees()
                else:
                    head_angle_deg = math.degrees(self.head_angle_current)
            except Exception:
                head_angle_deg = math.degrees(self.head_angle_current)
            
            # Write basic CSV entry
            mode_elapsed = time.time() - self.mode_start_time
            
            with open(self.csv_log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    mode_elapsed, time.time(), self.update_counter,
                    x_midpoint_pixels, x_midpoint_normalized,
                    person_data['x_camera'], person_data['z_camera'], person_data['z_camera'],
                    person_data['confidence'],
                    person_data['bounding_box']['xmin'], person_data['bounding_box']['ymin'],
                    person_data['bounding_box']['xmax'], person_data['bounding_box']['ymax'],
                    person_data['bounding_box']['xmax'] - person_data['bounding_box']['xmin'],
                    person_data['bounding_box']['ymax'] - person_data['bounding_box']['ymin'],
                    x_jump, is_large_jump,
                    0.0, 0.0, x_midpoint_pixels,  # Simplified variance
                    0.0, 0.0, 0.0,  
                    self.detection_count, self.detection_count, 0,
                    'stable', head_angle_deg, self.head_tracker is not None,
                    self.person_is_settled, 'COMPREHENSIVE_FIX',
                    self.filtered_x_position if self.filtered_x_position else x_midpoint_pixels,
                    self.is_in_deadband(x_midpoint_pixels), False
                ])
            
            # Enhanced logging
            if self.detection_count % 25 == 0:
                stats = self.get_anti_oscillation_stats()
                center_x = self.get_screen_center_x()
                distance = abs(x_midpoint_pixels - center_x)
                
                print(f"✅ COMPREHENSIVE CSV: {self.detection_count} rows, settled={self.person_is_settled}, "
                      f"head={head_angle_deg:.1f}°, moves={stats['movements_sent']}, "
                      f"blocks={stats['total_blocks']}")
                print(f"🔍 CURRENT: person={x_midpoint_pixels}px, center={center_x}px, "
                      f"distance={distance:.1f}px, deadband={self.deadband_pixels}px")
            
        except Exception as e:
            print(f"❌ CSV error: {e}")
    
    def get_anti_oscillation_stats(self):
        """Get statistics"""
        total_blocks = self.deadband_blocks + self.frequency_blocks + self.angle_blocks
        total_calls = total_blocks + self.movements_sent
        
        return {
            'movements_sent': self.movements_sent,
            'deadband_blocks': self.deadband_blocks,
            'frequency_blocks': self.frequency_blocks,
            'angle_blocks': self.angle_blocks,
            'total_blocks': total_blocks,
            'total_calls': total_calls,
            'movement_efficiency': f"{(self.movements_sent / max(total_calls, 1) * 100):.1f}%"
        }
    
    def log_detection_to_csv(self, person_data):
        """Use enhanced CSV logging"""
        self.independent_csv_logging(person_data)
    
    def conservative_head_tracking(self, person_data):
        """Disable parent method"""
        pass
    
    def update_conservative_head_tracking(self, person_data):
        """Disable parent method"""
        pass
    
    def update(self):
        """Main update loop"""
        try:
            self.update_counter += 1
            
            person_data = self.get_exact_person_detection()
            
            if person_data:
                bbox_center = person_data['bbox_center']
                x_pixels = bbox_center['x_pixels']
                
                smoothed_x = self.update_settlement_detection(x_pixels)
                self.update_head_tracking_with_hysteresis(smoothed_x)
                self.independent_csv_logging(person_data)
            
            if self.update_counter % 4 == 0:
                try:
                    self.update_display(person_data)
                except Exception:
                    pass
            
            return Status.RUNNING
            
        except Exception as e:
            print(f"⚠️ Update error: {e}")
            return Status.RUNNING
    
    def initialise(self):
        """Enhanced initialization with FORCED centering"""
        super().initialise()
        
        # Reset state
        self.last_significant_movement = time.time()
        self.person_is_settled = False
        self.position_smoothing_window.clear()
        self.head_angle_current = 0.0
        self.head_angle_target = 0.0
        self.last_head_command_time = 0
        self.last_commanded_angle_deg = 0.0
        self.detection_count = 0
        self.filtered_x_position = None
        self.filter_initialized = False
        self.last_smoothed_x = None
        
        # Reset counters
        self.deadband_blocks = 0
        self.frequency_blocks = 0
        self.angle_blocks = 0
        self.movements_sent = 0
        
        # CRITICAL: FORCE TRUE CENTER
        self.initialization_complete = False
        self.initialization_time = time.time()
        self.startup_delay = 3.0
        
        try:
            robot = self.get_robot()
            print("🎯 FORCING HEAD TO TRUE CENTER...")
            
            # FORCE servo controller to use center = 0.0
            if hasattr(robot, 'servo_controller') and robot.servo_controller:
                current_center = robot.servo_controller.get_center_position()
                print(f"🔍 CURRENT CENTER POSITION: {current_center}")
                
                if current_center != 0.0:
                    print(f"⚠️ FIXING: Forcing center from {current_center} to 0.0")
                    # Override the center position temporarily
                    original_center = robot.servo_controller.center_position
                    robot.servo_controller.center_position = 0.0
                    
                # Move to TRUE center (0.0 degrees)
                print("🎯 Moving to absolute 0.0 degrees...")
                robot.servo_controller.move_to_angle(0.0)
                time.sleep(1.0)
                
                # Verify position
                final_pos = robot.servo_controller.get_position()
                final_angle = robot.servo_controller.get_angle_degrees()
                print(f"🎯 CENTERED: position={final_pos:.3f}, angle={final_angle:.1f}°")
                
        except Exception as e:
            print(f"⚠️ Centering error: {e}")
        
        self.force_create_new_csv_file()
        
        print("✅ COMPREHENSIVE LiDAR Test initialized - All diagnostics active")
    
    def terminate(self, new_status):
        """Enhanced termination"""
        try:
            stats = self.get_anti_oscillation_stats()
            print("📊 COMPREHENSIVE RESULTS:")
            print(f"   Movements sent: {stats['movements_sent']}")
            print(f"   Deadband blocks: {stats['deadband_blocks']}")
            print(f"   Frequency blocks: {stats['frequency_blocks']}")
            print(f"   Angle blocks: {stats['angle_blocks']}")
            print(f"   Efficiency: {stats['movement_efficiency']}")
            print(f"   Coordinate system verified: {self.coordinate_system_verified}")
            
        except Exception:
            pass
        
        super().terminate(new_status)


# Export the comprehensive class
__all__ = ['FixedSettlementLidarTest']