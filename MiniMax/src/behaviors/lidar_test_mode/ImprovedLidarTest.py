"""
LIDAR Test Mode with PROPER centering at 960 pixels
Fixed to actually center the person and record CSV data
"""

import time
import math
import pygame
from py_trees.common import Status
from collections import deque

# Import base class
from .StandaloneLidarTest import ExactStandaloneLidarTest


class PIDController:
    """
    PID Controller for smooth movement without overshoot
    DIRECTLY FROM WORKING LidarTestBehavior.py
    """
    def __init__(self, kp=1.0, ki=0.1, kd=0.05, max_output=10.0, min_output=-10.0):
        self.kp = kp          # Proportional gain
        self.ki = ki          # Integral gain  
        self.kd = kd          # Derivative gain
        self.max_output = max_output    # Maximum output
        self.min_output = min_output    # Minimum output
        
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
    def reset(self):
        """Reset PID controller state"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
    def update(self, setpoint, current_value):
        """
        Calculate PID output for smooth movement
        """
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            dt = 0.01
            
        error = setpoint - current_value
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term (with windup protection)
        self.integral += error * dt
        self.integral = max(-5.0, min(5.0, self.integral))
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        
        # Calculate output
        output = proportional + integral + derivative
        
        # Limit output
        output = max(self.min_output, min(self.max_output, output))
        
        self.previous_error = error
        self.last_time = current_time
        
        return output


class ProperCenteringController:
    """
    Head tracking that ACTUALLY centers the person at 960 pixels
    """
    def __init__(self, robot):
        self.robot = robot
        
        # TARGET: Center person at 960 pixels!
        self.target_pixel_x = 960  # CENTER OF SCREEN
        self.pixel_tolerance = 30  # Consider centered if within ±30 pixels of 960
        
        # Head parameters
        self.head_max_angle = 85.0
        self.head_move_delay = 0.05  # 50ms between movements
        self.last_head_move_time = 0
        
        # Current state
        self.current_head_angle = 0
        self.last_person_x = 960
        
        # Anti-twitch: Only move if person moves significantly
        self.movement_threshold = 20  # pixels
        self.last_commanded_angle = 0
        
        # PID for smooth movement
        self.head_pid = PIDController(
            kp=0.8,      # Increased for more aggressive centering
            ki=0.02,     # Small integral
            kd=0.3,      # Good damping
            max_output=8.0,   # Larger steps allowed
            min_output=-8.0
        )
        
        # Initialize
        self.center_head_safely()

    def center_head_safely(self):
        """Center head at startup"""
        try:
            if hasattr(self.robot, 'head_velocity_manager') and self.robot.head_velocity_manager:
                self.robot.head_velocity_manager.set_head_position(0.0)
                time.sleep(0.3)
                return True
        except:
            pass
        
        try:
            if hasattr(self.robot, 'servo_controller') and self.robot.servo_controller:
                self.robot.servo_controller.move_to_angle(0.0)
                time.sleep(0.3)
                return True
        except:
            pass
        
        return False

    def get_current_head_angle(self):
        """Get current head angle in degrees"""
        try:
            if hasattr(self.robot, 'servo_controller') and self.robot.servo_controller:
                return self.robot.servo_controller.get_angle_degrees()
            elif hasattr(self.robot, 'head_velocity_manager') and self.robot.head_velocity_manager:
                servo_position = self.robot.head_velocity_manager.get_head_position()
                if servo_position is not None:
                    head_angle_degrees = servo_position * (90.0 / 0.98)
                    return max(-90.0, min(90.0, head_angle_degrees))
        except:
            pass
        return 0.0

    def set_head_angle(self, angle_degrees):
        """Set head to specific angle"""
        try:
            angle_degrees = max(-self.head_max_angle, min(self.head_max_angle, angle_degrees))
            
            if hasattr(self.robot, 'head_velocity_manager') and self.robot.head_velocity_manager:
                servo_position = angle_degrees / 90.0 * 0.98
                servo_position = max(-0.98, min(0.98, servo_position))
                return self.robot.head_velocity_manager.set_head_position(servo_position)
            elif hasattr(self.robot, 'servo_controller') and self.robot.servo_controller:
                return self.robot.servo_controller.move_to_angle(angle_degrees)
        except:
            pass
        return False

    def can_move_head(self):
        """Check if enough time has passed"""
        current_time = time.time()
        return (current_time - self.last_head_move_time) >= self.head_move_delay

    def track_person_to_center(self, person_x_pixels):
        """
        Track person to CENTER them at 960 pixels
        Returns: status string
        """
        if person_x_pixels is None:
            return "NO_PERSON"
        
        # Calculate pixel error from target (960)
        pixel_error = person_x_pixels - self.target_pixel_x
        
        # Check if already centered
        if abs(pixel_error) < self.pixel_tolerance:
            return f"CENTERED at {person_x_pixels:.0f}px"
        
        # Anti-twitch: Check if person moved significantly
        if abs(person_x_pixels - self.last_person_x) < self.movement_threshold:
            # Person hasn't moved much, don't twitch
            if abs(pixel_error) < self.pixel_tolerance * 2:
                return f"STABLE at {person_x_pixels:.0f}px"
        
        self.last_person_x = person_x_pixels
        
        # Calculate required angle adjustment
        # Estimate: ~2.5 degrees of head movement shifts view by ~100 pixels
        # This is an approximation that needs tuning
        pixels_per_degree = 40  # Tune this value based on your camera FOV
        
        # Current angle
        current_angle = self.get_current_head_angle()
        
        # Target angle to center the person
        # If person is at 500px (460px left of center), need to turn RIGHT
        # If person is at 1400px (440px right of center), need to turn LEFT
        angle_adjustment = -pixel_error / pixels_per_degree  # Negative because we need opposite movement
        target_angle = current_angle + angle_adjustment
        
        # Limit target angle
        target_angle = max(-self.head_max_angle, min(self.head_max_angle, target_angle))
        
        # Move head if enough time has passed
        if self.can_move_head():
            # Use PID for smooth movement
            pid_output = self.head_pid.update(target_angle, current_angle)
            new_angle = current_angle + pid_output
            new_angle = max(-self.head_max_angle, min(self.head_max_angle, new_angle))
            
            # Only move if change is significant (anti-twitch)
            if abs(new_angle - self.last_commanded_angle) > 0.5:
                if self.set_head_angle(new_angle):
                    self.last_head_move_time = time.time()
                    self.current_head_angle = new_angle
                    self.last_commanded_angle = new_angle
                    
                    if pixel_error < 0:
                        return f"Person at {person_x_pixels:.0f}px → Turning RIGHT to center"
                    else:
                        return f"Person at {person_x_pixels:.0f}px → Turning LEFT to center"
        
        return f"WAITING (Person at {person_x_pixels:.0f}px)"


class ImprovedLidarTest(ExactStandaloneLidarTest):
    """
    LIDAR test that PROPERLY centers the person at 960 pixels
    """
    
    def __init__(self):
        """Initialize"""
        super().__init__()
        self.name = "LIDAR Test - Proper Centering at 960px"
        
        # Screen parameters
        self.screen_width = 1920
        self.screen_center = 960
        
        # Controller
        self.centering_controller = None
        
        # Statistics
        self.movements_sent = 0
        self.tracking_status = "INITIALIZING"
        
        # Person detection smoothing
        self.person_buffer = deque(maxlen=8)  # More samples for stability
        
        # For display
        self.last_person_x_pixels = 960
        self.current_head_angle = 0
        
    def initialise(self):
        """Initialize the behavior"""
        super().initialise()
        
        print("\n" + "="*70)
        print("🎯 LIDAR TEST - PROPER CENTERING AT 960 PIXELS")
        print("="*70)
        print("📋 Goal: Center detected person at X=960 (screen center)")
        print("📋 Using PID control with anti-twitch")
        print("📋 CSV logging enabled")
        print("="*70 + "\n")
        
        # Get robot
        robot = self.get_robot()
        
        # Initialize controller
        self.centering_controller = ProperCenteringController(robot)
        
        # Reset
        self.movements_sent = 0
        self.tracking_status = "READY"
        self.person_buffer.clear()
        self.last_person_x_pixels = 960
        self.current_head_angle = 0
        
        print("✅ System ready - will center person at 960 pixels\n")
    
    def update(self):
        """Main update loop"""
        try:
            self.update_counter += 1
            
            # Get person detection
            person_data = self.get_exact_person_detection()
            
            if person_data and self.centering_controller:
                # Extract person position in pixels
                bbox_center = person_data.get('bbox_center', {})
                person_x_pixels = bbox_center.get('x_pixels', self.screen_center)
                
                # Add to buffer for smoothing
                self.person_buffer.append(person_x_pixels)
                
                # Use weighted average (recent samples weighted more)
                if len(self.person_buffer) >= 3:
                    weights = list(range(1, len(self.person_buffer) + 1))
                    weighted_sum = sum(x * w for x, w in zip(self.person_buffer, weights))
                    total_weight = sum(weights)
                    smoothed_x = weighted_sum / total_weight
                    
                    # Track person to center them at 960
                    self.tracking_status = self.centering_controller.track_person_to_center(smoothed_x)
                    
                    # Update stats
                    self.last_person_x_pixels = smoothed_x
                    self.current_head_angle = self.centering_controller.get_current_head_angle()
                    
                    if "Turning" in self.tracking_status:
                        self.movements_sent += 1
                        if self.movements_sent % 5 == 0:  # Print every 5th movement
                            pixel_error = smoothed_x - 960
                            print(f"📍 Movement #{self.movements_sent}: "
                                  f"Person at {smoothed_x:.0f}px (error: {pixel_error:+.0f}px) "
                                  f"Head: {self.current_head_angle:.1f}° "
                                  f"Status: {self.tracking_status}")
                
                # Log to CSV - add head angle to data
                try:
                    # Ensure head_angle is in person_data for CSV
                    person_data['head_angle_deg'] = self.current_head_angle
                    person_data['head_tracking_active'] = True
                    
                    # Use parent's CSV logging if available
                    if hasattr(super(), 'independent_csv_logging'):
                        super().independent_csv_logging(person_data)
                except Exception as e:
                    if self.update_counter % 100 == 0:
                        print(f"⚠️ CSV logging error: {e}")
            else:
                self.tracking_status = "NO_PERSON"
                self.last_person_x_pixels = 960
            
            # Update display
            if self.update_counter % 10 == 0:
                try:
                    self.update_display(person_data)
                    self._draw_centering_info()
                except:
                    pass
            
            return Status.RUNNING
            
        except Exception as e:
            if self.update_counter % 100 == 0:
                print(f"⚠️ Update error: {e}")
            return Status.RUNNING
    
    def _draw_centering_info(self):
        """Draw centering information on display"""
        try:
            if not hasattr(self, 'screen') or not self.screen:
                return
            
            font = pygame.font.Font(None, 28)
            small_font = pygame.font.Font(None, 20)
            
            # Calculate pixel error
            pixel_error = self.last_person_x_pixels - 960
            
            # Status info
            info = [
                f"TARGET: Center person at X=960",
                f"Person X: {self.last_person_x_pixels:.0f}px",
                f"Error: {pixel_error:+.0f}px from center",
                f"Head Angle: {self.current_head_angle:.1f}°",
                f"Status: {self.tracking_status}",
                f"Movements: {self.movements_sent}",
            ]
            
            y = 100
            for i, line in enumerate(info):
                # Color coding
                if i == 0:  # Target line
                    color = (0, 255, 255)  # Cyan
                elif "CENTERED" in line:
                    color = (0, 255, 0)  # Green
                elif abs(pixel_error) < 100 and i == 2:
                    color = (255, 255, 0)  # Yellow - close
                elif abs(pixel_error) > 200 and i == 2:
                    color = (255, 0, 0)  # Red - far
                elif "Turning" in line:
                    color = (255, 165, 0)  # Orange
                else:
                    color = (200, 200, 200)  # Light gray
                
                text = font.render(line, True, color)
                self.screen.blit(text, (10, y))
                y += 35
            
            # Draw center line indicator
            center_x = 960
            pygame.draw.line(self.screen, (0, 255, 255), 
                           (center_x, 50), (center_x, 100), 2)
            
            # Draw person position indicator
            if self.last_person_x_pixels != 960:
                person_x_screen = int(self.last_person_x_pixels * self.screen.get_width() / 1920)
                pygame.draw.line(self.screen, (255, 0, 0),
                               (person_x_screen, 50), (person_x_screen, 100), 2)
                
        except:
            pass
    
    def terminate(self, new_status):
        """Cleanup on termination"""
        try:
            print("\n" + "="*70)
            print(f"📊 Final Statistics:")
            print(f"   • Total movements: {self.movements_sent}")
            print(f"   • Final person X: {self.last_person_x_pixels:.0f}px")
            print(f"   • Final error from 960: {self.last_person_x_pixels - 960:+.0f}px")
            print(f"   • Final head angle: {self.current_head_angle:.1f}°")
            
            if self.centering_controller:
                # Center head
                print("🎯 Centering head before exit...")
                self.centering_controller.center_head_safely()
            
            print("="*70 + "\n")
            
            # Call parent terminate
            super().terminate(new_status)
            
        except Exception as e:
            print(f"⚠️ Termination error: {e}")
            super().terminate(new_status)
    
    # Override parent's CSV logging to ensure it works
    def independent_csv_logging(self, person_data):
        """Ensure CSV logging works"""
        try:
            # Call parent's CSV logging
            if hasattr(super(), 'independent_csv_logging'):
                super().independent_csv_logging(person_data)
            else:
                # If parent doesn't have it, do basic logging
                print(f"CSV: Person at {person_data.get('bbox_center', {}).get('x_pixels', 0):.0f}px, "
                      f"Head: {self.current_head_angle:.1f}°")
        except Exception as e:
            pass


# Export the improved class
__all__ = ['ImprovedLidarTest']