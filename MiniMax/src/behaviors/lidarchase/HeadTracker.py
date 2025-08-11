import math
import time
import threading
from typing import Optional, Tuple
from enum import Enum


class TrackingMode(Enum):
    """Head tracking modes"""
    DISABLED = "disabled"
    PERSON_TRACKING = "person_tracking" 
    MANUAL_POSITION = "manual_position"
    SCANNING = "scanning"


class HeadTracker:
    """
    Anti-oscillation head tracking system for keeping camera centered on targets
    IMPROVED with deadband, filtering, and reduced movement frequency
    """
    
    def __init__(self, head_velocity_manager, servo_controller=None):
        self.head_velocity_manager = head_velocity_manager
        self.servo_controller = servo_controller
        
        # Head physical limits (normalized: -1.0 = full left, +1.0 = full right)
        self.head_limit_left = -1.0    # Full left position
        self.head_limit_right = 1.0    # Full right position
        self.head_center = 0.0         # Center position
        
        # Tracking parameters
        self.tracking_mode = TrackingMode.DISABLED
        self.target_position = 0.0     # Target head position (-1.0 to 1.0)
        self.current_position = 0.0    # Current head position
        self.position_tolerance = 0.08 # INCREASED tolerance from 0.05 for stability
        
        # ANTI-OSCILLATION: Reduced movement speed and frequency
        self.max_movement_speed = 0.15  # REDUCED from 0.4 for gentler movement
        self.tracking_update_rate = 0.2  # SLOWER updates - every 200ms instead of 100ms
        self.smoothing_factor = 0.3    # REDUCED from 0.8 for less aggressive tracking
        
        # DEADBAND ZONE: 200 pixel deadband around screen center (1920x1080 screen)
        self.deadband_pixels = 200     # Don't move if within 200px of center
        self.screen_width = 1920       # Assuming 1920x1080 display
        self.deadband_normalized = self.deadband_pixels / (self.screen_width / 2)  # ~0.208
        
        # LOW-PASS FILTER: Exponential smoothing for camera X position
        self.filtered_x_position = 0.0
        self.filter_alpha = 0.2        # Smoothing factor - lower = more filtering
        self.filter_initialized = False
        
        # Person tracking state with REDUCED sensitivity
        self.person_angle_history = []
        self.max_history_length = 5   # INCREASED from 3 for more smoothing
        self.tracking_deadband = math.radians(8)  # INCREASED from 3 degrees for stability
        
        # MOVEMENT FREQUENCY CONTROL: Reduce servo command frequency
        self.last_movement_time = 0
        self.min_movement_interval = 0.5  # Wait at least 500ms between movements
        self.last_position_command = 0.0
        self.min_position_change = 0.03   # Minimum change to trigger movement
        
        # Scanning parameters (for when person is lost)
        self.scan_positions = [-0.5, -0.25, 0.0, 0.25, 0.5]  # REDUCED scan range
        self.scan_index = 0
        self.scan_dwell_time = 1.2     # SLOWER scanning for stability
        self.last_scan_move = 0
        
        # Threading control
        self.tracking_thread = None
        self.running = False
        self.thread_lock = threading.Lock()
        
        # Status tracking
        self.last_update_time = 0
        self.last_person_seen = 0
        self.person_lost_timeout = 2.0  # INCREASED timeout before scanning
        
        # Debug/status info
        self.movement_count = 0
        self.filtered_movements = 0
        self.deadband_blocks = 0
    
    def start_tracking(self):
        """Start the tracking thread"""
        if not self.running:
            self.running = True
            self.tracking_thread = threading.Thread(target=self._tracking_loop, daemon=True)
            self.tracking_thread.start()
    
    def stop_tracking(self):
        """Stop the tracking thread"""
        self.running = False
        if self.tracking_thread and self.tracking_thread.is_alive():
            self.tracking_thread.join(timeout=2.0)
    
    def set_person_tracking(self, person_angle_rad: float):
        """
        Set person tracking angle with IMPROVED filtering and deadband
        
        arguments:
            - person_angle_rad: Angle to person in radians
        """
        current_time = time.time()
        
        # Convert angle to normalized screen position for deadband check
        # Assuming 108° horizontal FOV (54° each side)
        max_angle_rad = math.radians(54)
        if abs(person_angle_rad) > max_angle_rad:
            person_angle_rad = max_angle_rad if person_angle_rad > 0 else -max_angle_rad
        
        # Convert to normalized position (-1.0 to 1.0)
        normalized_position = person_angle_rad / max_angle_rad
        
        # DEADBAND CHECK: Don't move if person is in center zone
        if abs(normalized_position) < self.deadband_normalized:
            self.deadband_blocks += 1
            return  # Person is in deadband - don't move
        
        # LOW-PASS FILTER: Apply exponential smoothing
        if not self.filter_initialized:
            self.filtered_x_position = normalized_position
            self.filter_initialized = True
        else:
            self.filtered_x_position = (self.filter_alpha * normalized_position + 
                                      (1.0 - self.filter_alpha) * self.filtered_x_position)
        
        # MOVEMENT FREQUENCY CONTROL: Don't move too often
        if current_time - self.last_movement_time < self.min_movement_interval:
            self.filtered_movements += 1
            return
        
        # Check if change is significant enough
        position_change = abs(self.filtered_x_position - self.last_position_command)
        if position_change < self.min_position_change:
            return
        
        # Update tracking state
        with self.thread_lock:
            self.tracking_mode = TrackingMode.PERSON_TRACKING
            self.target_position = self.filtered_x_position
            self.last_person_seen = current_time
            self.last_movement_time = current_time
            self.last_position_command = self.filtered_x_position
            self.movement_count += 1
    
    def set_manual_position(self, position: float):
        """Set manual head position"""
        with self.thread_lock:
            self.tracking_mode = TrackingMode.MANUAL_POSITION
            self.target_position = max(self.head_limit_left, min(self.head_limit_right, position))
    
    def enable_scanning(self):
        """Enable scanning mode"""
        with self.thread_lock:
            self.tracking_mode = TrackingMode.SCANNING
            self.scan_index = 0
            self.last_scan_move = time.time()
    
    def disable_tracking(self):
        """Disable all tracking"""
        with self.thread_lock:
            self.tracking_mode = TrackingMode.DISABLED
    
    def get_status(self):
        """Get current tracking status"""
        with self.thread_lock:
            return {
                'mode': self.tracking_mode.value,
                'target_position': self.target_position,
                'current_position': self.current_position,
                'last_person_seen': self.last_person_seen,
                'movement_count': self.movement_count,
                'filtered_movements': self.filtered_movements,
                'deadband_blocks': self.deadband_blocks,
                'filtered_x_position': self.filtered_x_position
            }
    
    def _execute_head_movement(self, target_position: float):
        """
        Execute smooth head movement with IMPROVED anti-oscillation control
        """
        try:
            # Get current position
            current_pos = self.current_position
            if self.head_velocity_manager:
                try:
                    actual_pos = self.head_velocity_manager.get_head_position()
                    if actual_pos is not None:
                        current_pos = actual_pos
                except Exception:
                    pass
            elif self.servo_controller:
                try:
                    actual_pos = self.servo_controller.get_position()
                    if actual_pos is not None:
                        # Convert servo position to normalized position
                        center_pos = self.servo_controller.get_center_position()
                        current_pos = (actual_pos - center_pos) / 0.98
                except Exception:
                    pass
            
            # Calculate movement needed
            position_diff = target_position - current_pos
            
            # Apply REDUCED movement speed and smoothing
            max_step = self.max_movement_speed * self.tracking_update_rate
            if abs(position_diff) > max_step:
                move_step = max_step if position_diff > 0 else -max_step
            else:
                move_step = position_diff * self.smoothing_factor
            
            new_position = current_pos + move_step
            
            # Only move if change is significant (INCREASED threshold)
            if abs(move_step) > 0.02:  # INCREASED from 0.008 for fewer commands
                # Try head_velocity_manager first
                if self.head_velocity_manager:
                    try:
                        # Convert normalized position back to servo position
                        center_pos = 0.15 if hasattr(self.head_velocity_manager.servo_controller, 'center_position') else 0.0
                        servo_position = center_pos + (new_position * 0.98)
                        
                        # Use non-blocking movement
                        self.head_velocity_manager.set_head_position(servo_position, wait_for_completion=False)
                        self.current_position = new_position
                        return True
                    except Exception:
                        pass
                
                # Fallback to servo_controller
                if self.servo_controller:
                    try:
                        # Convert normalized position to servo position
                        center_pos = self.servo_controller.get_center_position()
                        servo_position = center_pos + (new_position * 0.98)
                        
                        self.servo_controller.set_position(servo_position)
                        self.current_position = new_position
                        return True
                    except Exception:
                        pass
            
            return False
            
        except Exception:
            return False
    
    def _execute_scanning(self):
        """Execute scanning movement with REDUCED frequency"""
        current_time = time.time()
        
        # Check if it's time to move to next scan position
        if current_time - self.last_scan_move < self.scan_dwell_time:
            return
        
        # Move to next scan position
        target_position = self.scan_positions[self.scan_index]
        
        if self._execute_head_movement(target_position):
            self.scan_index = (self.scan_index + 1) % len(self.scan_positions)
            self.last_scan_move = current_time
    
    def _tracking_loop(self):
        """Main tracking loop running in background thread with REDUCED frequency"""
        while self.running:
            try:
                current_time = time.time()
                
                with self.thread_lock:
                    current_mode = self.tracking_mode
                    current_target = self.target_position
                
                if current_mode == TrackingMode.PERSON_TRACKING:
                    # Check if person was lost and switch to scanning
                    if current_time - self.last_person_seen > self.person_lost_timeout:
                        self.enable_scanning()
                        continue
                    
                    # Execute smooth movement towards person
                    self._execute_head_movement(current_target)
                
                elif current_mode == TrackingMode.SCANNING:
                    self._execute_scanning()
                
                elif current_mode == TrackingMode.MANUAL_POSITION:
                    self._execute_head_movement(current_target)
                
                # Update less frequently for stability
                time.sleep(self.tracking_update_rate)
                
            except Exception:
                time.sleep(0.1)  # Brief pause on error
                continue
    
    def reset_filters(self):
        """Reset all filters and state"""
        self.filtered_x_position = 0.0
        self.filter_initialized = False
        self.last_movement_time = 0
        self.last_position_command = 0.0
        self.movement_count = 0
        self.filtered_movements = 0
        self.deadband_blocks = 0
        self.person_angle_history.clear()
    
    def get_debug_info(self):
        """Get debug information about filtering and movement"""
        return {
            'deadband_pixels': self.deadband_pixels,
            'deadband_normalized': self.deadband_normalized,
            'filtered_x_position': self.filtered_x_position,
            'filter_alpha': self.filter_alpha,
            'movement_count': self.movement_count,
            'filtered_movements': self.filtered_movements,
            'deadband_blocks': self.deadband_blocks,
            'min_movement_interval': self.min_movement_interval,
            'max_movement_speed': self.max_movement_speed,
            'tracking_update_rate': self.tracking_update_rate
        }