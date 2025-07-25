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
    Asynchronous head tracking system for keeping camera centered on targets
    Runs in background thread without blocking main robot operations
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
        self.position_tolerance = 0.05 # 5% tolerance for position reached
        
        # Smoothing and speed control
        self.max_movement_speed = 0.4  # Maximum movement speed per update (increased for responsiveness)
        self.tracking_update_rate = 0.1  # Update every 100ms
        self.smoothing_factor = 0.8    # How much to smooth movements (increased for more responsive tracking)
        
        # Person tracking state
        self.person_angle_history = []
        self.max_history_length = 3   # Reduced for more responsive tracking
        self.tracking_deadband = math.radians(3)  # Smaller deadband for more precise tracking
        
        # Scanning parameters (for when person is lost)
        self.scan_positions = [-0.7, -0.4, 0.0, 0.4, 0.7]  # Wider scan range
        self.scan_index = 0
        self.scan_dwell_time = 0.8     # Faster scanning
        self.last_scan_move = 0
        
        # Threading control
        self.tracking_thread = None
        self.running = False
        self.thread_lock = threading.Lock()
        
        # Status tracking
        self.last_update_time = 0
        self.last_person_seen = 0
        self.person_lost_timeout = 1.5  # Start scanning sooner when person is lost
        
    def start_tracking(self):
        """Start the head tracking thread"""
        if not self.running:
            self.running = True
            self.tracking_thread = threading.Thread(target=self._tracking_loop, daemon=True)
            self.tracking_thread.start()
    
    def stop_tracking(self):
        """Stop the head tracking thread"""
        self.running = False
        if self.tracking_thread and self.tracking_thread.is_alive():
            self.tracking_thread.join(timeout=1.0)
        
        # Center head on stop
        self.set_manual_position(0.0)
        time.sleep(0.3)  # Give time for centering
    
    def set_person_tracking(self, person_angle_rad: float):
        """
        Set head to track a person at the given angle
        
        Args:
            person_angle_rad: Person angle in radians relative to robot center
                             (negative = left, positive = right)
        """
        with self.thread_lock:
            self.tracking_mode = TrackingMode.PERSON_TRACKING
            
            # Convert person angle to head position
            target_head_position = self._angle_to_head_position(person_angle_rad)
            
            # Add to history for smoothing
            self.person_angle_history.append(person_angle_rad)
            if len(self.person_angle_history) > self.max_history_length:
                self.person_angle_history.pop(0)
            
            # Calculate smoothed target position
            if len(self.person_angle_history) >= 2:
                # Use recent weighted average for smooth but responsive tracking
                weights = [0.3, 0.7] if len(self.person_angle_history) == 2 else [0.2, 0.3, 0.5]
                weighted_sum = sum(angle * weight for angle, weight in zip(self.person_angle_history, weights))
                avg_angle = weighted_sum
                target_head_position = self._angle_to_head_position(avg_angle)
            
            # Only update if change is significant (deadband)
            if len(self.person_angle_history) >= 2:
                angle_diff = abs(person_angle_rad - self.person_angle_history[-2])
            else:
                angle_diff = float('inf')  # Always update on first detection
                
            if angle_diff > self.tracking_deadband or len(self.person_angle_history) == 1:
                self.target_position = target_head_position
            
            self.last_person_seen = time.time()
    
    def set_manual_position(self, position: float):
        """
        Set head to manual position
        
        Args:
            position: Head position (-1.0 = full left, 1.0 = full right, 0.0 = center)
        """
        with self.thread_lock:
            self.tracking_mode = TrackingMode.MANUAL_POSITION
            self.target_position = max(self.head_limit_left, min(self.head_limit_right, position))
    
    def enable_scanning(self):
        """Enable scanning mode (sweep head to search for person)"""
        with self.thread_lock:
            # Only switch to scanning if not already tracking a person
            if self.tracking_mode != TrackingMode.PERSON_TRACKING or time.time() - self.last_person_seen > self.person_lost_timeout:
                self.tracking_mode = TrackingMode.SCANNING
                self.scan_index = 0
                self.last_scan_move = time.time()
    
    def disable_tracking(self):
        """Disable head tracking"""
        with self.thread_lock:
            self.tracking_mode = TrackingMode.DISABLED
    
    def _angle_to_head_position(self, angle_rad: float) -> float:
        """
        Convert person angle to head position
        
        Args:
            angle_rad: Angle in radians (negative = left, positive = right)
            
        Returns:
            Head position (-1.0 to 1.0)
        """
        # Convert radians to head position
        # Assuming max head range is about 90 degrees each way
        max_head_angle = math.radians(90)
        
        # FIXED: Direct mapping - if person is to the right (positive angle), 
        # head should turn right (positive position)
        head_position = angle_rad / max_head_angle
        
        # Clamp to head limits and apply a gain factor for better tracking
        head_position = head_position * 1.2  # Slight amplification for more responsive tracking
        
        return max(self.head_limit_left, min(self.head_limit_right, head_position))
    
    def _execute_head_movement(self, target_pos: float):
        """
        Execute smooth head movement to target position
        Non-blocking implementation
        """
        try:
            # Calculate smooth movement step
            position_diff = target_pos - self.current_position
            
            # Apply smoothing and speed limiting
            max_step = self.max_movement_speed * self.tracking_update_rate
            if abs(position_diff) > max_step:
                move_step = max_step if position_diff > 0 else -max_step
            else:
                move_step = position_diff * self.smoothing_factor
            
            new_position = self.current_position + move_step
            
            # Only move if change is significant
            if abs(move_step) > 0.008:  # Smaller threshold for more precise tracking
                # Try head_velocity_manager first
                if self.head_velocity_manager:
                    try:
                        # Use non-blocking movement
                        self.head_velocity_manager.set_head_position(new_position, wait_for_completion=False)
                        self.current_position = new_position
                        return True
                    except Exception:
                        pass
                
                # Fallback to servo_controller
                if self.servo_controller:
                    try:
                        self.servo_controller.set_position(new_position)
                        self.current_position = new_position
                        return True
                    except Exception:
                        pass
            
            return False
            
        except Exception:
            return False
    
    def _tracking_loop(self):
        """Main tracking loop running in background thread"""
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
                    # Scanning mode - sweep head to find person
                    if current_time - self.last_scan_move > self.scan_dwell_time:
                        self.scan_index = (self.scan_index + 1) % len(self.scan_positions)
                        with self.thread_lock:
                            self.target_position = self.scan_positions[self.scan_index]
                            self.last_scan_move = current_time
                    
                    self._execute_head_movement(current_target)
                
                elif current_mode == TrackingMode.MANUAL_POSITION:
                    # Manual position mode
                    self._execute_head_movement(current_target)
                
                # Update timing
                self.last_update_time = current_time
                
                # Sleep for update rate
                time.sleep(self.tracking_update_rate)
                
            except Exception:
                # Continue running even if there's an error
                time.sleep(self.tracking_update_rate)
    
    def get_status(self) -> dict:
        """Get current tracking status"""
        with self.thread_lock:
            return {
                'tracking_mode': self.tracking_mode.value,
                'current_position': self.current_position,
                'target_position': self.target_position,
                'running': self.running,
                'last_person_seen_ago': time.time() - self.last_person_seen if self.last_person_seen > 0 else -1,
                'person_tracking_active': self.tracking_mode == TrackingMode.PERSON_TRACKING,
                'scan_position_index': self.scan_index if self.tracking_mode == TrackingMode.SCANNING else -1,
                'position_history_length': len(self.person_angle_history)
            }
    
    def is_position_reached(self, tolerance: float = None) -> bool:
        """Check if head has reached target position"""
        if tolerance is None:
            tolerance = self.position_tolerance
        
        with self.thread_lock:
            return abs(self.current_position - self.target_position) <= tolerance
    
    def reset_tracking_history(self):
        """Reset person tracking history"""
        with self.thread_lock:
            self.person_angle_history.clear()
            self.last_person_seen = 0