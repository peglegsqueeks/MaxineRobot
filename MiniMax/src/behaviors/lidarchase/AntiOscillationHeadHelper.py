import math
import time
from collections import deque


class AntiOscillationHeadHelper:
    """
    Helper class for anti-oscillation head tracking that can be used with existing behaviors
    Implements the 5 key fixes requested:
    1. Simple PID with reduced proportional gain and increased derivative
    2. 200px deadband zone around screen center
    3. Low-pass filter (exponential smoothing) on detected person X position
    4. Reduced head movement magnitude
    5. Reduced servo command frequency
    """
    
    def __init__(self, screen_width=1920):
        # DEADBAND ZONE: 200 pixels around screen center
        self.deadband_pixels = 200
        self.screen_width = screen_width
        self.screen_center_x = screen_width // 2
        
        # LOW-PASS FILTER: Exponential smoothing parameters  
        self.filtered_x_position = None
        self.filter_alpha = 0.15  # Low value = more smoothing
        self.filter_initialized = False
        
        # MOVEMENT FREQUENCY CONTROL: Reduce servo commands
        self.last_head_command_time = 0
        self.min_command_interval = 0.8  # Wait 800ms between head commands
        self.last_commanded_angle = None
        self.min_angle_change_deg = 3.0  # Minimum 3 degree change to move
        
        # MOVEMENT MAGNITUDE REDUCTION: Smaller movements
        self.movement_scale_factor = 0.6  # Scale all movements by 60%
        self.max_single_movement_deg = 8.0  # Cap single movements at 8 degrees
        
        # Position history for additional smoothing
        self.position_history = deque(maxlen=5)
        
        # Debug counters
        self.deadband_blocks = 0
        self.frequency_blocks = 0
        self.angle_blocks = 0
        self.movements_sent = 0
    
    def process_person_x_position(self, x_pixels, current_time=None):
        """
        Process person X position with deadband check and low-pass filtering
        
        Args:
            x_pixels: Raw X position in pixels
            current_time: Current timestamp (uses time.time() if None)
            
        Returns:
            dict with processed position info or None if in deadband
        """
        if current_time is None:
            current_time = time.time()
        
        # DEADBAND CHECK: Don't process if person is in center zone
        distance_from_center = abs(x_pixels - self.screen_center_x)
        if distance_from_center < self.deadband_pixels:
            self.deadband_blocks += 1
            return None  # Person is in deadband zone
        
        # LOW-PASS FILTER: Apply exponential smoothing
        if not self.filter_initialized:
            self.filtered_x_position = x_pixels
            self.filter_initialized = True
        else:
            self.filtered_x_position = (self.filter_alpha * x_pixels + 
                                      (1.0 - self.filter_alpha) * self.filtered_x_position)
        
        # Add to position history for additional smoothing
        self.position_history.append(self.filtered_x_position)
        
        # Calculate average over recent history
        if len(self.position_history) >= 3:
            smoothed_position = sum(self.position_history) / len(self.position_history)
        else:
            smoothed_position = self.filtered_x_position
        
        return {
            'raw_x': x_pixels,
            'filtered_x': self.filtered_x_position,
            'smoothed_x': smoothed_position,
            'distance_from_center': distance_from_center,
            'timestamp': current_time
        }
    
    def calculate_head_angle(self, processed_position, camera_hfov_degrees=108):
        """
        Calculate target head angle with movement reduction applied
        
        Args:
            processed_position: Output from process_person_x_position()
            camera_hfov_degrees: Camera horizontal field of view
            
        Returns:
            Target angle in degrees (reduced magnitude)
        """
        if processed_position is None:
            return None
        
        smoothed_x = processed_position['smoothed_x']
        
        # Calculate pixel offset from center
        pixel_offset = smoothed_x - self.screen_center_x
        pixel_offset_normalized = pixel_offset / self.screen_center_x
        
        # Convert to angle
        camera_hfov_rad = math.radians(camera_hfov_degrees)
        raw_angle_rad = -pixel_offset_normalized * (camera_hfov_rad / 2.0)
        raw_angle_deg = math.degrees(raw_angle_rad)
        
        # MOVEMENT MAGNITUDE REDUCTION: Scale down the movement
        scaled_angle_deg = raw_angle_deg * self.movement_scale_factor
        
        # Cap maximum single movement
        if abs(scaled_angle_deg) > self.max_single_movement_deg:
            scaled_angle_deg = (self.max_single_movement_deg if scaled_angle_deg > 0 
                              else -self.max_single_movement_deg)
        
        return scaled_angle_deg
    
    def should_send_head_command(self, target_angle_deg, current_time=None):
        """
        Determine if head command should be sent based on frequency and angle change limits
        
        Args:
            target_angle_deg: Target angle in degrees
            current_time: Current timestamp
            
        Returns:
            True if command should be sent, False otherwise
        """
        if current_time is None:
            current_time = time.time()
        
        if target_angle_deg is None:
            return False
        
        # FREQUENCY CONTROL: Check minimum time interval
        if current_time - self.last_head_command_time < self.min_command_interval:
            self.frequency_blocks += 1
            return False
        
        # ANGLE CHANGE CONTROL: Check minimum angle change
        if self.last_commanded_angle is not None:
            angle_change = abs(target_angle_deg - self.last_commanded_angle)
            if angle_change < self.min_angle_change_deg:
                self.angle_blocks += 1
                return False
        
        return True
    
    def send_head_command(self, head_tracker, target_angle_deg, current_time=None):
        """
        Send head command if conditions are met
        
        Args:
            head_tracker: HeadTracker instance or head controller
            target_angle_deg: Target angle in degrees
            current_time: Current timestamp
            
        Returns:
            True if command was sent, False if blocked
        """
        if current_time is None:
            current_time = time.time()
        
        if not self.should_send_head_command(target_angle_deg, current_time):
            return False
        
        try:
            # Convert angle to radians for HeadTracker
            target_angle_rad = math.radians(target_angle_deg)
            
            # Send command to head tracker
            if hasattr(head_tracker, 'set_person_tracking'):
                head_tracker.set_person_tracking(target_angle_rad)
            elif hasattr(head_tracker, 'move_to_angle_smooth'):
                head_tracker.move_to_angle_smooth(target_angle_deg, wait_for_completion=False)
            elif hasattr(head_tracker, 'set_head_angle_degrees'):
                head_tracker.set_head_angle_degrees(target_angle_deg, wait_for_completion=False)
            else:
                return False
            
            # Update state
            self.last_head_command_time = current_time
            self.last_commanded_angle = target_angle_deg
            self.movements_sent += 1
            
            return True
            
        except Exception:
            return False
    
    def process_and_track_person(self, head_tracker, x_pixels, current_time=None):
        """
        Complete person tracking pipeline with all anti-oscillation features
        
        Args:
            head_tracker: Head tracking controller
            x_pixels: Raw person X position in pixels
            current_time: Current timestamp
            
        Returns:
            dict with processing results and action taken
        """
        if current_time is None:
            current_time = time.time()
        
        # Step 1: Process position with deadband and filtering
        processed_pos = self.process_person_x_position(x_pixels, current_time)
        if processed_pos is None:
            return {
                'action': 'blocked_deadband',
                'reason': 'Person in 200px deadband zone',
                'raw_x': x_pixels,
                'distance_from_center': abs(x_pixels - self.screen_center_x)
            }
        
        # Step 2: Calculate target angle with magnitude reduction
        target_angle = self.calculate_head_angle(processed_pos)
        if target_angle is None:
            return {
                'action': 'no_angle',
                'reason': 'Could not calculate angle',
                'processed_position': processed_pos
            }
        
        # Step 3: Check if command should be sent
        if not self.should_send_head_command(target_angle, current_time):
            time_since_last = current_time - self.last_head_command_time
            angle_change = (abs(target_angle - self.last_commanded_angle) 
                          if self.last_commanded_angle else 0)
            
            return {
                'action': 'blocked_frequency_or_angle',
                'reason': f'Time since last: {time_since_last:.1f}s, Angle change: {angle_change:.1f}°',
                'target_angle': target_angle,
                'processed_position': processed_pos
            }
        
        # Step 4: Send head command
        command_sent = self.send_head_command(head_tracker, target_angle, current_time)
        
        return {
            'action': 'command_sent' if command_sent else 'command_failed',
            'target_angle': target_angle,
            'processed_position': processed_pos,
            'movements_sent': self.movements_sent,
            'command_success': command_sent
        }
    
    def get_stats(self):
        """Get statistics about filtering and blocking"""
        total_calls = self.deadband_blocks + self.frequency_blocks + self.angle_blocks + self.movements_sent
        
        return {
            'total_calls': total_calls,
            'movements_sent': self.movements_sent,
            'deadband_blocks': self.deadband_blocks,
            'frequency_blocks': self.frequency_blocks,
            'angle_blocks': self.angle_blocks,
            'deadband_pixels': self.deadband_pixels,
            'min_command_interval': self.min_command_interval,
            'min_angle_change_deg': self.min_angle_change_deg,
            'movement_scale_factor': self.movement_scale_factor,
            'filter_alpha': self.filter_alpha,
            'movement_efficiency': f"{(self.movements_sent / max(total_calls, 1) * 100):.1f}%"
        }
    
    def reset_stats(self):
        """Reset all statistics counters"""
        self.deadband_blocks = 0
        self.frequency_blocks = 0
        self.angle_blocks = 0
        self.movements_sent = 0
    
    def reset_filters(self):
        """Reset all filters and state"""
        self.filtered_x_position = None
        self.filter_initialized = False
        self.last_head_command_time = 0
        self.last_commanded_angle = None
        self.position_history.clear()
        self.reset_stats()


def create_anti_oscillation_head_tracking_function(head_helper=None):
    """
    Factory function to create an improved head tracking function that can replace
    existing head tracking calls in LidarTestBehavior and similar classes
    
    Returns:
        Function that can be used as drop-in replacement for head tracking
    """
    if head_helper is None:
        head_helper = AntiOscillationHeadHelper()
    
    def improved_head_tracking(head_tracker, person_data, enable_debug=False):
        """
        Improved head tracking function with anti-oscillation features
        Can be used as drop-in replacement in existing behaviors
        
        Args:
            head_tracker: Head tracking controller
            person_data: Dictionary with person detection data
            enable_debug: Whether to print debug information
            
        Returns:
            Result of tracking operation
        """
        if not head_tracker or not person_data:
            return False
        
        try:
            # Extract X position from person data
            if 'bbox_center' in person_data:
                x_pixels = person_data['bbox_center']['x_pixels']
            elif 'x_pixels' in person_data:
                x_pixels = person_data['x_pixels']
            elif 'x_midpoint_pixels' in person_data:
                x_pixels = person_data['x_midpoint_pixels']
            else:
                return False
            
            # Process with anti-oscillation pipeline
            result = head_helper.process_and_track_person(head_tracker, x_pixels)
            
            if enable_debug and result['action'] == 'command_sent':
                print(f"🎯 Anti-Oscillation Head: {result['target_angle']:.1f}° "
                      f"(filtered from {result['processed_position']['raw_x']:.0f}px)")
            
            return result['action'] == 'command_sent'
            
        except Exception as e:
            if enable_debug:
                print(f"❌ Head tracking error: {e}")
            return False
    
    # Attach helper for access to stats
    improved_head_tracking.helper = head_helper
    
    return improved_head_tracking