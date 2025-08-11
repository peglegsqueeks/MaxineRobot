import time
from .ActionManager import ActionManager
from ..types.HeadMovementDirection import HeadMovementDirection
from py_trees.common import Status
import py_trees


class HeadVelocityManager(ActionManager[HeadMovementDirection]):
    """
    The Action Manager responsible for moving the robot's head.
    IMPROVED with anti-oscillation servo control and reduced movement sensitivity
    """

    # REDUCED step sizes for gentler movements
    DIRECTION_TO_SERVO_VALUES = {
        HeadMovementDirection.NONE: 0,
        HeadMovementDirection.LEFT: 0.05,   # REDUCED from 0.08
        HeadMovementDirection.RIGHT: -0.05,  # REDUCED from 0.08
    }

    def __init__(self, servo_controller) -> None:
        """
        Initializes the head velocity manager using improved anti-oscillation servo controller.

        args:
            - servo_controller: The improved ServoController instance with anti-oscillation PID control
        """
        super().__init__("Head Velocity Manager (Anti-Oscillation)")
        self.servo_controller = servo_controller
        self.current_head_position = 0.0
        
        # Movement timing control to reduce oscillation
        self.last_movement_time = 0
        self.min_movement_interval = 0.2  # Wait 200ms between movements
        
        if self.servo_controller.is_initialized:
            self.current_head_position = self.servo_controller.get_position()
        
    def center_head(self):
        """
        Center the head using the servo controller's corrected center position
        """
        if self.servo_controller.center():
            success = self.servo_controller.wait_for_position(timeout=8.0)
            if success:
                self.current_head_position = self.servo_controller.get_center_position()
                return True
            else:
                return False
        return False

    def get_center_position(self):
        """
        Get the corrected center position from servo controller
        """
        return self.servo_controller.get_center_position()

    def perform_action(self, direction: HeadMovementDirection):
        """
        Moves the robot head in a direction using improved anti-oscillation PID control
        """
        current_time = time.time()
        
        # Rate limiting to prevent oscillation
        if current_time - self.last_movement_time < self.min_movement_interval:
            return True  # Skip movement but return success
        
        step_amount = self.DIRECTION_TO_SERVO_VALUES[direction]
        
        if direction == HeadMovementDirection.NONE:
            return True
        
        current_target = self.servo_controller.target_position
        
        new_position = current_target + step_amount
        
        if new_position > 0.95:
            return False

        if new_position < -0.95:
            return False
        
        if self.servo_controller.set_position(new_position):
            self.last_movement_time = current_time
            return True
        return False

    def get_head_position(self):
        """
        Get current head position from servo controller
        """
        position = self.servo_controller.get_position()
        if position is not None:
            self.current_head_position = position
        return position
    
    def set_head_position(self, position: float, wait_for_completion=False):
        """
        Set head to specific position (-0.98 to +0.98) with anti-oscillation PID control
        
        arguments:
            - position: Target position
            - wait_for_completion: Whether to wait for movement to complete
        """
        current_time = time.time()
        
        # Rate limiting for frequent position commands
        if current_time - self.last_movement_time < 0.1:  # 100ms minimum for position commands
            if not wait_for_completion:
                return True  # Accept command but don't execute immediately
        
        if self.servo_controller.set_position(position):
            self.last_movement_time = current_time
            if wait_for_completion:
                success = self.servo_controller.wait_for_position(timeout=10.0)
                return success
            return True
        return False
    
    def get_head_angle_degrees(self):
        """
        Get current head angle in degrees relative to corrected center
        """
        return self.servo_controller.get_angle_degrees()
    
    def get_target_head_angle_degrees(self):
        """
        Get target head angle in degrees relative to corrected center
        """
        return self.servo_controller.get_target_angle_degrees()
    
    def set_head_angle_degrees(self, angle_degrees: float, wait_for_completion=False):
        """
        Set head to specific angle in degrees (-90 to +90) with anti-oscillation PID control
        Angles are relative to the corrected center position
        
        arguments:
            - angle_degrees: Target angle in degrees
            - wait_for_completion: Whether to wait for movement to complete
        """
        current_time = time.time()
        
        # Rate limiting for angle commands
        if current_time - self.last_movement_time < 0.15:  # 150ms minimum for angle commands
            if not wait_for_completion:
                return True
        
        if self.servo_controller.move_to_angle(angle_degrees):
            self.last_movement_time = current_time
            if wait_for_completion:
                success = self.servo_controller.wait_for_position(timeout=10.0)
                return success
            return True
        return False
    
    def get_position(self):
        """
        Compatibility method - get current position
        """
        return self.get_head_position()
    
    def move_left(self, step_size=None):
        """
        Move head left - compatibility method for LidarTestBehavior
        Uses improved anti-oscillation PID control
        """
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_movement_time < self.min_movement_interval:
            return True
        
        if step_size is None:
            return self.perform_action(HeadMovementDirection.LEFT)
        else:
            # Use smaller step size to prevent overshooting
            safe_step_size = min(step_size, 0.05)  # Cap at 0.05
            current_target = self.servo_controller.target_position
            new_position = current_target + safe_step_size
            
            if new_position > 0.95:
                return False
                
            if self.servo_controller.set_position(new_position):
                self.last_movement_time = current_time
                return True
            return False
    
    def move_right(self, step_size=None):
        """
        Move head right - compatibility method for LidarTestBehavior
        Uses improved anti-oscillation PID control
        """
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_movement_time < self.min_movement_interval:
            return True
        
        if step_size is None:
            return self.perform_action(HeadMovementDirection.RIGHT)
        else:
            # Use smaller step size to prevent overshooting
            safe_step_size = min(step_size, 0.05)  # Cap at 0.05
            current_target = self.servo_controller.target_position
            new_position = current_target - safe_step_size
            
            if new_position < -0.95:
                return False
                
            if self.servo_controller.set_position(new_position):
                self.last_movement_time = current_time
                return True
            return False
    
    def move_to_angle_smooth(self, angle_degrees: float, wait_for_completion=True):
        """
        Move to specific angle with smooth anti-oscillation PID control and optional waiting
        
        arguments:
            - angle_degrees: Target angle (-90 to +90 degrees) relative to corrected center
            - wait_for_completion: Whether to wait for movement to complete
        """
        return self.set_head_angle_degrees(angle_degrees, wait_for_completion)
    
    def scan_left_right(self, angle_range=30.0, steps=3, dwell_time=1.5):
        """
        Perform a smooth left-right scan pattern with REDUCED parameters to prevent oscillation
        
        arguments:
            - angle_range: Total angle range (degrees) to scan - REDUCED from 45.0
            - steps: Number of scan positions - REDUCED from 5
            - dwell_time: Time to pause at each position - INCREASED from 1.0
        """
        if not self.servo_controller or not self.servo_controller.is_initialized:
            return False
            
        try:
            # Calculate scan positions with reduced range
            half_range = angle_range / 2.0
            if steps <= 1:
                positions = [0.0]
            else:
                step_size = angle_range / (steps - 1)
                positions = [-half_range + i * step_size for i in range(steps)]
            
            # Execute scan with longer dwell times
            for angle in positions:
                if not self.servo_controller.move_to_angle(angle):
                    return False
                
                # Wait for movement to complete
                if not self.servo_controller.wait_for_position(timeout=8.0):
                    return False
                
                # Dwell at position
                time.sleep(dwell_time)
            
            # Return to center
            if not self.servo_controller.move_to_angle(0.0):
                return False
                
            return self.servo_controller.wait_for_position(timeout=8.0)
            
        except Exception as e:
            return False
    
    def get_movement_stats(self):
        """
        Get movement statistics for debugging oscillation issues
        """
        return {
            'last_movement_time': self.last_movement_time,
            'min_movement_interval': self.min_movement_interval,
            'current_head_position': self.current_head_position,
            'servo_target_position': self.servo_controller.target_position if self.servo_controller else None,
            'servo_current_position': self.servo_controller.current_position if self.servo_controller else None
        }