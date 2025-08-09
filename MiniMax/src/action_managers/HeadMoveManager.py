import time
from .ActionManager import ActionManager
from ..types.HeadMovementDirection import HeadMovementDirection
from py_trees.common import Status
import py_trees


class HeadVelocityManager(ActionManager[HeadMovementDirection]):
    """
    The Action Manager responsible for moving the robot's head.
    Now uses a shared ServoController with PID control for smooth, safe movements
    """

    DIRECTION_TO_SERVO_VALUES = {
        HeadMovementDirection.NONE: 0,
        HeadMovementDirection.LEFT: 0.08,
        HeadMovementDirection.RIGHT: -0.08,
    }

    def __init__(self, servo_controller) -> None:
        """
        Initializes the head velocity manager using a shared servo controller.

        args:
            - servo_controller: The shared ServoController instance with PID control
        """
        super().__init__("Head Velocity Manager")
        self.servo_controller = servo_controller
        self.current_head_position = 0.0
        
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
        Moves the robot head in a direction using PID-controlled servo movement
        """
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
        Set head to specific position (-0.98 to +0.98) with PID control
        
        arguments:
            - position: Target position
            - wait_for_completion: Whether to wait for movement to complete
        """
        if self.servo_controller.set_position(position):
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
        Set head to specific angle in degrees (-90 to +90) with PID control
        Angles are relative to the corrected center position
        
        arguments:
            - angle_degrees: Target angle in degrees
            - wait_for_completion: Whether to wait for movement to complete
        """
        if self.servo_controller.move_to_angle(angle_degrees):
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
        Uses PID control for smooth movement
        """
        if step_size is None:
            return self.perform_action(HeadMovementDirection.LEFT)
        else:
            current_target = self.servo_controller.target_position
            new_position = current_target + step_size
            
            if new_position > 0.95:
                return False
                
            return self.servo_controller.set_position(new_position)
    
    def move_right(self, step_size=None):
        """
        Move head right - compatibility method for LidarTestBehavior
        Uses PID control for smooth movement
        """
        if step_size is None:
            return self.perform_action(HeadMovementDirection.RIGHT)
        else:
            current_target = self.servo_controller.target_position
            new_position = current_target - step_size
            
            if new_position < -0.95:
                return False
                
            return self.servo_controller.set_position(new_position)
    
    def move_to_angle_smooth(self, angle_degrees: float, wait_for_completion=True):
        """
        Move to specific angle with smooth PID control and optional waiting
        
        arguments:
            - angle_degrees: Target angle (-90 to +90 degrees) relative to corrected center
            - wait_for_completion: Whether to wait for movement to complete
        """
        return self.set_head_angle_degrees(angle_degrees, wait_for_completion)
    
    def scan_left_right(self, angle_range=45.0, steps=5, dwell_time=1.0):
        """
        Perform a smooth left-right scan pattern
        
        arguments:
            - angle_range: Total angle range (degrees) to scan
            - steps: Number of stop points in the scan
            - dwell_time: Time to pause at each position
        """
        try:
            half_range = angle_range / 2
            if steps <= 1:
                positions = [0]
            else:
                positions = []
                for i in range(steps):
                    angle = -half_range + (i * angle_range / (steps - 1))
                    positions.append(angle)
            
            for i, angle in enumerate(positions):
                if self.move_to_angle_smooth(angle, wait_for_completion=True):
                    if dwell_time > 0:
                        time.sleep(dwell_time)
            
            self.move_to_angle_smooth(0.0, wait_for_completion=True)
            
            return True
            
        except Exception:
            return False
    
    def emergency_stop(self):
        """
        Emergency stop - immediately halt all head movement
        """
        self.servo_controller.stop_movement()
    
    def get_movement_status(self):
        """
        Get detailed status of head movement for debugging
        """
        status = self.servo_controller.get_status()
        status['head_manager_position'] = self.current_head_position
        return status
    
    def is_moving(self):
        """
        Check if head is currently moving
        """
        status = self.servo_controller.get_status()
        return status['movement_active'] and status['position_error'] > 0.01
    
    def wait_for_movement_complete(self, timeout=10.0):
        """
        Wait for any ongoing head movement to complete
        
        arguments:
            - timeout: Maximum time to wait
            
        returns:
            - True if movement completed, False if timeout
        """
        return self.servo_controller.wait_for_position(timeout)