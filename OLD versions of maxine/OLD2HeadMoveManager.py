import time
import logging
from .ActionManager import ActionManager
from typing import Optional
from ..types.HeadMovementDirection import HeadMovementDirection

class HeadVelocityManager(ActionManager[HeadMovementDirection]):

    DIRECTION_TO_SERVO_VALUES = {
        HeadMovementDirection.NONE: (0),
        HeadMovementDirection.LEFT: (0.02),
        HeadMovementDirection.RIGHT: (-0.02),
    }

    def __init__(self, servo_channel: int, max_retries: int = 5, retry_delay: float = 0.5):
        self.servo_channel = servo_channel
        self.max_retries = max_retries
        self.retry_delay = retry_delay
        self.logger = logging.getLogger(f"HeadVelocityManager_{servo_channel}")
        
        # Initialize servo connection with retries
        self._initialize_servo_connection()
        
        # Center the head with robust error handling
        self.center_head()
    
    def _initialize_servo_connection(self):
        """Initialize servo connection with retries for post-boot stability"""
        for attempt in range(self.max_retries):
            try:
                # Add delay for servo controller to stabilize after boot
                if attempt > 0:
                    self.logger.info(f"Servo initialization attempt {attempt + 1}/{self.max_retries}")
                    time.sleep(self.retry_delay * (attempt + 1))  # Progressive delay
                
                # Test servo communication
                position = self._get_servo_position_safe()
                if position is not None:
                    self.logger.info(f"Servo channel {self.servo_channel} initialized successfully at position {position}")
                    return
                
            except Exception as e:
                self.logger.warning(f"Servo initialization attempt {attempt + 1} failed: {e}")
        
        self.logger.error(f"Failed to initialize servo after {self.max_retries} attempts")
    
    def _get_servo_position_safe(self) -> Optional[float]:
        """Safely get servo position with error handling"""
        try:
            # Replace this with your actual servo position reading code
            # Example assuming UltraBorg servo interface:
            # position = self.ultraborg.GetServoPosition(self.servo_channel)
            
            # Placeholder - replace with your actual servo reading method
            position = self.get_servo_position()  # Your existing method
            
            # Validate the position is a valid number
            if position is None:
                return None
            
            # Ensure it's a float and within reasonable bounds (-1.0 to 1.0 typically)
            position = float(position)
            if not (-2.0 <= position <= 2.0):  # Reasonable servo range
                self.logger.warning(f"Servo position {position} outside expected range")
                return None
            
            return position
            
        except (TypeError, ValueError, AttributeError) as e:
            self.logger.warning(f"Error reading servo position: {e}")
            return None
        except Exception as e:
            self.logger.error(f"Unexpected error reading servo position: {e}")
            return None
    
    def center_head(self, timeout: float = 10.0, tolerance: float = 0.02):
        """Center the head with robust error handling and timeout"""
        self.logger.info("Centering head...")
        start_time = time.time()
        attempts = 0
        max_attempts = 20
        
        try:
            while attempts < max_attempts:
                # Check timeout
                if time.time() - start_time > timeout:
                    self.logger.error(f"Head centering timeout after {timeout}s")
                    break
                
                # Get current position with error handling
                position = self._get_servo_position_safe()
                
                if position is None:
                    attempts += 1
                    self.logger.warning(f"Cannot read servo position (attempt {attempts}/{max_attempts})")
                    
                    # Try to reset/reinitialize servo
                    if attempts % 5 == 0:  # Every 5th attempt
                        self.logger.info("Attempting servo reset...")
                        self._reset_servo()
                    
                    time.sleep(0.2)
                    continue
                
                # Check if already centered
                if abs(position) < tolerance:
                    self.logger.info(f"Head centered successfully at position {position:.3f}")
                    return
                
                # Move towards center
                try:
                    target_position = 0.0  # Center position
                    self._move_servo_to_position(target_position)
                    time.sleep(0.1)  # Allow movement time
                    attempts += 1
                    
                except Exception as e:
                    self.logger.error(f"Error moving servo: {e}")
                    attempts += 1
                    time.sleep(0.2)
            
            # Final position check
            final_position = self._get_servo_position_safe()
            if final_position is not None:
                self.logger.info(f"Head centering completed at position {final_position:.3f}")
            else:
                self.logger.warning("Head centering completed but cannot verify final position")
                
        except Exception as e:
            self.logger.error(f"Unexpected error during head centering: {e}")
            # Don't raise the exception - allow robot to continue with non-centered head
    
    def _reset_servo(self):
        """Reset servo connection/settings"""
        try:
            # Add your servo reset logic here
            # Example for UltraBorg:
            # self.ultraborg.SetServoPosition(self.servo_channel, 0.0)
            time.sleep(0.5)
            self.logger.info("Servo reset attempted")
        except Exception as e:
            self.logger.warning(f"Servo reset failed: {e}")
    
    def _move_servo_to_position(self, position: float):
        """Move servo to specified position with error handling"""
        try:
            # Clamp position to safe range
            position = max(-1.0, min(1.0, position))
            
            # Replace with your actual servo movement code
            # Example: self.ultraborg.SetServoPosition(self.servo_channel, position)
            self.set_servo_position(position)  # Your existing method
            
        except Exception as e:
            self.logger.error(f"Failed to move servo to position {position}: {e}")
            raise
    
    def get_servo_position(self):
        """Your existing servo position reading method - placeholder"""
        # Replace this with your actual implementation
        pass
    
    def set_servo_position(self, position: float):
        """Your existing servo position setting method - placeholder"""
        # Replace this with your actual implementation
        pass

    def perform_action(self, direction: HeadMovementDirection):
        """
        Moves the robot in a direction by setting servo values
        """
        # get servo positions for direction
        step_amount= self.DIRECTION_TO_SERVO_VALUES[direction]
        new_position = self.current_head_position + step_amount
        
        if new_position > 0.98:
            return

        if new_position < -0.98:
            return
        
        # set servo positions
        self.current_head_position = new_position
        self.board.SetServoPosition3(new_position)