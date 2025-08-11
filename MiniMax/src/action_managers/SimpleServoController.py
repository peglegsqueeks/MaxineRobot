"""
Fixed SimpleServoController.py - Corrected UltraBorg initialization
Uses -1 to +1 values with proper hardware communication
"""

import time
import threading
import UltraBorg


class SimpleServoController:
    """
    Fixed servo controller using -1 to +1 values with correct UltraBorg initialization
    """
    
    def __init__(self, i2c_address=11):
        """
        Initialize servo controller with correct parameters
        
        Args:
            i2c_address: I2C address for UltraBorg (default 11 = 0x0b)
        """
        self.i2c_address = i2c_address  # Default is 11 (0x0b), not 0x36
        self.board = None
        self.is_initialized = False
        
        # Position limits: -1.0 (full left) to +1.0 (full right)
        self.min_position = -0.98  # Safety margin
        self.max_position = 0.98   # Safety margin
        self.center_position = 0.0  # True center
        
        # Current state
        self.current_position = 0.0
        self.target_position = 0.0
        
        # Movement parameters
        self.position_tolerance = 0.015  # Position is "reached" within this tolerance
        self.update_rate = 0.05         # 20Hz update rate
        
        # Thread control
        self.movement_lock = threading.Lock()
        self.movement_thread = None
        self.movement_active = False
    
    def initialize(self):
        """
        Initialize connection to servo hardware with CORRECT UltraBorg initialization
        
        Returns:
            bool: True if successful, False otherwise
        """
        if self.is_initialized:
            return True
        
        try:
            # CORRECT UltraBorg initialization - no arguments to constructor!
            self.board = UltraBorg.UltraBorg()
            self.board.i2cAddress = self.i2c_address  # Set address as property
            self.board.Init()  # Initialize the board
            
            # Check if board was found
            if not self.board.foundChip:
                print(f"❌ UltraBorg not found at address {hex(self.i2c_address)}")
                return False
            
            print(f"✅ UltraBorg connected at address {hex(self.i2c_address)}")
            
            # Configure servo 3 parameters (from working ServoController)
            try:
                self.board.SetWithRetry(self.board.SetServoMaximum3, self.board.GetServoMaximum3, 5085, 5)
                self.board.SetWithRetry(self.board.SetServoMinimum3, self.board.GetServoMinimum3, 1550, 5)
                self.board.SetWithRetry(self.board.SetServoStartup3, self.board.GetServoStartup3, 3565, 5)
                print("✅ Servo 3 configured with proper limits")
            except Exception as e:
                print(f"⚠️ Error configuring servo parameters: {e}")
            
            # Get current position
            try:
                current_pos = self.board.GetServoPosition3()
                if current_pos is not None:
                    self.current_position = self._clamp_position(current_pos)
                    print(f"📍 Current servo position: {self.current_position:.3f}")
                else:
                    self.current_position = 0.0
                    print("⚠️ Could not read servo position, assuming 0.0")
            except Exception as e:
                print(f"⚠️ Error reading initial position: {e}")
                self.current_position = 0.0
            
            self.target_position = self.current_position
            self.is_initialized = True
            
            # Center the servo
            print("🎯 Centering servo...")
            self.center_immediate()  # Use immediate centering for initialization
            time.sleep(1.0)  # Wait for centering
            
            # Verify center position
            actual_pos = self.get_position()
            print(f"✅ Servo initialized at position: {actual_pos:.3f}")
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to initialize servo controller: {e}")
            self.is_initialized = False
            return False
    
    def _clamp_position(self, position):
        """
        Clamp position to safe limits
        
        Args:
            position: Raw position value
            
        Returns:
            float: Clamped position between min and max
        """
        return max(self.min_position, min(self.max_position, position))
    
    def _movement_thread_function(self):
        """
        Background thread for smooth movement
        """
        try:
            consecutive_errors = 0
            
            while self.movement_active:
                with self.movement_lock:
                    if not self.movement_active:
                        break
                    
                    # Get actual position from hardware
                    try:
                        actual_pos = self.board.GetServoPosition3()
                        if actual_pos is not None:
                            self.current_position = actual_pos
                            consecutive_errors = 0
                        else:
                            consecutive_errors += 1
                            if consecutive_errors > 5:
                                print("⚠️ Lost communication with servo")
                                self.movement_active = False
                                break
                    except Exception as e:
                        consecutive_errors += 1
                        if consecutive_errors > 5:
                            print(f"⚠️ Servo communication error: {e}")
                            self.movement_active = False
                            break
                    
                    # Calculate error
                    error = self.target_position - self.current_position
                    
                    # Check if we've reached target
                    if abs(error) <= self.position_tolerance:
                        continue
                    
                    # Calculate step with damping
                    max_step = 0.03  # Maximum step per update
                    if abs(error) > max_step:
                        step = max_step if error > 0 else -max_step
                    else:
                        step = error * 0.5  # Damping factor
                    
                    # Calculate new position
                    new_position = self.current_position + step
                    new_position = self._clamp_position(new_position)
                    
                    # Send to hardware
                    try:
                        self.board.SetServoPosition3(new_position)
                    except Exception as e:
                        print(f"⚠️ Failed to set servo position: {e}")
                
                time.sleep(self.update_rate)
                
        except Exception as e:
            print(f"⚠️ Movement thread error: {e}")
        finally:
            self.movement_active = False
    
    def set_position(self, position: float):
        """
        Set servo to target position (absolute positioning)
        
        Args:
            position: Target position (-1.0 to +1.0)
            
        Returns:
            bool: True if command accepted, False otherwise
        """
        if not self.is_initialized:
            print("⚠️ Servo controller not initialized")
            return False
        
        # Clamp position to safe limits
        position = self._clamp_position(position)
        
        try:
            # First, try to get actual current position
            try:
                actual_pos = self.board.GetServoPosition3()
                if actual_pos is not None:
                    self.current_position = actual_pos
            except:
                pass
            
            # Update target
            with self.movement_lock:
                self.target_position = position
                
                # Start movement thread if not running
                if not self.movement_active:
                    self.movement_active = True
                    self.movement_thread = threading.Thread(
                        target=self._movement_thread_function,
                        daemon=True
                    )
                    self.movement_thread.start()
            
            return True
            
        except Exception as e:
            print(f"⚠️ Failed to set position: {e}")
            return False
    
    def set_position_immediate(self, position: float):
        """
        Set servo position immediately without smooth movement
        
        Args:
            position: Target position (-1.0 to +1.0)
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_initialized:
            print("⚠️ Servo controller not initialized")
            return False
        
        position = self._clamp_position(position)
        
        try:
            # Stop smooth movement
            with self.movement_lock:
                self.movement_active = False
            
            # Wait for thread to stop
            if self.movement_thread and self.movement_thread.is_alive():
                self.movement_thread.join(timeout=0.5)
            
            # Set position directly
            self.board.SetServoPosition3(position)
            self.current_position = position
            self.target_position = position
            
            # Verify movement
            time.sleep(0.1)
            actual_pos = self.board.GetServoPosition3()
            if actual_pos is not None:
                self.current_position = actual_pos
                if abs(actual_pos - position) > 0.1:
                    print(f"⚠️ Servo did not reach target: commanded {position:.3f}, actual {actual_pos:.3f}")
            
            return True
            
        except Exception as e:
            print(f"⚠️ Failed to set immediate position: {e}")
            return False
    
    def get_position(self):
        """
        Get current servo position from hardware
        
        Returns:
            float: Current position (-1.0 to +1.0)
        """
        if not self.is_initialized:
            return self.current_position
        
        try:
            position = self.board.GetServoPosition3()
            if position is not None:
                self.current_position = position
                return position
            else:
                return self.current_position
        except Exception:
            return self.current_position
    
    def get_target_position(self):
        """
        Get target position
        
        Returns:
            float: Target position (-1.0 to +1.0)
        """
        return self.target_position
    
    def center(self):
        """
        Move servo to center position (0.0) smoothly
        
        Returns:
            bool: True if command accepted, False otherwise
        """
        return self.set_position(0.0)
    
    def center_immediate(self):
        """
        Center servo immediately without smooth movement
        
        Returns:
            bool: True if successful, False otherwise
        """
        return self.set_position_immediate(0.0)
    
    def wait_for_position(self, timeout=10.0):
        """
        Wait for servo to reach target position
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            bool: True if position reached, False if timeout
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # Get actual position from hardware
            try:
                actual_pos = self.board.GetServoPosition3()
                if actual_pos is not None:
                    self.current_position = actual_pos
            except:
                pass
            
            error = abs(self.target_position - self.current_position)
            if error <= self.position_tolerance:
                return True
            time.sleep(0.1)
        
        print(f"⚠️ Timeout waiting for position: target={self.target_position:.3f}, current={self.current_position:.3f}")
        return False
    
    def stop(self):
        """
        Stop all servo movement
        """
        with self.movement_lock:
            self.movement_active = False
        
        if self.movement_thread and self.movement_thread.is_alive():
            self.movement_thread.join(timeout=1.0)
    
    def shutdown(self):
        """
        Shutdown servo controller - center and cleanup
        """
        if self.is_initialized:
            try:
                print("🔄 Shutting down servo controller...")
                
                # Stop movement
                self.stop()
                time.sleep(0.2)
                
                # Center servo
                print("🎯 Centering servo before shutdown...")
                self.center_immediate()
                time.sleep(0.5)
                
                print("✅ Servo controller shutdown complete")
                
            except Exception as e:
                print(f"⚠️ Error during shutdown: {e}")
        
        # Mark as not initialized
        self.is_initialized = False
    
    def is_at_position(self, position: float, tolerance: float = None):
        """
        Check if servo is at specified position
        
        Args:
            position: Position to check
            tolerance: Position tolerance (uses default if None)
            
        Returns:
            bool: True if at position within tolerance
        """
        if tolerance is None:
            tolerance = self.position_tolerance
        
        # Get actual position from hardware
        actual_pos = self.get_position()
        return abs(actual_pos - position) <= tolerance
    
    def is_at_target(self):
        """
        Check if servo has reached target position
        
        Returns:
            bool: True if at target position
        """
        return self.is_at_position(self.target_position)
    
    def is_centered(self):
        """
        Check if servo is at center position
        
        Returns:
            bool: True if centered
        """
        return self.is_at_position(0.0)
    
    # Compatibility methods for existing code
    def get_center_position(self):
        """Get center position (always 0.0)"""
        return 0.0
    
    def move_to_angle(self, angle_degrees: float):
        """
        Compatibility method - converts angle to position
        
        Args:
            angle_degrees: Angle in degrees (-90 to +90)
            
        Returns:
            bool: True if command accepted
        """
        # Convert angle to position
        position = (angle_degrees / 90.0) * 0.98
        position = self._clamp_position(position)
        print(f"📐 move_to_angle: {angle_degrees}° → position {position:.3f}")
        return self.set_position(position)
    
    def get_angle_degrees(self):
        """
        Compatibility method - converts position to angle
        
        Returns:
            float: Angle in degrees
        """
        # Convert position to angle
        angle = (self.current_position / 0.98) * 90.0
        return angle
    
    def get_head_position(self):
        """Compatibility method"""
        return self.get_position()
    
    def move_by(self, delta: float):
        """
        Move servo by a relative amount
        
        Args:
            delta: Amount to move (-1.0 to +1.0)
            
        Returns:
            bool: True if command accepted, False otherwise
        """
        new_position = self.target_position + delta
        return self.set_position(new_position)