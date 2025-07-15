import time
import threading
import UltraBorg


class PIDController:
    """
    PID Controller for smooth servo movements
    """
    def __init__(self, kp=2.0, ki=0.1, kd=0.05, max_output=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        
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
        Calculate PID output
        
        arguments:
            - setpoint: Target position
            - current_value: Current position
            
        returns:
            - output: PID output (step to take)
        """
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            dt = 0.01
            
        error = setpoint - current_value
        
        proportional = self.kp * error
        
        self.integral += error * dt
        self.integral = max(-1.0, min(1.0, self.integral))
        integral = self.ki * self.integral
        
        derivative = self.kd * (error - self.previous_error) / dt
        
        output = proportional + integral + derivative
        
        output = max(-self.max_output, min(self.max_output, output))
        
        self.previous_error = error
        self.last_time = current_time
        
        return output


class ServoController:
    """
    Enhanced servo controller with PID control for smooth, safe head movements
    """

    def __init__(self, i2c_address: int = 11):
        """
        Initialize the servo controller with PID control
        
        arguments:
            - i2c_address: I2C address of the UltraBorg board (default 11, which is 0x0b in hex)
        """
        self.i2c_address = i2c_address
        self.board = None
        self.is_initialized = False
        
        self.min_position = -0.98
        self.max_position = 0.98
        
        # Corrected center position - further counter-clockwise for proper alignment
        self.center_position = 0.15
        
        self.current_position = 0.0
        self.target_position = self.center_position
        
        self.pid = PIDController(
            kp=1.5,
            ki=0.05,
            kd=0.08,
            max_output=0.02
        )
        
        self.movement_thread = None
        self.movement_active = False
        self.movement_lock = threading.Lock()
        self.position_tolerance = 0.01
        self.update_rate = 0.05
        
    def initialize(self):
        """
        Initialize the UltraBorg board and center the head properly
        """
        try:
            self.board = UltraBorg.UltraBorg()
            self.board.i2cAddress = self.i2c_address
            self.board.Init()
            
            if not self.board.foundChip:
                return False
            
            try:
                self.board.SetWithRetry(self.board.SetServoMaximum3, self.board.GetServoMaximum3, 5085, 5)
                self.board.SetWithRetry(self.board.SetServoMinimum3, self.board.GetServoMinimum3, 1550, 5)
                self.board.SetWithRetry(self.board.SetServoStartup3, self.board.GetServoStartup3, 3565, 5) #3565
            except Exception:
                pass
            
            try:
                self.current_position = self.board.GetServoPosition3()
                if self.current_position is None:
                    self.current_position = 0.0
            except Exception:
                self.current_position = 0.0
                
            self.is_initialized = True
            
            try:
                self.center_head_smooth()
            except Exception:
                self.target_position = self.center_position
            
            return True
            
        except Exception:
            self.is_initialized = False
            return False
    
    def center_head_smooth(self):
        """
        Smoothly center the head to the corrected center position using PID control
        """
        if not self.is_initialized:
            return False
            
        try:
            try:
                current_pos = self.board.GetServoPosition3()
                if current_pos is None:
                    current_pos = 0.0
            except Exception:
                current_pos = 0.0
                
            self.current_position = current_pos
            
            self.pid.reset()
            
            target = self.center_position
            max_iterations = 100
            iteration = 0
            
            while abs(self.current_position - target) > self.position_tolerance and iteration < max_iterations:
                try:
                    current_pos = self.board.GetServoPosition3()
                    if current_pos is not None:
                        self.current_position = current_pos
                except Exception:
                    break
                    
                pid_output = self.pid.update(target, self.current_position)
                
                new_position = self.current_position + pid_output
                new_position = max(self.min_position, min(self.max_position, new_position))
                
                try:
                    self.board.SetServoPosition3(new_position)
                except Exception:
                    break
                
                time.sleep(self.update_rate)
                
                iteration += 1
            
            try:
                final_position = self.board.GetServoPosition3()
                if final_position is not None:
                    self.current_position = final_position
            except Exception:
                pass
                
            self.target_position = self.current_position
            
            return True
            
        except Exception:
            self.target_position = self.center_position
            return True
    
    def _movement_thread_function(self):
        """
        Background thread function for smooth PID-controlled movement
        """
        try:
            while self.movement_active:
                with self.movement_lock:
                    if not self.movement_active:
                        break
                        
                    current_pos = self.board.GetServoPosition3()
                    if current_pos is not None:
                        self.current_position = current_pos
                    
                    error = abs(self.target_position - self.current_position)
                    if error <= self.position_tolerance:
                        continue
                    
                    pid_output = self.pid.update(self.target_position, self.current_position)
                    
                    new_position = self.current_position + pid_output
                    new_position = max(self.min_position, min(self.max_position, new_position))
                    
                    self.board.SetServoPosition3(new_position)
                
                time.sleep(self.update_rate)
                
        except Exception:
            pass
        finally:
            self.movement_active = False
    
    def set_position(self, position: float):
        """
        Set servo position with smooth PID-controlled movement
        
        arguments:
            - position: Target position (-0.98 to +0.98)
        """
        if not self.is_initialized:
            return False
            
        position = max(self.min_position, min(self.max_position, position))
        
        try:
            with self.movement_lock:
                self.target_position = position
                
                self.pid.reset()
                
                if not self.movement_active:
                    self.movement_active = True
                    self.movement_thread = threading.Thread(target=self._movement_thread_function, daemon=True)
                    self.movement_thread.start()
            
            return True
            
        except Exception:
            return False
    
    def set_position_immediate(self, position: float):
        """
        Set servo position immediately without PID
        """
        if not self.is_initialized:
            return False
            
        position = max(self.min_position, min(self.max_position, position))
        
        try:
            with self.movement_lock:
                self.movement_active = False
                
            self.board.SetServoPosition3(position)
            self.current_position = position
            self.target_position = position
            return True
            
        except Exception:
            return False
    
    def wait_for_position(self, timeout=10.0):
        """
        Wait for servo to reach target position
        
        arguments:
            - timeout: Maximum time to wait in seconds
            
        returns:
            - True if position reached, False if timeout
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            error = abs(self.target_position - self.current_position)
            if error <= self.position_tolerance:
                return True
            time.sleep(0.1)
            
        return False
    
    def get_position(self):
        """
        Get current servo position from hardware
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
    
    def get_head_position(self):
        """
        Compatibility method for existing code that calls get_head_position()
        """
        return self.get_position()
    
    def get_center_position(self):
        """
        Get the corrected center position for this robot
        """
        return self.center_position
    
    def move_left(self, step_size: float = 0.1):
        """
        Move servo left by step size (with PID control)
        """
        new_position = self.target_position + step_size
        if new_position > self.max_position:
            return False
        return self.set_position(new_position)
    
    def move_right(self, step_size: float = 0.1):
        """
        Move servo right by step size (with PID control)
        """
        new_position = self.target_position - step_size
        if new_position < self.min_position:
            return False
        return self.set_position(new_position)
    
    def move_to_angle(self, angle_degrees: float):
        """
        Move to specific angle in degrees (-90 to +90) with PID control
        Angles are relative to the corrected center position
        """
        position_offset = (angle_degrees / 90.0) * 0.98
        position = self.center_position + position_offset
        return self.set_position(position)
    
    def get_angle_degrees(self):
        """
        Get current angle in degrees relative to corrected center
        """
        position_offset = self.current_position - self.center_position
        return (position_offset / 0.98) * 90.0
    
    def get_target_angle_degrees(self):
        """
        Get target angle in degrees relative to corrected center
        """
        position_offset = self.target_position - self.center_position
        return (position_offset / 0.98) * 90.0
    
    def center(self):
        """
        Center the servo to the corrected center position with PID control
        """
        return self.set_position(self.center_position)
    
    def stop_movement(self):
        """
        Stop any ongoing movement and hold current position
        """
        with self.movement_lock:
            self.movement_active = False
            self.target_position = self.current_position
            
    def shutdown(self):
        """
        Shutdown servo controller - center servo smoothly and cleanup
        """
        if self.is_initialized:
            try:
                self.stop_movement()
                
                time.sleep(0.2)
                
                self.center()
                self.wait_for_position(timeout=5.0)
                
            except Exception:
                pass
                
        with self.movement_lock:
            self.movement_active = False
            
        if self.movement_thread and self.movement_thread.is_alive():
            self.movement_thread.join(timeout=1.0)
                
        self.is_initialized = False
    
    def get_status(self):
        """
        Get detailed status information for debugging
        """
        return {
            'initialized': self.is_initialized,
            'current_position': self.current_position,
            'target_position': self.target_position,
            'center_position': self.center_position,
            'current_angle_deg': self.get_angle_degrees(),
            'target_angle_deg': self.get_target_angle_degrees(),
            'movement_active': self.movement_active,
            'position_error': abs(self.target_position - self.current_position)
        }