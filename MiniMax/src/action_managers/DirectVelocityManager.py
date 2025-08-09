import time
from ..types.MovementDirection import MovementDirection
from ..action_managers.VelocityManager import VelocityConfig
from ..action_managers.ActionManager import ActionManager
import Jetson.GPIO as GPIO
import motoron


class DirectVelocityManager(ActionManager):
    """
    Direct velocity manager that controls robot movement using motoron controller
    Fixed to properly work with VelocityConfig and LiDAR Chase Mode
    """
    
    REFERENCE_MV = 3300
    PIN = 7
    TYPE = motoron.CurrentSenseType.MOTORON_24V16
    VIN_TYPE = motoron.VinSenseType.MOTORON_HP
    
    MAX_ACCELERATION = 50
    MAX_DECELERATION = 40
    CURRENT_LIMIT = 15000

    # Motor speed mappings for different movement directions
    # Adjusted speeds for better robot control
    DIRECTION_TO_MOTOR_SPEED = {
        MovementDirection.NONE: (0, 0),
        MovementDirection.FORWARDS: (-200, -196),
        MovementDirection.RIGHT: (170, -170),
        MovementDirection.LEFT: (-170, 170),
        MovementDirection.FORWARDS_LEFT: (-270, -200),
        MovementDirection.BACKWARDS_LEFT: (300, 200),
        MovementDirection.BACKWARDS: (200, 200),
        MovementDirection.BACKWARDS_RIGHT: (-170, -205),
        MovementDirection.FORWARDS_RIGHT: (-170, -205),
    }

    def __init__(self, version, maxaccel, maxdecel, current_limit, simulate=False) -> None:
        super().__init__("Direct Motor Control Velocity Manager")
        
        self.TYPE = motoron.CurrentSenseType.MOTORON_24V16
        self.CURRENT_LIMIT = current_limit
        self.MAX_ACCELERATION = maxaccel
        self.MAX_DECELERATION = maxdecel
        self.simulate = simulate
        
        GPIO.setmode(GPIO.BOARD)  
        GPIO.setup(self.PIN, GPIO.OUT)
        
        if not self.simulate:
            self.initialise_motor_control()
            self.turn_brake_off()

    def shutdown(self):
        """Shutdown motors and engage brake"""
        if self.simulate:
            self.turn_brake_on() 
            return

        try:
            if hasattr(self, 'mc') and self.mc:
                self.mc.set_speed(1, 0)
                self.mc.set_speed(2, 0)
        except Exception:
            pass
        
        self.turn_brake_on()

    def turn_brake_on(self):
        """Engage brake (safety feature - brake on when pin is LOW)"""
        GPIO.output(self.PIN, GPIO.LOW)
        
    def turn_brake_off(self):
        """Disengage brake (brake off when pin is HIGH)"""
        GPIO.output(self.PIN, GPIO.HIGH)

    def initialise_motor_control(self):
        """Initialize the motoron motor controller"""
        if not self.simulate:
            self.turn_brake_off() 
            
            # Initialize the motor controller
            self.mc = motoron.MotoronI2C(bus=7)
            self.mc.reinitialize() 
            self.mc.disable_crc()
            self.mc.clear_reset_flag()
            self.mc.disable_command_timeout()

            # Configure both motors 
            self.configure_motor(1)
            self.configure_motor(2)

    def configure_motor(self, motor_id):
        """Configure individual motor settings"""
        # Clear motor status
        self.mc.clear_motor_fault()
        
        # Set acceleration / deceleration maximums
        self.mc.set_max_acceleration(motor_id, self.MAX_ACCELERATION)
        self.mc.set_max_deceleration(motor_id, self.MAX_DECELERATION)

        # Set current limit on motor
        self.mc.set_braking(motor_id, 0)
        self.mc.set_speed(motor_id, 0)
        current_offset = self.mc.get_current_sense_offset(motor_id)
        limit = motoron.calculate_current_limit(self.CURRENT_LIMIT, self.TYPE, self.REFERENCE_MV, current_offset)
        self.mc.set_current_limit(motor_id, limit)

    def perform_action(self, config: VelocityConfig):
        """
        Perform movement action using VelocityConfig
        
        Args:
            config: VelocityConfig object with direction and speed
        """
        direction = config.direction
        speed = config.speed
        
        # Get base motor speeds for the direction
        motor_1, motor_2 = self.DIRECTION_TO_MOTOR_SPEED[direction]

        # Apply speed multiplier
        motor_1 = int(motor_1 * speed)
        motor_2 = int(motor_2 * speed)

        # Clamp motor speeds to safe range
        motor_1 = max(-800, min(800, motor_1))
        motor_2 = max(-800, min(800, motor_2))

        if self.simulate:
            # Simulation mode - just log the movement
            if direction == MovementDirection.NONE:
                pass
            elif direction == MovementDirection.FORWARDS:
                pass
            elif direction == MovementDirection.BACKWARDS:
                pass
            elif direction == MovementDirection.LEFT:
                pass
            elif direction == MovementDirection.RIGHT:
                pass
            elif direction == MovementDirection.FORWARDS_LEFT:
                pass
            elif direction == MovementDirection.FORWARDS_RIGHT:
                pass
            elif direction == MovementDirection.BACKWARDS_LEFT:
                pass
            elif direction == MovementDirection.BACKWARDS_RIGHT:
                pass
            return

        # Real movement - send commands to motors
        try:
            if hasattr(self, 'mc') and self.mc:
                self.mc.set_speed(1, motor_1)
                self.mc.set_speed(2, motor_2)
        except Exception:
            pass