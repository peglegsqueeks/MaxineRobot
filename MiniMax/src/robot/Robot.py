"""
Robot.py - COMPLETE file with SimpleServoController integration
This is the FULL Robot class file with all methods and SimpleServoController support
"""

from ..sensors.CameraSensor import CameraSensor
from ..sensors.DistanceSensor import DistanceSensor
from ..sensors.LidarSensor import LidarSensor
from ..types.CameraMode import CameraMode
from ..types.RobotModes import RobotMode
from ..sensors.KeyboardSensor import KeyboardSensor
from ..action_managers.FacialAnimationManager import FacialAnimationManager
from ..action_managers.SoundManager import SoundManager
from ..action_managers.SpeechManager import SpeechManager
from ..action_managers.DirectVelocityManager import DirectVelocityManager
from queue import LifoQueue


class Robot:
    """
    The main robot class.
    This class just holds all the Action managers and sensors the robot has.
    It also stores the current mode.

    Behaviors will have access to the robot class to do stuff with it
    CAMERA SENSOR CAN BE None: DirectPipelineLidarTest handles camera directly
    """

    def __init__(
        self,
        keyboard_sensor: KeyboardSensor,
        facial_animation_manager: FacialAnimationManager,
        sound_manager: SoundManager,
        speech_manager: SpeechManager,
        velocity_manager: DirectVelocityManager,
        head_move_manager,  # Can be None initially, will be set by new_main.py
        distance_sensor: DistanceSensor,
        camera_sensor,  # CHANGED: Can now be None (was CameraSensor)
        lidar_sensor: LidarSensor,
        errors_ocurred: list[str],
    ) -> None:
        """
        Initialises the robot with the relevant sensors and action managers
        CAMERA SENSOR CAN BE None: DirectPipelineLidarTest handles camera directly
        """
        self.errors_ocurred = errors_ocurred
        self.keyboard_sensor = keyboard_sensor
        self.facial_animation_manager = facial_animation_manager
        self.sound_manager = sound_manager
        self.speech_manager = speech_manager
        self.velocity_manager = velocity_manager
        self.head_move_manager = head_move_manager

        self.distance_sensor = distance_sensor
        self.camera_sensor = camera_sensor  # Can be None now
        self.lidar_sensor = lidar_sensor

        # robot starts in IDLE mode
        self.current_mode = RobotMode.IDLE
        
        # New servo system attributes (will be set by new_main.py)
        self.servo_controller = None
        self.head_velocity_manager = None
        
        # Compatibility aliases (for backward compatibility)
        self.head_manager = head_move_manager  # Set initial value
        self.head_servo = None
        
        # Camera sensor status logging
        if self.camera_sensor is None:
            print("🚫 Robot initialized with NO camera sensor - DirectPipelineLidarTest will handle camera")
        else:
            print("📷 Robot initialized with camera sensor")
    
    def initialize_servo_system(self, servo_controller, head_velocity_manager):
        """
        Initialize the simplified servo system
        Called by new_main.py after robot creation
        """
        self.servo_controller = servo_controller
        self.head_velocity_manager = head_velocity_manager
        
        # Set up compatibility aliases
        self.head_move_manager = head_velocity_manager
        self.head_manager = head_velocity_manager
        self.head_servo = servo_controller
    
    def initialize_servo_controller(self):
        """
        Initialize servo controller - try SimpleServoController first, fall back to original
        This method can be called to upgrade the servo controller after robot creation
        """
        try:
            # Try to use the new SimpleServoController first
            from src.action_managers.SimpleServoController import SimpleServoController
            
            print("🔧 Attempting to initialize SimpleServoController...")
            # Use correct I2C address - 11 (0x0b) not 0x36
            self.servo_controller = SimpleServoController(i2c_address=11)
            
            if self.servo_controller.initialize():
                print("✅ SimpleServoController initialized successfully")
                
                # Also initialize head velocity manager with the new controller
                if not hasattr(self, 'head_velocity_manager') or self.head_velocity_manager is None:
                    from src.action_managers.HeadMoveManager import HeadVelocityManager
                    self.head_velocity_manager = HeadVelocityManager(self.servo_controller)
                    print("✅ Head velocity manager initialized with SimpleServoController")
                
                # Update compatibility aliases
                self.head_move_manager = self.head_velocity_manager
                self.head_manager = self.head_velocity_manager
                self.head_servo = self.servo_controller
                
                return True
            else:
                print("⚠️ SimpleServoController initialization failed, trying original...")
                
        except ImportError:
            print("⚠️ SimpleServoController not available, using original ServoController")
        
        # Fall back to original ServoController
        try:
            from src.action_managers.ServoController import ServoController
            
            print("🔧 Initializing original ServoController...")
            # Use correct I2C address - 11 (0x0b) not 0x36
            self.servo_controller = ServoController(i2c_address=11)
            
            if self.servo_controller.initialize():
                print("✅ Original ServoController initialized")
                
                # Initialize head velocity manager
                if not hasattr(self, 'head_velocity_manager') or self.head_velocity_manager is None:
                    from src.action_managers.HeadMoveManager import HeadVelocityManager
                    self.head_velocity_manager = HeadVelocityManager(self.servo_controller)
                    print("✅ Head velocity manager initialized")
                
                # Update compatibility aliases
                self.head_move_manager = self.head_velocity_manager
                self.head_manager = self.head_velocity_manager
                self.head_servo = self.servo_controller
                
                return True
            else:
                print("❌ Original ServoController initialization failed")
                self.servo_controller = None
                return False
                
        except Exception as e:
            print(f"❌ Failed to initialize any servo controller: {e}")
            self.servo_controller = None
            return False
    
    def set_mode(self, mode: RobotMode):
        """
        Set the robot mode
        """
        self.current_mode = mode
    
    def shutdown(self):
        """
        Enhanced shutdown method to include servo controller
        """
        try:
            # Shutdown servo controller first
            if hasattr(self, 'servo_controller') and self.servo_controller:
                self.servo_controller.shutdown()
                print("✅ Servo controller shutdown")
            
            # Shutdown velocity manager
            if hasattr(self, 'velocity_manager') and self.velocity_manager:
                self.velocity_manager.shutdown()
                print("✅ Velocity manager shutdown")
            
            # Other shutdowns could go here
            print("✅ Robot shutdown complete")
            
        except Exception as e:
            print(f"⚠️ Error during robot shutdown: {e}")
    
    def get_head_position(self):
        """
        Get current head position - unified method
        Returns position in -1.0 to +1.0 range
        """
        if hasattr(self, 'servo_controller') and self.servo_controller:
            return self.servo_controller.get_position()
        elif hasattr(self, 'head_velocity_manager') and self.head_velocity_manager:
            return self.head_velocity_manager.get_head_position()
        return 0.0
    
    def set_head_position(self, position):
        """
        Set head position - unified method
        Position should be in -1.0 to +1.0 range
        """
        if hasattr(self, 'servo_controller') and self.servo_controller:
            return self.servo_controller.set_position(position)
        elif hasattr(self, 'head_velocity_manager') and self.head_velocity_manager:
            return self.head_velocity_manager.set_head_position(position)
        return False
    
    def get_head_angle(self):
        """
        Get current head angle - unified method
        NOTE: Deprecated, use get_head_position() instead for better accuracy
        """
        if hasattr(self, 'servo_controller') and self.servo_controller:
            if hasattr(self.servo_controller, 'get_angle_degrees'):
                return self.servo_controller.get_angle_degrees()
            else:
                # Convert position to angle for SimpleServoController
                position = self.servo_controller.get_position()
                return (position / 0.98) * 90.0
        elif hasattr(self, 'head_velocity_manager') and self.head_velocity_manager:
            return self.head_velocity_manager.get_head_angle_degrees()
        return 0.0
    
    def set_head_angle(self, angle_degrees):
        """
        Set head angle - unified method
        NOTE: Deprecated, use set_head_position() instead for better accuracy
        """
        # Convert angle to position
        position = (angle_degrees / 90.0) * 0.98
        position = max(-0.98, min(0.98, position))
        return self.set_head_position(position)
    
    def center_head(self):
        """
        Center the head - unified method
        """
        if hasattr(self, 'servo_controller') and self.servo_controller:
            return self.servo_controller.center()
        elif hasattr(self, 'head_velocity_manager') and self.head_velocity_manager:
            return self.head_velocity_manager.center_head()
        return False