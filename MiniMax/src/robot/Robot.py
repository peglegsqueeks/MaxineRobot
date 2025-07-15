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
        camera_sensor: CameraSensor,
        lidar_sensor: LidarSensor,
        errors_ocurred: list[str],
    ) -> None:
        """
        Initialises the robot with the relevant sensors and action managers
        """
        self.errors_ocurred = errors_ocurred
        self.keyboard_sensor = keyboard_sensor
        self.facial_animation_manager = facial_animation_manager
        self.sound_manager = sound_manager
        self.speech_manager = speech_manager
        self.velocity_manager = velocity_manager
        self.head_move_manager = head_move_manager

        self.distance_sensor = distance_sensor
        self.camera_sensor = camera_sensor
        self.lidar_sensor = lidar_sensor

        # robot starts in IDLE mode
        self.current_mode = RobotMode.IDLE
        
        # New servo system attributes (will be set by new_main.py)
        self.servo_controller = None
        self.head_velocity_manager = None
        
        # Compatibility aliases (for backward compatibility)
        self.head_manager = head_move_manager  # Set initial value
        self.head_servo = None
    
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
    
    def get_head_angle(self):
        """
        Get current head angle - unified method
        """
        if hasattr(self, 'servo_controller') and self.servo_controller:
            return self.servo_controller.get_angle_degrees()
        elif hasattr(self, 'head_velocity_manager') and self.head_velocity_manager:
            return self.head_velocity_manager.get_head_angle_degrees()
        return 0.0
    
    def set_head_angle(self, angle_degrees):
        """
        Set head angle - unified method
        """
        if hasattr(self, 'servo_controller') and self.servo_controller:
            return self.servo_controller.move_to_angle(angle_degrees)
        elif hasattr(self, 'head_velocity_manager') and self.head_velocity_manager:
            return self.head_velocity_manager.set_head_angle_degrees(angle_degrees)
        return False
    
    def center_head(self):
        """
        Center the head - unified method
        """
        if hasattr(self, 'servo_controller') and self.servo_controller:
            return self.servo_controller.center()
        elif hasattr(self, 'head_velocity_manager') and self.head_velocity_manager:
            return self.head_velocity_manager.center_head()
        return False