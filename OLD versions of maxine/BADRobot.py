from src.action_managers.HeadMoveManager import HeadVelocityManager
from src.multithreading.PathFindingThread import PathFindingThread
from src.path_finding.LidarPlot import LidarPlot
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
import logging

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
        head_move_manager: HeadVelocityManager,
        distance_sensor: DistanceSensor,
        camera_sensor: CameraSensor,
        lidar_sensor: LidarSensor,
        errors_ocurred: list[str],
        board=None,  # Add board parameter
    ) -> None:
        """
        Initialises the robot with the relevant sensors and action managers
        """
        # CRITICAL: Store board reference FIRST
        self.board = board
        self.logger = logging.getLogger("Robot")
        
        # Log board status for debugging
        #if self.board is None:
        #    self.logger.warning("Robot initialized with None board - servo operations will fail")
        #else:
        #    self.logger.info(f"Robot initialized with board: {type(self.board)}")
        #    # Test if board has required servo methods
        #    if hasattr(self.board, 'SetServoPosition3') and hasattr(self.board, 'GetServoPosition3'):
        #        self.logger.info("Board has required servo methods")
        #    else:
        #        self.logger.error("Board missing required servo methods!")
        
        self.errors_ocurred = errors_ocurred
        self.keyboard_sensor = keyboard_sensor
        self.facial_animation_manager = facial_animation_manager
        self.sound_manager = sound_manager
        self.speech_manager = speech_manager
        self.velocity_manager = velocity_manager
        self.distance_sensor = distance_sensor
        self.camera_sensor = camera_sensor
        self.lidar_sensor = lidar_sensor
        
        # Use the passed head_move_manager OR create a new one if None
        if head_move_manager is not None:
            self.head_move_manager = head_move_manager
        else:
            # Create new head manager if none provided
            self.logger.info("Creating new HeadVelocityManager with board reference")
            self.head_move_manager = HeadVelocityManager(
                servo_channel=3,
                board=self.board,  # Now self.board is properly set
                max_retries=1,
                retry_delay=0.5
            )
        
        # robot starts in IDLE mode
        self.current_mode = RobotMode.IDLE
    
    def set_mode(self, mode: RobotMode):
        """
        Sets the mode of the robot
        """
        self.current_mode = mode
    
    def shutdown(self):
        """
        Shutdown the robot and all its components
        """
        self.logger.info("Shutting down robot...")
        
        # switch speed to 0 and turn brake back on
        self.velocity_manager.shutdown()
        
        # Shutdown head manager if it has a shutdown method
        if hasattr(self.head_move_manager, 'stop'):
            self.head_move_manager.stop()
            
        self.logger.info("Robot shutdown complete")
    
    def get_board_status(self) -> dict:
        """
        Get status information about the board connection
        """
        status = {
            "board_available": self.board is not None,
            "board_type": type(self.board).__name__ if self.board else None,
            "has_servo_methods": False
        }
        
        #if self.board:
        #    status["has_servo_methods"] = (
        #        hasattr(self.board, 'SetServoPosition3') and 
        #        hasattr(self.board, 'GetServoPosition3')
        #    )
        
        return status