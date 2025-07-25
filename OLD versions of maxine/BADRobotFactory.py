import pygame
import yaml

from src.action_managers.HeadMoveManager import HeadVelocityManager

from ..sensors.CameraSensor import CameraSensor
from ..sensors.I2CSensor import I2CSensor

from ..sensors.DistanceSensor import DistanceSensor
from ..sensors.LidarSensor import LidarSensor
from ..sensors.KeyboardSensor import KeyboardSensor
from ..action_managers.FacialAnimationManager import FacialAnimationManager
from ..action_managers.SoundManager import SoundManager
from ..action_managers.SpeechManager import SpeechManager
from ..action_managers.DirectVelocityManager import DirectVelocityManager
from ..action_managers.DebugActionManager import DebugActionManager
from ..types.DirectionSensorLocation import DirectionSensorLocation

from .Robot import Robot


class RobotFactory:
    """
    Builds a Robot object from a config file
    """

    def __init__(self, config_filename: str, board=None) -> None:
        """
        Initialises the factory

        arguments:
            - config_filename: the filename of the configurations file
            - board: the hardware board interface (e.g., for servo control)
        """
        # load the configs
        self.config = self.load_config_file(config_filename)
        self.errors_occurred = []
        self.board = board  # Store board reference

    def load_config_file(self, file_name: str):
        """
        Loads the configuration file from disk
        """
        with open(file_name, "r") as file:
            return yaml.safe_load(file)

    def build_robot(self) -> Robot:
        """
        Builds the robot based on the config file
        """

        display = self.init_pygame(self.config["pygame"])

        # build action managers
        animation_manager = self.build_facial_animation_manager(
            self.config["animation_manager"]
        )
        sound_manager = self.build_sound_manger(self.config["sound_manager"])
        speech_manager = self.build_speech_manager(self.config["speech_manager"])
        
        try:
            velocity_manager = self.build_velocity_manager(self.config["velocity_manager"])
        except Exception as e:
            self.errors_occurred.append(f'Velocity Manager failed to initialise: {e}')
            velocity_manager = None

        # Build head movement manager with board reference
        try:
            head_movement_manager = self.build_head_movement_manager(self.config.get("head_manager", {}))
        except Exception as e:
            self.errors_occurred.append(f'Head Movement Manager failed to initialise: {e}')
            head_movement_manager = None

        # build sensors
        keyboard_sensor = self.build_keyboard_sensor()
        distance_sensor = self.build_distance_sensor(self.config["distance_sensors"])
        camera_sensor = self.build_camera_sensor()
        
        try:
            lidar_sensor = self.build_lidar_sensor()
        except Exception as e:
            self.errors_occurred.append(f'Lidar failed to initialise: {e}')
            lidar_sensor = None
            
        # build robot with board reference
        return Robot(
            keyboard_sensor,
            animation_manager,
            sound_manager,
            speech_manager,
            velocity_manager,
            head_movement_manager,
            distance_sensor,
            camera_sensor,
            lidar_sensor,
            self.errors_occurred,
            board=self.board  # Pass board to Robot constructor
        )

    def build_head_movement_manager(self, config) -> HeadVelocityManager:
        """
        Builds the head movement manager with proper configuration
        """
        # if in manager is in debug mode return a debug manager
        if config.get("debug", False):
            return DebugActionManager("Head Movement Manager")
            
        servo_channel = config.get("servo_channel", 3)  # Default to channel 3
        max_retries = config.get("max_retries", 1)
        retry_delay = config.get("retry_delay", 0.5)
        
        return HeadVelocityManager(
            servo_channel=servo_channel,
            board=self.board,
            max_retries=max_retries,
            retry_delay=retry_delay
        )

    def build_lidar_sensor(self) -> LidarSensor:
        return LidarSensor()

    def build_keyboard_sensor(self) -> KeyboardSensor:
        """
        Builds the keyboard sensor
        """
        return KeyboardSensor()

    def init_pygame(self, config) -> pygame.Surface:
        """
        Initialises pygame according to configs
        """
        pygame.init()

        # initialise audio
        frequency = config["audio_frequency"]
        size = config["audio_size"]
        channels = config["audio_channels"]
        buffer = config["audio_buffer"]
        pygame.mixer.pre_init(frequency, size, channels, buffer)
        pygame.mixer.init()

    def build_facial_animation_manager(self, config) -> FacialAnimationManager:
        # if in manager is in debug mode return a debug manager
        if config["debug"]:
            return DebugActionManager("Facial Animation Manager")

        return FacialAnimationManager()

    def build_sound_manger(self, config) -> SoundManager:
        # if in manager is in debug mode return a debug manager
        if config["debug"]:
            return DebugActionManager("Sound Manager")

        return SoundManager(config["channel"])

    def build_speech_manager(self, config) -> SpeechManager:
        # if in manager is in debug mode return a debug manager
        if config["debug"]:
            return DebugActionManager("Speech Manager")

        rate = config["rate"]
        voice = config["voice"]
        volume = config["volume"]
        return SpeechManager(rate, voice, volume)

    def build_velocity_manager(self, config) -> DirectVelocityManager:
        # if in manager is in debug mode return a debug manager
        if config["debug"]:
            return DebugActionManager("Velocity Manager")

        return DirectVelocityManager(
            config["version"],
            config["maxaccel"],
            config["maxdeccel"],
            config["current_limit"],
            simulate=config.get("simulate", False)
        )

    def build_distance_sensor(self, config) -> DistanceSensor:
        """
        Builds the distance sensor
        """

        # builds the sensors from the config
        def build_sensors():
            sensors = {}
            for direction, i2c_config in config.items():
                sensor = I2CSensor(
                    int(i2c_config["ic2_adress"]), int(i2c_config["port"])
                )
                sensors[DirectionSensorLocation(direction)] = sensor
            return sensors

        sensors = []  # build_sensors() - commented out as in original
        return DistanceSensor(sensors)

    def build_camera_sensor(self) -> CameraSensor:
        """
        Builds the camera sensor
        """
        return CameraSensor()