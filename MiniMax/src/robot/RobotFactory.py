import pygame
import yaml

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

    def __init__(self, config_filename: str) -> None:
        """
        Initiliases the factory

        arguments:
            - config_filename: the filename of the configurations file
        """
        # load the configs
        self.config = self.load_config_file(config_filename)
        self.errors_occurred=[]

    def load_config_file(self, file_name: str):
        """
        Loads the configuration file from disk
        """
        with open(file_name, "r") as file:
            return yaml.safe_load(file)

    def build_robot(self) -> Robot:
        """
        Builds the robot based on the config file
        Note: Head movement manager is now handled by new_main.py, not here
        """

        display = self.init_pygame(self.config["pygame"])

        # build action managers
        animimation_manager = self.build_facial_animation_manager(
            self.config["animation_manager"]
        )
        sound_manager = self.build_sound_manger(self.config["sound_manager"])
        speech_manager = self.build_speech_manager(self.config["speech_manager"])
        
        try:
            velocity_manager = self.build_velocity_manager(self.config["velocity_manager"])
        except:
            self.errors_occurred.append('Velocity Manager failed to initialise')
            velocity_manager=None

        # Head movement manager is now handled by new_main.py - don't create it here
        head_movement_manager = None

        # build sensors
        keyboard_sensor = self.build_keyboard_sensor()
        distance_sensor = self.build_distance_sensor(self.config["distance_sensors"])
        camera_sensor = self.build_camera_sensor()
        
        # Build lidar sensor with proper initialization readiness
        try:
            lidar_sensor = self.build_lidar_sensor()
        except:
            self.errors_occurred.append('Lidar failed to initialise')
            lidar_sensor=None
            
        # build robot
        return Robot(
            keyboard_sensor,
            animimation_manager,
            sound_manager,
            speech_manager,
            velocity_manager,
            head_movement_manager,  # This will be None, to be set later by new_main.py
            distance_sensor,
            camera_sensor,
            lidar_sensor,
            self.errors_occurred
        )

    def build_lidar_sensor(self) -> LidarSensor:
        """
        Build lidar sensor ready for initialization when needed
        The sensor is created but not initialized - initialization happens in SetRobotMode
        """
        print("🔧 Creating LiDAR sensor (ready for initialization)")
        return LidarSensor()

    def build_keyboard_sensor(self) -> KeyboardSensor:
        """
        Builds the keyboard sensor
        """
        return KeyboardSensor()

    def init_pygame(self, config) -> pygame.Surface:
        """
        Initilises pygame according to configs
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

        sensors = [] #build_sensors()
        return DistanceSensor(sensors)

    def build_camera_sensor(self) -> CameraSensor:
        """
        Builds the camera sensor
        """
        return CameraSensor()