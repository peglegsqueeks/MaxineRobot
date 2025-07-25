import cv2
import pygame
from py_trees.common import Status

from ..types.CameraMode import CameraMode
from .MaxineBehavior import MaxineBehavior
from ..types.RobotModes import RobotMode


class SetRobotMode(MaxineBehavior):
    """
    CLEAN robot mode setter - no LiDAR interference
    """

    # Camera modes for each robot mode
    ROBOT_MODE_TO_CAMERA_MODE = {
        RobotMode.IDLE: CameraMode.DISABLED,
        RobotMode.EXIT: CameraMode.DISABLED,
        RobotMode.KEYBOARD_CONTROL: CameraMode.DISABLED,
        RobotMode.CHASE: CameraMode.OBJECT_DETECTION,
        RobotMode.LIDARCHASE: CameraMode.OBJECT_DETECTION,
        RobotMode.DISCOVERY: CameraMode.OBJECT_DETECTION,
        RobotMode.HEADTURN: CameraMode.DISABLED,
        RobotMode.DIAGNOSTIC: CameraMode.DISABLED,
        RobotMode.PLAYGAME: CameraMode.DISABLED,
        RobotMode.LIDAR_TEST: CameraMode.OBJECT_DETECTION,
        RobotMode.HEAD_ALIGN: CameraMode.DISABLED,
    }

    MODE_SAYING = {
        RobotMode.IDLE: "idle",
        RobotMode.KEYBOARD_CONTROL: "keyboard control",
        RobotMode.CHASE: "chase",
        RobotMode.EXIT: "Exit",
        RobotMode.LIDARCHASE: "Lidar Chase Mode",
        RobotMode.DISCOVERY: "Discovery",
        RobotMode.HEADTURN: "Head Turn",
        RobotMode.DIAGNOSTIC: "Diagnostic",
        RobotMode.PLAYGAME: "Play Game",
        RobotMode.LIDAR_TEST: "Test Lidar",
        RobotMode.HEAD_ALIGN: "HEAD ALIGN",
    }

    def __init__(self, mode: RobotMode):
        super().__init__(f"Set to {mode.name} mode")
        self.mode = mode

    def safely_destroy_camera_window(self):
        """Safely destroy OpenCV window"""
        try:
            if cv2.getWindowProperty("Camera", cv2.WND_PROP_VISIBLE) >= 0:
                cv2.destroyWindow("Camera")
        except:
            pass

    def update_camera_only(self):
        """ONLY handle camera - no LiDAR interference"""
        robot = self.get_robot()

        # Set camera mode
        camera_mode = self.ROBOT_MODE_TO_CAMERA_MODE[self.mode]
        robot.camera_sensor.switch_mode(camera_mode)

        # Clean up OpenCV window
        self.safely_destroy_camera_window()

        # Initialize pygame if needed for LIDARCHASE
        if self.mode == RobotMode.LIDARCHASE:
            if not pygame.get_init():
                pygame.init()
                print("🎮 Pygame initialized for LiDAR Chase")

        # Clean up pygame for other modes
        elif self.mode != RobotMode.LIDARCHASE and pygame.get_init():
            try:
                pygame.event.clear()
                # Don't quit pygame - other modes might need it
            except:
                pass
   
    def update(self) -> Status:
        """Clean mode setting"""
        # Set robot mode
        robot = self.get_robot()
        robot.set_mode(self.mode)
        
        # Announce mode
        speech_text = self.MODE_SAYING[self.mode]
        robot.speech_manager.perform_action(speech_text)
        print(f"Setting to {self.mode}")

        # Clear keyboard
        robot.keyboard_sensor.flush_readings()

        # ONLY handle camera - let LiDAR Chase handle its own LiDAR
        self.update_camera_only()

        # Success
        return Status.SUCCESS