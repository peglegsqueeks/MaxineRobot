import cv2
import pygame
from py_trees.common import Status

from ..types.CameraMode import CameraMode
from .MaxineBehavior import MaxineBehavior
from ..types.RobotModes import RobotMode


class SetRobotMode(MaxineBehavior):
    """
    FIXED robot mode setter - handles NO camera sensor gracefully
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
        RobotMode.LIDAR_TEST: CameraMode.DISABLED,  # CHANGED: Set to DISABLED as requested
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
        """
        FIXED: Handle camera mode switching with graceful NULL camera sensor handling
        """
        robot = self.get_robot()

        # CRITICAL FIX: Check if camera sensor exists before using it
        if not hasattr(robot, 'camera_sensor') or robot.camera_sensor is None:
            print(f"⚠️ No camera sensor available - skipping camera mode switch for {self.mode.name}")
            return True  # Return success even without camera
        
        try:
            # Set camera mode only if camera sensor exists
            camera_mode = self.ROBOT_MODE_TO_CAMERA_MODE[self.mode]
            robot.camera_sensor.switch_mode(camera_mode)
            print(f"✅ Camera mode set to {camera_mode.name} for {self.mode.name}")
            
            # Clean up OpenCV window
            self.safely_destroy_camera_window()
            
            return True
            
        except Exception as e:
            print(f"⚠️ Camera mode switch failed for {self.mode.name}: {e}")
            # Continue anyway - don't let camera issues crash the mode switch
            return True

    def update(self) -> Status:
        """
        FIXED: Set robot mode with comprehensive error handling
        """
        try:
            print(f"🔄 Setting robot mode to {self.mode.name}")
            
            robot = self.get_robot()
            
            # Set the robot mode first
            robot.set_mode(self.mode)
            print(f"✅ Robot mode set to {self.mode.name}")
            
            # Handle camera mode switching with error handling
            try:
                camera_success = self.update_camera_only()
                if not camera_success:
                    print(f"⚠️ Camera update failed for {self.mode.name}, but continuing")
            except Exception as e:
                print(f"⚠️ Camera error for {self.mode.name}: {e}, but continuing")
            
            # Handle TTS announcement with error handling
            try:
                if hasattr(robot, 'tts') and robot.tts and self.mode in self.MODE_SAYING:
                    robot.tts.say_text(self.MODE_SAYING[self.mode])
                    print(f"🔊 TTS announcement: {self.MODE_SAYING[self.mode]}")
            except Exception as e:
                print(f"⚠️ TTS announcement failed for {self.mode.name}: {e}, but continuing")
            
            return Status.SUCCESS
            
        except Exception as e:
            print(f"❌ CRITICAL: Failed to set robot mode {self.mode.name}: {e}")
            return Status.FAILURE