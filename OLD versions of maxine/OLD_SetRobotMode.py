import cv2
from py_trees.common import Status

from ..types.CameraMode import CameraMode
from .MaxineBehavior import MaxineBehavior
from ..types.RobotModes import RobotMode


class SetRobotMode(MaxineBehavior):
    """
    Behavior that sets the robot in a particular mode
    Used to switch modes
    """

    # stores the camera mode for each robot mode
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
    }

    MODE_SAYING = {
        RobotMode.IDLE: "idle",
        RobotMode.KEYBOARD_CONTROL: "keyboard control",
        RobotMode.CHASE: "chase",
        RobotMode.EXIT: "Exit",
        RobotMode.LIDARCHASE: "Lidar",
        RobotMode.DISCOVERY: "Discovery",
        RobotMode.HEADTURN: "Head Turn",
        RobotMode.DIAGNOSTIC: "Diagnostic",
        RobotMode.PLAYGAME: "Play Game",
    }

    def __init__(self, mode: RobotMode):
        """
        Initialises the Behavior

        arguments:
            - mode: the mode to set the robot in
        """
        super().__init__(f"Set to {mode.name} mode")
        self.mode = mode

    def update_camera(self):
        """
        Handles updating the camera sensor realted settings upon changing robot mode
        """
        robot = self.get_robot()

        # switch the camera mode on camera sensor
        camera_mode = self.ROBOT_MODE_TO_CAMERA_MODE[self.mode]
        robot.camera_sensor.switch_mode(camera_mode)

        # close the camera window
        try:
            cv2.destroyWindow("Camera")
        except:
            # window does not exist -> can ignore
            pass

        # open camera window if required
        if camera_mode != CameraMode.DISABLED:
            # cv2.namedWindow("Camera", cv2.WINDOW_GUI_NORMAL)
            pass
   

    def update(self) -> Status:
        # set the robot mode
        robot = self.get_robot()
        robot.set_mode(self.mode)
        robot.speech_manager.perform_action(
            "Entering " + self.MODE_SAYING[self.mode] + " mode"
        )

        print(f"Setting to {self.mode}")

        # clear keyboard sensor currently pressed keys when swithing modes
        robot.keyboard_sensor.flush_readings()

        # update the camera settings
        self.update_camera()

        # this behavior always passes
        return Status.SUCCESS
