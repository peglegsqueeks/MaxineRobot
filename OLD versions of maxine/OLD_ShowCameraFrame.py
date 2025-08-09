import cv2
from py_trees.common import Status
from ..behaviors.MaxineBehavior import MaxineBehavior


class ShowCameraFrame(MaxineBehavior):
    """
    Displays the current camera frame in a CV2 window
    """

    def __init__(self):
        """
        Initilialises the behavior
        """
        super().__init__("Show Camera Frame")

    def update(self) -> Status:
        # get the current frame
        camera = self.get_robot().camera_sensor
        reading = camera.get_reading()
        frame = reading.get_frame()

        # display the current frame
        cv2.imshow("Camera", frame)
        cv2.waitKey(1)

        # behavior always passes
        return Status.SUCCESS
