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
        self.window_created = False

    def update(self) -> Status:
        # get the current frame
        camera = self.get_robot().camera_sensor
        reading = camera.get_reading()
        
        # Check if reading is None (camera disabled or no frame available)
        if reading is None:
            return Status.SUCCESS
            
        frame = reading.get_frame()
        
        # Check if frame is None
        if frame is None:
            return Status.SUCCESS

        # Create window if not already created
        if not self.window_created:
            cv2.namedWindow("Camera", cv2.WINDOW_AUTOSIZE)
            self.window_created = True

        # display the current frame
        cv2.imshow("Camera", frame)
        cv2.waitKey(1)

        # behavior always passes
        return Status.SUCCESS

    def terminate(self, new_status: Status):
        """
        Clean up the OpenCV window when behavior terminates
        """
        self.cleanup_window()
        super().terminate(new_status)

    def cleanup_window(self):
        """
        Safely destroy the OpenCV window
        """
        if self.window_created:
            try:
                # Check if window still exists before destroying
                if cv2.getWindowProperty("Camera", cv2.WND_PROP_VISIBLE) >= 0:
                    cv2.destroyWindow("Camera")
            except cv2.error:
                # Window doesn't exist, that's fine
                pass
            finally:
                self.window_created = False