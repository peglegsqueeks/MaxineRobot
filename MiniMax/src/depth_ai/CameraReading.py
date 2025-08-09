class CameraReading:
    """
    The base class for all camera reading
    All camera readings should be able to get a cv2 frame to display the robots view
    """

    # the key of the queue readings that corresponds to the image
    IMAGE_KEY = ""

    def __init__(self, queue_readings) -> None:
        """
        Initialises the reading

        arguments:
            - queue_readings: The latest reading from the camera for each type of output
        """
        self.queue_readings = queue_readings

    def get_frame(self):
        """
        Returns the latest cv frame
        """
        image = self.queue_readings[self.IMAGE_KEY]
        return image.getCvFrame()
