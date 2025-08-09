from ..depth_ai.CameraReading import CameraReading
from ..depth_ai.ObjectDetectionReading import ObjectDetectionReading
from ..types.CameraMode import CameraMode
from ..depth_ai.DepthAiPipeline import DepthAiPipeline
from ..depth_ai.ObjectDetectionPipeline import ObjectDetectionPipeline
from ..sensors.RobotSensor import RobotSensor
import depthai as dai


class CameraSensor(RobotSensor):
    """
    The camera sensor runs different pipelines on the oak camera based on the current mode
    """

    # mappings between camera mode and the pipeline/reading types
    MODE_PIPELINES = {CameraMode.OBJECT_DETECTION: ObjectDetectionPipeline()}
    MODE_READINGS = {CameraMode.OBJECT_DETECTION: ObjectDetectionReading}

    def __init__(self, starting_mode: CameraMode = CameraMode.DISABLED) -> None:
        """
        Initialises the camera
        """
        super().__init__("Camera")
        self.device = None
        self.switch_mode(starting_mode)

    def stop_camera(self):
        """
        Stops the camera pipeline from running
        """
        if self.device is not None:
            self.device.close()
            self.device = None

    def start_camera(self, pipeline: DepthAiPipeline):
        """
        Starts a pipeline on the camera
        """
        if self.device is not None:
            raise Exception(
                "Trying to start camera but camera is already started (did you forget to close the previous camera connection?)"
            )

        self.device = dai.Device()
        self.device.startPipeline(pipeline.pipeline)

    def switch_mode(self, mode: CameraMode):
        """
        Switch the camera mode
        """
        # stop the current pipeline
        self.stop_camera()

        # if camera is disabled nothing to do
        if mode == CameraMode.DISABLED:
            self.current_mode = CameraMode.DISABLED
            return

        # start the relevant mode
        pipeline = self.MODE_PIPELINES[mode]
        self.start_camera(pipeline)

        self.current_mode = mode

        # set the queues to match this mode
        self.output_queues = pipeline.get_output_queues(self.device)

    def get_reading(self) -> CameraReading:
        """
        Returns the current reading from the pipeline
        """

        # disabled mode has no readings
        if self.current_mode == CameraMode.DISABLED:
            return None

        # get the lastest reading from each queue for this mode
        readings = {}
        for name, queue in self.output_queues.items():
            values = queue.getAll()
            if len(values) == 0:
                readings[name] = None

            readings[name] = values[-1]

        # build a CameraReading object from the queue readings
        return self.MODE_READINGS[self.current_mode](readings)
