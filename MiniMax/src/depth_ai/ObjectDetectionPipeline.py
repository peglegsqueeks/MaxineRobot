from enum import Enum
from ..depth_ai.DepthAiPipeline import DepthAiPipeline
import depthai as dai
from typing import Dict
import os
import glob


class ObjectDetectionPipeline(DepthAiPipeline):
    """
    Stable object detection pipeline for reliable person detection
    """

    @staticmethod
    def find_best_compatible_model():
        """Find the best compatible model for person detection"""
        original_mobilenet = "/home/jetson/depthai-python/examples/models/mobilenet-ssd_openvino_2021.4_6shave.blob"
        if os.path.exists(original_mobilenet):
            return original_mobilenet
        
        blob_files = glob.glob("/home/jetson/models/*.blob")
        if blob_files:
            for blob_file in blob_files:
                if 'mobilenet' in blob_file.lower():
                    return blob_file
            return blob_files[0]
        
        return original_mobilenet

    def __init__(self):
        self.SPATIAL_NETWORK_PATH = self.find_best_compatible_model()
        self.expected_input_size = 300
        self.minimum_confidence = 0.65  # Increased to 65% for more stable detection
        super().__init__()

    class ObjectDetectionQueues(Enum):
        IMAGE_OUT = "image_out"
        OBJECT_OUT = "object_out"
        DEPTH_OUT = "depth_out"

    def make_detection_net(self) -> dai.node.MobileNetSpatialDetectionNetwork:
        """
        Makes stable spatial detection network for reliable person detection
        """
        confidence_threshold = self.minimum_confidence
        queue_size = 4
        
        self.expected_input_size = 300

        try:
            spatial_net: dai.node.MobileNetSpatialDetectionNetwork = self.make_net(
                dai.node.MobileNetSpatialDetectionNetwork,
                self.SPATIAL_NETWORK_PATH,
                confidence_threshold,
                queue_size,
                blocking=False,
            )

            spatial_net.setBoundingBoxScaleFactor(0.3)
            spatial_net.setDepthLowerThreshold(200)
            spatial_net.setDepthUpperThreshold(8000)
            
            spatial_net.input.setBlocking(False)
            spatial_net.input.setQueueSize(queue_size)

            return spatial_net
            
        except Exception as e:
            raise

    def configure(self):
        """
        Configure pipeline with stable settings
        """
        try:
            left_mono = self.make_mono_camera("left")
            right_mono = self.make_mono_camera("right")
            
            left_mono.setFps(30)
            right_mono.setFps(30)
            
            stereo_sensor = self.make_stereo_sensor()
            detection_net = self.make_detection_net()
            
            input_size = self.expected_input_size
            image_manip = self.make_image_manip(input_size, True)

            left_mono.out.link(stereo_sensor.left)
            right_mono.out.link(stereo_sensor.right)
            
            stereo_sensor.rectifiedRight.link(image_manip.inputImage)
            image_manip.out.link(detection_net.input)
            stereo_sensor.depth.link(detection_net.inputDepth)

            self.make_output(image_manip.out, self.ObjectDetectionQueues.IMAGE_OUT.value)
            self.make_output(detection_net.out, self.ObjectDetectionQueues.OBJECT_OUT.value)
            self.make_output(detection_net.passthroughDepth, self.ObjectDetectionQueues.DEPTH_OUT.value)
            
        except Exception as e:
            raise

    def get_output_queues(
        self, device: dai.Device
    ) -> Dict[ObjectDetectionQueues, dai.DataOutputQueue]:
        """
        Get output queues with stable settings
        """
        try:
            queues = {}
            for queue_type in self.ObjectDetectionQueues:
                try:
                    queue = device.getOutputQueue(queue_type.value, maxSize=4, blocking=False)
                    queues[queue_type] = queue
                except Exception as e:
                    pass
            
            return queues
            
        except Exception as e:
            raise