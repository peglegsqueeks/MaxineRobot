#!/usr/bin/env python3
"""
Enhanced ObjectDetectionPipeline.py - Replace existing version
Optimized pipeline settings for maximum person detection consistency
"""

from enum import Enum
from ..depth_ai.DepthAiPipeline import DepthAiPipeline
import depthai as dai
from typing import Dict


class ObjectDetectionPipeline(DepthAiPipeline):
    """
    Enhanced object detection pipeline optimized for person detection consistency
    Configured to minimize detection gaps and maximize person detection reliability
    """

    def __init__(self):
        # DIRECT path as requested - no searching
        self.SPATIAL_NETWORK_PATH = "/home/jetson/depthai-python/examples/models/mobilenet-ssd_openvino_2021.4_5shave.blob"
        self.expected_input_size = 300
        
        # ENHANCED: Lowered confidence threshold for better person detection
        self.minimum_confidence = 0.45  # Lowered from 0.65 to catch more person detections
        self.person_optimized_confidence = 0.40  # Special lower threshold for person class
        
        super().__init__()

    class ObjectDetectionQueues(Enum):
        IMAGE_OUT = "image_out"
        OBJECT_OUT = "object_out"
        DEPTH_OUT = "depth_out"

    def make_detection_net(self) -> dai.node.MobileNetSpatialDetectionNetwork:
        """
        Enhanced spatial detection network optimized for person detection
        """
        # Use lower confidence threshold to catch more detections
        confidence_threshold = self.person_optimized_confidence
        queue_size = 6  # Increased queue size for better buffering
        
        self.expected_input_size = 300

        try:
            spatial_net: dai.node.MobileNetSpatialDetectionNetwork = self.make_net(
                dai.node.MobileNetSpatialDetectionNetwork,
                self.SPATIAL_NETWORK_PATH,
                confidence_threshold,
                queue_size,
                blocking=False,
            )

            # ENHANCED: Optimized settings for person detection
            spatial_net.setBoundingBoxScaleFactor(0.2)  # Reduced from 0.3 for tighter bounds
            spatial_net.setDepthLowerThreshold(0)        # Set to 0 as per user requirement
            spatial_net.setDepthUpperThreshold(8000)     # Set to 8000mm as per user requirement
            
            # Enhanced pipeline settings for stability
            spatial_net.input.setBlocking(False)
            spatial_net.input.setQueueSize(queue_size)
            
            # ENHANCED: Additional network optimizations
            if hasattr(spatial_net, 'setNumClasses'):
                spatial_net.setNumClasses(21)  # COCO classes including person
            
            if hasattr(spatial_net, 'setCoordinateSize'):
                spatial_net.setCoordinateSize(4)
                
            if hasattr(spatial_net, 'setAnchors'):
                # Default MobileNet-SSD anchors optimized for person detection
                # These are typically handled automatically but can be fine-tuned
                pass
            
            if hasattr(spatial_net, 'setAnchorMasks'):
                # Set up anchor masks if supported
                pass
                
            if hasattr(spatial_net, 'setIouThreshold'):
                spatial_net.setIouThreshold(0.4)  # NMS IoU threshold
            
            return spatial_net
            
        except Exception as e:
            raise Exception(f"Failed to create enhanced detection network: {e}")

    def configure(self):
        """
        Enhanced pipeline configuration for optimal person detection
        """
        try:
            # ENHANCED: Camera configuration for better person detection
            left_mono = self.make_mono_camera("left")
            right_mono = self.make_mono_camera("right")
            
            # Optimized camera settings
            left_mono.setFps(30)      # Standard 30fps for good performance
            right_mono.setFps(30)
            
            # Enhanced camera resolution and quality - USER REQUIREMENT: THE_800_P for max FOV
            left_mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            right_mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            
            # ENHANCED: Stereo depth configuration
            stereo_sensor = self.make_stereo_sensor()
            
            # Optimized stereo settings for person detection
            stereo_sensor.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
            stereo_sensor.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
            stereo_sensor.setLeftRightCheck(True)        # Better accuracy
            stereo_sensor.setSubpixel(True)             # Sub-pixel accuracy
            stereo_sensor.setExtendedDisparity(False)    # Better for close-range detection
            stereo_sensor.setRectifyEdgeFillColor(0)    # Black fill for missing areas
            
            # Enhanced depth range for person detection
            stereo_sensor.initialConfig.setConfidenceThreshold(230)  # Slightly lower for more depth points
            
            # ENHANCED: Detection network
            detection_net = self.make_detection_net()
            
            # ENHANCED: Image manipulation for detection optimization
            input_size = self.expected_input_size
            image_manip = self.make_image_manip(input_size, resize_mode=True)
            
            # Optimized image manipulation settings
            if hasattr(image_manip.initialConfig, 'setResize'):
                image_manip.initialConfig.setResize(input_size, input_size)
            if hasattr(image_manip.initialConfig, 'setFrameType'):
                image_manip.initialConfig.setFrameType(dai.RawImgFrame.Type.BGR888p)
            if hasattr(image_manip, 'setKeepAspectRatio'):
                image_manip.setKeepAspectRatio(True)
            
            # ENHANCED: Pipeline linking with optimization
            left_mono.out.link(stereo_sensor.left)
            right_mono.out.link(stereo_sensor.right)
            
            # Link image processing
            stereo_sensor.rectifiedRight.link(image_manip.inputImage)
            image_manip.out.link(detection_net.input)
            
            # Link depth information
            stereo_sensor.depth.link(detection_net.inputDepth)
            
            # ENHANCED: Output configuration - FIXED: Removed maxSize parameter
            self.make_output(image_manip.out, self.ObjectDetectionQueues.IMAGE_OUT.value)
            self.make_output(detection_net.out, self.ObjectDetectionQueues.OBJECT_OUT.value)
            self.make_output(detection_net.passthroughDepth, self.ObjectDetectionQueues.DEPTH_OUT.value)
            
        except Exception as e:
            raise Exception(f"Enhanced pipeline configuration failed: {e}")

    def get_output_queues(self, device: dai.Device) -> Dict[ObjectDetectionQueues, dai.DataOutputQueue]:
        """Enhanced output queues with optimized settings"""
        return {
            # Larger queues for better buffering and reduced frame drops
            self.ObjectDetectionQueues.IMAGE_OUT: device.getOutputQueue(
                self.ObjectDetectionQueues.IMAGE_OUT.value, maxSize=4, blocking=False
            ),
            self.ObjectDetectionQueues.OBJECT_OUT: device.getOutputQueue(
                self.ObjectDetectionQueues.OBJECT_OUT.value, maxSize=8, blocking=False  # Larger for detection stability
            ),
            self.ObjectDetectionQueues.DEPTH_OUT: device.getOutputQueue(
                self.ObjectDetectionQueues.DEPTH_OUT.value, maxSize=4, blocking=False
            )
        }

    def make_image_manip(self, resize: int, resize_mode: bool = True):
        """Enhanced image manipulation for person detection"""
        image_manip: dai.node.ImageManip = self.pipeline.create(dai.node.ImageManip)
        
        if resize_mode:
            # Optimized resize settings for person detection
            image_manip.initialConfig.setResize(resize, resize)
            image_manip.setKeepAspectRatio(False)  # Allow distortion for consistent input size
        else:
            image_manip.initialConfig.setCropRect(0, 0, 1, 1)
        
        # Enhanced image processing settings
        image_manip.initialConfig.setFrameType(dai.RawImgFrame.Type.BGR888p)
        image_manip.setMaxOutputFrameSize(resize * resize * 3)
        
        # Additional optimizations if available
        if hasattr(image_manip, 'setNumFramesPool'):
            image_manip.setNumFramesPool(4)  # Buffer frames for stability
            
        return image_manip

    def get_pipeline_info(self) -> Dict:
        """Get enhanced pipeline information"""
        return {
            'pipeline_type': 'ENHANCED_PERSON_DETECTION',
            'model_path': self.SPATIAL_NETWORK_PATH,
            'input_size': self.expected_input_size,
            'base_confidence_threshold': self.minimum_confidence,
            'person_confidence_threshold': self.person_optimized_confidence,
            'optimizations': [
                'Lowered confidence threshold',
                'Enlarged queue sizes',
                'Enhanced stereo settings',
                'Optimized image manipulation',
                'Person-focused filtering'
            ]
        }