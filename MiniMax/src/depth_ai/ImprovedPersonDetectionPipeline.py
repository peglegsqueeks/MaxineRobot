from enum import Enum
from ..depth_ai.DepthAiPipeline import DepthAiPipeline
import depthai as dai
from typing import Dict


class ImprovedPersonDetectionPipeline(DepthAiPipeline):
    """
    An improved DAI pipeline for person detection using YOLOv5/v8
    More accurate than MobileNet-SSD for person detection
    """

    # Recommended models (download from OpenVINO Model Zoo or convert from PyTorch):
    
    # Option 1: YOLOv5s person detection (faster, good accuracy)
    YOLO_PERSON_MODEL_PATH = "/home/jetson/models/yolov5s-person_openvino_2022.1_6shave.blob"
    
    # Option 2: YOLOv8n person detection (newest, best balance)
    # YOLO_PERSON_MODEL_PATH = "/home/jetson/models/yolov8n-person_openvino_2022.1_6shave.blob"
    
    # Option 3: Fallback to original MobileNet if YOLO models not available
    FALLBACK_MODEL_PATH = "/home/jetson/depthai-python/examples/models/mobilenet-ssd_openvino_2021.4_6shave.blob"

    # Queue outputs
    class PersonDetectionQueues(Enum):
        IMAGE_OUT = "image_out"
        PERSON_OUT = "person_out"
        DEPTH_OUT = "depth_out"

    def __init__(self, use_yolo=True):
        """
        Initialize with option to use YOLO or fallback to MobileNet
        
        Args:
            use_yolo: If True, try to use YOLO model, fallback to MobileNet if not found
        """
        self.use_yolo = use_yolo
        self.model_path = self.YOLO_PERSON_MODEL_PATH if use_yolo else self.FALLBACK_MODEL_PATH
        super().__init__()

    def make_person_detection_net(self) -> dai.node.DetectionNetwork:
        """
        Makes an optimized person detection network
        """
        try:
            # Try to use YOLO model first
            if self.use_yolo:
                detection_net = self.pipeline.create(dai.node.DetectionNetwork)
                detection_net.setConfidenceThreshold(0.5)  # YOLO can use lower threshold
                detection_net.setBlobPath(self.model_path)
                detection_net.input.setQueueSize(2)
                detection_net.input.setBlocking(False)
                return detection_net
            else:
                # Fallback to spatial detection network with improved settings
                spatial_net: dai.node.MobileNetSpatialDetectionNetwork = self.make_net(
                    dai.node.MobileNetSpatialDetectionNetwork,
                    self.FALLBACK_MODEL_PATH,
                    0.8,  # Higher confidence threshold
                    4,    # Smaller queue size for faster processing
                    blocking=False,
                )
                
                # Optimized settings for person detection
                spatial_net.setBoundingBoxScaleFactor(0.3)  # Tighter bounding boxes
                spatial_net.setDepthLowerThreshold(200)     # Ignore very close objects
                spatial_net.setDepthUpperThreshold(4000)    # Limit to 4 meters
                
                return spatial_net
                
        except Exception as e:
            print(f"Error loading {self.model_path}, falling back to MobileNet")
            # Fallback to MobileNet with optimized settings
            spatial_net: dai.node.MobileNetSpatialDetectionNetwork = self.make_net(
                dai.node.MobileNetSpatialDetectionNetwork,
                self.FALLBACK_MODEL_PATH,
                0.8,
                4,
                blocking=False,
            )
            spatial_net.setBoundingBoxScaleFactor(0.3)
            spatial_net.setDepthLowerThreshold(200)
            spatial_net.setDepthUpperThreshold(4000)
            return spatial_net

    def configure(self):
        # Build sensors with optimized settings
        left_mono = self.make_mono_camera("left")
        right_mono = self.make_mono_camera("right")
        
        # Higher resolution for better detection
        left_mono.setFps(15)  # Reduce FPS for better quality
        right_mono.setFps(15)
        
        stereo_sensor = self.make_stereo_sensor()
        
        # Optimized stereo settings for person detection
        stereo_sensor.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        stereo_sensor.setLeftRightCheck(True)  # Better depth accuracy
        stereo_sensor.setSubpixel(True)        # Sub-pixel accuracy
        
        # Image manipulation for detection
        image_manip = self.make_image_manip(416, True)  # YOLO typically uses 416x416
        detection_net = self.make_person_detection_net()

        # Link pipeline
        left_mono.out.link(stereo_sensor.left)
        right_mono.out.link(stereo_sensor.right)
        image_manip.out.link(detection_net.input)
        stereo_sensor.rectifiedRight.link(image_manip.inputImage)
        
        # Link depth for spatial detection (if using MobileNet)
        if hasattr(detection_net, 'inputDepth'):
            stereo_sensor.depth.link(detection_net.inputDepth)

        # Build outputs
        self.make_output(image_manip.out, self.PersonDetectionQueues.IMAGE_OUT.value)
        self.make_output(detection_net.out, self.PersonDetectionQueues.PERSON_OUT.value)
        
        # Depth output
        if hasattr(detection_net, 'passthroughDepth'):
            self.make_output(detection_net.passthroughDepth, self.PersonDetectionQueues.DEPTH_OUT.value)
        else:
            self.make_output(stereo_sensor.depth, self.PersonDetectionQueues.DEPTH_OUT.value)

    def get_output_queues(self, device: dai.Device) -> Dict[PersonDetectionQueues, dai.DataOutputQueue]:
        """Returns optimized output queues for person detection"""
        return {
            queue: device.getOutputQueue(queue.value, maxSize=2, blocking=False)  # Smaller queues
            for queue in self.PersonDetectionQueues
        }


# Improved Person Detection Reading class
class ImprovedPersonDetectionReading:
    """
    Enhanced person detection reading with better filtering
    """
    
    def __init__(self, queue_readings):
        self.queue_readings = queue_readings
        self.confidence_threshold = 0.85
        self.distance_range = (300, 4000)  # 0.3m to 4m
        
    def get_filtered_people_locations(self):
        """Get people locations with advanced filtering"""
        try:
            detections = self.queue_readings[ImprovedPersonDetectionPipeline.PersonDetectionQueues.PERSON_OUT]
            
            if not detections or not hasattr(detections, 'detections'):
                return []
                
            filtered_people = []
            
            for detection in detections.detections:
                try:
                    # For MobileNet, filter by person class
                    if hasattr(detection, 'label'):
                        # MobileNet SSD labels (person = 15)
                        if detection.label != 15:  # person class
                            continue
                    
                    # Quality filters
                    if detection.confidence < self.confidence_threshold:
                        continue
                        
                    # Distance filter
                    if hasattr(detection, 'spatialCoordinates'):
                        z_dist = detection.spatialCoordinates.z
                        if z_dist < self.distance_range[0] or z_dist > self.distance_range[1]:
                            continue
                    
                    # Bounding box size filter
                    box_width = detection.xmax - detection.xmin
                    box_height = detection.ymax - detection.ymin
                    
                    if box_width < 0.05 or box_height < 0.1:
                        continue
                        
                    if box_width > 0.8 or box_height > 0.95:
                        continue
                        
                    # Aspect ratio filter (people are taller than wide)
                    aspect_ratio = box_height / box_width
                    if aspect_ratio < 1.2 or aspect_ratio > 4.0:
                        continue
                        
                    filtered_people.append(detection)
                    
                except Exception as e:
                    continue
                    
            # Apply Non-Maximum Suppression
            final_people = self.apply_nms(filtered_people)
            
            # Return only the closest person
            if final_people:
                closest = min(final_people, 
                            key=lambda x: x.spatialCoordinates.z if hasattr(x, 'spatialCoordinates') else float('inf'))
                return [closest]
            
            return []
            
        except Exception as e:
            return []
    
    def apply_nms(self, detections, iou_threshold=0.3):
        """Apply Non-Maximum Suppression"""
        if len(detections) <= 1:
            return detections
            
        # Sort by confidence
        detections = sorted(detections, key=lambda x: x.confidence, reverse=True)
        
        kept = []
        for current in detections:
            keep = True
            for kept_det in kept:
                if self.calculate_iou(current, kept_det) > iou_threshold:
                    keep = False
                    break
            if keep:
                kept.append(current)
                
        return kept
    
    def calculate_iou(self, det1, det2):
        """Calculate IoU between two detections"""
        try:
            x1 = max(det1.xmin, det2.xmin)
            y1 = max(det1.ymin, det2.ymin)
            x2 = min(det1.xmax, det2.xmax)
            y2 = min(det1.ymax, det2.ymax)
            
            if x2 <= x1 or y2 <= y1:
                return 0.0
                
            intersection = (x2 - x1) * (y2 - y1)
            area1 = (det1.xmax - det1.xmin) * (det1.ymax - det1.ymin)
            area2 = (det2.xmax - det2.xmin) * (det2.ymax - det2.ymin)
            union = area1 + area2 - intersection
            
            return intersection / union if union > 0 else 0.0
            
        except:
            return 0.0