from typing import List, Dict, Optional
from .ObjectDetectionPipeline import ObjectDetectionPipeline
from .CameraReading import CameraReading
import depthai as dai
import cv2
import numpy as np
import time
import math


class ObjectDetectionReading(CameraReading):
    """
    Stable person detection for reliable operation
    """

    IMAGE_KEY = ObjectDetectionPipeline.ObjectDetectionQueues.IMAGE_OUT

    LABEL_MAP = [
        "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair",
        "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
    ]

    def __init__(self, queue_readings):
        super().__init__(queue_readings)
        
        self.confidence_threshold = 0.65  # Increased to 65% for more stable detection
        self.distance_range = (150, 8000)
        self.min_box_area = 0.0005
        self.max_box_area = 0.95
        
        self.person_label_index = self.LABEL_MAP.index("person")
        
        # Occlusion detection and position stabilization
        self.last_stable_position = None
        self.last_stable_time = 0
        self.position_stability_timeout = 2.0  # Keep stable position for 2 seconds
        self.min_person_bbox_width = 0.03  # Minimum expected person width (3% of frame)
        self.min_person_bbox_height = 0.08  # Minimum expected person height (8% of frame)
        self.depth_consistency_threshold = 500  # 50cm depth consistency check

    def get_people_locations(self) -> List[dai.SpatialImgDetection]:
        """
        Stable people detection with pure bounding box positioning
        """
        objects = self.queue_readings[ObjectDetectionPipeline.ObjectDetectionQueues.OBJECT_OUT]
        
        if not objects or not hasattr(objects, 'detections'):
            return []

        person_detections = []
        for detection in objects.detections:
            if (detection.label == self.person_label_index and 
                detection.confidence >= self.confidence_threshold):
                person_detections.append(detection)

        if not person_detections:
            return []

        filtered_people = self.apply_basic_filters(person_detections)
        
        if not filtered_people:
            return []

        closest_person = self.get_closest_person(filtered_people)
        result = [closest_person] if closest_person else []
        
        return result

    def detect_occlusion(self, detection: dai.SpatialImgDetection) -> bool:
        """
        Detect if person detection is likely occluded by obstacles
        """
        try:
            # Calculate bounding box dimensions
            bbox_width = detection.xmax - detection.xmin
            bbox_height = detection.ymax - detection.ymin
            bbox_area = bbox_width * bbox_height
            
            # Check for unusually small bounding box (indicates occlusion)
            if bbox_width < self.min_person_bbox_width or bbox_height < self.min_person_bbox_height:
                return True
            
            # Check for unusual aspect ratio (person partially cut off)
            aspect_ratio = bbox_width / bbox_height if bbox_height > 0 else 0
            if aspect_ratio > 1.0 or aspect_ratio < 0.2:  # Person should be taller than wide
                return True
            
            # Check if bounding box is at edge of frame (likely partial visibility)
            edge_threshold = 0.05  # 5% from edge
            if (detection.xmin < edge_threshold or detection.xmax > (1.0 - edge_threshold) or
                detection.ymin < edge_threshold or detection.ymax > (1.0 - edge_threshold)):
                return True
            
            # Check depth consistency - if depth is much closer than expected, likely hitting obstacle
            if hasattr(detection, 'spatialCoordinates') and detection.spatialCoordinates:
                z_depth = detection.spatialCoordinates.z
                
                # If depth is very close but bounding box suggests distant person, likely occlusion
                if z_depth < 1000 and bbox_area < 0.1:  # Close depth but small visible area
                    return True
            
            return False
            
        except Exception:
            return False
    
    def is_depth_consistent(self, detection: dai.SpatialImgDetection, expected_distance_range=(800, 6000)) -> bool:
        """
        Check if depth reading is consistent with a person (not an obstacle)
        """
        try:
            if not hasattr(detection, 'spatialCoordinates') or not detection.spatialCoordinates:
                return False
            
            z_depth = detection.spatialCoordinates.z
            
            # Basic range check
            if z_depth < expected_distance_range[0] or z_depth > expected_distance_range[1]:
                return False
            
            # Check bounding box size vs distance consistency
            bbox_area = (detection.xmax - detection.xmin) * (detection.ymax - detection.ymin)
            
            # Expected area decreases with distance
            expected_min_area = max(0.01, 0.2 * (1000 / max(z_depth, 1000)))
            
            if bbox_area < expected_min_area:
                return False  # Too small for that distance
            
            return True
            
        except Exception:
            return False
    
    def get_stabilized_position(self, detection: dai.SpatialImgDetection) -> dict:
        """
        Get person position with occlusion handling and position stabilization
        """
        current_time = time.time()
        
        # Check if current detection is likely occluded
        is_occluded = self.detect_occlusion(detection)
        depth_consistent = self.is_depth_consistent(detection)
        
        # Calculate current position
        current_position = self.get_person_bounding_box_center(detection)
        
        if current_position and not is_occluded and depth_consistent:
            # Good detection - update stable position
            self.last_stable_position = current_position.copy()
            self.last_stable_time = current_time
            return current_position
        
        elif (self.last_stable_position and 
              current_time - self.last_stable_time < self.position_stability_timeout):
            # Use last stable position if recent and current detection is problematic
            
            # Update Z depth if current reading seems reasonable
            if (current_position and 
                abs(current_position['z_depth'] - self.last_stable_position['z_depth']) < 1000):
                self.last_stable_position['z_depth'] = current_position['z_depth']
            
            return self.last_stable_position
        
        elif current_position:
            # No recent stable position, use current even if problematic
            # But try to correct obvious depth errors
            if is_occluded and self.last_stable_position:
                # Keep last stable angle, use current depth if reasonable
                corrected_position = self.last_stable_position.copy()
                if abs(current_position['z_depth'] - self.last_stable_position['z_depth']) < 2000:
                    corrected_position['z_depth'] = current_position['z_depth']
                return corrected_position
            else:
                return current_position
        
        else:
            return None

    def get_person_bounding_box_center(self, detection: dai.SpatialImgDetection) -> dict:
        """
        Calculate person position using ONLY bounding box center and Z depth
        No obstacle influence - pure geometric calculation
        """
        try:
            # Calculate the center of the bounding box (normalized 0-1 coordinates)
            x_center_norm = (detection.xmin + detection.xmax) / 2.0
            y_center_norm = (detection.ymin + detection.ymax) / 2.0
            
            # Get Z depth from spatial coordinates
            z_depth = detection.spatialCoordinates.z if hasattr(detection, 'spatialCoordinates') else 1000
            
            # Convert normalized center to camera angle
            # Assuming camera FOV and converting from normalized coordinates
            # Camera FOV is typically around 70-80 degrees horizontal
            camera_hfov_rad = math.radians(75)  # 75 degree horizontal FOV
            
            # Convert normalized position to angle offset from camera center
            # x_center_norm = 0.5 means center, 0.0 means far left, 1.0 means far right
            angle_offset_norm = (x_center_norm - 0.5) * 2.0  # Range: -1 to +1
            camera_angle_offset = angle_offset_norm * (camera_hfov_rad / 2.0)
            
            return {
                'x_center_norm': x_center_norm,
                'y_center_norm': y_center_norm,
                'camera_angle_offset': camera_angle_offset,
                'z_depth': z_depth,
                'bounding_box': {
                    'xmin': detection.xmin,
                    'xmax': detection.xmax,
                    'ymin': detection.ymin,
                    'ymax': detection.ymax
                },
                'bbox_area': (detection.xmax - detection.xmin) * (detection.ymax - detection.ymin),
                'detection_quality': 'occluded' if self.detect_occlusion(detection) else 'clear'
            }
            
        except Exception as e:
            return None

    def apply_basic_filters(self, detections: List[dai.SpatialImgDetection]) -> List[dai.SpatialImgDetection]:
        """Basic quality filters"""
        filtered = []
        
        for detection in detections:
            try:
                if hasattr(detection, 'spatialCoordinates') and detection.spatialCoordinates:
                    z_dist = detection.spatialCoordinates.z
                    if z_dist < self.distance_range[0] or z_dist > self.distance_range[1]:
                        continue
                else:
                    continue
                
                box_area = (detection.xmax - detection.xmin) * (detection.ymax - detection.ymin)
                if box_area < self.min_box_area or box_area > self.max_box_area:
                    continue
                
                filtered.append(detection)
                
            except Exception:
                continue
        
        return filtered

    def get_closest_person(self, detections: List[dai.SpatialImgDetection]) -> Optional[dai.SpatialImgDetection]:
        """Get closest person"""
        if not detections:
            return None
        
        closest_person = None
        min_distance = float('inf')
        
        for detection in detections:
            try:
                if hasattr(detection, 'spatialCoordinates') and detection.spatialCoordinates:
                    z_distance = detection.spatialCoordinates.z
                    if 0 < z_distance < min_distance:
                        min_distance = z_distance
                        closest_person = detection
            except Exception:
                continue
        
        return closest_person

    def get_closest_person_with_depth(self) -> Optional[Dict]:
        """
        Get closest person with depth info and stabilized position handling
        """
        people = self.get_people_locations()
        
        if not people:
            return None
        
        person = people[0]
        
        try:
            spatial_coords = person.spatialCoordinates
            
            # Get stabilized position data that handles occlusion
            bbox_data = self.get_stabilized_position(person)
            
            result = {
                'detection': person,
                'x_mm': spatial_coords.x,
                'y_mm': spatial_coords.y, 
                'z_mm': spatial_coords.z,
                'confidence': person.confidence,
                'bounding_box': {
                    'xmin': person.xmin,
                    'ymin': person.ymin,
                    'xmax': person.xmax,
                    'ymax': person.ymax
                }
            }
            
            # Add stabilized position data if available
            if bbox_data:
                result['bbox_center'] = bbox_data
                result['detection_quality'] = bbox_data.get('detection_quality', 'unknown')
                result['is_occluded'] = bbox_data.get('detection_quality') == 'occluded'
            
            return result
            
        except Exception:
            return None

    def get_object_locations(self) -> List[dai.SpatialImgDetection]:
        """Get all object detections"""
        objects = self.queue_readings[ObjectDetectionPipeline.ObjectDetectionQueues.OBJECT_OUT]
        
        if not objects or not hasattr(objects, 'detections'):
            return []
            
        filtered_objects = []
        for obj in objects.detections:
            if obj.confidence >= self.confidence_threshold:
                filtered_objects.append(obj)
                
        return filtered_objects

    def get_detection_status(self) -> Dict:
        """Get detection status"""
        people = self.get_people_locations()
        
        return {
            'people_detected': len(people),
            'confidence_threshold': self.confidence_threshold * 100,
            'detection_range_m': f"{self.distance_range[0]/1000:.1f}-{self.distance_range[1]/1000:.1f}",
            'processing_mode': 'STABLE',
            'closest_person_distance': people[0].spatialCoordinates.z if people else None
        }

    def get_frame(self):
        """Stable frame rendering"""
        base_frame = super().get_frame()
        
        if base_frame is None:
            return None

        def to_image_coord(x, y):
            return (np.array((x, y)).clip(0, 1) * base_frame.shape[:2]).astype(np.int32)

        objects = self.queue_readings[ObjectDetectionPipeline.ObjectDetectionQueues.OBJECT_OUT]

        if not objects or not hasattr(objects, 'detections'):
            return base_frame

        detection_count = 0
        for detection in objects.detections:
            if (detection.label == self.person_label_index and 
                detection.confidence >= self.confidence_threshold):
                
                detection_count += 1
                
                start_point = to_image_coord(detection.xmin, detection.ymin)
                end_point = to_image_coord(detection.xmax, detection.ymax)

                people = self.get_people_locations()
                is_closest = people and people[0] == detection
                
                color = (0, 255, 0) if is_closest else (0, 255, 255)
                thickness = 4 if is_closest else 2

                confidence = detection.confidence
                distance_text = ""
                if hasattr(detection, 'spatialCoordinates') and detection.spatialCoordinates:
                    distance_text = f" {detection.spatialCoordinates.z:.0f}mm"
                
                text = f"Person {confidence:.2f}{distance_text}"
                if is_closest:
                    text += " [CLOSEST]"

                base_frame = cv2.rectangle(base_frame, start_point, end_point, color, thickness)

                base_frame = cv2.putText(
                    base_frame, text, start_point + np.array((0, -10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2
                )

        return base_frame