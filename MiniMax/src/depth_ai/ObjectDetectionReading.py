#!/usr/bin/env python3
"""
Enhanced ObjectDetectionReading.py - Replace existing version
Optimized for consistent person detection with reduced gaps
Ignores all non-person detections for maximum focus
"""

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
    Enhanced object detection reading focused exclusively on person detection
    Optimized for maximum consistency and minimal detection gaps
    """

    IMAGE_KEY = ObjectDetectionPipeline.ObjectDetectionQueues.IMAGE_OUT

    LABEL_MAP = [
        "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair",
        "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
    ]

    def __init__(self, queue_readings):
        super().__init__(queue_readings)
        
        # ENHANCED: Optimized thresholds for person detection consistency
        self.confidence_threshold = 0.50  # Lowered from 0.65 to reduce detection gaps
        self.high_confidence_threshold = 0.75  # For premium detections
        self.minimum_confidence_threshold = 0.40  # Absolute minimum with temporal boost
        
        # Person-specific parameters - USER REQUIREMENTS: 0mm min, 8000mm max
        self.distance_range = (0, 8000)     # 0mm to 8m - as per user specification
        self.min_box_area = 0.0008   # Slightly smaller minimum for distant people
        self.max_box_area = 0.92     # Slightly smaller maximum to avoid false positives
        
        self.person_label_index = self.LABEL_MAP.index("person")
        
        # ENHANCED: Person-specific filtering
        self.min_person_width = 0.015   # Minimum 1.5% of frame width
        self.min_person_height = 0.04   # Minimum 4% of frame height
        self.max_person_width = 0.85    # Maximum 85% of frame width
        self.max_person_height = 0.90   # Maximum 90% of frame height
        
        # ENHANCED: Temporal smoothing for consistency
        self.detection_history = []
        self.max_history_frames = 15    # Track more frames for stability
        self.confidence_boost_factor = 0.15  # Boost for consistent detections
        
        # ENHANCED: Position stability tracking
        self.stable_positions = {}
        self.position_tolerance = 0.08  # 8% tolerance for position changes
        self.stability_frames = 3       # Frames needed for stability
        
        # Detection quality metrics
        self.frame_count = 0
        self.successful_detections = 0
        
        # Legacy compatibility
        self.last_stable_position = None
        self.last_stable_time = 0
        self.position_stability_timeout = 3.0
        self.depth_consistency_threshold = 300  # Tightened to 30cm

    def get_people_locations(self) -> List[dai.SpatialImgDetection]:
        """
        ENHANCED: Get people locations with maximum consistency and stability
        Focuses ONLY on person detections, ignores all other objects
        """
        objects = self.queue_readings[ObjectDetectionPipeline.ObjectDetectionQueues.OBJECT_OUT]
        
        if not objects or not hasattr(objects, 'detections'):
            return []

        self.frame_count += 1
        current_time = time.time()

        # STEP 1: Extract ONLY person detections with enhanced filtering
        person_candidates = []
        for detection in objects.detections:
            if self._is_enhanced_person_detection(detection):
                person_candidates.append(detection)

        if not person_candidates:
            # Try to recover from detection gap using history
            return self._attempt_detection_recovery()

        # STEP 2: Apply temporal confidence boosting
        boosted_detections = self._apply_confidence_boosting(person_candidates, current_time)

        # STEP 3: Apply enhanced filtering
        filtered_people = self._apply_enhanced_filtering(boosted_detections)

        # STEP 4: Apply intelligent NMS (Non-Maximum Suppression)
        final_people = self._apply_intelligent_nms(filtered_people)

        # STEP 5: Update detection history and position tracking
        self._update_detection_tracking(final_people, current_time)

        if final_people:
            self.successful_detections += 1

        return final_people

    def _is_enhanced_person_detection(self, detection) -> bool:
        """Enhanced person detection validation - ONLY people allowed"""
        try:
            # CRITICAL: Must be person class (ignore everything else)
            if detection.label != self.person_label_index:
                return False
            
            # Must meet minimum confidence threshold
            if detection.confidence < self.minimum_confidence_threshold:
                return False
            
            # Enhanced bounding box validation
            width = detection.xmax - detection.xmin
            height = detection.ymax - detection.ymin
            area = width * height
            
            # Person-specific size constraints
            if (width < self.min_person_width or width > self.max_person_width or
                height < self.min_person_height or height > self.max_person_height or
                area < self.min_box_area or area > self.max_box_area):
                return False
            
            # Must have valid spatial coordinates
            if not hasattr(detection, 'spatialCoordinates') or not detection.spatialCoordinates:
                return False
            
            # Enhanced distance validation
            distance = detection.spatialCoordinates.z
            if distance < self.distance_range[0] or distance > self.distance_range[1]:
                return False
            
            # Aspect ratio check (people are typically taller than wide)
            aspect_ratio = height / width if width > 0 else 0
            if aspect_ratio < 0.5 or aspect_ratio > 8.0:  # Very wide range for various poses
                return False
            
            return True
            
        except Exception:
            return False

    def _apply_confidence_boosting(self, detections, current_time):
        """Apply temporal confidence boosting for consistent detections"""
        boosted_detections = []
        
        for detection in detections:
            try:
                # Calculate position signature
                center_x = (detection.xmin + detection.xmax) / 2
                center_y = (detection.ymin + detection.ymax) / 2
                distance = detection.spatialCoordinates.z
                
                position_key = self._get_position_key(center_x, center_y, distance)
                
                # Calculate confidence boost based on position stability
                boost = self._calculate_confidence_boost(position_key, current_time)
                
                # Apply boost (capped at 1.0)
                original_confidence = detection.confidence
                boosted_confidence = min(1.0, original_confidence + boost)
                
                # Create boosted detection
                boosted_detection = detection
                boosted_detection.confidence = boosted_confidence
                boosted_detections.append(boosted_detection)
                
            except Exception:
                # If boosting fails, keep original
                boosted_detections.append(detection)
        
        return boosted_detections

    def _get_position_key(self, x, y, distance):
        """Generate position key for stability tracking"""
        x_bucket = int(x / self.position_tolerance)
        y_bucket = int(y / self.position_tolerance)
        d_bucket = int(distance / 200)  # 200mm distance buckets
        return f"{x_bucket}_{y_bucket}_{d_bucket}"

    def _calculate_confidence_boost(self, position_key, current_time):
        """Calculate confidence boost for stable positions"""
        if position_key not in self.stable_positions:
            self.stable_positions[position_key] = {
                'first_seen': current_time,
                'last_seen': current_time,
                'count': 1
            }
            return 0.0
        
        stable_info = self.stable_positions[position_key]
        stable_info['last_seen'] = current_time
        stable_info['count'] += 1
        
        # Apply boost for positions seen multiple times
        if stable_info['count'] >= self.stability_frames:
            boost = min(0.25, self.confidence_boost_factor * stable_info['count'] / 10)
            return boost
        
        return 0.0

    def _apply_enhanced_filtering(self, detections):
        """Apply enhanced filtering with dynamic thresholds"""
        filtered_people = []
        
        for detection in detections:
            try:
                # Dynamic confidence threshold based on detection characteristics
                required_confidence = self._get_dynamic_threshold(detection)
                
                if detection.confidence >= required_confidence:
                    filtered_people.append(detection)
                    
            except Exception:
                continue
        
        return filtered_people

    def _get_dynamic_threshold(self, detection):
        """Calculate dynamic confidence threshold"""
        base_threshold = self.confidence_threshold
        
        try:
            # Larger detections (closer people) get lower threshold
            width = detection.xmax - detection.xmin
            height = detection.ymax - detection.ymin
            size_factor = width * height
            
            if size_factor > 0.20:  # Very large detection
                return max(0.35, base_threshold - 0.20)
            elif size_factor > 0.12:  # Large detection
                return max(0.40, base_threshold - 0.15)
            elif size_factor > 0.06:  # Medium detection
                return max(0.45, base_threshold - 0.10)
            else:  # Small detection
                return base_threshold
                
        except Exception:
            return base_threshold

    def _apply_intelligent_nms(self, detections):
        """Apply intelligent Non-Maximum Suppression for person detection"""
        if len(detections) <= 1:
            return detections

        # Sort by confidence (highest first)
        sorted_detections = sorted(detections, key=lambda x: x.confidence, reverse=True)
        
        final_detections = []
        for current in sorted_detections:
            should_keep = True
            
            for kept in final_detections:
                iou = self._calculate_iou(current, kept)
                if iou > 0.25:  # 25% overlap threshold for people
                    should_keep = False
                    break
            
            if should_keep:
                final_detections.append(current)
        
        # For navigation, return only the closest person
        if final_detections:
            closest = min(final_detections, key=lambda x: x.spatialCoordinates.z)
            return [closest]
        
        return []

    def _calculate_iou(self, det1, det2):
        """Calculate Intersection over Union"""
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
            
        except Exception:
            return 0.0

    def _attempt_detection_recovery(self):
        """Attempt to recover from detection gaps using recent history"""
        if not self.detection_history:
            return []
        
        # Look for recent successful detection within last 1 second
        current_time = time.time()
        recent_threshold = current_time - 1.0
        
        for frame_data in reversed(self.detection_history):
            if (frame_data['time'] > recent_threshold and 
                frame_data['detections']):
                
                # Return the most recent successful detection
                # Note: This is a fallback - confidence might be slightly outdated
                return frame_data['detections'][:1]  # Return only closest
        
        return []

    def _update_detection_tracking(self, detections, current_time):
        """Update detection history and clean up old data"""
        try:
            # Add to history
            self.detection_history.append({
                'time': current_time,
                'detections': detections.copy(),
                'count': len(detections)
            })
            
            # Limit history size
            while len(self.detection_history) > self.max_history_frames:
                self.detection_history.pop(0)
            
            # Clean up old position tracking (>10 seconds)
            cutoff_time = current_time - 10.0
            keys_to_remove = [
                key for key, info in self.stable_positions.items()
                if info['last_seen'] < cutoff_time
            ]
            
            for key in keys_to_remove:
                del self.stable_positions[key]
                
        except Exception:
            pass

    def get_detection_status(self) -> Dict:
        """Get enhanced detection status"""
        people_count = len(self.get_people_locations())
        success_rate = self.successful_detections / max(1, self.frame_count)
        
        return {
            'people_detected': people_count,
            'confidence_threshold': self.confidence_threshold,
            'min_threshold': self.minimum_confidence_threshold,
            'success_rate': success_rate,
            'total_frames': self.frame_count,
            'successful_detections': self.successful_detections,
            'stable_positions': len(self.stable_positions),
            'history_frames': len(self.detection_history),
            'detection_mode': 'ENHANCED_PERSON_ONLY'
        }

    # Legacy methods for compatibility
    def apply_basic_filters(self, detections):
        """Legacy compatibility method"""
        return self._apply_enhanced_filtering(detections)

    def get_closest_person_with_depth(self) -> Optional[Dict]:
        """Enhanced closest person detection"""
        people = self.get_people_locations()
        
        if not people:
            return None
        
        person = people[0]  # Already filtered to closest
        
        try:
            spatial_coords = person.spatialCoordinates
            
            result = {
                'detection': person,
                'x_mm': spatial_coords.x,
                'y_mm': spatial_coords.y, 
                'z_mm': spatial_coords.z,
                'confidence': person.confidence,
                'enhanced': True,
                'detection_quality': 'high' if person.confidence > 0.75 else 'medium' if person.confidence > 0.60 else 'acceptable',
                'bounding_box': {
                    'xmin': person.xmin,
                    'ymin': person.ymin,
                    'xmax': person.xmax,
                    'ymax': person.ymax,
                    'width': person.xmax - person.xmin,
                    'height': person.ymax - person.ymin,
                    'area': (person.xmax - person.xmin) * (person.ymax - person.ymin)
                }
            }
            
            return result
            
        except Exception:
            return None

    def get_object_locations(self) -> List[dai.SpatialImgDetection]:
        """ENHANCED: Returns ONLY person detections (ignores all other objects)"""
        return self.get_people_locations()

    def get_frame(self):
        """Enhanced frame rendering with person-only focus"""
        base_frame = super().get_frame()
        
        if base_frame is None:
            return None

        def to_image_coord(x, y):
            return (np.array((x, y)).clip(0, 1) * base_frame.shape[:2]).astype(np.int32)

        people = self.get_people_locations()

        for detection in people:
            start_point = to_image_coord(detection.xmin, detection.ymin)
            end_point = to_image_coord(detection.xmax, detection.ymax)

            # Enhanced visualization
            confidence = detection.confidence
            distance = detection.spatialCoordinates.z if hasattr(detection, 'spatialCoordinates') else 0
            
            # Color coding by confidence
            if confidence > 0.75:
                color = (0, 255, 0)      # Green for high confidence
                thickness = 4
            elif confidence > 0.60:
                color = (0, 255, 255)    # Yellow for medium confidence  
                thickness = 3
            else:
                color = (255, 165, 0)    # Orange for acceptable confidence
                thickness = 2

            text = f"Person {confidence:.2f}"
            if distance > 0:
                text += f" {distance:.0f}mm"
            
            text += f" [ENHANCED]"

            base_frame = cv2.rectangle(base_frame, start_point, end_point, color, thickness)
            base_frame = cv2.putText(
                base_frame, text, start_point + np.array((0, -10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2
            )

        return base_frame