#!/usr/bin/env python3
"""
Enhanced Person Detection System
Optimized for maximum person detection consistency and stability
Ignores all non-person detections and focuses on robust person tracking
"""

import time
import math
from typing import List, Dict, Optional, Tuple
import depthai as dai


class EnhancedPersonDetectionFilter:
    """
    Enhanced person detection filter for maximum consistency
    Designed to reduce detection holes and improve stability
    """
    
    def __init__(self):
        # Optimized thresholds for person detection
        self.base_confidence_threshold = 0.55  # Lowered from 0.65 to catch more detections
        self.high_confidence_threshold = 0.75  # For high-quality detections
        self.minimum_confidence_threshold = 0.45  # Absolute minimum for temporal smoothing
        
        # Person-specific filtering
        self.person_label_index = 15  # "person" in COCO/MobileNet models
        self.min_person_width = 0.02   # Minimum 2% of frame width
        self.min_person_height = 0.05  # Minimum 5% of frame height
        self.max_person_width = 0.90   # Maximum 90% of frame width
        self.max_person_height = 0.95  # Maximum 95% of frame height
        
        # Distance filtering for realistic person detection
        self.min_distance_mm = 0    # 0cm minimum
        self.max_distance_mm = 8000    # 6m maximum
        
        # Temporal smoothing for consistency
        self.detection_history = []
        self.max_history_frames = 10   # Track last 10 frames
        self.temporal_confidence_boost = 0.1  # Boost confidence for consistent detections
        
        # Detection stability tracking
        self.stable_detections = {}
        self.stability_threshold = 3   # Frames to consider detection stable
        self.position_tolerance = 0.1  # 10% tolerance for position changes
        
    def filter_person_detections(self, raw_detections) -> List[dai.SpatialImgDetection]:
        """
        Enhanced person detection filtering with temporal smoothing
        Returns only the most reliable person detections
        """
        if not raw_detections or not hasattr(raw_detections, 'detections'):
            return []
        
        current_time = time.time()
        
        # Step 1: Extract only person detections with basic filtering
        person_candidates = []
        for detection in raw_detections.detections:
            if self._is_valid_person_detection(detection):
                person_candidates.append(detection)
        
        # Step 2: Apply temporal smoothing and stability analysis
        enhanced_detections = self._apply_temporal_enhancement(person_candidates, current_time)
        
        # Step 3: Apply person-specific quality filters
        quality_filtered = self._apply_quality_filters(enhanced_detections)
        
        # Step 4: Remove overlapping detections (NMS)
        final_detections = self._apply_smart_nms(quality_filtered)
        
        # Step 5: Update detection history for next frame
        self._update_detection_history(final_detections, current_time)
        
        return final_detections
    
    def _is_valid_person_detection(self, detection) -> bool:
        """Basic person detection validation"""
        try:
            # Must be person class
            if detection.label != self.person_label_index:
                return False
            
            # Must meet minimum confidence (lowered threshold)
            if detection.confidence < self.minimum_confidence_threshold:
                return False
            
            # Must have reasonable bounding box dimensions
            width = detection.xmax - detection.xmin
            height = detection.ymax - detection.ymin
            
            if (width < self.min_person_width or width > self.max_person_width or
                height < self.min_person_height or height > self.max_person_height):
                return False
            
            # Must have valid spatial coordinates
            if not hasattr(detection, 'spatialCoordinates') or not detection.spatialCoordinates:
                return False
            
            # Must be within reasonable distance range
            distance = detection.spatialCoordinates.z
            if distance < self.min_distance_mm or distance > self.max_distance_mm:
                return False
            
            return True
            
        except Exception:
            return False
    
    def _apply_temporal_enhancement(self, detections, current_time) -> List[dai.SpatialImgDetection]:
        """
        Apply temporal smoothing to boost confidence of consistent detections
        """
        enhanced_detections = []
        
        for detection in detections:
            try:
                # Calculate position key for tracking
                center_x = (detection.xmin + detection.xmax) / 2
                center_y = (detection.ymin + detection.ymax) / 2
                distance = detection.spatialCoordinates.z
                
                position_key = self._get_position_key(center_x, center_y, distance)
                
                # Check if this detection is consistent with recent history
                consistency_boost = self._calculate_consistency_boost(
                    position_key, detection.confidence, current_time
                )
                
                # Apply confidence boost for consistent detections
                enhanced_confidence = min(1.0, detection.confidence + consistency_boost)
                
                # Create enhanced detection (modify confidence)
                enhanced_detection = detection
                enhanced_detection.confidence = enhanced_confidence
                
                enhanced_detections.append(enhanced_detection)
                
            except Exception:
                # If enhancement fails, keep original detection
                enhanced_detections.append(detection)
        
        return enhanced_detections
    
    def _apply_quality_filters(self, detections) -> List[dai.SpatialImgDetection]:
        """Apply person-specific quality filters"""
        quality_detections = []
        
        for detection in detections:
            try:
                # Dynamic confidence threshold based on detection quality
                required_confidence = self._calculate_required_confidence(detection)
                
                if detection.confidence >= required_confidence:
                    quality_detections.append(detection)
                    
            except Exception:
                continue
        
        return quality_detections
    
    def _calculate_required_confidence(self, detection) -> float:
        """
        Calculate dynamic confidence threshold based on detection characteristics
        """
        base_threshold = self.base_confidence_threshold
        
        try:
            # Lower threshold for larger detections (likely closer/clearer)
            width = detection.xmax - detection.xmin
            height = detection.ymax - detection.ymin
            size_factor = width * height
            
            # Larger detections get lower threshold (easier to detect)
            if size_factor > 0.15:  # Large detection
                return max(0.45, base_threshold - 0.15)
            elif size_factor > 0.08:  # Medium detection
                return max(0.50, base_threshold - 0.10)
            else:  # Small detection
                return base_threshold
                
        except Exception:
            return base_threshold
    
    def _apply_smart_nms(self, detections) -> List[dai.SpatialImgDetection]:
        """
        Apply Non-Maximum Suppression optimized for person detection
        """
        if len(detections) <= 1:
            return detections
        
        # Sort by enhanced confidence
        sorted_detections = sorted(detections, key=lambda x: x.confidence, reverse=True)
        
        final_detections = []
        for current in sorted_detections:
            should_keep = True
            
            for kept in final_detections:
                if self._calculate_iou(current, kept) > 0.3:  # 30% overlap threshold
                    # Choose detection with higher confidence
                    should_keep = False
                    break
            
            if should_keep:
                final_detections.append(current)
        
        # Return only the closest person for navigation consistency
        if final_detections:
            closest = min(final_detections, 
                         key=lambda x: x.spatialCoordinates.z)
            return [closest]
        
        return []
    
    def _calculate_iou(self, det1, det2) -> float:
        """Calculate Intersection over Union"""
        try:
            # Calculate intersection rectangle
            x1 = max(det1.xmin, det2.xmin)
            y1 = max(det1.ymin, det2.ymin)
            x2 = min(det1.xmax, det2.xmax)
            y2 = min(det1.ymax, det2.ymax)
            
            # No intersection
            if x2 <= x1 or y2 <= y1:
                return 0.0
            
            # Calculate areas
            intersection = (x2 - x1) * (y2 - y1)
            area1 = (det1.xmax - det1.xmin) * (det1.ymax - det1.ymin)
            area2 = (det2.xmax - det2.xmin) * (det2.ymax - det2.ymin)
            union = area1 + area2 - intersection
            
            return intersection / union if union > 0 else 0.0
            
        except Exception:
            return 0.0
    
    def _get_position_key(self, x: float, y: float, distance: float) -> str:
        """Generate position key for tracking detections"""
        # Quantize position for stability
        x_bucket = int(x / self.position_tolerance)
        y_bucket = int(y / self.position_tolerance)
        d_bucket = int(distance / 200)  # 200mm buckets for distance
        
        return f"{x_bucket}_{y_bucket}_{d_bucket}"
    
    def _calculate_consistency_boost(self, position_key: str, confidence: float, current_time: float) -> float:
        """Calculate confidence boost based on detection consistency"""
        try:
            # Update stability tracking
            if position_key not in self.stable_detections:
                self.stable_detections[position_key] = {
                    'count': 1,
                    'first_seen': current_time,
                    'last_seen': current_time,
                    'avg_confidence': confidence
                }
                return 0.0
            else:
                stable_info = self.stable_detections[position_key]
                stable_info['count'] += 1
                stable_info['last_seen'] = current_time
                stable_info['avg_confidence'] = (
                    (stable_info['avg_confidence'] * (stable_info['count'] - 1) + confidence) / 
                    stable_info['count']
                )
                
                # Apply boost for stable detections
                if stable_info['count'] >= self.stability_threshold:
                    return min(0.2, self.temporal_confidence_boost * stable_info['count'] / 5)
                
                return 0.0
                
        except Exception:
            return 0.0
    
    def _update_detection_history(self, detections: List[dai.SpatialImgDetection], current_time: float):
        """Update detection history for next frame"""
        try:
            # Add current frame to history
            self.detection_history.append({
                'time': current_time,
                'count': len(detections),
                'detections': detections.copy() if detections else []
            })
            
            # Limit history size
            while len(self.detection_history) > self.max_history_frames:
                self.detection_history.pop(0)
            
            # Clean up old stable detections (>5 seconds old)
            cutoff_time = current_time - 5.0
            keys_to_remove = [
                key for key, info in self.stable_detections.items()
                if info['last_seen'] < cutoff_time
            ]
            
            for key in keys_to_remove:
                del self.stable_detections[key]
                
        except Exception:
            pass
    
    def get_detection_stats(self) -> Dict:
        """Get detection statistics for debugging"""
        recent_frames = len(self.detection_history)
        stable_positions = len(self.stable_detections)
        
        detection_rate = 0.0
        if recent_frames > 0:
            frames_with_detections = sum(1 for frame in self.detection_history if frame['count'] > 0)
            detection_rate = frames_with_detections / recent_frames
        
        return {
            'recent_frames': recent_frames,
            'stable_positions': stable_positions,
            'detection_rate': detection_rate,
            'confidence_threshold': self.base_confidence_threshold,
            'min_threshold': self.minimum_confidence_threshold
        }


class EnhancedPersonDetectionReading:
    """
    Enhanced reading class that uses the improved person detection filter
    """
    
    def __init__(self, queue_readings):
        self.queue_readings = queue_readings
        self.person_filter = EnhancedPersonDetectionFilter()
        
        # Override original thresholds
        self.confidence_threshold = 0.55  # Match filter's base threshold
        
    def get_people_locations(self) -> List[dai.SpatialImgDetection]:
        """
        Get enhanced person detections with improved consistency
        """
        try:
            from .ObjectDetectionPipeline import ObjectDetectionPipeline
            
            # Get raw detections
            objects = self.queue_readings[ObjectDetectionPipeline.ObjectDetectionQueues.OBJECT_OUT]
            
            if not objects:
                return []
            
            # Apply enhanced filtering
            enhanced_people = self.person_filter.filter_person_detections(objects)
            
            return enhanced_people
            
        except Exception:
            return []
    
    def get_detection_stats(self) -> Dict:
        """Get detection statistics"""
        return self.person_filter.get_detection_stats()