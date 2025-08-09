#!/usr/bin/env python3
"""
Maxine Robot - Optimized Detection System
Based on optimization test results from enhanced_camera_tester.py

Key findings implemented:
- Optimal confidence threshold: 0.5 (best detection rate with fewest gaps)
- Gap bridging: Max 3.0s tolerance based on test data
- Edge detection smoothing: 60.9% flicker rate on edges requires filtering
- Z-depth stability: Moving average filter for 1145mm variance reduction

Test Results Summary:
- baseline (0.5 conf): 51.6% detection, 2 gaps ✅ OPTIMAL
- low_conf (0.3 conf): 48.7% detection, 16 gaps ❌
- very_low_conf (0.2): 46.4% detection, 9 gaps ❌  
- wide_bbox (0.7 scale): 51.6% detection, 14 gaps ❌
"""

import depthai as dai
import numpy as np
import time
import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, List, Tuple

@dataclass
class PersonDetection:
    """Optimized person detection data structure for Maxine"""
    x_camera: float
    y_camera: float
    z_depth: float
    confidence: float
    timestamp: float
    edge_zone: str  # 'left', 'center', 'right'
    is_stable: bool  # Z-depth stability flag
    gap_duration: float = 0.0  # Gap before this detection

class MaxineOptimizedDetector:
    """
    Optimized person detection system for Maxine robot
    Implements findings from camera optimization tests
    """
    
    def __init__(self):
        # OPTIMAL SETTINGS from test results
        self.CONFIDENCE_THRESHOLD = 0.5  # Best performance: 51.6% detection, 2 gaps
        self.BBOX_SCALE = 0.5           # Balanced bounding box scale
        self.MAX_GAP_TOLERANCE = 3.0    # Based on 3.04s max gap from tests
        
        # Edge detection optimization (based on 127° FOV)
        self.CAMERA_HFOV_DEGREES = 127
        self.EDGE_THRESHOLD = 0.6       # Center vs edge boundary
        self.EDGE_FLICKER_FILTER_SIZE = 5  # Smooth 60.9% edge flicker rate
        
        # Z-depth stability optimization (address 49.6% stability)
        self.Z_FLICKER_THRESHOLD = 300  # mm
        self.Z_STABILITY_FILTER_SIZE = 5  # Moving average for 1145mm variance
        self.MIN_STABLE_Z = 500         # mm
        self.MAX_STABLE_Z = 8000        # mm
        
        # Gap bridging system
        self.last_valid_detection = None
        self.last_detection_time = 0
        self.gap_start_time = None
        self.current_gap_duration = 0
        
        # Stability filters
        self.z_history = deque(maxlen=self.Z_STABILITY_FILTER_SIZE)
        self.edge_detection_history = deque(maxlen=self.EDGE_FLICKER_FILTER_SIZE)
        
        # DepthAI setup
        self.device = None
        self.pipeline = None
        self.detection_queue = None
        
        # Performance tracking
        self.total_detections = 0
        self.stable_detections = 0
        self.gap_events = []
        self.edge_performance = {'left': 0, 'center': 0, 'right': 0}
        
        print("🤖 Maxine Optimized Detector Initialized")
        print(f"   Confidence: {self.CONFIDENCE_THRESHOLD} (optimal from tests)")
        print(f"   BBox Scale: {self.BBOX_SCALE}")
        print(f"   Gap Tolerance: {self.MAX_GAP_TOLERANCE}s")
        print(f"   Edge Filter: {self.EDGE_FLICKER_FILTER_SIZE} frames")
        print(f"   Z-Stability Filter: {self.Z_STABILITY_FILTER_SIZE} frames")
    
    def create_optimized_pipeline(self):
        """Create camera pipeline with optimized settings"""
        try:
            pipeline = dai.Pipeline()
            
            # Camera setup - same as working configuration
            cam_rgb = pipeline.create(dai.node.ColorCamera)
            cam_rgb.setPreviewSize(416, 416)
            cam_rgb.setInterleaved(False)
            cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            cam_rgb.setFps(15)
            
            # Image manipulation
            manip = pipeline.create(dai.node.ImageManip)
            manip.initialConfig.setResize(300, 300)
            manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
            cam_rgb.preview.link(manip.inputImage)
            
            # Stereo depth
            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_right = pipeline.create(dai.node.MonoCamera)
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            
            depth = pipeline.create(dai.node.StereoDepth)
            depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
            depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
            depth.initialConfig.setConfidenceThreshold(160)
            depth.setLeftRightCheck(True)
            depth.setSubpixel(True)
            depth.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            
            mono_left.out.link(depth.left)
            mono_right.out.link(depth.right)
            
            # MobileNet with OPTIMAL settings from tests
            detection_nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
            detection_nn.setBlobPath("./mobilenet-ssd_openvino_2022.1_6shave.blob")
            
            # Apply optimal parameters
            detection_nn.setConfidenceThreshold(self.CONFIDENCE_THRESHOLD)
            detection_nn.setBoundingBoxScaleFactor(self.BBOX_SCALE)
            detection_nn.setDepthLowerThreshold(150)
            detection_nn.setDepthUpperThreshold(10000)
            
            # Connect pipeline
            manip.out.link(detection_nn.input)
            depth.depth.link(detection_nn.inputDepth)
            
            # Output
            detection_out = pipeline.create(dai.node.XLinkOut)
            detection_out.setStreamName("detections")
            detection_nn.out.link(detection_out.input)
            
            return pipeline
            
        except Exception as e:
            print(f"❌ Failed to create optimized pipeline: {e}")
            return None
    
    def initialize_camera(self):
        """Initialize camera with optimized settings"""
        try:
            print("📷 Initializing Maxine optimized camera...")
            
            self.pipeline = self.create_optimized_pipeline()
            if not self.pipeline:
                return False
            
            self.device = dai.Device(self.pipeline)
            self.detection_queue = self.device.getOutputQueue("detections", maxSize=4, blocking=False)
            
            print("✅ Maxine optimized camera initialized")
            return True
            
        except Exception as e:
            print(f"❌ Camera initialization failed: {e}")
            return False
    
    def calculate_edge_zone(self, x_camera: float) -> Tuple[str, float]:
        """Calculate edge zone with Maxine's 127° FOV"""
        max_x_at_1m = 1000 * math.tan(math.radians(self.CAMERA_HFOV_DEGREES / 2))
        x_position_ratio = x_camera / max_x_at_1m if max_x_at_1m > 0 else 0
        x_position_ratio = max(-1.0, min(1.0, x_position_ratio))
        
        distance_from_center = abs(x_position_ratio)
        
        # Edge zone classification (based on test data showing issues at edges)
        if x_position_ratio < -self.EDGE_THRESHOLD:
            edge_zone = 'left'
        elif x_position_ratio > self.EDGE_THRESHOLD:
            edge_zone = 'right'
        else:
            edge_zone = 'center'
        
        return edge_zone, distance_from_center
    
    def apply_z_stability_filter(self, z_depth: float) -> Tuple[float, bool]:
        """Apply Z-depth stability filter (address 49.6% stability rate)"""
        self.z_history.append(z_depth)
        
        if len(self.z_history) < 3:
            return z_depth, True
        
        # Calculate moving average for stability
        recent_z_values = list(self.z_history)
        avg_z = sum(recent_z_values) / len(recent_z_values)
        z_variance = sum((z - avg_z) ** 2 for z in recent_z_values) / len(recent_z_values)
        z_std = math.sqrt(z_variance)
        
        # Check if current reading is stable (based on test variance of 1145mm)
        z_change = abs(z_depth - avg_z)
        is_stable = z_change < self.Z_FLICKER_THRESHOLD and z_std < 500  # 500mm tolerance
        
        # Return filtered value for stable readings
        if is_stable and len(self.z_history) >= self.Z_STABILITY_FILTER_SIZE:
            filtered_z = avg_z
        else:
            filtered_z = z_depth
        
        return filtered_z, is_stable
    
    def apply_edge_detection_filter(self, detection_data: PersonDetection) -> PersonDetection:
        """Apply edge detection smoothing (address 60.9% edge flicker)"""
        self.edge_detection_history.append(detection_data)
        
        if len(self.edge_detection_history) < 3:
            return detection_data
        
        # For edge detections, apply position smoothing
        if detection_data.edge_zone != 'center':
            recent_detections = list(self.edge_detection_history)[-3:]
            
            # Smooth X position for edge stability
            avg_x = sum(d.x_camera for d in recent_detections) / len(recent_detections)
            avg_z = sum(d.z_depth for d in recent_detections) / len(recent_detections)
            
            # Create smoothed detection
            smoothed_detection = PersonDetection(
                x_camera=avg_x,
                y_camera=detection_data.y_camera,
                z_depth=avg_z,
                confidence=detection_data.confidence,
                timestamp=detection_data.timestamp,
                edge_zone=detection_data.edge_zone,
                is_stable=detection_data.is_stable,
                gap_duration=detection_data.gap_duration
            )
            
            return smoothed_detection
        
        return detection_data
    
    def handle_detection_gap(self, current_time: float) -> Optional[PersonDetection]:
        """Handle detection gaps with bridging (max 3.0s based on test data)"""
        if self.gap_start_time is None:
            self.gap_start_time = current_time
            return None
        
        self.current_gap_duration = current_time - self.gap_start_time
        
        # Gap bridging within tolerance
        if (self.current_gap_duration <= self.MAX_GAP_TOLERANCE and 
            self.last_valid_detection is not None):
            
            # Return interpolated/predicted position
            gap_ratio = self.current_gap_duration / self.MAX_GAP_TOLERANCE
            
            # Simple prediction - could be enhanced with velocity tracking
            predicted_detection = PersonDetection(
                x_camera=self.last_valid_detection.x_camera,
                y_camera=self.last_valid_detection.y_camera,
                z_depth=self.last_valid_detection.z_depth,
                confidence=self.last_valid_detection.confidence * (1 - gap_ratio * 0.5),  # Decay confidence
                timestamp=current_time,
                edge_zone=self.last_valid_detection.edge_zone,
                is_stable=False,  # Mark as interpolated
                gap_duration=self.current_gap_duration
            )
            
            return predicted_detection
        
        # Gap too long - report loss
        if self.current_gap_duration > self.MAX_GAP_TOLERANCE:
            self.gap_events.append({
                'duration': self.current_gap_duration,
                'last_position': self.last_valid_detection.x_camera if self.last_valid_detection else None,
                'timestamp': current_time
            })
            
            # Reset gap tracking
            self.gap_start_time = None
            self.current_gap_duration = 0
            self.last_valid_detection = None
        
        return None
    
    def process_detections(self) -> Optional[PersonDetection]:
        """Process detections with all optimizations applied"""
        current_time = time.time()
        
        try:
            detections = self.detection_queue.tryGet()
            if detections:
                # Find person detections (class 15)
                person_detections = [det for det in detections.detections if det.label == 15]
                
                if person_detections:
                    # Get best person detection
                    best_person = max(person_detections, key=lambda p: p.confidence)
                    
                    # Extract basic data
                    x_camera = best_person.spatialCoordinates.x
                    y_camera = best_person.spatialCoordinates.y
                    z_depth = best_person.spatialCoordinates.z
                    confidence = best_person.confidence
                    
                    # Validate detection
                    if (confidence >= self.CONFIDENCE_THRESHOLD and 
                        self.MIN_STABLE_Z < z_depth < self.MAX_STABLE_Z):
                        
                        # Calculate gap duration
                        gap_duration = 0
                        if self.gap_start_time:
                            gap_duration = current_time - self.gap_start_time
                            self.gap_start_time = None
                            self.current_gap_duration = 0
                        
                        # Calculate edge zone
                        edge_zone, distance_from_center = self.calculate_edge_zone(x_camera)
                        
                        # Apply Z-stability filter
                        filtered_z, is_stable = self.apply_z_stability_filter(z_depth)
                        
                        # Create detection object
                        detection = PersonDetection(
                            x_camera=x_camera,
                            y_camera=y_camera,
                            z_depth=filtered_z,
                            confidence=confidence,
                            timestamp=current_time,
                            edge_zone=edge_zone,
                            is_stable=is_stable,
                            gap_duration=gap_duration
                        )
                        
                        # Apply edge detection smoothing
                        smoothed_detection = self.apply_edge_detection_filter(detection)
                        
                        # Update tracking
                        self.total_detections += 1
                        if is_stable:
                            self.stable_detections += 1
                        
                        self.edge_performance[edge_zone] += 1
                        self.last_valid_detection = smoothed_detection
                        self.last_detection_time = current_time
                        
                        return smoothed_detection
            
            # No valid detection - handle gap
            return self.handle_detection_gap(current_time)
            
        except Exception as e:
            print(f"❌ Detection processing error: {e}")
            return None
    
    def get_navigation_target(self) -> Optional[Tuple[float, float, float]]:
        """Get navigation target for Maxine robot (closest person)"""
        detection = self.process_detections()
        
        if detection:
            # Return position in robot coordinate system
            # X: left/right, Z: forward distance, Y: height
            return detection.x_camera, detection.z_depth, detection.y_camera
        
        return None
    
    def get_performance_stats(self) -> dict:
        """Get performance statistics for monitoring"""
        stability_rate = self.stable_detections / max(self.total_detections, 1)
        
        return {
            'total_detections': self.total_detections,
            'stability_rate': stability_rate,
            'gap_events': len(self.gap_events),
            'edge_performance': self.edge_performance.copy(),
            'current_gap_duration': self.current_gap_duration,
            'last_detection_time': self.last_detection_time
        }
    
    def cleanup(self):
        """Cleanup camera resources"""
        if self.device:
            self.device.close()
            
        # Print final performance report
        stats = self.get_performance_stats()
        print(f"\n📊 Maxine Detector Performance:")
        print(f"   Total Detections: {stats['total_detections']}")
        print(f"   Stability Rate: {stats['stability_rate']:.1%}")
        print(f"   Gap Events: {stats['gap_events']}")
        print(f"   Edge Performance: {stats['edge_performance']}")


# Example usage for Maxine robot navigation
def maxine_navigation_example():
    """Example implementation for Maxine robot navigation"""
    detector = MaxineOptimizedDetector()
    
    if not detector.initialize_camera():
        print("❌ Failed to initialize camera")
        return
    
    print("🤖 Maxine robot navigation active...")
    print("🎯 Searching for closest person to navigate to...")
    
    try:
        while True:
            target = detector.get_navigation_target()
            
            if target:
                x, z, y = target
                print(f"🎯 Target: X={x:.0f}mm, Z={z:.0f}mm (distance)")
                
                # Navigation logic for Maxine would go here:
                # - Convert to wheelchair base coordinates
                # - Account for LIDAR offset (50cm height diff, 13cm setback)
                # - Execute movement while avoiding obstacles
                # - Rotate head servo to track person
                
            else:
                print("👀 Searching for person...")
            
            time.sleep(0.1)  # 10 Hz update rate
            
    except KeyboardInterrupt:
        print("\n🛑 Navigation stopped")
    finally:
        detector.cleanup()


if __name__ == "__main__":
    maxine_navigation_example()