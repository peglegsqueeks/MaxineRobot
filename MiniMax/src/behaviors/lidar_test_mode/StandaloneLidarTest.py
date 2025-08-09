#!/usr/bin/env python3
"""
EXACT Standalone LiDAR Test - TRUE COPY of detection_consistency_test.py Pipeline
Uses EXACTLY the same pipeline settings that achieve ±37px consistency
NO DEVIATIONS from the working standalone test configuration
"""

import pygame
import math
import time
import threading
import queue
import py_trees
import csv
import statistics
import os
from collections import deque
from datetime import datetime
from py_trees.common import Status
from pyrplidar import PyRPlidar

from src.behaviors.MaxineBehavior import MaxineBehavior
from src.types.RobotModes import RobotMode
from src.types.MovementDirection import MovementDirection
from src.action_managers.VelocityManager import VelocityConfig

# Import existing LiDAR system
from .LidarTestBehavior import UltraStableLidarSystem, EnhancedFacialAnimationRestorer

# Import DepthAI for direct pipeline creation
try:
    import depthai as dai
    import cv2
    import numpy as np
    DEPTHAI_AVAILABLE = True
except ImportError as e:
    print(f"❌ DepthAI not available: {e}")
    DEPTHAI_AVAILABLE = False


class ExactStandaloneLidarTest(MaxineBehavior):
    """
    EXACT Copy of detection_consistency_test.py pipeline in LiDAR Test Mode
    Uses IDENTICAL settings that achieve ±37px consistency in standalone test
    """
    
    def __init__(self):
        super().__init__("Exact Standalone LiDAR Test - True Copy")
        
        # py_trees Lifecycle: __init__ - Just setup variables, NO hardware initialization
        
        # Blackboard setup
        self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.READ)
        self.blackboard.register_key("HEAD_CENTER_POSITION", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.WRITE)
        
        # Component references - NO INITIALIZATION HERE
        self.lidar_system = None
        self.screen = None
        self.head_tracker = None
        self.facial_restorer = None
        self.robot = None
        
        # EXACT COPY of standalone camera system settings
        self.device = None
        self.pipeline = None
        self.detection_queue = None
        self.preview_queue = None
        self.has_detection = False
        self.camera_initialized = False
        self.camera_error_message = ""
        
        # EXACT COPY of standalone camera settings - NO CHANGES
        self.camera_resolution_width = 300
        self.camera_resolution_height = 300
        self.camera_hfov_degrees = 114
        self.target_fps = 25
        # EXACT COPY: NO confidence threshold changes
        
        # Display parameters
        self.center_x = 0
        self.center_y = 0
        self.scale = 0
        self.update_counter = 0
        self.display_update_rate = 3
        
        # EXACT COPY of standalone CSV logging
        self.csv_log_filename = "LIDARTEST.csv"
        self.csv_initialized = False
        self.mode_start_time = 0
        
        # EXACT COPY of standalone consistency tracking
        self.x_midpoints = deque(maxlen=1000)
        self.x_midpoints_pixels = deque(maxlen=1000)
        self.x_midpoints_normalized = deque(maxlen=1000)
        self.variance_window = deque(maxlen=100)
        
        # Real-time variance calculation
        self.current_variance_pixels = 0.0
        self.current_std_dev_pixels = 0.0
        self.current_mean_pixels = 0.0
        
        # Consistency metrics
        self.detection_count = 0
        self.consistent_detection_count = 0
        self.large_jumps_count = 0
        self.jump_threshold_pixels = 50
        self.last_x_midpoint = None
        
        # EXACT COPY of standalone variance periods
        self.short_term_variance = deque(maxlen=25)   # 1-second variance (at 25fps)
        self.medium_term_variance = deque(maxlen=125) # 5-second variance  
        self.long_term_variance = deque(maxlen=750)   # 30-second variance
        
        # Stability classification
        self.stability_zones = {
            'stable': 0, 'moderate': 0, 'unstable': 0, 'very_unstable': 0
        }
        
        # EXACT COPY of standalone Z-depth smoothing - NO CHANGES
        self.z_depth_smoother = deque(maxlen=5)  # EXACT: 5 samples
        self.confidence_weights = deque(maxlen=5)
        self.smoothed_z_depth = 0
        self.depth_trust_threshold = 0.6  # EXACT: 0.6
        
        # CONSERVATIVE head tracking parameters (only for settled person)
        self.head_tracking_enabled = True
        self.angle_history = []
        self.max_angle_history = 15  # Conservative smoothing
        self.last_sent_angle = None
        self.angle_change_threshold = math.radians(15)  # Large threshold
        self.update_counter_tracking = 0
        self.tracking_update_interval = 12  # Conservative updates
        self.last_person_detected = 0
        self.person_lost_timeout = 3.0
        
        # Person settlement detection
        self.person_movement_threshold = 100  # pixels
        self.person_settled_time = 2.0  # seconds
        self.last_significant_movement = 0
        self.person_is_settled = False
        
        # EXACT COPY of standalone performance settings
        self.detection_skip_frames = 1
        self.frame_counter = 0
        
        # Grid overlay for potential field
        self.grid_size = 40
        self.show_grid = True
        self.last_potential_grid = {}
        
        # Initialize pygame if not already done
        if not pygame.get_init():
            pygame.init()
        pygame.font.init()
    
    def setup(self, **kwargs):
        """py_trees Lifecycle: setup() - Prepare for potential execution"""
        return True
    
    def initialise(self):
        """py_trees Lifecycle: initialise() - Called when behavior STARTS running"""
        print("🎯 EXACT Standalone LiDAR Test initializing...")
        
        # Get robot reference
        self.robot = self.get_robot()
        
        # Stop robot movement first
        self.stop_robot()
        
        # Initialize display
        self.initialize_display()
        
        # Initialize facial restorer
        self.facial_restorer = EnhancedFacialAnimationRestorer()
        
        # Initialize EXACT COPY camera system
        self.initialize_exact_camera()
        
        # Initialize conservative head tracker
        self.initialize_head_tracker()
        
        # Initialize LiDAR system
        self.initialize_lidar()
        
        # Initialize CSV logging
        self.mode_start_time = time.time()
        self.initialize_csv_log()
        
        # Reset tracking data
        self.reset_tracking_data()
        
        print("✅ EXACT Standalone LiDAR Test initialized successfully")
    
    def initialize_display(self):
        """Initialize pygame display"""
        try:
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
            pygame.display.set_caption("MAXINE EXACT STANDALONE LIDAR TEST - TRUE COPY")
            
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            self.scale = min(display_info.current_w, display_info.current_h) // 7
            
            # Draw initial interface
            self.draw_clean_interface()
            pygame.display.flip()
            
            print(f"📺 Display initialized: {display_info.current_w}x{display_info.current_h}")
            
        except Exception as e:
            print(f"❌ Display initialization failed: {e}")
    
    def check_camera_connection(self):
        """Check camera connection - EXACT COPY"""
        try:
            devices = dai.Device.getAllAvailableDevices()
            return len(devices) > 0, f"Found {len(devices)} device(s)"
        except Exception as e:
            return False, f"Device detection error: {str(e)}"
    
    def create_exact_pipeline(self):
        """Create EXACT COPY of detection_consistency_test.py pipeline - NO CHANGES"""
        try:
            pipeline = dai.Pipeline()
            
            local_blob_path = "./mobilenet-ssd_openvino_2021.4_5shave.blob"
            if not os.path.exists(local_blob_path):
                raise Exception(f"Blob file not found: {local_blob_path}")
            
            # EXACT COPY: Exact working camera setup from standalone test
            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            mono_left.setFps(self.target_fps)
            
            mono_right = pipeline.create(dai.node.MonoCamera)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            mono_right.setFps(self.target_fps)
            
            # EXACT COPY: Exact working ImageManip setup from standalone test
            manip_nn = pipeline.create(dai.node.ImageManip)
            manip_nn.initialConfig.setResize(300, 300)
            manip_nn.initialConfig.setKeepAspectRatio(False)
            manip_nn.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
            mono_right.out.link(manip_nn.inputImage)
            
            # Display manip for camera preview
            manip_display = pipeline.create(dai.node.ImageManip)
            manip_display.initialConfig.setResize(600, 450)  # Same as standalone
            manip_display.initialConfig.setKeepAspectRatio(True)
            manip_display.initialConfig.setFrameType(dai.ImgFrame.Type.GRAY8)
            mono_right.out.link(manip_display.inputImage)
            
            # EXACT COPY: Exact working stereo depth settings from standalone test
            depth = pipeline.create(dai.node.StereoDepth)
            depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)  # EXACT: DEFAULT
            depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
            depth.initialConfig.setConfidenceThreshold(180)  # EXACT: 180
            depth.setLeftRightCheck(True)
            depth.setSubpixel(False)  # EXACT: False
            depth.setDepthAlign(dai.CameraBoardSocket.CAM_C)
            
            mono_left.out.link(depth.left)
            mono_right.out.link(depth.right)
            
            # EXACT COPY: Exact working detection network settings from standalone test
            detection_nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
            detection_nn.setConfidenceThreshold(0.4)  # EXACT: 0.4
            detection_nn.setBlobPath(local_blob_path)
            detection_nn.setBoundingBoxScaleFactor(0.5)  # EXACT: 0.5
            detection_nn.setDepthLowerThreshold(100)    # EXACT: 100
            detection_nn.setDepthUpperThreshold(8000)   # EXACT: 8000
            
            manip_nn.out.link(detection_nn.input)
            depth.depth.link(detection_nn.inputDepth)
            
            # Outputs
            detection_out = pipeline.create(dai.node.XLinkOut)
            detection_out.setStreamName("detections")
            detection_nn.out.link(detection_out.input)
            
            preview_out = pipeline.create(dai.node.XLinkOut)
            preview_out.setStreamName("preview")
            manip_display.out.link(preview_out.input)
            
            return pipeline
        except Exception as e:
            print(f"❌ Pipeline creation failed: {e}")
            return None
    
    def initialize_exact_camera(self):
        """Initialize EXACT COPY of standalone camera system"""
        if not DEPTHAI_AVAILABLE:
            print("❌ DepthAI not available - camera disabled")
            self.camera_initialized = False
            return
        
        try:
            print("📷 Initializing EXACT COPY of standalone camera system...")
            
            connected, message = self.check_camera_connection()
            if not connected:
                self.camera_error_message = message
                print(f"❌ Camera connection failed: {message}")
                self.camera_initialized = False
                return
            
            self.pipeline = self.create_exact_pipeline()
            if not self.pipeline:
                self.camera_error_message = "Pipeline creation failed"
                print("❌ Pipeline creation failed")
                self.camera_initialized = False
                return
            
            try:
                self.device = dai.Device(self.pipeline)
                print("✅ DepthAI device connected")
            except Exception as e:
                self.camera_error_message = f"Device connection failed: {str(e)}"
                print(f"❌ Device connection failed: {e}")
                self.camera_initialized = False
                return
            
            # EXACT COPY: Device optimization from standalone test
            try:
                if hasattr(self.device, 'setLogLevel'):
                    self.device.setLogLevel(dai.LogLevel.WARN)
                self.device.setIrLaserDotProjectorIntensity(900)  # EXACT: 900
                print("✅ Device optimized (exact copy)")
            except Exception:
                pass
            
            # EXACT COPY: Queue settings from standalone test
            try:
                self.detection_queue = self.device.getOutputQueue("detections", maxSize=4, blocking=False)  # EXACT: 4
                self.has_detection = True
                print("✅ Detection queue initialized (exact copy)")
            except Exception:
                self.detection_queue = None
                self.has_detection = False
            
            try:
                self.preview_queue = self.device.getOutputQueue("preview", maxSize=4, blocking=False)  # EXACT: 4
                print("✅ Preview queue initialized (exact copy)")
            except Exception:
                self.preview_queue = None
            
            # EXACT COPY: Camera frame testing from standalone test
            if self.preview_queue:
                for i in range(20):  # EXACT: 20 iterations
                    test_frame = self.preview_queue.tryGet()
                    if test_frame:
                        self.camera_initialized = True
                        break
                    time.sleep(0.1)
            
            time.sleep(2)  # EXACT: 2 seconds
            print("✅ EXACT COPY camera system initialized successfully")
            
        except Exception as e:
            self.camera_error_message = f"Camera initialization error: {str(e)}"
            print(f"❌ Camera initialization error: {e}")
            self.camera_initialized = False
    
    def initialize_head_tracker(self):
        """Initialize conservative head tracker"""
        if not self.head_tracking_enabled:
            return
        
        try:
            if (hasattr(self.robot, 'head_velocity_manager') and self.robot.head_velocity_manager) or \
               (hasattr(self.robot, 'servo_controller') and self.robot.servo_controller):
                try:
                    from src.behaviors.lidarchase.HeadTracker import HeadTracker
                    self.head_tracker = HeadTracker(
                        self.robot.head_velocity_manager if hasattr(self.robot, 'head_velocity_manager') else None,
                        self.robot.servo_controller if hasattr(self.robot, 'servo_controller') else None
                    )
                    self.head_tracker.start_tracking()
                    self.head_tracker.set_manual_position(0.0)
                    print("✅ Conservative head tracker initialized")
                except Exception as e:
                    print(f"⚠️ Head tracker initialization failed: {e}")
                    self.head_tracker = None
            else:
                self.head_tracker = None
        except Exception as e:
            print(f"⚠️ Head tracker setup error: {e}")
            self.head_tracker = None
    
    def initialize_lidar(self):
        """Initialize LiDAR system"""
        try:
            print("🚀 Starting LiDAR system...")
            self.lidar_system = UltraStableLidarSystem()
            success = self.lidar_system.start()
            if success:
                self.blackboard.set("LIDAR_SYSTEM", self.lidar_system)
                print("✅ LiDAR system started successfully")
                time.sleep(2)
            else:
                print("❌ Failed to start LiDAR system")
                self.lidar_system = None
        except Exception as e:
            print(f"❌ LiDAR initialization error: {e}")
            self.lidar_system = None
    
    def initialize_csv_log(self):
        """Initialize CSV log - Enhanced with Z-depth analysis"""
        if self.csv_initialized:
            return
        
        try:
            with open(self.csv_log_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'mode_time_elapsed', 'timestamp', 'frame_number',
                    'x_midpoint_pixels', 'x_midpoint_normalized', 'x_camera_mm',
                    'z_depth_mm', 'raw_z_depth_mm', 'confidence',
                    'bbox_xmin', 'bbox_ymin', 'bbox_xmax', 'bbox_ymax', 'bbox_width', 'bbox_height',
                    'x_jump_from_previous', 'is_large_jump', 
                    'rolling_variance_pixels', 'rolling_std_dev_pixels', 'rolling_mean_pixels',
                    'short_term_variance', 'medium_term_variance', 'long_term_variance',
                    'detection_count', 'consistent_detections', 'large_jumps_count',
                    'stability_classification', 'head_angle_deg', 'head_tracking_active',
                    'person_is_settled', 'detection_method'
                ])
            self.csv_initialized = True
            print(f"✅ EXACT COPY CSV logging to: {self.csv_log_filename}")
        except Exception as e:
            print(f"⚠️ CSV initialization failed: {e}")
    
    def reset_tracking_data(self):
        """Reset all tracking data for new session"""
        self.detection_count = 0
        self.consistent_detection_count = 0
        self.large_jumps_count = 0
        self.last_x_midpoint = None
        
        # Clear tracking data
        self.x_midpoints.clear()
        self.x_midpoints_pixels.clear()
        self.x_midpoints_normalized.clear()
        self.variance_window.clear()
        self.short_term_variance.clear()
        self.medium_term_variance.clear()
        self.long_term_variance.clear()
        
        # Reset stability zones
        for zone in self.stability_zones:
            self.stability_zones[zone] = 0
    
    def smooth_z_depth(self, raw_z_depth, confidence):
        """EXACT COPY of standalone Z-depth smoothing - NO CHANGES"""
        try:
            self.z_depth_smoother.append(raw_z_depth)
            self.confidence_weights.append(confidence)
            
            if len(self.z_depth_smoother) < 2:
                self.smoothed_z_depth = raw_z_depth
                return raw_z_depth
            
            total_weight = 0
            weighted_sum = 0
            
            for i, (z_val, conf) in enumerate(zip(self.z_depth_smoother, self.confidence_weights)):
                recency_weight = (i + 1) / len(self.z_depth_smoother)
                confidence_weight = max(0.1, conf)
                combined_weight = recency_weight * confidence_weight
                
                weighted_sum += z_val * combined_weight
                total_weight += combined_weight
            
            smoothed = weighted_sum / total_weight if total_weight > 0 else raw_z_depth
            
            if confidence < self.depth_trust_threshold and len(self.z_depth_smoother) > 1:
                prev_z = self.z_depth_smoother[-2]
                max_jump = 500
                if abs(smoothed - prev_z) > max_jump:
                    blend_factor = confidence / self.depth_trust_threshold
                    smoothed = prev_z + (smoothed - prev_z) * blend_factor
            
            self.smoothed_z_depth = smoothed
            return smoothed
        except Exception:
            return raw_z_depth
    
    def get_exact_person_detection(self):
        """Get person detection using EXACT COPY of standalone logic"""
        if not self.camera_initialized or not self.detection_queue:
            return None
        
        start_time = time.time()
        self.frame_counter += 1
        
        if self.frame_counter % self.detection_skip_frames != 0:
            return None
        
        try:
            detections = self.detection_queue.tryGet()
            if not detections:
                return None
            
            # EXACT COPY: Same person detection logic as standalone test
            person_detections = [det for det in detections.detections if det.label == 15]
            if not person_detections:
                return None
            
            # EXACT COPY: Use closest person (same as standalone test)
            closest_person = min(person_detections, key=lambda p: p.spatialCoordinates.z)
            
            x_camera = closest_person.spatialCoordinates.x
            y_camera = closest_person.spatialCoordinates.y
            raw_z_depth = closest_person.spatialCoordinates.z
            confidence = closest_person.confidence
            
            if raw_z_depth <= 0 or raw_z_depth > 15000:
                return None
            
            smoothed_z_depth = self.smooth_z_depth(raw_z_depth, confidence)
            
            # EXACT COPY: Same X midpoint calculations as standalone test
            bbox_xmin = closest_person.xmin
            bbox_xmax = closest_person.xmax
            bbox_ymin = closest_person.ymin
            bbox_ymax = closest_person.ymax
            
            # EXACT COPY: Same screen coordinate conversion
            x_midpoint_normalized = (bbox_xmin + bbox_xmax) / 2.0
            x_midpoint_pixels = int(x_midpoint_normalized * self.screen.get_width())
            
            processing_time = (time.time() - start_time) * 1000
            
            return {
                'x_camera': x_camera,
                'y_camera': y_camera,
                'z_camera': smoothed_z_depth,
                'raw_z_depth': raw_z_depth,
                'confidence': confidence,
                'bbox_center': {
                    'x_normalized': x_midpoint_normalized,
                    'y_normalized': (bbox_ymin + bbox_ymax) / 2.0,
                    'x_pixels': x_midpoint_pixels,
                    'y_pixels': int(((bbox_ymin + bbox_ymax) / 2.0) * self.screen.get_height())
                },
                'bounding_box': {
                    'xmin': bbox_xmin,
                    'xmax': bbox_xmax,
                    'ymin': bbox_ymin,
                    'ymax': bbox_ymax
                },
                'processing_time_ms': processing_time,
                'detection_method': 'EXACT_STANDALONE_COPY'
            }
            
        except Exception as e:
            print(f"⚠️ Detection error: {e}")
            return None
    
    def update(self) -> Status:
        """py_trees Lifecycle: update() - Main behavior execution"""
        try:
            self.update_counter += 1
            
            # Handle pygame events
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        print("🔄 ESC pressed - returning to IDLE mode")
                        return Status.SUCCESS
                elif event.type == pygame.QUIT:
                    print("🔄 Window closed - returning to IDLE mode")
                    return Status.SUCCESS
            
            # Handle person detection using EXACT COPY logic
            person_data = self.get_exact_person_detection()
            if person_data:
                self.log_detection_to_csv(person_data)
                self.update_conservative_head_tracking(person_data)
                self.last_person_detected = time.time()
            elif time.time() - self.last_person_detected > self.person_lost_timeout and self.head_tracker:
                # Clear tracking when person is lost
                self.angle_history.clear()
                self.last_sent_angle = None
                self.head_tracker.set_manual_position(0.0)
                self.person_is_settled = False
            
            # Display update
            if self.update_counter % self.display_update_rate == 0:
                try:
                    self.update_display(person_data)
                except Exception as e:
                    print(f"Display error: {e}")
            
            # Grid overlay update
            if self.show_grid and self.update_counter % 8 == 0:
                self.update_potential_field_grid(person_data)
            
            return Status.RUNNING
            
        except Exception as e:
            print(f"Update error: {e}")
            return Status.SUCCESS
    
    def update_conservative_head_tracking(self, person_data):
        """Conservative head tracking - only when person is settled"""
        if not self.head_tracker or not person_data or not self.head_tracking_enabled:
            return
        
        try:
            bbox_center = person_data['bbox_center']
            x_pixels = bbox_center['x_pixels']
            z_camera = person_data['z_camera']
            
            if z_camera <= 0:
                return
            
            # Person movement detection
            current_time = time.time()
            if self.last_x_midpoint is not None:
                movement = abs(x_pixels - self.last_x_midpoint)
                if movement > self.person_movement_threshold:
                    self.last_significant_movement = current_time
                    self.person_is_settled = False
                elif current_time - self.last_significant_movement > self.person_settled_time:
                    self.person_is_settled = True
            
            # Conservative frame skipping
            self.update_counter_tracking += 1
            if self.update_counter_tracking % self.tracking_update_interval != 0:
                return
            
            # Only track if person is settled
            if not self.person_is_settled:
                return
            
            # Calculate angle
            screen_center_x = self.screen.get_width() // 2
            pixel_offset = x_pixels - screen_center_x
            pixel_offset_normalized = pixel_offset / screen_center_x
            
            camera_hfov_rad = math.radians(108)
            raw_angle_rad = -pixel_offset_normalized * (camera_hfov_rad / 2.0)
            
            # Conservative smoothing
            self.angle_history.append(raw_angle_rad)
            if len(self.angle_history) > self.max_angle_history:
                self.angle_history.pop(0)
            
            # Weighted smoothing
            if len(self.angle_history) >= 10:
                weights = [i+1 for i in range(len(self.angle_history))]
                weighted_sum = sum(angle * weight for angle, weight in zip(self.angle_history, weights))
                total_weight = sum(weights)
                smoothed_angle_rad = weighted_sum / total_weight
            else:
                return  # Need enough samples
            
            # Large dead zone
            dead_zone_rad = math.radians(20)
            is_in_dead_zone = abs(smoothed_angle_rad) <= dead_zone_rad
            
            # Large change threshold
            significant_change = (self.last_sent_angle is None or 
                                abs(smoothed_angle_rad - self.last_sent_angle) > self.angle_change_threshold)
            
            # Conservative movement - only when outside dead zone, significant change, and person settled
            will_send_command = (not is_in_dead_zone) and significant_change and self.person_is_settled
            
            if will_send_command:
                print(f"🎯 Head tracking: {math.degrees(smoothed_angle_rad):.1f}° (Person settled)")
                self.head_tracker.set_person_tracking(smoothed_angle_rad)
                self.last_sent_angle = smoothed_angle_rad
            
        except Exception as e:
            print(f"⚠️ Head tracking error: {e}")
    
    # EXACT COPY of standalone variance calculation methods
    def calculate_x_midpoint_variance(self):
        """EXACT COPY of standalone variance calculation"""
        try:
            if len(self.variance_window) < 2:
                return 0.0, 0.0, 0.0
            
            window_data = list(self.variance_window)
            
            if len(window_data) < 2:
                return 0.0, 0.0, 0.0
            
            mean_val = statistics.mean(window_data)
            variance_val = statistics.variance(window_data) if len(window_data) > 1 else 0.0
            std_dev_val = math.sqrt(variance_val)
            
            return variance_val, std_dev_val, mean_val
        except Exception:
            return 0.0, 0.0, 0.0
    
    def calculate_multi_term_variance(self, x_pixel):
        """EXACT COPY of standalone multi-term variance calculation"""
        self.short_term_variance.append(x_pixel)
        self.medium_term_variance.append(x_pixel)
        self.long_term_variance.append(x_pixel)
        
        try:
            short_var = statistics.variance(list(self.short_term_variance)) if len(self.short_term_variance) > 1 else 0.0
            medium_var = statistics.variance(list(self.medium_term_variance)) if len(self.medium_term_variance) > 1 else 0.0
            long_var = statistics.variance(list(self.long_term_variance)) if len(self.long_term_variance) > 1 else 0.0
            
            return short_var, medium_var, long_var
        except Exception:
            return 0.0, 0.0, 0.0
    
    def classify_stability(self, std_dev_pixels):
        """EXACT COPY of standalone stability classification"""
        if std_dev_pixels < 10:
            return 'stable'
        elif std_dev_pixels < 25:
            return 'moderate'
        elif std_dev_pixels < 50:
            return 'unstable'
        else:
            return 'very_unstable'
    
    def log_detection_to_csv(self, person_data):
        """Log detection data - same structure as standalone test"""
        try:
            if not self.csv_initialized:
                self.initialize_csv_log()
            
            bbox_center = person_data['bbox_center']
            x_midpoint_pixels = bbox_center['x_pixels']
            x_midpoint_normalized = bbox_center['x_normalized']
            
            # EXACT COPY: Track midpoints same as standalone test
            self.x_midpoints.append(person_data['x_camera'])
            self.x_midpoints_pixels.append(x_midpoint_pixels)
            self.x_midpoints_normalized.append(x_midpoint_normalized)
            self.variance_window.append(x_midpoint_pixels)
            
            # Calculate jump from previous detection
            x_jump = 0
            is_large_jump = False
            if self.last_x_midpoint is not None:
                x_jump = abs(x_midpoint_pixels - self.last_x_midpoint)
                is_large_jump = x_jump > self.jump_threshold_pixels
                if is_large_jump:
                    self.large_jumps_count += 1
            
            self.last_x_midpoint = x_midpoint_pixels
            
            # Update detection counts
            self.detection_count += 1
            if not is_large_jump:
                self.consistent_detection_count += 1
            
            # Calculate variance metrics
            self.current_variance_pixels, self.current_std_dev_pixels, self.current_mean_pixels = self.calculate_x_midpoint_variance()
            
            # Calculate multi-term variance
            short_var, medium_var, long_var = self.calculate_multi_term_variance(x_midpoint_pixels)
            
            # Classify stability
            stability_class = self.classify_stability(self.current_std_dev_pixels)
            self.stability_zones[stability_class] += 1
            
            # Get current head tracking info
            head_angle_deg = math.degrees(self.last_sent_angle) if self.last_sent_angle else 0.0
            head_tracking_active = self.head_tracker is not None and self.head_tracking_enabled
            
            # Calculate mode elapsed time
            mode_elapsed = time.time() - self.mode_start_time
            
            # Write to CSV
            with open(self.csv_log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    mode_elapsed, time.time(), self.update_counter,
                    x_midpoint_pixels, x_midpoint_normalized,
                    person_data['x_camera'], person_data['z_camera'], person_data['raw_z_depth'], person_data['confidence'],
                    person_data['bounding_box']['xmin'], person_data['bounding_box']['ymin'],
                    person_data['bounding_box']['xmax'], person_data['bounding_box']['ymax'],
                    person_data['bounding_box']['xmax'] - person_data['bounding_box']['xmin'],
                    person_data['bounding_box']['ymax'] - person_data['bounding_box']['ymin'],
                    x_jump, is_large_jump,
                    self.current_variance_pixels, self.current_std_dev_pixels, self.current_mean_pixels,
                    short_var, medium_var, long_var,
                    self.detection_count, self.consistent_detection_count, self.large_jumps_count,
                    stability_class, head_angle_deg, head_tracking_active,
                    self.person_is_settled, person_data.get('detection_method', 'EXACT_STANDALONE_COPY')
                ])
        except Exception:
            pass
    
    # Display methods (simplified for focus on detection)
    def draw_clean_interface(self):
        """Draw clean radar interface"""
        self.screen.fill((0, 0, 0))
        self.draw_radar_grid()
        self.draw_robot()
    
    def draw_radar_grid(self):
        """Draw radar-style grid"""
        for distance in [1000, 2000, 3000, 4000, 5000, 6000]:
            radius = distance * self.scale // 1000
            if radius < min(self.center_x, self.center_y) - 50:
                line_width = 3 if distance == 6000 else 2
                color = (0, 150, 0) if distance < 6000 else (255, 255, 0)
                pygame.draw.circle(self.screen, color, (self.center_x, self.center_y), radius, line_width)
        
        for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
            display_angle_rad = math.radians(90 - angle)
            line_length = min(self.center_x, self.center_y) - 80
            end_x = self.center_x + int(line_length * math.cos(display_angle_rad))
            end_y = self.center_y - int(line_length * math.sin(display_angle_rad))
            line_width = 3 if angle % 90 == 0 else 1
            pygame.draw.line(self.screen, (0, 150, 0), (self.center_x, self.center_y), (end_x, end_y), line_width)
    
    def draw_robot(self):
        """Draw robot representation"""
        pygame.draw.circle(self.screen, (0, 255, 0), (self.center_x, self.center_y), 15, 3)
        arrow_end_x = self.center_x
        arrow_end_y = self.center_y - 30
        pygame.draw.line(self.screen, (0, 255, 0), (self.center_x, self.center_y), (arrow_end_x, arrow_end_y), 5)
    
    def draw_lidar_data(self, obstacles):
        """Draw LiDAR obstacles"""
        if not obstacles:
            return 0
        
        obstacle_count = 0
        for angle, distance in obstacles:
            corrected_angle_rad = math.radians(90 - angle)
            
            x = self.center_x + math.cos(corrected_angle_rad) * (distance * self.scale // 1000)
            y = self.center_y - math.sin(corrected_angle_rad) * (distance * self.scale // 1000)
            
            if (0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height()):
                pygame.draw.circle(self.screen, (255, 255, 255), (int(x), int(y)), 2)
                obstacle_count += 1
        
        return obstacle_count
    
    def draw_person_detection(self, person_data):
        """Draw person detection with stability info"""
        try:
            x_camera = person_data['x_camera']
            z_camera = person_data['z_camera']
            confidence = person_data['confidence']
            bbox_center = person_data['bbox_center']
            
            if z_camera <= 0:
                return
            
            # Calculate position for radar display
            person_angle_rad = math.atan2(x_camera, z_camera)
            person_angle_deg = math.degrees(person_angle_rad)
            
            display_angle_deg = 360 - person_angle_deg
            while display_angle_deg < 0:
                display_angle_deg += 360
            while display_angle_deg >= 360:
                display_angle_deg -= 360
            
            distance_m = z_camera / 1000.0
            display_angle_rad = math.radians(90 - display_angle_deg)
            
            x = self.center_x + int(distance_m * self.scale * math.cos(display_angle_rad))
            y = self.center_y - int(distance_m * self.scale * math.sin(display_angle_rad))
            
            # Color by stability
            stability_class = self.classify_stability(self.current_std_dev_pixels)
            if stability_class == 'stable':
                color = (0, 255, 0)
            elif stability_class == 'moderate':
                color = (255, 255, 0)
            elif stability_class == 'unstable':
                color = (255, 165, 0)
            else:
                color = (255, 0, 0)
            
            # Draw detection
            if 0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height():
                radius = 10 if self.person_is_settled else 6
                pygame.draw.circle(self.screen, color, (x, y), radius)
                pygame.draw.circle(self.screen, (255, 255, 255), (x, y), radius + 1, 2)
            
            # Draw coordinate information
            self.draw_coordinate_info(bbox_center, z_camera)
            
        except Exception:
            pass
    
    def draw_coordinate_info(self, bbox_center, z_camera):
        """Draw coordinate info"""
        try:
            screen_width = self.screen.get_width()
            
            x_norm = bbox_center['x_normalized']
            x_pixels = bbox_center['x_pixels']
            
            large_font = pygame.font.Font(None, 72)
            medium_font = pygame.font.Font(None, 48)
            
            main_text = f"Person X: {x_pixels}px | Z: {z_camera:.0f}mm (EXACT COPY)"
            detail_text = f"Variance: ±{self.current_std_dev_pixels:.1f}px | Target: ±37px"
            consistency_text = f"Detections: {self.detection_count} | Jumps: {self.large_jumps_count}"
            settlement_text = f"Person: {'SETTLED' if self.person_is_settled else 'MOVING'} | Head: {'ACTIVE' if self.person_is_settled else 'PAUSED'}"
            
            # Color based on target achievement
            if self.current_std_dev_pixels <= 37:
                main_color = (0, 255, 0)      # Green - achieved target
            elif self.current_std_dev_pixels <= 75:
                main_color = (255, 255, 0)    # Yellow - close to target
            elif self.current_std_dev_pixels <= 150:
                main_color = (255, 165, 0)    # Orange - needs work
            else:
                main_color = (255, 0, 0)      # Red - far from target
            
            # Render text
            main_surface = large_font.render(main_text, True, main_color)
            detail_surface = medium_font.render(detail_text, True, (255, 255, 255))
            consistency_surface = medium_font.render(consistency_text, True, (0, 255, 255))
            settlement_surface = medium_font.render(settlement_text, True, (255, 255, 0))
            
            # Position
            main_x = (screen_width - main_surface.get_width()) // 2
            main_y = 30
            
            detail_x = (screen_width - detail_surface.get_width()) // 2
            detail_y = main_y + main_surface.get_height() + 5
            
            consistency_x = (screen_width - consistency_surface.get_width()) // 2
            consistency_y = detail_y + detail_surface.get_height() + 5
            
            settlement_x = (screen_width - settlement_surface.get_width()) // 2
            settlement_y = consistency_y + consistency_surface.get_height() + 5
            
            # Draw background
            total_height = (main_surface.get_height() + detail_surface.get_height() + 
                          consistency_surface.get_height() + settlement_surface.get_height() + 20)
            max_width = max(main_surface.get_width(), detail_surface.get_width(), 
                          consistency_surface.get_width(), settlement_surface.get_width())
            status_bg = pygame.Rect(main_x - 10, main_y - 5, max_width + 20, total_height)
            
            pygame.draw.rect(self.screen, (0, 0, 0), status_bg)
            pygame.draw.rect(self.screen, main_color, status_bg, 3)
            
            # Draw text
            self.screen.blit(main_surface, (main_x, main_y))
            self.screen.blit(detail_surface, (detail_x, detail_y))
            self.screen.blit(consistency_surface, (consistency_x, consistency_y))
            self.screen.blit(settlement_surface, (settlement_x, settlement_y))
            
        except Exception:
            pass
    
    def update_display(self, person_data):
        """Update pygame display"""
        if not self.screen:
            return
        
        # Clear and draw base interface
        self.screen.fill((0, 0, 0))
        self.draw_radar_grid()
        self.draw_robot()
        
        # Draw LiDAR obstacles
        obstacle_count = 0
        if self.lidar_system:
            obstacles = self.lidar_system.get_display_obstacles()
            if obstacles:
                obstacle_count = self.draw_lidar_data(obstacles)
        
        # Draw person detection
        if person_data:
            self.draw_person_detection(person_data)
        
        # Draw grid overlay
        if self.show_grid:
            self.draw_grid_lines()
            self.draw_cached_potential_numbers()
        
        # Draw info
        self.draw_info(obstacle_count)
        
        pygame.display.flip()
    
    def draw_info(self, obstacle_count):
        """Draw test information"""
        try:
            tracking_status = "CONSERVATIVE" if (self.head_tracker and self.head_tracking_enabled) else "OFF"
            lidar_status = "ACTIVE" if self.lidar_system else "INACTIVE"
            camera_status = "EXACT_COPY" if self.camera_initialized else "INACTIVE"
            
            current_head_angle = math.degrees(self.last_sent_angle) if self.last_sent_angle else 0.0
            consistency_rate = (self.consistent_detection_count / max(1, self.detection_count)) * 100
            
            info_lines = [
                f"MAXINE EXACT COPY - LiDAR: {lidar_status} | Camera: {camera_status} | Head: {tracking_status}",
                f"Obstacles: {obstacle_count} | Head: {current_head_angle:.1f}° | Target: ±37px",
                f"Consistency: {consistency_rate:.1f}% (±{self.current_std_dev_pixels:.1f}px)",
                f"Detections: {self.detection_count} | Jumps: {self.large_jumps_count}",
                f"CSV: {self.csv_log_filename} | ESC: Exit | EXACT COPY MODE"
            ]
            
            y_offset = self.screen.get_height() - 150
            font = pygame.font.Font(None, 32)
            
            for i, line in enumerate(info_lines):
                if i == 0:
                    color = (0, 255, 255)  # Cyan for header
                elif "Target" in line:
                    if self.current_std_dev_pixels <= 37:
                        color = (0, 255, 0)      # Green - achieved target
                    elif self.current_std_dev_pixels <= 75:
                        color = (255, 255, 0)    # Yellow - close
                    else:
                        color = (255, 0, 0)      # Red - needs work
                elif "EXACT COPY" in line:
                    color = (255, 165, 0)
                else:
                    color = (255, 255, 255)
                
                text_surface = font.render(line, True, color)
                self.screen.blit(text_surface, (20, y_offset + i * 28))
            
        except Exception:
            pass
    
    # Implement remaining utility methods (grid overlay, cleanup, etc.)
    def update_potential_field_grid(self, person_data):
        """Update potential field grid"""
        try:
            if person_data:
                x_camera = person_data['x_camera']
                z_camera = person_data['z_camera']
                
                if z_camera > 0:
                    person_angle_rad = math.atan2(x_camera, z_camera)
                    person_angle_deg = math.degrees(person_angle_rad)
                    
                    display_angle_deg = 360 - person_angle_deg
                    while display_angle_deg < 0:
                        display_angle_deg += 360
                    while display_angle_deg >= 360:
                        display_angle_deg -= 360
                    
                    distance_m = z_camera / 1000.0
                    display_angle_rad = math.radians(90 - display_angle_deg)
                    
                    person_x = self.center_x + int(distance_m * self.scale * math.cos(display_angle_rad))
                    person_y = self.center_y - int(distance_m * self.scale * math.sin(display_angle_rad))
                    
                    grid_values = {}
                    
                    for dx in [-40, 0, 40]:
                        for dy in [-40, 0, 40]:
                            px = person_x + dx
                            py = person_y + dy
                            
                            if 0 <= px < self.screen.get_width() and 0 <= py < self.screen.get_height():
                                grid_col = px // self.grid_size
                                grid_row = py // self.grid_size
                                
                                if dx == 0 and dy == 0:
                                    grid_values[(grid_col, grid_row)] = "+9"
                                else:
                                    grid_values[(grid_col, grid_row)] = "+5"
                    
                    self.last_potential_grid = grid_values
        except Exception:
            pass
    
    def draw_grid_lines(self):
        """Draw grid lines overlay"""
        try:
            grid_color = (100, 100, 100)
            screen_width = self.screen.get_width()
            screen_height = self.screen.get_height()
            
            for x in range(0, screen_width, self.grid_size):
                pygame.draw.line(self.screen, grid_color, (x, 0), (x, screen_height), 1)
            
            for y in range(0, screen_height, self.grid_size):
                pygame.draw.line(self.screen, grid_color, (0, y), (screen_width, y), 1)
                
        except Exception:
            pass
    
    def draw_cached_potential_numbers(self):
        """Draw potential field numbers"""
        try:
            font = pygame.font.Font(None, 24)
            
            for (grid_col, grid_row), text in self.last_potential_grid.items():
                grid_center_x = grid_col * self.grid_size + self.grid_size // 2
                grid_center_y = grid_row * self.grid_size + self.grid_size // 2
                
                color = (0, 255, 0) if text.startswith('+') else (255, 0, 0)
                
                text_surface = font.render(text, True, color)
                text_rect = text_surface.get_rect(center=(grid_center_x, grid_center_y))
                
                bg_rect = text_rect.inflate(6, 4)
                pygame.draw.rect(self.screen, (0, 0, 0), bg_rect)
                self.screen.blit(text_surface, text_rect)
                
        except Exception:
            pass
    
    def stop_robot(self):
        """Stop robot movement"""
        try:
            if not self.robot:
                return
                
            velocity_manager = None
            if hasattr(self.robot, 'direct_velocity_manager') and self.robot.direct_velocity_manager:
                velocity_manager = self.robot.direct_velocity_manager
            elif hasattr(self.robot, 'velocity_manager') and self.robot.velocity_manager:
                velocity_manager = self.robot.velocity_manager
            
            if velocity_manager:
                stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                velocity_manager.perform_action(stop_config)
                
        except Exception:
            pass
    
    def center_head_for_idle_mode(self):
        """Center head for idle mode"""
        try:
            if not self.robot:
                return
                
            if hasattr(self.robot, 'servo_controller') and self.robot.servo_controller:
                self.robot.servo_controller.center()
                time.sleep(0.5)
            elif hasattr(self.robot, 'head_velocity_manager') and self.robot.head_velocity_manager:
                self.robot.head_velocity_manager.center_head()
                time.sleep(0.5)
                    
        except Exception:
            pass
    
    def cleanup_exact_camera(self):
        """Cleanup exact camera system"""
        try:
            if self.device:
                print("🛑 Stopping exact copy camera...")
                self.device.close()
                self.device = None
                print("✅ Exact copy camera stopped")
        except Exception as e:
            print(f"⚠️ Camera cleanup warning: {e}")
    
    def terminate(self, new_status: Status):
        """py_trees Lifecycle: terminate() - Enhanced cleanup"""
        print("🔄 EXACT Standalone LiDAR Test terminating...")
        
        try:
            # Generate summary
            if self.detection_count > 0:
                overall_variance = statistics.variance(list(self.x_midpoints_pixels)) if len(self.x_midpoints_pixels) > 1 else 0
                overall_std_dev = math.sqrt(overall_variance)
                consistency_rate = (self.consistent_detection_count / self.detection_count) * 100
                
                print(f"\n📊 EXACT COPY CONSISTENCY SUMMARY:")
                print(f"   Detection Method: EXACT_STANDALONE_COPY (No pipeline changes)")
                print(f"   Total Detections: {self.detection_count}")
                print(f"   Consistency Rate: {consistency_rate:.1f}%")
                print(f"   X-Coordinate Std Dev: ±{overall_std_dev:.2f} pixels")
                print(f"   TARGET: ±37 pixels")
                print(f"   Achievement: {('✅ TARGET ACHIEVED' if overall_std_dev <= 37 else f'❌ {(overall_std_dev/37):.1f}x over target')}")
                print(f"   Large Jumps: {self.large_jumps_count}")
                print(f"   CSV Data: {self.csv_log_filename}")
            
            # Stop robot movement
            self.stop_robot()
            
            # Center head
            self.center_head_for_idle_mode()
            
            # Stop head tracker
            if self.head_tracker:
                self.head_tracker.stop_tracking()
                self.head_tracker = None
            
            # Stop LiDAR system
            if self.lidar_system:
                print("🛑 Stopping LiDAR system...")
                self.lidar_system.stop()
                self.lidar_system = None
                print("✅ LiDAR system stopped")
            
            # Cleanup exact camera
            self.cleanup_exact_camera()
            
            # Clear pygame events
            if pygame.get_init():
                pygame.event.clear()
            
            # Restore facial animation
            if self.facial_restorer and self.robot:
                self.facial_restorer.restore_resting_face_immediately(self.robot)
            
            # Clean up blackboard
            try:
                if self.blackboard.exists("LIDAR_SYSTEM"):
                    self.blackboard.unset("LIDAR_SYSTEM")
            except Exception:
                pass
            
            # Set robot mode to IDLE
            if self.robot:
                print("🎯 Setting robot mode to IDLE")
                self.robot.set_mode(RobotMode.IDLE)
            
            print("✅ EXACT Standalone LiDAR Test terminated successfully")
            
        except Exception as e:
            print(f"⚠️ Termination error: {e}")
            try:
                if self.robot:
                    self.robot.set_mode(RobotMode.IDLE)
            except Exception:
                pass
        
        super().terminate(new_status)


# Use the exact copy implementation
StandaloneLidarTest = ExactStandaloneLidarTest