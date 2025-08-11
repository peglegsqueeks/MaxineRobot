#!/usr/bin/env python3
"""
LIDAR TEST BEHAVIOR - USING EXACT PIPELINE FROM detection_consistency_test.py
This version uses the EXACT SAME camera pipeline that achieves stable detection
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

# Import DepthAI for DIRECT pipeline (EXACT same as detection_consistency_test.py)
try:
    import depthai as dai
    import cv2
    import numpy as np
    DEPTHAI_AVAILABLE = True
except ImportError as e:
    print(f"❌ DepthAI not available: {e}")
    DEPTHAI_AVAILABLE = False


class UltraStableLidarSystem:
    """Ultra stable LiDAR system with strong filtering"""
    
    def __init__(self):
        self.lidar = None
        self.scanning = False
        self.data_queue = queue.Queue(maxsize=10)
        self.worker_thread = None
        self.latest_scan = None
        self.scan_lock = threading.Lock()
        
        # Ultra stable filtering
        self.smoothing_alpha = 0.15
        self.previous_scan = {}
        self.confidence_threshold = 200
        self.outlier_distance = 100
        self.historical_scans = deque(maxlen=5)
        
    def start(self):
        """Start LiDAR scanning"""
        try:
            self.lidar = PyRPlidar()
            self.lidar.connect(port="/dev/ttyUSB0", baudrate=256000, timeout=3)
            info = self.lidar.get_info()
            health = self.lidar.get_health()
            samplerate = self.lidar.get_samplerate()
            
            self.lidar.set_motor_pwm(550)
            time.sleep(2)
            
            self.scanning = True
            self.worker_thread = threading.Thread(target=self._scan_worker, daemon=True)
            self.worker_thread.start()
            
            return True
        except Exception as e:
            print(f"Failed to start LiDAR: {e}")
            return False
    
    def _scan_worker(self):
        """Worker thread for continuous scanning"""
        # FIX: start_scan_express() only takes scan_type parameter
        scan_generator = self.lidar.start_scan_express(0)  # FIXED: Removed 'stability' argument
        
        while self.scanning:
            try:
                scan_data = {}
                for count, scan in enumerate(scan_generator()):
                    angle = scan.angle
                    distance = scan.distance
                    quality = scan.quality
                    
                    if 200 <= distance <= 5000 and quality >= self.confidence_threshold:
                        scan_data[angle] = distance
                    
                    if count >= 1500:
                        break
                
                if scan_data:
                    processed_scan = self._apply_ultra_filtering(scan_data)
                    with self.scan_lock:
                        self.latest_scan = processed_scan
                        if not self.data_queue.full():
                            self.data_queue.put(processed_scan)
                    
            except Exception:
                continue
    
    def _apply_ultra_filtering(self, scan_data):
        """Apply ultra strong temporal and spatial filtering"""
        filtered = {}
        
        for angle, distance in scan_data.items():
            if angle in self.previous_scan:
                prev_distance = self.previous_scan[angle]
                if abs(distance - prev_distance) > self.outlier_distance:
                    distance = prev_distance + (distance - prev_distance) * 0.1
                
                smoothed = prev_distance * (1 - self.smoothing_alpha) + distance * self.smoothing_alpha
                filtered[angle] = smoothed
            else:
                filtered[angle] = distance
        
        self.previous_scan = filtered.copy()
        self.historical_scans.append(filtered)
        
        return filtered
    
    def get_latest_scan(self):
        """Get the latest filtered scan data"""
        with self.scan_lock:
            return self.latest_scan.copy() if self.latest_scan else {}
    
    def get_display_obstacles(self):
        """Get obstacles formatted for display"""
        scan = self.get_latest_scan()
        obstacles = []
        for angle, distance in scan.items():
            obstacles.append((angle, distance))
        return obstacles
    
    def stop(self):
        """Stop LiDAR scanning"""
        self.scanning = False
        if self.worker_thread:
            self.worker_thread.join(timeout=2)
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.set_motor_pwm(0)
                self.lidar.disconnect()
            except:
                pass


class LidarTestBehavior(MaxineBehavior):
    """
    LiDAR Test using EXACT SAME PIPELINE as detection_consistency_test.py
    """
    
    def __init__(self):
        super().__init__("LiDAR Test Mode - Direct Pipeline")
        
        # py_trees Lifecycle: __init__ - Just setup variables, NO hardware initialization
        
        # Blackboard keys
        self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("HEAD_CENTER_POSITION", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.WRITE)
        
        # Component references
        self.lidar_system = None
        self.screen = None
        self.robot = None
        
        # EXACT COPY FROM detection_consistency_test.py - Camera system
        self.device = None
        self.pipeline = None
        self.detection_queue = None
        self.preview_queue = None
        self.has_detection = False
        self.camera_initialized = False
        self.camera_error_message = ""
        
        # EXACT COPY - Camera specs
        self.camera_resolution_width = 300
        self.camera_resolution_height = 300
        self.camera_hfov_degrees = 114
        self.target_fps = 25
        
        # EXACT COPY - Performance settings
        self.detection_skip_frames = 1
        self.frame_counter = 0
        
        # EXACT COPY - Camera display settings
        self.camera_display_size = (600, 450)
        self.camera_surface = None
        self.show_camera_debug = True
        
        # Display parameters
        self.center_x = 0
        self.center_y = 0
        self.scale = 0
        self.update_counter = 0
        self.display_update_rate = 3
        
        # GRID OVERLAY - PRESERVED FEATURE
        self.show_grid = True
        self.grid_size = 80
        self.last_potential_grid = {}
        
        # Person tracking parameters
        self.last_person_detected = 0
        self.person_lost_timeout = 2.0
        
        # Head tracking parameters
        self.angle_history = []
        self.max_angle_history = 5
        self.last_sent_angle = None
        self.angle_change_threshold = math.radians(8)
        self.update_counter_tracking = 0
        self.tracking_update_interval = 6
        
        # CSV LOGGING AND VARIANCE TRACKING
        self.csv_log_filename = "LIDARTEST.csv"
        self.csv_initialized = False
        self.mode_start_time = 0
        
        # X-midpoint tracking for consistency analysis
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
        
        # Multi-term variance tracking
        self.short_term_variance = deque(maxlen=75)   # 3-second variance at 25fps
        self.medium_term_variance = deque(maxlen=250) # 10-second variance
        self.long_term_variance = deque(maxlen=1500)  # 60-second variance
        
        # Stability classification
        self.stability_zones = {
            'stable': 0,      # <10px variance
            'moderate': 0,    # 10-25px variance
            'unstable': 0,    # 25-50px variance  
            'very_unstable': 0  # >50px variance
        }
        
        # EXACT COPY - Z-depth tracking
        self.z_depth_smoother = deque(maxlen=5)
        self.confidence_weights = deque(maxlen=5)
        self.smoothed_z_depth = 0
        self.depth_trust_threshold = 0.6
        
        # Console output control
        self.last_console_output_time = 0
        self.console_output_interval = 5.0
        self.last_variance_print_time = 0
        self.variance_print_interval = 1.0
        
        # Position smoothing for stability
        self.x_position_smoother = deque(maxlen=5)
        self.smoothed_x_position = None
        self.outlier_rejection_threshold = 500  # Reject jumps > 500 pixels
        
        if not pygame.get_init():
            pygame.init()
        pygame.font.init()
    
    def setup(self, **kwargs):
        """py_trees: setup() - Prepare for potential execution, NO hardware initialization"""
        return True
    
    def initialise(self):
        """py_trees: initialise() - Called when behavior STARTS running (when mode changes)"""
        print("\n" + "="*60)
        print("🎯 LIDAR TEST MODE - EXACT PIPELINE FROM detection_consistency_test.py")
        print("="*60)
        
        # Reset mode start time
        self.mode_start_time = time.time()
        
        # Get robot reference
        self.robot = self.get_robot()
        
        # Stop robot movement
        self.stop_robot()
        
        # Initialize display
        self.initialize_display()
        
        # Initialize LiDAR
        self.initialize_lidar()
        
        # CRITICAL: Initialize camera using EXACT pipeline from detection_consistency_test.py
        if DEPTHAI_AVAILABLE:
            self.initialize_camera_direct()
        else:
            print("⚠️ DepthAI not available - running without camera")
        
        # Initialize CSV logging
        self.initialize_csv_log()
        
        # Reset tracking data
        self.reset_tracking_data()
        
        # Reset console output timers
        self.last_console_output_time = time.time()
        self.last_variance_print_time = time.time()
        
        print("✅ LiDAR Test Mode initialized with direct camera pipeline")
        print("📊 Logging variance data to: LIDARTEST.csv")
        print("📍 Grid overlay: ON | LiDAR obstacles: ON | Person circle: ON")
        print("-"*60 + "\n")
    
    def check_camera_connection(self):
        """EXACT COPY from detection_consistency_test.py"""
        try:
            devices = dai.Device.getAllAvailableDevices()
            return len(devices) > 0, f"Found {len(devices)} device(s)"
        except Exception as e:
            return False, f"Device detection error: {str(e)}"
    
    def create_pipeline(self):
        """EXACT COPY from detection_consistency_test.py - Create pipeline with PRESERVED settings"""
        try:
            pipeline = dai.Pipeline()
            
            local_blob_path = "./mobilenet-ssd_openvino_2021.4_5shave.blob"
            if not os.path.exists(local_blob_path):
                raise Exception(f"Blob file not found: {local_blob_path}")
            
            # PRESERVED: Exact working camera setup
            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            mono_left.setFps(self.target_fps)
            
            mono_right = pipeline.create(dai.node.MonoCamera)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            mono_right.setFps(self.target_fps)
            
            # PRESERVED: Exact working ImageManip setup
            manip_nn = pipeline.create(dai.node.ImageManip)
            manip_nn.initialConfig.setResize(300, 300)
            manip_nn.initialConfig.setKeepAspectRatio(False)
            manip_nn.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
            mono_right.out.link(manip_nn.inputImage)
            
            manip_display = pipeline.create(dai.node.ImageManip)
            manip_display.initialConfig.setResize(self.camera_display_size[0], self.camera_display_size[1])
            manip_display.initialConfig.setKeepAspectRatio(True)
            manip_display.initialConfig.setFrameType(dai.ImgFrame.Type.GRAY8)
            mono_right.out.link(manip_display.inputImage)
            
            # PRESERVED: Exact working stereo depth settings
            depth = pipeline.create(dai.node.StereoDepth)
            depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
            depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
            depth.initialConfig.setConfidenceThreshold(180)
            depth.setLeftRightCheck(True)
            depth.setSubpixel(False)
            depth.setDepthAlign(dai.CameraBoardSocket.CAM_C)
            
            mono_left.out.link(depth.left)
            mono_right.out.link(depth.right)
            
            # PRESERVED: Exact working detection network settings
            detection_nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
            detection_nn.setConfidenceThreshold(0.4)  # PRESERVED
            detection_nn.setBlobPath(local_blob_path)
            detection_nn.setBoundingBoxScaleFactor(0.5)  # PRESERVED
            detection_nn.setDepthLowerThreshold(100)    # PRESERVED
            detection_nn.setDepthUpperThreshold(8000)   # PRESERVED
            
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
        except Exception:
            return None
    
    def initialize_camera_direct(self):
        """EXACT COPY from detection_consistency_test.py - Initialize camera"""
        try:
            connected, message = self.check_camera_connection()
            if not connected:
                self.camera_error_message = message
                print(f"⚠️ Camera not connected: {message}")
                return False
            
            self.pipeline = self.create_pipeline()
            if not self.pipeline:
                self.camera_error_message = "Pipeline creation failed"
                print("⚠️ Pipeline creation failed")
                return False
            
            try:
                self.device = dai.Device(self.pipeline)
                print("✅ Camera device connected")
            except Exception as e:
                self.camera_error_message = f"Device connection failed: {str(e)}"
                print(f"⚠️ Device connection failed: {str(e)}")
                return False
            
            # Optimize device
            try:
                if hasattr(self.device, 'setLogLevel'):
                    self.device.setLogLevel(dai.LogLevel.WARN)
                self.device.setIrLaserDotProjectorIntensity(900)
            except Exception:
                pass
            
            # Get queues
            try:
                self.detection_queue = self.device.getOutputQueue("detections", maxSize=4, blocking=False)
                self.has_detection = True
                print("✅ Detection queue initialized")
            except Exception:
                self.detection_queue = None
                self.has_detection = False
                print("⚠️ Detection queue failed")
            
            try:
                self.preview_queue = self.device.getOutputQueue("preview", maxSize=4, blocking=False)
                print("✅ Preview queue initialized")
            except Exception:
                self.preview_queue = None
                print("⚠️ Preview queue failed")
            
            # Test camera frames
            if self.preview_queue:
                for i in range(20):
                    test_frame = self.preview_queue.tryGet()
                    if test_frame:
                        self.camera_initialized = True
                        break
                    time.sleep(0.1)
            
            time.sleep(2)
            
            if self.camera_initialized:
                print("✅ Camera system initialized successfully")
            else:
                print("⚠️ Camera initialization incomplete")
            
            return True
        except Exception as e:
            self.camera_error_message = f"Camera initialization error: {str(e)}"
            print(f"⚠️ Camera initialization error: {str(e)}")
            return False
    
    def smooth_z_depth(self, raw_z_depth, confidence):
        """EXACT COPY from detection_consistency_test.py - Apply confidence-weighted temporal smoothing"""
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
    
    def process_detections_direct(self):
        """EXACT COPY from detection_consistency_test.py - Process detections with X MIDPOINT CONSISTENCY ANALYSIS"""
        if not (self.has_detection and self.detection_queue and self.camera_initialized):
            return None
        
        start_time = time.time()
        self.frame_counter += 1
        
        if self.frame_counter % self.detection_skip_frames != 0:
            return None
        
        try:
            detections = self.detection_queue.tryGet()
            if not detections:
                return None
            
            person_detections = [det for det in detections.detections if det.label == 15]
            if not person_detections:
                return None
            
            # DEBUG: Print number of detections
            if len(person_detections) > 1 and self.frame_counter % 30 == 0:
                print(f"⚠️ Multiple people detected: {len(person_detections)}")
                for i, p in enumerate(person_detections):
                    x_mid = (p.xmin + p.xmax) / 2.0
                    x_pixels = int(x_mid * 1920)
                    print(f"  Person {i}: X={x_pixels}px, Z={p.spatialCoordinates.z:.0f}mm, Conf={p.confidence:.2f}")
            
            # Get closest person - FIXED: When multiple people, choose closest to center
            if len(person_detections) > 1:
                # Multiple people - choose the one closest to screen center (960px)
                screen_center = 960
                def distance_from_center(p):
                    x_mid = (p.xmin + p.xmax) / 2.0
                    x_pixels = x_mid * screen_width
                    return abs(x_pixels - screen_center)
                
                # Sort by distance from center
                person_detections_by_center = sorted(person_detections, key=distance_from_center)
                
                # Pick closest to center with reasonable confidence
                closest_person = None
                for person in person_detections_by_center:
                    if person.confidence >= 0.5:
                        closest_person = person
                        x_mid = (person.xmin + person.xmax) / 2.0
                        x_pixels = int(x_mid * screen_width)
                        if self.frame_counter % 30 == 0:
                            print(f"✅ Selected person at {x_pixels}px (closest to center)")
                        break
                
                if not closest_person:
                    # Fallback to highest confidence
                    closest_person = max(person_detections, key=lambda p: p.confidence)
            else:
                # Single person - use them
                closest_person = person_detections[0]
            
            x_camera = closest_person.spatialCoordinates.x
            y_camera = closest_person.spatialCoordinates.y
            raw_z_depth = closest_person.spatialCoordinates.z
            confidence = closest_person.confidence
            
            if raw_z_depth <= 0 or raw_z_depth > 15000:
                return None
            
            smoothed_z_depth = self.smooth_z_depth(raw_z_depth, confidence)
            
            # CONSISTENCY ANALYSIS - Calculate X midpoint
            bbox_xmin = closest_person.xmin
            bbox_xmax = closest_person.xmax
            bbox_ymin = closest_person.ymin
            bbox_ymax = closest_person.ymax
            
            # X midpoint calculations
            x_midpoint_normalized = (bbox_xmin + bbox_xmax) / 2.0
            
            # Use actual screen width, not hardcoded 1920
            screen_width = self.screen.get_width() if self.screen else 1920
            screen_height = self.screen.get_height() if self.screen else 1080
            
            # Sanity check on screen dimensions
            if screen_width != 1920 and self.frame_counter % 100 == 0:
                print(f"ℹ️ Screen width is {screen_width}px, not 1920px")
            
            x_midpoint_pixels_raw = int(x_midpoint_normalized * screen_width)
            
            # OUTLIER REJECTION: Reject extreme jumps when sitting still
            x_midpoint_pixels = x_midpoint_pixels_raw
            if self.last_x_midpoint is not None:
                jump_distance = abs(x_midpoint_pixels_raw - self.last_x_midpoint)
                
                # If jump is extreme (>500 pixels) and we have history, reject it
                if jump_distance > 500 and len(self.x_midpoints_pixels) > 10:
                    # Use median of recent values instead
                    recent_values = list(self.x_midpoints_pixels)[-10:]
                    median_x = sorted(recent_values)[len(recent_values)//2]
                    
                    # Only accept the jump if confidence is very high
                    if confidence < 0.7:
                        x_midpoint_pixels = median_x
                        if self.frame_counter % 30 == 0:
                            print(f"🛡️ Rejected outlier: {x_midpoint_pixels_raw}px → Using median: {median_x}px")
                    else:
                        # High confidence, might be real movement
                        x_midpoint_pixels = x_midpoint_pixels_raw
                        if self.frame_counter % 30 == 0:
                            print(f"⚠️ Large jump accepted (high conf): {self.last_x_midpoint}px → {x_midpoint_pixels_raw}px")
            
            # Populate TARGET_PERSON on blackboard for other behaviors
            self.blackboard.set("TARGET_PERSON", closest_person)
            
            return {
                'x_camera': x_camera,
                'y_camera': y_camera,
                'z_camera': smoothed_z_depth,
                'raw_z_depth': raw_z_depth,
                'confidence': confidence,
                'bbox_center': {
                    'x_normalized': x_midpoint_pixels / screen_width,  # Recalculate based on filtered value
                    'y_normalized': (bbox_ymin + bbox_ymax) / 2.0,
                    'x_pixels': x_midpoint_pixels,  # Filtered value
                    'x_pixels_raw': x_midpoint_pixels_raw,  # Original value for debugging
                    'y_pixels': int(((bbox_ymin + bbox_ymax) / 2.0) * (self.screen.get_height() if self.screen else 1080))
                },
                'bounding_box': {
                    'xmin': bbox_xmin,
                    'xmax': bbox_xmax,
                    'ymin': bbox_ymin,
                    'ymax': bbox_ymax
                },
                'detection_method': 'direct_pipeline'
            }
        except Exception:
            return None
    
    def update(self):
        """py_trees: update() - Called during behavior execution"""
        self.update_counter += 1
        
        # Check for exit
        keys = pygame.key.get_pressed()
        if keys[pygame.K_ESCAPE]:
            return Status.SUCCESS
        
        # Toggle grid with 'G' key
        if keys[pygame.K_g] and self.update_counter % 30 == 0:  # Debounce
            self.show_grid = not self.show_grid
            print(f"Grid overlay: {'ON' if self.show_grid else 'OFF'}")
        
        # Get person detection using DIRECT pipeline (EXACT same as detection_consistency_test.py)
        person_data = self.process_detections_direct()
        
        if person_data:
            self.last_person_detected = time.time()
            
            # Log variance data to CSV and console
            self.log_detection_consistency_to_csv(person_data)
            
            # Track head angle
            self.track_person(person_data)
        else:
            # Check timeout for centering head
            if time.time() - self.last_person_detected > self.person_lost_timeout:
                self.center_head()
        
        # Update display with ALL features
        self.update_display_full_features(person_data)
        
        return Status.RUNNING
    
    def reset_tracking_data(self):
        """Reset all tracking data for fresh start"""
        self.x_midpoints_pixels.clear()
        self.x_midpoints_normalized.clear()
        self.variance_window.clear()
        self.short_term_variance.clear()
        self.medium_term_variance.clear()
        self.long_term_variance.clear()
        
        self.detection_count = 0
        self.consistent_detection_count = 0
        self.large_jumps_count = 0
        self.last_x_midpoint = None
        
        self.stability_zones = {
            'stable': 0,
            'moderate': 0,
            'unstable': 0,
            'very_unstable': 0
        }
        
        self.update_counter = 0
        self.frame_counter = 0
        
        # Reset position smoother
        self.x_position_smoother.clear()
        self.smoothed_x_position = None
    
    def initialize_csv_log(self):
        """Initialize CSV log for variance analysis - ALWAYS OVERWRITE"""
        try:
            # Always create new file, overwriting any existing
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
            print(f"✅ CSV log created: {self.csv_log_filename}")
        except Exception as e:
            print(f"❌ CSV initialization failed: {e}")
            self.csv_initialized = False
    
    def calculate_x_midpoint_variance(self):
        """Calculate real-time X midpoint variance"""
        try:
            if len(self.variance_window) < 2:
                return 0.0, 0.0, 0.0
            
            window_data = list(self.variance_window)
            mean_val = statistics.mean(window_data)
            variance_val = statistics.variance(window_data) if len(window_data) > 1 else 0.0
            std_dev_val = math.sqrt(variance_val)
            
            return variance_val, std_dev_val, mean_val
        except Exception:
            return 0.0, 0.0, 0.0
    
    def calculate_multi_term_variance(self, x_pixel):
        """Calculate short, medium, and long-term variance"""
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
        """Classify detection stability"""
        if std_dev_pixels < 10:
            return 'stable'
        elif std_dev_pixels < 25:
            return 'moderate'
        elif std_dev_pixels < 50:
            return 'unstable'
        else:
            return 'very_unstable'
    
    def log_detection_consistency_to_csv(self, person_data):
        """Log detection consistency data to CSV"""
        if not self.csv_initialized:
            self.initialize_csv_log()
            if not self.csv_initialized:
                return
        
        try:
            # Extract data from person_data
            bbox_center = person_data.get('bbox_center', {})
            x_midpoint_pixels = bbox_center.get('x_pixels', 0)
            x_midpoint_normalized = bbox_center.get('x_normalized', 0.5)
            
            # Track midpoints for variance analysis
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
            
            # Get current state info
            head_angle_deg = math.degrees(self.last_sent_angle) if self.last_sent_angle else 0.0
            head_tracking_active = 1
            person_is_settled = 1 if self.current_std_dev_pixels < 25 else 0
            
            # Calculate mode elapsed time
            mode_elapsed = time.time() - self.mode_start_time
            
            # Write to CSV file
            with open(self.csv_log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    mode_elapsed,
                    time.time(),
                    self.frame_counter,
                    x_midpoint_pixels,
                    x_midpoint_normalized,
                    person_data.get('x_camera', 0),
                    person_data.get('z_camera', 0),
                    person_data.get('raw_z_depth', person_data.get('z_camera', 0)),
                    person_data.get('confidence', 0),
                    person_data.get('bounding_box', {}).get('xmin', 0),
                    person_data.get('bounding_box', {}).get('ymin', 0),
                    person_data.get('bounding_box', {}).get('xmax', 0),
                    person_data.get('bounding_box', {}).get('ymax', 0),
                    person_data.get('bounding_box', {}).get('xmax', 0) - person_data.get('bounding_box', {}).get('xmin', 0),
                    person_data.get('bounding_box', {}).get('ymax', 0) - person_data.get('bounding_box', {}).get('ymin', 0),
                    x_jump,
                    1 if is_large_jump else 0,
                    self.current_variance_pixels,
                    self.current_std_dev_pixels,
                    self.current_mean_pixels,
                    short_var,
                    medium_var,
                    long_var,
                    self.detection_count,
                    self.consistent_detection_count,
                    self.large_jumps_count,
                    stability_class,
                    head_angle_deg,
                    head_tracking_active,
                    person_is_settled,
                    person_data.get('detection_method', 'direct_pipeline')
                ])
                csvfile.flush()  # Force write to disk
            
            # Console output - Real-time variance (every 1 second)
            current_time = time.time()
            if current_time - self.last_variance_print_time >= self.variance_print_interval:
                # Color code the output
                if self.current_std_dev_pixels < 10:
                    status = "✅ STABLE"
                elif self.current_std_dev_pixels < 25:
                    status = "⚠️ MODERATE"
                elif self.current_std_dev_pixels < 50:
                    status = "⚠️ UNSTABLE"
                else:
                    status = "❌ VERY UNSTABLE"
                
                # Show raw vs filtered if different
                raw_x = bbox_center.get('x_pixels_raw', x_midpoint_pixels)
                if raw_x != x_midpoint_pixels:
                    print(f"📍 X:{x_midpoint_pixels:4d}px (raw:{raw_x:4d}px) | Variance: ±{self.current_std_dev_pixels:5.1f}px | {status} | Z:{person_data.get('z_camera', 0):.0f}mm")
                else:
                    print(f"📍 X:{x_midpoint_pixels:4d}px | Variance: ±{self.current_std_dev_pixels:5.1f}px | {status} | Z:{person_data.get('z_camera', 0):.0f}mm")
                self.last_variance_print_time = current_time
            
            # Console output - Periodic summary (every 5 seconds)
            if current_time - self.last_console_output_time >= self.console_output_interval:
                self.print_variance_summary()
                self.last_console_output_time = current_time
                
        except Exception as e:
            if self.update_counter % 100 == 0:  # Only print errors occasionally
                print(f"⚠️ CSV logging error: {e}")
    
    def print_variance_summary(self):
        """Print periodic variance summary to console"""
        print("\n" + "-"*60)
        print("📊 VARIANCE SUMMARY")
        print("-"*60)
        
        # Calculate consistency rate
        if self.detection_count > 0:
            consistency_rate = (self.consistent_detection_count / self.detection_count) * 100
        else:
            consistency_rate = 0
        
        # Calculate stability percentages
        total_classifications = sum(self.stability_zones.values())
        if total_classifications > 0:
            stable_pct = (self.stability_zones['stable'] / total_classifications) * 100
            moderate_pct = (self.stability_zones['moderate'] / total_classifications) * 100
            unstable_pct = (self.stability_zones['unstable'] / total_classifications) * 100
            very_unstable_pct = (self.stability_zones['very_unstable'] / total_classifications) * 100
        else:
            stable_pct = moderate_pct = unstable_pct = very_unstable_pct = 0
        
        print(f"Total Detections: {self.detection_count}")
        print(f"Consistency Rate: {consistency_rate:.1f}%")
        print(f"Large Jumps (>{self.jump_threshold_pixels}px): {self.large_jumps_count}")
        print(f"Current Std Dev: ±{self.current_std_dev_pixels:.1f}px")
        print(f"Current Mean X: {self.current_mean_pixels:.0f}px")
        print(f"\nStability Distribution:")
        print(f"  Stable (<10px):      {stable_pct:5.1f}%")
        print(f"  Moderate (10-25px):  {moderate_pct:5.1f}%")
        print(f"  Unstable (25-50px):  {unstable_pct:5.1f}%")
        print(f"  Very Unstable (>50px): {very_unstable_pct:5.1f}%")
        print("-"*60 + "\n")
    
    def initialize_display(self):
        """Initialize pygame display"""
        try:
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
            pygame.display.set_caption("MAXINE LIDAR TEST - DIRECT PIPELINE")
            
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            self.scale = min(display_info.current_w, display_info.current_h) // 7
            
            print(f"✅ Display initialized: {display_info.current_w}x{display_info.current_h}")
            print(f"   Screen center: ({self.center_x}, {self.center_y})")
        except Exception as e:
            print(f"⚠️ Display initialization error: {e}")
    
    def initialize_lidar(self):
        """Initialize LiDAR system"""
        try:
            self.lidar_system = UltraStableLidarSystem()
            success = self.lidar_system.start()
            
            if success:
                self.blackboard.set("LIDAR_SYSTEM", self.lidar_system)
                print("✅ LiDAR system initialized")
            else:
                print("⚠️ LiDAR system failed to initialize")
                self.lidar_system = None
        except Exception as e:
            print(f"⚠️ LiDAR initialization error: {e}")
            self.lidar_system = None
    
    def update_display_full_features(self, person_data):
        """Update display with ALL features preserved"""
        if not self.screen:
            return
        
        # Clear screen
        self.screen.fill((0, 0, 0))
        
        # Draw radar grid - PRESERVED
        self.draw_radar_grid()
        
        # Draw robot at center - PRESERVED
        self.draw_robot()
        
        # Draw LiDAR obstacles - PRESERVED
        obstacle_count = 0
        if self.lidar_system:
            obstacles = self.lidar_system.get_display_obstacles()
            if obstacles:
                obstacle_count = self.draw_lidar_data(obstacles)
        
        # Draw person detection circle - PRESERVED
        if person_data:
            self.draw_person_detection(person_data)
        
        # Draw grid overlay if enabled - PRESERVED
        if self.show_grid:
            self.draw_grid_lines()
            if person_data:
                self.update_potential_field_grid(person_data)
                self.draw_cached_potential_numbers()
        
        # Draw variance info on screen
        self.draw_variance_info()
        
        # Draw info text
        self.draw_info_text(obstacle_count, person_data)
        
        # Update display
        pygame.display.flip()
    
    def draw_radar_grid(self):
        """Draw radar-style grid - PRESERVED FROM ORIGINAL"""
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
        """Draw robot representation - PRESERVED FROM ORIGINAL"""
        pygame.draw.circle(self.screen, (0, 255, 0), (self.center_x, self.center_y), 15, 3)
        arrow_end_x = self.center_x
        arrow_end_y = self.center_y - 30
        pygame.draw.line(self.screen, (0, 255, 0), (self.center_x, self.center_y), (arrow_end_x, arrow_end_y), 5)
    
    def draw_lidar_data(self, obstacles):
        """Draw LiDAR obstacles - PRESERVED FROM ORIGINAL"""
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
        """Draw person detection circle - PRESERVED WITH COLOR CODING"""
        try:
            if not person_data:
                return
            
            x_camera = person_data.get('x_camera', 0)
            z_camera = person_data.get('z_camera', 0)
            
            if z_camera > 0:
                # Convert to display coordinates
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
                
                # Color-code by stability
                stability_class = self.classify_stability(self.current_std_dev_pixels)
                if stability_class == 'stable':
                    color = (0, 255, 0)      # Green
                elif stability_class == 'moderate':
                    color = (255, 255, 0)    # Yellow
                elif stability_class == 'unstable':
                    color = (255, 165, 0)    # Orange
                else:
                    color = (255, 0, 0)      # Red
                
                # Draw detection circle
                if 0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height():
                    pygame.draw.circle(self.screen, color, (x, y), 8)
                    pygame.draw.circle(self.screen, (255, 255, 255), (x, y), 9, 2)
            
        except Exception:
            pass
    
    def draw_grid_lines(self):
        """Draw grid lines overlay - PRESERVED FROM ORIGINAL"""
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
    
    def update_potential_field_grid(self, person_data):
        """Update potential field grid - PRESERVED FROM ORIGINAL"""
        try:
            if person_data:
                x_camera = person_data.get('x_camera', 0)
                z_camera = person_data.get('z_camera', 0)
                
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
    
    def draw_cached_potential_numbers(self):
        """Draw potential field numbers - PRESERVED FROM ORIGINAL"""
        try:
            font = pygame.font.Font(None, 24)
            
            for (grid_col, grid_row), text in self.last_potential_grid.items():
                grid_center_x = grid_col * self.grid_size + self.grid_size // 2
                grid_center_y = grid_row * self.grid_size + self.grid_size // 2
                
                if text.startswith('+'):
                    color = (0, 255, 0)
                else:
                    color = (255, 0, 0)
                
                text_surface = font.render(text, True, color)
                text_rect = text_surface.get_rect(center=(grid_center_x, grid_center_y))
                
                bg_rect = text_rect.inflate(6, 4)
                pygame.draw.rect(self.screen, (0, 0, 0), bg_rect)
                self.screen.blit(text_surface, text_rect)
                
        except Exception:
            pass
    
    def draw_variance_info(self):
        """Draw variance information on screen"""
        try:
            font = pygame.font.Font(None, 36)
            
            # Determine color based on stability
            if self.current_std_dev_pixels < 10:
                color = (0, 255, 0)  # Green - stable
            elif self.current_std_dev_pixels < 25:
                color = (255, 255, 0)  # Yellow - moderate
            elif self.current_std_dev_pixels < 50:
                color = (255, 165, 0)  # Orange - unstable
            else:
                color = (255, 0, 0)  # Red - very unstable
            
            y_offset = 50
            
            variance_text = f"Variance: ±{self.current_std_dev_pixels:.1f}px"
            text_surface = font.render(variance_text, True, color)
            self.screen.blit(text_surface, (50, y_offset))
            
            mean_text = f"Mean X: {self.current_mean_pixels:.0f}px"
            text_surface = font.render(mean_text, True, (255, 255, 255))
            self.screen.blit(text_surface, (50, y_offset + 40))
            
            if self.detection_count > 0:
                consistency_rate = (self.consistent_detection_count / self.detection_count) * 100
                consistency_text = f"Consistency: {consistency_rate:.1f}%"
                text_surface = font.render(consistency_text, True, (255, 255, 255))
                self.screen.blit(text_surface, (50, y_offset + 80))
            
        except Exception:
            pass
    
    def draw_info_text(self, obstacle_count, person_data):
        """Draw info text at bottom"""
        try:
            font = pygame.font.Font(None, 32)
            y_offset = self.screen.get_height() - 150
            
            lidar_status = "ACTIVE" if self.lidar_system else "INACTIVE"
            camera_status = "DIRECT" if self.camera_initialized else "INACTIVE"
            head_angle = math.degrees(self.last_sent_angle) if self.last_sent_angle else 0.0
            
            info_lines = [
                f"MAXINE LIDAR TEST - LiDAR: {lidar_status} | Camera: {camera_status} | Head: {head_angle:.1f}°",
                f"Obstacles: {obstacle_count} | Detections: {self.detection_count} | Jumps: {self.large_jumps_count}",
                f"Grid: {'ON' if self.show_grid else 'OFF'} (G to toggle) | CSV: {self.csv_log_filename}",
                f"ESC: Exit to IDLE mode"
            ]
            
            for i, line in enumerate(info_lines):
                color = (0, 255, 255) if i == 0 else (255, 255, 255)
                text_surface = font.render(line, True, color)
                self.screen.blit(text_surface, (20, y_offset + i * 35))
                
        except Exception:
            pass
    
    def track_person(self, person_data):
        """Track person with head movement"""
        try:
            bbox_center = person_data.get('bbox_center', {})
            x_normalized = bbox_center.get('x_normalized', 0.5)
            
            angle_radians = (x_normalized - 0.5) * math.radians(114)
            
            self.angle_history.append(angle_radians)
            if len(self.angle_history) > self.max_angle_history:
                self.angle_history.pop(0)
            
            avg_angle = sum(self.angle_history) / len(self.angle_history)
            
            self.update_counter_tracking += 1
            if self.update_counter_tracking >= self.tracking_update_interval:
                if self.last_sent_angle is None or abs(avg_angle - self.last_sent_angle) > self.angle_change_threshold:
                    angle_degrees = math.degrees(avg_angle)
                    angle_degrees = max(-90, min(90, angle_degrees))
                    
                    if self.robot:
                        self.robot.set_head_angle(angle_degrees)
                    
                    self.last_sent_angle = avg_angle
                
                self.update_counter_tracking = 0
        except Exception:
            pass
    
    def center_head(self):
        """Center the head when no person detected"""
        try:
            if self.last_sent_angle is not None and abs(self.last_sent_angle) > 0.01:
                if self.robot:
                    self.robot.set_head_angle(0)
                self.last_sent_angle = 0
                self.angle_history.clear()
        except:
            pass
    
    def stop_robot(self):
        """Stop robot movement"""
        try:
            if self.robot:
                velocity_config = VelocityConfig(linear_velocity=0, angular_velocity=0)
                self.robot.move(MovementDirection.STOP, velocity_config)
        except:
            pass
    
    def terminate(self, new_status):
        """py_trees: terminate() - Called when behavior STOPS running"""
        print("\n" + "="*60)
        print("📊 LIDAR TEST MODE - FINAL VARIANCE SUMMARY")
        print("="*60)
        
        # Print final summary
        if self.detection_count > 0:
            consistency_rate = (self.consistent_detection_count / self.detection_count) * 100
            overall_variance = statistics.variance(list(self.x_midpoints_pixels)) if len(self.x_midpoints_pixels) > 1 else 0
            overall_std_dev = math.sqrt(overall_variance)
            
            print(f"Total Test Duration: {time.time() - self.mode_start_time:.1f} seconds")
            print(f"Total Detections: {self.detection_count}")
            print(f"Consistency Rate: {consistency_rate:.1f}%")
            print(f"Overall Std Dev: ±{overall_std_dev:.1f}px")
            print(f"Large Jumps (>{self.jump_threshold_pixels}px): {self.large_jumps_count}")
            print(f"Data saved to: {self.csv_log_filename}")
        else:
            print("No detections recorded during test")
        
        print("="*60 + "\n")
        
        # Cleanup camera
        if self.device:
            try:
                self.device.close()
                print("✅ Camera device closed")
            except:
                pass
        
        # Cleanup LiDAR
        if self.lidar_system:
            self.lidar_system.stop()
        
        # Cleanup robot
        if self.robot:
            self.stop_robot()
            self.robot.set_head_angle(0)
        
        # Cleanup display
        if self.screen:
            pygame.display.quit()