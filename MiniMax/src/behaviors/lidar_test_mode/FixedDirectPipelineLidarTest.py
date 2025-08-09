#!/usr/bin/env python3
"""
FIXED Direct Pipeline LiDAR Test - PROPER py_trees Methodology
- NO hardware initialization in __init__() - follows py_trees lifecycle correctly
- Graceful error handling that doesn't crash the entire robot
- Proper exit to IDLE mode using py_trees structure
- Compatible with robot having NO camera sensor (camera_sensor = None)
- FIXED: Removed pygame.event.clear() that was blocking ESC key detection
"""

import pygame
import math
import time
import py_trees
import csv
import os
import statistics
from collections import deque
from datetime import datetime
from py_trees.common import Status

# CRITICAL: Import depthai DIRECTLY only when needed, not in __init__
try:
    import depthai as dai
    import cv2
    import numpy as np
    CAMERA_AVAILABLE = True
except ImportError as e:
    print(f"❌ Missing required camera libraries: {e}")
    CAMERA_AVAILABLE = False

# Import LiDAR directly
try:
    from pyrplidar import PyRPlidar
    LIDAR_AVAILABLE = True
except ImportError as e:
    print(f"❌ Missing PyRPlidar: {e}")
    LIDAR_AVAILABLE = False

from src.behaviors.MaxineBehavior import MaxineBehavior
from src.types.RobotModes import RobotMode
from src.types.MovementDirection import MovementDirection
from src.action_managers.VelocityManager import VelocityConfig


class SimpleLidarSystem:
    """Simple LiDAR system for obstacle detection"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        self.is_connected = False
        self.scan_generator = None
        self.scan_iterator = None
        self.latest_obstacles = []
        
    def start(self):
        """Start LiDAR system"""
        if not LIDAR_AVAILABLE:
            print("⚠️ LiDAR not available - running without LiDAR")
            return True
            
        try:
            print("🚀 Starting simple LiDAR system...")
            self.lidar = PyRPlidar()
            self.lidar.connect(port=self.port, baudrate=self.baudrate, timeout=2.0)
            self.is_connected = True
            
            # Start scanning
            self.lidar.stop()
            time.sleep(1.0)
            self.lidar.set_motor_pwm(600)
            time.sleep(3.0)
            
            try:
                self.scan_generator = self.lidar.start_scan_express(4)
                self.scan_iterator = self.scan_generator()
                print("✅ Simple LiDAR system started")
                return True
            except Exception:
                try:
                    self.scan_generator = self.lidar.force_scan()
                    self.scan_iterator = self.scan_generator()
                    print("✅ Simple LiDAR system started (force mode)")
                    return True
                except Exception:
                    print("❌ Failed to start LiDAR scanning")
                    return False
                    
        except Exception as e:
            print(f"❌ LiDAR connection failed: {e}")
            self.is_connected = False
            return False
    
    def stop(self):
        """Stop LiDAR system"""
        try:
            if self.lidar and self.is_connected:
                self.lidar.stop()
                time.sleep(0.5)
                self.lidar.set_motor_pwm(0)
                time.sleep(0.5)
                self.lidar.disconnect()
                self.is_connected = False
                print("✅ Simple LiDAR system stopped")
        except Exception:
            pass
    
    def get_display_obstacles(self):
        """Get obstacles for display"""
        if not self.is_connected or not self.scan_iterator:
            return []
        
        obstacles = []
        try:
            # Get a few measurements
            for _ in range(50):  # Limit to prevent blocking
                try:
                    measurement = next(self.scan_iterator)
                    if measurement:
                        quality = getattr(measurement, 'quality', 0)
                        angle = getattr(measurement, 'angle', 0)
                        distance = getattr(measurement, 'distance', 0)
                        
                        if quality > 8 and 200 < distance < 6000:
                            obstacles.append({
                                'angle': angle,
                                'distance': distance,
                                'quality': quality
                            })
                except StopIteration:
                    break
                except Exception:
                    continue
                    
        except Exception:
            pass
            
        self.latest_obstacles = obstacles
        return obstacles


class FixedDirectPipelineLidarTest(MaxineBehavior):
    """
    FIXED Direct Pipeline LiDAR Test - PROPER py_trees methodology
    NO hardware initialization in __init__() - follows py_trees lifecycle correctly
    FIXED: Does NOT call pygame.event.clear() that was blocking ESC detection
    """
    
    def __init__(self):
        super().__init__("FIXED Direct Pipeline LiDAR Test")
        
        # py_trees Lifecycle: __init__ - ONLY setup variables, NO hardware initialization
        
        # Blackboard setup
        self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.READ)
        self.blackboard.register_key("HEAD_CENTER_POSITION", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.WRITE)
        
        # Component references - NO INITIALIZATION HERE
        self.lidar_system = None
        self.screen = None
        self.robot = None
        self.initialization_successful = False
        
        # DIRECT CAMERA SYSTEM - Variables only, no objects created
        self.device = None
        self.pipeline = None
        self.detection_queue = None
        self.preview_queue = None
        self.has_detection = False
        self.camera_initialized = False
        self.camera_error_message = ""
        
        # Camera settings - Just constants
        self.target_fps = 25
        self.confidence_threshold = 0.4
        self.camera_display_size = (600, 450)
        self.camera_surface = None
        
        # Detection settings - Variables only
        self.detection_skip_frames = 1
        self.frame_counter = 0
        
        # Display parameters - Variables only
        self.center_x = 0
        self.center_y = 0
        self.scale = 0
        self.update_counter = 0
        self.display_update_rate = 3
        
        # CSV LOGGING - Variables only  
        self.csv_log_filename = "LIDARTEST.csv"
        self.csv_initialized = False
        self.mode_start_time = 0
        
        # X-midpoint consistency tracking - Variables only
        self.x_midpoints_pixels = deque(maxlen=1000)
        self.x_midpoints_normalized = deque(maxlen=1000)
        self.variance_window = deque(maxlen=100)
        
        # Variance calculation variables
        self.current_variance_pixels = 0.0
        self.current_std_dev_pixels = 0.0
        self.current_mean_pixels = 0.0
        
        # Consistency metrics - Variables only
        self.detection_count = 0
        self.consistent_detection_count = 0
        self.large_jumps_count = 0
        self.jump_threshold_pixels = 50
        self.last_x_midpoint = None
        
        # Multi-term variance tracking - Variables only
        self.short_term_variance = deque(maxlen=75)
        self.medium_term_variance = deque(maxlen=250)
        self.long_term_variance = deque(maxlen=1500)
        
        # Stability classification - Variables only
        self.stability_zones = {
            'stable': 0, 'moderate': 0, 'unstable': 0, 'very_unstable': 0
        }
        
        # Z-depth smoothing variables
        self.z_depth_smoother = deque(maxlen=5)
        self.confidence_weights = deque(maxlen=5)
        self.smoothed_z_depth = 0
        self.depth_trust_threshold = 0.6
        
        # Person tracking
        self.last_person_detected = 0
        self.person_lost_timeout = 2.0
        
        # CRITICAL: NO pygame initialization here - moved to initialise()
        # CRITICAL: NO hardware initialization here - follows py_trees methodology
    
    def setup(self, **kwargs):
        """
        py_trees Lifecycle: setup() - Prepare for potential execution, NO hardware initialization
        """
        # Just return success - no hardware initialization allowed here
        return True
    
    def initialise(self):
        """
        py_trees Lifecycle: initialise() - Called when behavior STARTS running
        THIS is where ALL hardware initialization happens
        """
        print("🎯 FIXED Direct Pipeline LiDAR Test initializing...")
        
        # Get robot reference
        self.robot = self.get_robot()
        
        # Stop robot movement first
        self.stop_robot()
        
        # PROPER py_trees: Initialize pygame HERE, not in __init__()
        if not pygame.get_init():
            pygame.init()
        pygame.font.init()
        
        # Initialize display
        self.initialize_display()
        
        # Initialize LiDAR system
        self.initialize_lidar()
        
        # CRITICAL: Initialize camera ONLY if available, with graceful failure
        if CAMERA_AVAILABLE:
            self.initialize_direct_camera_pipeline()
        else:
            print("⚠️ Camera not available - running LiDAR-only mode")
            self.camera_initialized = False
        
        # Initialize CSV logging
        self.mode_start_time = time.time()
        self.initialize_csv_log()
        
        # Reset tracking data
        self.reset_tracking_data()
        
        # Mark initialization as successful
        self.initialization_successful = True
        
        print("✅ FIXED Direct Pipeline LiDAR Test initialized successfully")
    
    def initialize_display(self):
        """Initialize pygame display with error handling"""
        try:
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
            pygame.display.set_caption("MAXINE FIXED DIRECT PIPELINE LIDAR TEST")
            
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            self.scale = min(display_info.current_w, display_info.current_h) // 7
            
            # Draw initial interface
            self.draw_clean_interface()
            pygame.display.flip()
            
            print("✅ Display initialized")
        except Exception as e:
            print(f"⚠️ Display initialization error: {e}")
            # Continue without display - don't fail the behavior
    
    def initialize_lidar(self):
        """Initialize LiDAR system with graceful failure handling"""
        try:
            self.lidar_system = SimpleLidarSystem()
            success = self.lidar_system.start()
            
            if success:
                # Store in blackboard for compatibility
                self.blackboard.set("LIDAR_SYSTEM", self.lidar_system)
                print("✅ LiDAR system initialized")
            else:
                print("⚠️ LiDAR system failed to initialize - continuing without LiDAR")
                self.lidar_system = None
        except Exception as e:
            print(f"⚠️ LiDAR initialization error: {e} - continuing without LiDAR")
            self.lidar_system = None
    
    def check_camera_connection(self):
        """Check camera connection"""
        if not CAMERA_AVAILABLE:
            return False, "DepthAI libraries not available"
            
        try:
            devices = dai.Device.getAllAvailableDevices()
            return len(devices) > 0, f"Found {len(devices)} device(s)"
        except Exception as e:
            return False, f"Device detection error: {str(e)}"
    
    def create_pipeline(self):
        """Create camera pipeline"""
        if not CAMERA_AVAILABLE:
            return None
            
        try:
            pipeline = dai.Pipeline()
            
            local_blob_path = "./mobilenet-ssd_openvino_2021.4_5shave.blob"
            if not os.path.exists(local_blob_path):
                print(f"⚠️ Blob file not found: {local_blob_path}")
                return None
            
            # Camera setup
            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            mono_left.setFps(self.target_fps)
            
            mono_right = pipeline.create(dai.node.MonoCamera)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            mono_right.setFps(self.target_fps)
            
            # ImageManip setup
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
            
            # Stereo depth
            depth = pipeline.create(dai.node.StereoDepth)
            depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
            depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
            depth.initialConfig.setConfidenceThreshold(180)
            depth.setLeftRightCheck(True)
            depth.setSubpixel(False)
            depth.setDepthAlign(dai.CameraBoardSocket.CAM_C)
            
            mono_left.out.link(depth.left)
            mono_right.out.link(depth.right)
            
            # Detection network
            detection_nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
            detection_nn.setConfidenceThreshold(self.confidence_threshold)
            detection_nn.setBlobPath(local_blob_path)
            detection_nn.setBoundingBoxScaleFactor(0.5)
            detection_nn.setDepthLowerThreshold(100)
            detection_nn.setDepthUpperThreshold(8000)
            
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
            print(f"⚠️ Pipeline creation error: {e}")
            return None
    
    def initialize_direct_camera_pipeline(self):
        """Initialize camera with GRACEFUL failure handling"""
        try:
            print("📷 Initializing DIRECT camera pipeline...")
            
            connected, message = self.check_camera_connection()
            if not connected:
                self.camera_error_message = message
                print(f"⚠️ Camera connection failed: {message} - continuing without camera")
                return False
            
            self.pipeline = self.create_pipeline()
            if not self.pipeline:
                self.camera_error_message = "Pipeline creation failed"
                print("⚠️ Pipeline creation failed - continuing without camera")
                return False
            
            try:
                self.device = dai.Device(self.pipeline)
                print("✅ Camera device connected")
            except Exception as e:
                self.camera_error_message = f"Device connection failed: {str(e)}"
                print(f"⚠️ Device connection failed: {str(e)} - continuing without camera")
                return False
            
            # Optimize device
            try:
                if hasattr(self.device, 'setLogLevel'):
                    self.device.setLogLevel(dai.LogLevel.WARN)
                self.device.setIrLaserDotProjectorIntensity(900)
                print("✅ Device optimizations applied")
            except Exception:
                pass
            
            # Get queues
            try:
                self.detection_queue = self.device.getOutputQueue("detections", maxSize=4, blocking=False)
                self.has_detection = True
                print("✅ Detection queue created")
            except Exception:
                self.detection_queue = None
                self.has_detection = False
                print("⚠️ Detection queue creation failed")
            
            try:
                self.preview_queue = self.device.getOutputQueue("preview", maxSize=4, blocking=False)
                print("✅ Preview queue created")
            except Exception:
                self.preview_queue = None
                print("⚠️ Preview queue creation failed")
            
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
                print("✅ DIRECT camera pipeline initialized successfully")
                return True
            else:
                print("⚠️ Camera pipeline test failed - continuing without camera")
                return False
                
        except Exception as e:
            self.camera_error_message = f"Camera initialization error: {str(e)}"
            print(f"⚠️ Camera initialization error: {str(e)} - continuing without camera")
            return False
    
    def process_detections_direct(self):
        """Process detections using DIRECT pipeline"""
        if not (self.has_detection and self.detection_queue and self.camera_initialized):
            return None
        
        try:
            detections = self.detection_queue.tryGet()
            if not detections:
                return None
            
            person_detections = [det for det in detections.detections if det.label == 15]
            if not person_detections:
                return None
            
            closest_person = min(person_detections, key=lambda p: p.spatialCoordinates.z)
            
            x_camera = closest_person.spatialCoordinates.x
            y_camera = closest_person.spatialCoordinates.y
            raw_z_depth = closest_person.spatialCoordinates.z
            confidence = closest_person.confidence
            
            if raw_z_depth <= 0 or raw_z_depth > 15000:
                return None
            
            smoothed_z_depth = self.smooth_z_depth(raw_z_depth, confidence)
            
            # X midpoint calculations
            bbox_xmin = closest_person.xmin
            bbox_xmax = closest_person.xmax
            bbox_ymin = closest_person.ymin
            bbox_ymax = closest_person.ymax
            
            x_midpoint_normalized = (bbox_xmin + bbox_xmax) / 2.0
            screen_width = self.screen.get_width() if self.screen else 1920
            x_midpoint_pixels = int(x_midpoint_normalized * screen_width)
            
            person_data = {
                'x_camera': x_camera,
                'y_camera': y_camera,
                'z_camera': smoothed_z_depth,
                'raw_z_depth': raw_z_depth,
                'confidence': confidence,
                'x_midpoint_pixels': x_midpoint_pixels,
                'x_midpoint_normalized': x_midpoint_normalized,
                'bounding_box': {
                    'xmin': bbox_xmin,
                    'ymin': bbox_ymin,
                    'xmax': bbox_xmax,
                    'ymax': bbox_ymax
                },
                'detection_method': 'DIRECT_PIPELINE'
            }
            
            self.log_detection_consistency_to_csv(person_data)
            
            return person_data
        except Exception:
            return None
    
    def smooth_z_depth(self, raw_z_depth, confidence):
        """Apply confidence-weighted temporal smoothing"""
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
    
    def update(self):
        """
        py_trees Lifecycle: update() - Called during behavior execution
        PROPER error handling to prevent robot shutdown
        FIXED: Does NOT touch pygame events - let exit_behavior handle them
        """
        # CRITICAL: Check if initialization was successful
        if not self.initialization_successful:
            print("⚠️ Initialization failed - exiting to IDLE mode gracefully")
            return Status.SUCCESS  # Return SUCCESS to exit gracefully, not FAILURE
        
        self.update_counter += 1
        
        try:
            # FIXED: Do NOT touch pygame events - let exit_behavior handle ALL events
            # This was the bug preventing proper exit via ESC key!
            # The working modes like keyboard_mode never call pygame.event.clear()
            
            # Update camera frame if available
            if self.camera_initialized:
                self.update_camera_frame()
            
            # Process detections if camera available
            person_data = None
            if self.camera_initialized:
                person_data = self.process_detections_direct()
                if person_data:
                    self.last_person_detected = time.time()
            
            # Handle LiDAR data
            lidar_obstacles = []
            if self.lidar_system:
                obstacles = self.lidar_system.get_display_obstacles()
                if obstacles:
                    lidar_obstacles = obstacles
            
            # Update display
            if self.update_counter % self.display_update_rate == 0:
                try:
                    self.update_display_with_info(person_data, lidar_obstacles)
                except Exception as e:
                    print(f"Display error: {e}")
            
            # PROPER py_trees: Continue running
            return Status.RUNNING
            
        except Exception as e:
            print(f"Update error: {e} - exiting gracefully")
            # CRITICAL: Return SUCCESS to exit gracefully, not FAILURE which crashes robot
            return Status.SUCCESS
    
    def update_camera_frame(self):
        """Update camera display"""
        if not self.preview_queue or not self.camera_initialized:
            return
        
        try:
            frame_data = self.preview_queue.tryGet()
            if frame_data is not None:
                frame = frame_data.getFrame()
                if frame is not None and frame.size > 0:
                    if frame.shape[0] != self.camera_display_size[1] or frame.shape[1] != self.camera_display_size[0]:
                        frame = cv2.resize(frame, self.camera_display_size)
                    
                    if len(frame.shape) == 2:
                        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
                    else:
                        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    
                    rgb_frame_transposed = np.transpose(rgb_frame, (1, 0, 2))
                    self.camera_surface = pygame.surfarray.make_surface(rgb_frame_transposed)
        except Exception:
            pass
    
    def update_display_with_info(self, person_data, lidar_obstacles):
        """Update display with all information"""
        if not self.screen:
            return
        
        # Clear and draw base interface
        self.screen.fill((0, 0, 0))
        self.draw_radar_grid()
        self.draw_robot()
        
        # Draw LiDAR obstacles
        obstacle_count = 0
        if lidar_obstacles:
            obstacle_count = self.draw_lidar_data(lidar_obstacles)
        
        # Draw person detection
        if person_data:
            self.draw_person_detection(person_data)
        
        # Draw camera feed
        self.draw_camera_feed()
        
        # Draw info
        self.draw_info_with_status(obstacle_count, person_data)
        
        pygame.display.flip()
    
    def draw_clean_interface(self):
        """Draw clean radar interface"""
        if not self.screen:
            return
        self.screen.fill((0, 0, 0))
        self.draw_radar_grid()
        self.draw_robot()
    
    def draw_radar_grid(self):
        """Draw radar grid"""
        if not self.screen:
            return
        try:
            # Draw circles
            for radius in [self.scale, self.scale * 2, self.scale * 3, self.scale * 4]:
                pygame.draw.circle(self.screen, (0, 100, 0), (self.center_x, self.center_y), radius, 1)
            
            # Draw lines
            for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
                end_x = self.center_x + int(self.scale * 4 * math.cos(math.radians(angle - 90)))
                end_y = self.center_y + int(self.scale * 4 * math.sin(math.radians(angle - 90)))
                pygame.draw.line(self.screen, (0, 100, 0), (self.center_x, self.center_y), (end_x, end_y), 1)
                
        except Exception:
            pass
    
    def draw_robot(self):
        """Draw robot at center"""
        if not self.screen:
            return
        try:
            robot_size = 10
            pygame.draw.rect(self.screen, (255, 255, 255), 
                           (self.center_x - robot_size//2, self.center_y - robot_size//2, 
                            robot_size, robot_size))
        except Exception:
            pass
    
    def draw_lidar_data(self, obstacles):
        """Draw LiDAR obstacles"""
        if not self.screen:
            return 0
        try:
            obstacle_count = 0
            for obstacle in obstacles:
                if obstacle.get('distance', 0) > 0:
                    angle_rad = math.radians(obstacle['angle'] - 90)
                    distance = min(obstacle['distance'], 4000)  # Cap at 4m for display
                    scale_distance = (distance / 1000.0) * self.scale
                    
                    x = self.center_x + int(scale_distance * math.cos(angle_rad))
                    y = self.center_y + int(scale_distance * math.sin(angle_rad))
                    
                    pygame.draw.circle(self.screen, (255, 0, 0), (x, y), 3)
                    obstacle_count += 1
            
            return obstacle_count
        except Exception:
            return 0
    
    def draw_person_detection(self, person_data):
        """Draw person detection on radar"""
        if not self.screen or not person_data:
            return
        try:
            x_camera = person_data.get('x_camera', 0)
            z_camera = person_data.get('z_camera', 0)
            
            if z_camera > 0:
                # Convert camera coordinates to display coordinates
                distance = min(z_camera, 4000)  # Cap at 4m for display
                scale_distance = (distance / 1000.0) * self.scale
                
                # Convert x_camera to angle
                angle_offset = math.atan2(x_camera, z_camera)
                angle_rad = angle_offset
                
                x = self.center_x + int(scale_distance * math.sin(angle_rad))
                y = self.center_y - int(scale_distance * math.cos(angle_rad))
                
                # Color code by stability
                stability_class = self.classify_stability(self.current_std_dev_pixels)
                if stability_class == 'stable':
                    color = (0, 255, 0)  # Green
                elif stability_class == 'moderate':
                    color = (255, 255, 0)  # Yellow
                elif stability_class == 'unstable':
                    color = (255, 165, 0)  # Orange
                else:
                    color = (255, 0, 0)  # Red
                
                pygame.draw.circle(self.screen, color, (x, y), 8)
                pygame.draw.circle(self.screen, (255, 255, 255), (x, y), 8, 2)
                
        except Exception:
            pass
    
    def draw_camera_feed(self):
        """Draw camera feed in corner"""
        if not self.screen or not self.camera_surface:
            return
        try:
            display_info = pygame.display.Info()
            camera_x = display_info.current_w - self.camera_display_size[0] - 50
            camera_y = 50
            
            # Draw camera background
            pygame.draw.rect(self.screen, (50, 50, 50), 
                           (camera_x - 5, camera_y - 5, 
                            self.camera_display_size[0] + 10, 
                            self.camera_display_size[1] + 10))
            
            # Draw camera surface
            self.screen.blit(self.camera_surface, (camera_x, camera_y))
            
            # Draw border
            pygame.draw.rect(self.screen, (0, 255, 255), 
                           (camera_x - 2, camera_y - 2, 
                            self.camera_display_size[0] + 4, 
                            self.camera_display_size[1] + 4), 2)
        except Exception:
            pass
    
    def draw_info_with_status(self, obstacle_count, person_data):
        """Draw information with system status"""
        if not self.screen:
            return
        try:
            font = pygame.font.Font(None, 36)
            y_offset = 50
            
            # Title
            title_text = font.render("FIXED DIRECT PIPELINE LIDAR TEST", True, (0, 255, 255))
            self.screen.blit(title_text, (50, y_offset))
            y_offset += 40
            
            # System status
            lidar_status = "ACTIVE" if self.lidar_system else "INACTIVE"
            camera_status = "ACTIVE" if self.camera_initialized else "INACTIVE"
            
            status_text = font.render(f"LiDAR: {lidar_status} | Camera: {camera_status}", True, (255, 255, 255))
            self.screen.blit(status_text, (50, y_offset))
            y_offset += 35
            
            # Fixed ESC handling info
            esc_info_text = font.render("FIXED: pygame.event.clear() removed for proper ESC handling", True, (0, 255, 255))
            self.screen.blit(esc_info_text, (50, y_offset))
            y_offset += 35
            
            # Detection info
            if person_data:
                stability_class = self.classify_stability(self.current_std_dev_pixels)
                std_dev = self.current_std_dev_pixels
                
                if stability_class == 'stable':
                    stability_color = (0, 255, 0)
                elif stability_class == 'moderate':
                    stability_color = (255, 255, 0)
                elif stability_class == 'unstable':
                    stability_color = (255, 165, 0)
                else:
                    stability_color = (255, 0, 0)
                
                stability_text = font.render(f"Person Detection: {stability_class.upper()} (±{std_dev:.1f}px)", True, stability_color)
                self.screen.blit(stability_text, (50, y_offset))
                y_offset += 35
                
                if self.detection_count > 0:
                    consistency_rate = (self.consistent_detection_count / self.detection_count) * 100
                    consist_color = (0, 255, 0) if consistency_rate > 90 else (255, 255, 0) if consistency_rate > 80 else (255, 0, 0)
                    consist_text = font.render(f"Consistency: {consistency_rate:.1f}% ({self.detection_count} detections)", True, consist_color)
                    self.screen.blit(consist_text, (50, y_offset))
                    y_offset += 35
            else:
                no_person_text = font.render("No Person Detected", True, (255, 255, 0))
                self.screen.blit(no_person_text, (50, y_offset))
                y_offset += 35
            
            # LiDAR info
            lidar_text = font.render(f"LiDAR Obstacles: {obstacle_count}", True, (255, 255, 255))
            self.screen.blit(lidar_text, (50, y_offset))
            y_offset += 35
            
            # Exit instructions
            exit_text = font.render("Press ESC to return to IDLE mode", True, (255, 255, 255))
            self.screen.blit(exit_text, (50, y_offset + 20))
            
        except Exception:
            pass
    
    # Variance calculation methods
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
    
    def initialize_csv_log(self):
        """Initialize CSV log"""
        if self.csv_initialized:
            return
        
        try:
            with open(self.csv_log_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'mode_time_elapsed', 'timestamp', 'frame_number',
                    'x_midpoint_pixels', 'x_midpoint_normalized',
                    'x_camera_mm', 'z_depth_mm', 'confidence',
                    'bbox_xmin', 'bbox_ymin', 'bbox_xmax', 'bbox_ymax', 'bbox_width', 'bbox_height',
                    'x_jump_from_previous', 'is_large_jump', 
                    'rolling_variance_pixels', 'rolling_std_dev_pixels', 'rolling_mean_pixels',
                    'short_term_variance', 'medium_term_variance', 'long_term_variance',
                    'detection_count', 'consistent_detections', 'large_jumps_count',
                    'stability_classification', 'head_angle_deg', 'head_tracking_active',
                    'detection_method'
                ])
            self.csv_initialized = True
            print(f"✅ CSV logging to: {self.csv_log_filename}")
        except Exception as e:
            print(f"⚠️ CSV initialization failed: {e}")
    
    def log_detection_consistency_to_csv(self, person_data):
        """Log detection consistency data"""
        try:
            if not self.csv_initialized:
                self.initialize_csv_log()
            
            x_midpoint_pixels = person_data['x_midpoint_pixels']
            x_midpoint_normalized = person_data['x_midpoint_normalized']
            
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
            
            # Calculate mode elapsed time
            mode_elapsed = time.time() - self.mode_start_time
            
            # Write to CSV
            with open(self.csv_log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    mode_elapsed, time.time(), self.update_counter,
                    x_midpoint_pixels, x_midpoint_normalized,
                    person_data['x_camera'], person_data['z_camera'], person_data['confidence'],
                    person_data['bounding_box']['xmin'], person_data['bounding_box']['ymin'],
                    person_data['bounding_box']['xmax'], person_data['bounding_box']['ymax'],
                    person_data['bounding_box']['xmax'] - person_data['bounding_box']['xmin'],
                    person_data['bounding_box']['ymax'] - person_data['bounding_box']['ymin'],
                    x_jump, is_large_jump,
                    self.current_variance_pixels, self.current_std_dev_pixels, self.current_mean_pixels,
                    short_var, medium_var, long_var,
                    self.detection_count, self.consistent_detection_count, self.large_jumps_count,
                    stability_class, 0.0, False,  # Head tracking disabled
                    person_data.get('detection_method', 'DIRECT_PIPELINE')
                ])
        except Exception:
            pass
    
    def reset_tracking_data(self):
        """Reset all tracking data for new session"""
        self.detection_count = 0
        self.consistent_detection_count = 0
        self.large_jumps_count = 0
        self.last_x_midpoint = None
        
        # Clear tracking data
        self.x_midpoints_pixels.clear()
        self.x_midpoints_normalized.clear()
        self.variance_window.clear()
        self.short_term_variance.clear()
        self.medium_term_variance.clear()
        self.long_term_variance.clear()
        
        # Reset stability zones
        for zone in self.stability_zones:
            self.stability_zones[zone] = 0
    
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
    
    def terminate(self, new_status: Status):
        """
        py_trees Lifecycle: terminate() - Called when behavior STOPS running
        PROPER cleanup that doesn't crash the robot
        """
        print("🔄 FIXED Direct Pipeline LiDAR Test terminating...")
        
        try:
            # Generate final summary
            if self.detection_count > 0:
                overall_variance = statistics.variance(list(self.x_midpoints_pixels)) if len(self.x_midpoints_pixels) > 1 else 0
                overall_std_dev = math.sqrt(overall_variance)
                consistency_rate = (self.consistent_detection_count / self.detection_count) * 100
                
                print(f"\n📊 FIXED PIPELINE CONSISTENCY SUMMARY:")
                print(f"   Detection Method: DIRECT_PIPELINE (No CameraSensor threading)")
                print(f"   Total Detections: {self.detection_count}")
                print(f"   Consistency Rate: {consistency_rate:.1f}%")
                print(f"   Overall Std Dev: ±{overall_std_dev:.2f} pixels")
                print(f"   CSV Data: {self.csv_log_filename}")
            
            # Stop robot movement
            self.stop_robot()
            
            # Center head
            self.center_head_for_idle_mode()
            
            # Stop LiDAR system gracefully
            if self.lidar_system:
                print("🛑 Stopping LiDAR system...")
                self.lidar_system.stop()
                self.lidar_system = None
                print("✅ LiDAR system stopped")
            
            # Close direct camera device gracefully
            if self.device:
                print("📷 Closing direct camera device...")
                self.device.close()
                self.device = None
                print("✅ Direct camera device closed")
            
            # FIXED: Do NOT clear pygame events - let exit_behavior handle cleanup
            # The pygame.event.clear() call was interfering with proper exit handling
            
            # Clean up blackboard
            try:
                if self.blackboard.exists("LIDAR_SYSTEM"):
                    self.blackboard.unset("LIDAR_SYSTEM")
            except Exception:
                pass
            
            # CRITICAL: Restore facial animation display
            try:
                if self.robot and hasattr(self.robot, 'facial_animation_manager') and self.robot.facial_animation_manager:
                    print("🎭 Restoring facial animation display...")
                    self.robot.facial_animation_manager.bring_to_front()
                    print("✅ Facial animation display restored")
            except Exception as e:
                print(f"⚠️ Facial animation restoration warning: {e}")
            
            # PROPER py_trees: Set robot mode to IDLE (this should happen via exit behavior)
            # The exit behavior should handle mode transition, not the main behavior
            print("✅ FIXED Direct Pipeline LiDAR Test terminated successfully")
            
        except Exception as e:
            print(f"⚠️ Termination error: {e}")
            # Ensure robot doesn't crash even if cleanup fails
            try:
                if self.robot and hasattr(self.robot, 'facial_animation_manager') and self.robot.facial_animation_manager:
                    self.robot.facial_animation_manager.bring_to_front()
            except Exception:
                pass
        
        # Call parent terminate
        super().terminate(new_status)


# Use the fixed implementation as the main class
DirectPipelineLidarTest = FixedDirectPipelineLidarTest