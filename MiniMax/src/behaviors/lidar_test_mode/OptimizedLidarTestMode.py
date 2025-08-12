#!/usr/bin/env python3
"""
Optimized LiDAR Test Mode - Standalone with Integrated Detection System
Uses EXACT detection pipeline from detection_consistency_test.py for <37px variance
Includes all necessary LiDAR components directly (no external imports needed)
FILE LOCATION: src/behaviors/lidar_test_mode/OptimizedLidarTestMode.py

CRITICAL: Follows project py_trees methodology instructions:
- Does NOT handle ESC key directly (per instruction #9)
- Returns Status.RUNNING to let exit_mode_behavior handle ESC
- Uses memory=False in __init__.py to allow exit checking
- Properly implements terminate() for cleanup when exit triggered
"""

import pygame
import math
import time
import threading
import queue
import py_trees
import csv
import os
import statistics
import numpy as np
import cv2
from collections import deque
from datetime import datetime
from py_trees.common import Status
from pyrplidar import PyRPlidar

from src.behaviors.MaxineBehavior import MaxineBehavior
from src.types.RobotModes import RobotMode
from src.types.MovementDirection import MovementDirection
from src.action_managers.VelocityManager import VelocityConfig

try:
    import depthai as dai
except ImportError:
    dai = None
    print("⚠️ DepthAI not available - will use fallback detection")


class UltraStablePyRPLidarA3:
    """Ultra-stable LiDAR class for robust obstacle detection"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000, timeout=2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.lidar = None
        self.is_connected = False
        self.scan_generator = None
        self.scan_iterator = None
        
        # Ultra-Stable Parameters
        self.motor_pwm = 600
        self.stability_mode = 4
        self.current_mode = None

    def connect(self):
        """Connect to the LiDAR device"""
        try:
            self.lidar = PyRPlidar()
            self.lidar.connect(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            self.is_connected = True
            return True
        except Exception:
            self.is_connected = False
            return False

    def disconnect(self):
        """Graceful disconnect from the LiDAR device"""
        try:
            if self.lidar and self.is_connected:
                self.lidar.stop()
                time.sleep(0.5)
                self.lidar.set_motor_pwm(0)
                time.sleep(0.5)
                self.lidar.disconnect()
                self.is_connected = False
        except Exception:
            pass

    def start_scanning(self, mode='ultra_stable'):
        """Start scanning using ultra-stable settings"""
        try:
            if not self.is_connected:
                return False
                
            self.lidar.stop()
            time.sleep(1.0)
            
            self.lidar.set_motor_pwm(self.motor_pwm)
            time.sleep(3.0)
            
            if mode == 'ultra_stable':
                try:
                    self.scan_generator = self.lidar.start_scan_express(self.stability_mode)
                    self.scan_iterator = self.scan_generator()
                    self.current_mode = 'stability'
                    return True
                except Exception:
                    pass
                
                try:
                    self.scan_generator = self.lidar.force_scan()
                    self.scan_iterator = self.scan_generator()
                    self.current_mode = 'force'
                    return True
                except Exception:
                    return False
            
            return True
            
        except Exception:
            return False

    def get_scan_data_generator(self, shutdown_flag):
        """Generator for scan data with stability improvements"""
        scan_buffer = []
        last_angle = None
        consecutive_failures = 0
        max_failures = 50
        
        while not shutdown_flag[0] and consecutive_failures < max_failures:
            try:
                measurement = next(self.scan_iterator)
                
                if measurement:
                    consecutive_failures = 0
                    
                    try:
                        quality = getattr(measurement, 'quality', 0)
                        angle = getattr(measurement, 'angle', 0)
                        distance = getattr(measurement, 'distance', 0)
                        
                        if quality > 8 and 200 < distance < 6000:
                            scan_buffer.append((quality, angle, distance))
                            
                            if (last_angle is not None and 
                                angle < last_angle and 
                                len(scan_buffer) > 100):
                                
                                yield scan_buffer
                                scan_buffer = []
                            
                            last_angle = angle
                        
                    except Exception:
                        continue
                else:
                    consecutive_failures += 1
                    time.sleep(0.1)
                    continue
                    
            except StopIteration:
                consecutive_failures += 1
                scan_buffer.clear()
                last_angle = None
                continue
            except Exception:
                consecutive_failures += 1
                time.sleep(0.15)


class UltraStableLidarSystem:
    """Ultra-stable LiDAR system for reliable robot navigation"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        
        # Ultra-stable data structures
        self.scan_queue = queue.Queue(maxsize=2)
        self.latest_obstacles = []
        self.data_lock = threading.Lock()
        
        # Conservative obstacle mapping
        self.obstacle_confidence = {}
        self.scan_cycle_count = 0
        self.angle_resolution = 1.0
        self.distance_resolution = 50
        
        # Conservative confidence parameters
        self.confidence_threshold = 0.15
        self.confidence_increment = 0.25
        self.confidence_decay = 0.05
        self.max_confidence = 1.0
        
        # Performance monitoring
        self.scan_rate = 0
        self.last_scan_time = time.time()
        self.scan_count = 0
        
        # Control flags
        self.running = False
        self.shutdown_flag = [False]
        self.threads = []
        
    def data_acquisition_thread(self):
        """Ultra-stable data acquisition thread"""
        try:
            self.lidar = UltraStablePyRPLidarA3(self.port, self.baudrate, timeout=2.0)
            
            if not self.lidar.connect():
                return
            
            if not self.lidar.start_scanning('ultra_stable'):
                return
            
            try:
                for scan_data in self.lidar.get_scan_data_generator(self.shutdown_flag):
                    if not self.running or self.shutdown_flag[0]:
                        break
                    
                    try:
                        if scan_data and len(scan_data) > 0:
                            self.scan_count += 1
                            current_time = time.time()
                            if current_time - self.last_scan_time >= 5.0:
                                self.scan_rate = self.scan_count / (current_time - self.last_scan_time)
                                self.scan_count = 0
                                self.last_scan_time = current_time
                            
                            self.update_obstacle_confidence_stable(scan_data)
                            stable_obstacles = self.get_confident_obstacles()
                            
                            if len(stable_obstacles) > 0:
                                with self.data_lock:
                                    self.latest_obstacles = stable_obstacles
                                    
                                try:
                                    self.scan_queue.put_nowait(stable_obstacles)
                                except queue.Full:
                                    try:
                                        self.scan_queue.get_nowait()
                                        self.scan_queue.put_nowait(stable_obstacles)
                                    except queue.Empty:
                                        pass
                    
                    except Exception:
                        continue
                        
            except Exception:
                pass
                
        except Exception:
            pass
        finally:
            if self.lidar:
                self.lidar.disconnect()

    def update_obstacle_confidence_stable(self, scan_data):
        """Update obstacle confidence with stable mapping"""
        seen_obstacles = set()
        
        for quality, angle, distance in scan_data:
            if quality > 8 and 200 < distance < 6000:
                # Quantize for stability
                quantized_angle = round(angle / self.angle_resolution) * self.angle_resolution
                quantized_distance = round(distance / self.distance_resolution) * self.distance_resolution
                
                key = (quantized_angle, quantized_distance)
                seen_obstacles.add(key)
                
                # Confidence update
                current_confidence = self.obstacle_confidence.get(key, 0)
                self.obstacle_confidence[key] = min(
                    self.max_confidence, 
                    current_confidence + self.confidence_increment
                )
        
        # Decay for unseen obstacles
        obstacles_to_remove = []
        for key, confidence in list(self.obstacle_confidence.items()):
            if key not in seen_obstacles:
                new_confidence = confidence - self.confidence_decay
                if new_confidence <= 0:
                    obstacles_to_remove.append(key)
                else:
                    self.obstacle_confidence[key] = new_confidence
        
        for key in obstacles_to_remove:
            del self.obstacle_confidence[key]
    
    def get_confident_obstacles(self):
        """Get stable obstacles that exceed confidence threshold"""
        stable_obstacles = []
        for (angle, distance), confidence in self.obstacle_confidence.items():
            if confidence >= self.confidence_threshold:
                stable_obstacles.append((angle, distance))
        return stable_obstacles
    
    def get_display_obstacles(self):
        """Get obstacles for display"""
        with self.data_lock:
            return self.latest_obstacles.copy() if self.latest_obstacles else []
    
    def start(self):
        """Start the LiDAR system"""
        self.running = True
        self.shutdown_flag[0] = False
        
        data_thread = threading.Thread(target=self.data_acquisition_thread, daemon=True)
        data_thread.start()
        self.threads.append(data_thread)
        
        return True
    
    def stop(self):
        """Stop the system properly"""
        self.running = False
        self.shutdown_flag[0] = True
        
        for thread in self.threads:
            thread.join(timeout=2.0)
        
        if self.lidar:
            self.lidar.disconnect()


class OptimizedDetectionSystem:
    """EXACT detection system from detection_consistency_test.py for <37px variance"""
    
    def __init__(self):
        # EXACT settings from standalone test
        self.device = None
        self.pipeline = None
        self.detection_queue = None
        self.has_detection = False
        self.camera_initialized = False
        
        # Camera specs - EXACT from standalone
        self.camera_resolution_width = 300
        self.camera_resolution_height = 300
        self.camera_hfov_degrees = 114
        self.target_fps = 25
        
        # Detection settings - EXACT from standalone
        self.confidence_threshold = 0.4
        self.detection_skip_frames = 1
        self.frame_counter = 0
        
        # Z-depth smoothing - EXACT from standalone
        self.z_depth_smoother = deque(maxlen=5)
        self.confidence_weights = deque(maxlen=5)
        self.smoothed_z_depth = 0
        self.depth_trust_threshold = 0.6
    
    def create_pipeline(self):
        """Create EXACT pipeline from detection_consistency_test.py"""
        if not dai:
            return None
            
        try:
            pipeline = dai.Pipeline()
            
            local_blob_path = "./mobilenet-ssd_openvino_2021.4_5shave.blob"
            if not os.path.exists(local_blob_path):
                print(f"⚠️ Blob file not found: {local_blob_path}")
                return None
            
            # EXACT camera setup from standalone
            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            mono_left.setFps(self.target_fps)
            
            mono_right = pipeline.create(dai.node.MonoCamera)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            mono_right.setFps(self.target_fps)
            
            # EXACT ImageManip setup from standalone
            manip_nn = pipeline.create(dai.node.ImageManip)
            manip_nn.initialConfig.setResize(300, 300)
            manip_nn.initialConfig.setKeepAspectRatio(False)
            manip_nn.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
            mono_right.out.link(manip_nn.inputImage)
            
            # EXACT stereo depth settings from standalone
            depth = pipeline.create(dai.node.StereoDepth)
            depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
            depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
            depth.initialConfig.setConfidenceThreshold(180)
            depth.setLeftRightCheck(True)
            depth.setSubpixel(False)
            depth.setDepthAlign(dai.CameraBoardSocket.CAM_C)
            
            mono_left.out.link(depth.left)
            mono_right.out.link(depth.right)
            
            # EXACT detection network settings from standalone
            detection_nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
            detection_nn.setConfidenceThreshold(0.4)
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
            
            return pipeline
            
        except Exception as e:
            print(f"Pipeline creation error: {e}")
            return None
    
    def initialize(self):
        """Initialize camera with EXACT settings from standalone"""
        if not dai:
            print("⚠️ DepthAI not available - using fallback")
            return False
            
        try:
            devices = dai.Device.getAllAvailableDevices()
            if len(devices) == 0:
                print("⚠️ No OAK devices found")
                return False
            
            self.pipeline = self.create_pipeline()
            if not self.pipeline:
                return False
            
            self.device = dai.Device(self.pipeline)
            
            # Optimize device
            try:
                if hasattr(self.device, 'setLogLevel'):
                    self.device.setLogLevel(dai.LogLevel.WARN)
                self.device.setIrLaserDotProjectorIntensity(900)
            except Exception:
                pass
            
            # Get detection queue
            try:
                self.detection_queue = self.device.getOutputQueue("detections", maxSize=4, blocking=False)
                self.has_detection = True
            except Exception:
                self.detection_queue = None
                self.has_detection = False
            
            time.sleep(2)
            
            self.camera_initialized = True
            print("✅ Optimized detection system initialized")
            return True
            
        except Exception as e:
            print(f"Detection system initialization error: {e}")
            return False
    
    def smooth_z_depth(self, raw_z_depth, confidence):
        """EXACT z-depth smoothing from standalone"""
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
    
    def get_detection(self):
        """Get detection using EXACT method from standalone"""
        if not self.has_detection or not self.detection_queue or not self.camera_initialized:
            return None
        
        try:
            self.frame_counter += 1
            
            if self.frame_counter % self.detection_skip_frames != 0:
                return None
            
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
            
            bbox_xmin = closest_person.xmin
            bbox_xmax = closest_person.xmax
            bbox_ymin = closest_person.ymin
            bbox_ymax = closest_person.ymax
            
            x_midpoint_normalized = (bbox_xmin + bbox_xmax) / 2.0
            y_midpoint_normalized = (bbox_ymin + bbox_ymax) / 2.0
            
            return {
                'x_camera': x_camera,
                'y_camera': y_camera,
                'z_camera': smoothed_z_depth,
                'raw_z_depth': raw_z_depth,
                'confidence': confidence,
                'bbox_center': {
                    'x_normalized': x_midpoint_normalized,
                    'y_normalized': y_midpoint_normalized,
                },
                'bounding_box': {
                    'xmin': bbox_xmin,
                    'xmax': bbox_xmax,
                    'ymin': bbox_ymin,
                    'ymax': bbox_ymax
                }
            }
            
        except Exception:
            return None
    
    def shutdown(self):
        """Shutdown detection system"""
        try:
            if self.device:
                self.device.close()
                self.device = None
        except Exception:
            pass


class LidarTestBehavior(MaxineBehavior):
    """
    Optimized LiDAR Test with standalone detection system for <37px variance
    
    PROPER py_trees implementation (per project instructions):
    - Does NOT handle ESC directly - that's handled by exit_mode_behavior
    - Always returns Status.RUNNING in update() 
    - Implements proper terminate() for cleanup
    - Works with memory=False in the py_trees Selector/Sequence
    """
    
    def __init__(self):
        super().__init__("Optimized LiDAR Test")
        
        # Blackboard setup
        self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.READ)
        self.blackboard.register_key("HEAD_CENTER_POSITION", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.WRITE)
        
        # Core components - UNCHANGED
        self.detection_system = None
        self.lidar_system = None
        self.screen = None
        self.initialized = False
        
        # Display parameters - UNCHANGED
        self.center_x = 0
        self.center_y = 0
        self.scale = 0
        self.update_counter = 0
        self.display_update_rate = 3
        
        # CSV LOGGING - UNCHANGED
        self.csv_log_filename = "LIDARTEST_OPTIMIZED.csv"
        self.csv_initialized = False
        self.mode_start_time = time.time()
        
        # X-midpoint consistency tracking - UNCHANGED
        self.x_midpoints_pixels = deque(maxlen=1000)
        self.x_midpoints_normalized = deque(maxlen=1000)
        self.variance_window = deque(maxlen=100)
        
        # Real-time variance calculation - UNCHANGED
        self.current_variance_pixels = 0.0
        self.current_std_dev_pixels = 0.0
        self.current_mean_pixels = 0.0
        
        # Consistency metrics - UNCHANGED
        self.detection_count = 0
        self.consistent_detection_count = 0
        self.large_jumps_count = 0
        self.jump_threshold_pixels = 50
        self.last_x_midpoint = None
        
        # Multi-term variance tracking - UNCHANGED
        self.short_term_variance = deque(maxlen=75)
        self.medium_term_variance = deque(maxlen=250)
        self.long_term_variance = deque(maxlen=1500)
        
        # Stability zones - UNCHANGED
        self.stability_zones = {
            'stable': 0, 'moderate': 0, 'unstable': 0, 'very_unstable': 0
        }
        
        # HEAD TRACKING - NEW (Optional layer, doesn't affect existing functionality)
        self.head_tracker = None
        self.head_tracking_enabled = True  # Set to False to disable head tracking completely
        
        # Optimized head tracking parameters based on excellent ±4.23px variance
        self.angle_history = []
        self.max_angle_history = 3  # Reduced from 5 for faster response with stable detection
        self.last_sent_angle = None
        self.angle_change_threshold = math.radians(4)  # Reduced from 8° - more responsive with stable detection
        self.tracking_update_interval = 3  # Reduced from 6 - 2x faster updates safe with low variance
        self.dead_zone_degrees = 8  # Reduced from 12° - tighter tracking possible with ±4.23px variance
        self.update_counter_tracking = 0
        self.last_person_detected = 0
        self.person_lost_timeout = 2.0
        
        if not pygame.get_init():
            pygame.init()
        pygame.font.init()
    
    def setup(self, **kwargs):
        """Setup behavior"""
        return True
    
    def initialise(self):
        """Initialize when behavior starts - called by py_trees when mode changes"""
        print("🎯 LiDAR Test initializing (ESC handled by py_trees exit behavior)...")
        
        if not self.initialized:
            self.initialize_components()
        self.stop_robot()
        
        # Reset tracking for new session
        self.mode_start_time = time.time()
        self.csv_initialized = False
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
    
    def initialize_components(self):
        """Initialize test mode components"""
        if self.initialized:
            return True
        
        try:
            # Initialize display - UNCHANGED
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
            pygame.display.set_caption("MAXINE OPTIMIZED LIDAR TEST")
            
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            self.scale = min(display_info.current_w, display_info.current_h) // 7
            
            # Draw initial interface - UNCHANGED
            self.draw_clean_interface()
            pygame.display.flip()
            
            # Initialize OPTIMIZED detection system - UNCHANGED
            print("🎯 Initializing optimized detection system...")
            self.detection_system = OptimizedDetectionSystem()
            if self.detection_system.initialize():
                print("✅ Optimized detection system ready")
            else:
                print("⚠️ Using fallback detection")
            
            # Initialize LiDAR system - UNCHANGED
            self.start_stable_lidar()
            
            # Initialize CSV logging - UNCHANGED
            self.initialize_csv_log()
            
            # Initialize HEAD TRACKING - NEW (Optional, non-interfering)
            if self.head_tracking_enabled:
                self.initialize_head_tracker()
            
            self.initialized = True
            return True
            
        except Exception as e:
            print(f"Initialization error: {e}")
            self.initialized = False
            return False
    
    def initialize_head_tracker(self):
        """Initialize head tracking system - NEW METHOD (optional, non-interfering)"""
        try:
            robot = self.get_robot()
            
            # Check if robot has head control capability
            if hasattr(robot, 'servo_controller') and robot.servo_controller:
                print("🎯 Initializing head tracking with servo controller...")
                # Try to import HeadTracker if available
                try:
                    from src.behaviors.lidarchase.HeadTracker import HeadTracker
                    self.head_tracker = HeadTracker(
                        head_velocity_manager=None,
                        servo_controller=robot.servo_controller
                    )
                    self.head_tracker.start_tracking()
                    self.head_tracker.set_manual_position(0.0)  # Center head
                    print("✅ Head tracking initialized (servo controller)")
                except ImportError:
                    print("⚠️ HeadTracker not available, trying direct servo control...")
                    # Fallback to direct servo control
                    robot.servo_controller.center()
                    self.head_tracker = None
                    
            elif hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager:
                print("🎯 Initializing head tracking with velocity manager...")
                try:
                    from src.behaviors.lidarchase.HeadTracker import HeadTracker
                    self.head_tracker = HeadTracker(
                        head_velocity_manager=robot.head_velocity_manager,
                        servo_controller=None
                    )
                    self.head_tracker.start_tracking()
                    self.head_tracker.set_manual_position(0.0)  # Center head
                    print("✅ Head tracking initialized (velocity manager)")
                except ImportError:
                    print("⚠️ HeadTracker not available")
                    self.head_tracker = None
            else:
                print("ℹ️ No head control available - head tracking disabled")
                self.head_tracker = None
                self.head_tracking_enabled = False
                
        except Exception as e:
            print(f"⚠️ Head tracking initialization failed: {e}")
            print("   Continuing without head tracking...")
            self.head_tracker = None
            self.head_tracking_enabled = False
    
    def initialize_csv_log(self):
        """Initialize CSV log"""
        if self.csv_initialized:
            return
        
        try:
            with open(self.csv_log_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'mode_time_elapsed', 'timestamp', 'frame_number',
                    'x_midpoint_pixels', 'x_midpoint_normalized', 'x_camera_mm',
                    'z_depth_mm', 'confidence',
                    'rolling_variance_pixels', 'rolling_std_dev_pixels', 'rolling_mean_pixels',
                    'detection_count', 'consistent_detections', 'large_jumps_count',
                    'stability_classification', 'head_angle_degrees', 'head_tracking_active'
                ])
            self.csv_initialized = True
            print(f"✅ CSV logging to: {self.csv_log_filename}")
        except Exception as e:
            print(f"⚠️ CSV initialization failed: {e}")
    
    def start_stable_lidar(self):
        """Start stable LiDAR system"""
        try:
            if not self.lidar_system:
                print("🚀 Starting LiDAR system...")
                self.lidar_system = UltraStableLidarSystem()
                success = self.lidar_system.start()
                if success:
                    self.blackboard.set("LIDAR_SYSTEM", self.lidar_system)
                    print("✅ LiDAR system started")
                    time.sleep(3)
                else:
                    print("❌ Failed to start LiDAR")
                    self.lidar_system = None
        except Exception as e:
            print(f"❌ LiDAR error: {e}")
            self.lidar_system = None
    
    def get_person_detection(self):
        """Get person detection using OPTIMIZED system or fallback"""
        # Try optimized detection first
        if self.detection_system and self.detection_system.camera_initialized:
            detection = self.detection_system.get_detection()
            if detection:
                # Convert to screen pixels
                screen_width = self.screen.get_width() if self.screen else 1920
                screen_height = self.screen.get_height() if self.screen else 1080
                
                x_pixels = int(detection['bbox_center']['x_normalized'] * screen_width)
                y_pixels = int(detection['bbox_center']['y_normalized'] * screen_height)
                
                detection['bbox_center']['x_pixels'] = x_pixels
                detection['bbox_center']['y_pixels'] = y_pixels
                
                return detection
        
        # Fallback to robot's camera sensor
        try:
            robot = self.get_robot()
            if not hasattr(robot, 'camera_sensor') or not robot.camera_sensor:
                return None
            
            reading = robot.camera_sensor.get_reading()
            if not reading or not hasattr(reading, 'get_people_locations'):
                return None
            
            people = reading.get_people_locations()
            if not people:
                return None
            
            closest_person = people[0]
            
            if closest_person.confidence < 0.4:
                return None
            
            x_center_normalized = (closest_person.xmax + closest_person.xmin) / 2.0
            y_center_normalized = (closest_person.ymax + closest_person.ymin) / 2.0
            
            screen_width = self.screen.get_width() if self.screen else 1920
            screen_height = self.screen.get_height() if self.screen else 1080
            
            x_center_pixels = int(x_center_normalized * screen_width)
            y_center_pixels = int(y_center_normalized * screen_height)
            
            return {
                'x_camera': closest_person.spatialCoordinates.x,
                'z_camera': closest_person.spatialCoordinates.z,
                'confidence': closest_person.confidence,
                'bbox_center': {
                    'x_normalized': x_center_normalized,
                    'y_normalized': y_center_normalized,
                    'x_pixels': x_center_pixels,
                    'y_pixels': y_center_pixels
                },
                'bounding_box': {
                    'xmin': closest_person.xmin,
                    'xmax': closest_person.xmax,
                    'ymin': closest_person.ymin,
                    'ymax': closest_person.ymax
                }
            }
            
        except Exception:
            return None
    
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
    
    def draw_person_detection(self):
        """Draw person detection with consistency information - UNCHANGED CORE FUNCTIONALITY"""
        person_data = self.get_person_detection()
        
        if not person_data:
            return None  # Return None when no person
        
        try:
            x_camera = person_data['x_camera']
            z_camera = person_data['z_camera']
            bbox_center = person_data['bbox_center']
            
            if z_camera <= 0:
                return None
            
            # Log consistency data - UNCHANGED
            self.log_detection_consistency_to_csv(person_data)
            
            # Calculate angle and position for radar display - UNCHANGED
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
            
            # Color-code by stability - UNCHANGED
            stability_class = self.classify_stability(self.current_std_dev_pixels)
            if stability_class == 'stable':
                color = (0, 255, 0)
            elif stability_class == 'moderate':
                color = (255, 255, 0)
            elif stability_class == 'unstable':
                color = (255, 165, 0)
            else:
                color = (255, 0, 0)
            
            # Draw detection - UNCHANGED
            if 0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height():
                pygame.draw.circle(self.screen, color, (x, y), 8)
                pygame.draw.circle(self.screen, (255, 255, 255), (x, y), 9, 2)
            
            # Draw large x-coordinate information - UNCHANGED
            self.draw_person_x_coordinate_info(bbox_center)
            
            return person_data  # Return the person data for head tracking
            
        except Exception:
            return None
    
    def draw_person_x_coordinate_info(self, bbox_center):
        """Draw person's x-coordinate center with consistency info"""
        try:
            screen_width = self.screen.get_width()
            
            x_norm = bbox_center['x_normalized']
            x_pixels = bbox_center['x_pixels']
            
            large_font = pygame.font.Font(None, 72)
            medium_font = pygame.font.Font(None, 48)
            
            main_text = f"Person X-Center: {x_pixels}px"
            detail_text = f"Variance: ±{self.current_std_dev_pixels:.1f}px | Target: <37px"
            
            # Color based on consistency
            if self.current_std_dev_pixels <= 37:
                main_color = (0, 255, 0)  # Green - meeting target
            elif self.current_std_dev_pixels <= 50:
                main_color = (255, 255, 0)  # Yellow
            else:
                main_color = (255, 0, 0)  # Red
            
            # Render text
            main_surface = large_font.render(main_text, True, main_color)
            detail_surface = medium_font.render(detail_text, True, (255, 255, 255))
            
            # Position text
            main_x = (screen_width - main_surface.get_width()) // 2
            main_y = 50
            
            detail_x = (screen_width - detail_surface.get_width()) // 2
            detail_y = main_y + main_surface.get_height() + 10
            
            # Draw text
            self.screen.blit(main_surface, (main_x, main_y))
            self.screen.blit(detail_surface, (detail_x, detail_y))
            
        except Exception:
            pass
    
    def update_head_tracking(self, person_data):
        """Update head tracking based on person detection - NEW METHOD (optional layer)"""
        if not self.head_tracking_enabled or not self.head_tracker or not person_data:
            return
        
        try:
            bbox_center = person_data['bbox_center']
            x_pixels = bbox_center['x_pixels']
            z_camera = person_data.get('z_camera', 0)
            
            if z_camera <= 0:
                return
            
            # Only process every N frames (optimized for ±4.23px variance)
            self.update_counter_tracking += 1
            if self.update_counter_tracking % self.tracking_update_interval != 0:
                return
            
            # Calculate pixel-based angle
            screen_center_x = self.screen.get_width() // 2 if self.screen else 960
            pixel_offset = x_pixels - screen_center_x
            pixel_offset_normalized = pixel_offset / screen_center_x
            
            # Convert to angle with correct polarity
            camera_hfov_rad = math.radians(108)  # Typical camera FOV
            raw_angle_rad = -pixel_offset_normalized * (camera_hfov_rad / 2.0)
            
            # Smooth angle with optimized history for low variance
            self.angle_history.append(raw_angle_rad)
            if len(self.angle_history) > self.max_angle_history:
                self.angle_history.pop(0)
            
            if len(self.angle_history) >= 2:
                smoothed_angle_rad = sum(self.angle_history) / len(self.angle_history)
            else:
                smoothed_angle_rad = raw_angle_rad
            
            smoothed_angle_deg = math.degrees(smoothed_angle_rad)
            
            # Optimized dead zone for excellent detection stability
            dead_zone_rad = math.radians(self.dead_zone_degrees)
            is_in_dead_zone = abs(smoothed_angle_rad) <= dead_zone_rad
            
            # Check for significant change
            significant_change = (self.last_sent_angle is None or 
                                abs(smoothed_angle_rad - self.last_sent_angle) > self.angle_change_threshold)
            
            # Send head movement command if needed
            if (not is_in_dead_zone) and significant_change:
                self.head_tracker.set_person_tracking(smoothed_angle_rad)
                self.last_sent_angle = smoothed_angle_rad
            
            self.last_person_detected = time.time()
            
        except Exception as e:
            # Silently handle errors to not affect detection/LiDAR
            pass
    
    def get_head_angle_degrees(self):
        """Get current head angle in degrees for display/logging"""
        if self.last_sent_angle is not None:
            return math.degrees(self.last_sent_angle)
        return 0.0
    
    def draw_info(self, obstacle_count):
        """Draw test information with head tracking status"""
        try:
            lidar_status = "ACTIVE" if self.lidar_system else "INACTIVE"
            detection_status = "OPTIMIZED" if (self.detection_system and self.detection_system.camera_initialized) else "FALLBACK"
            consistency_rate = (self.consistent_detection_count / max(1, self.detection_count)) * 100
            
            # Head tracking status - NEW
            if self.head_tracker and self.head_tracking_enabled:
                head_angle = self.get_head_angle_degrees()
                head_status = f"HEAD: {head_angle:.1f}° (±{self.dead_zone_degrees}° dead zone)"
            elif self.head_tracking_enabled:
                head_status = "HEAD: Initializing..."
            else:
                head_status = "HEAD: Disabled"
            
            info_lines = [
                f"OPTIMIZED LIDAR TEST - Detection: {detection_status} | LiDAR: {lidar_status}",
                f"Obstacles: {obstacle_count} | Consistency: {consistency_rate:.1f}% | {head_status}",
                f"Variance: ±{self.current_std_dev_pixels:.1f}px (Target <37px) | Detections: {self.detection_count}",
                f"CSV: {self.csv_log_filename} | Press ESC to exit to IDLE mode"
            ]
            
            y_offset = self.screen.get_height() - 120
            font = pygame.font.Font(None, 36)
            
            for i, line in enumerate(info_lines):
                color = (0, 255, 255) if i == 0 else (255, 255, 255)
                if "Variance" in line:
                    # Color code variance line
                    if self.current_std_dev_pixels <= 10:
                        color = (0, 255, 0)  # Green - excellent
                    elif self.current_std_dev_pixels <= 37:
                        color = (255, 255, 0)  # Yellow - good
                    else:
                        color = (255, 0, 0)  # Red - poor
                text_surface = font.render(line, True, color)
                self.screen.blit(text_surface, (20, y_offset + i * 30))
            
        except Exception:
            pass
        """Draw test information"""
        try:
            lidar_status = "ACTIVE" if self.lidar_system else "INACTIVE"
            detection_status = "OPTIMIZED" if (self.detection_system and self.detection_system.camera_initialized) else "FALLBACK"
            consistency_rate = (self.consistent_detection_count / max(1, self.detection_count)) * 100
            
            info_lines = [
                f"OPTIMIZED LIDAR TEST - Detection: {detection_status} | LiDAR: {lidar_status}",
                f"Obstacles: {obstacle_count} | Consistency: {consistency_rate:.1f}%",
                f"CSV: {self.csv_log_filename} | Press ESC to exit to IDLE mode"
            ]
            
            y_offset = self.screen.get_height() - 100
            font = pygame.font.Font(None, 36)
            
            for i, line in enumerate(info_lines):
                color = (0, 255, 255) if i == 0 else (255, 255, 255)
                text_surface = font.render(line, True, color)
                self.screen.blit(text_surface, (20, y_offset + i * 30))
            
        except Exception:
            pass
    
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
        """Calculate multi-term variance"""
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
        """Log detection consistency data with head tracking info"""
        try:
            if not self.csv_initialized:
                self.initialize_csv_log()
            
            bbox_center = person_data['bbox_center']
            x_midpoint_pixels = bbox_center['x_pixels']
            x_midpoint_normalized = bbox_center['x_normalized']
            
            # Track midpoints - UNCHANGED
            self.x_midpoints_pixels.append(x_midpoint_pixels)
            self.x_midpoints_normalized.append(x_midpoint_normalized)
            self.variance_window.append(x_midpoint_pixels)
            
            # Calculate jump - UNCHANGED
            x_jump = 0
            is_large_jump = False
            if self.last_x_midpoint is not None:
                x_jump = abs(x_midpoint_pixels - self.last_x_midpoint)
                is_large_jump = x_jump > self.jump_threshold_pixels
                if is_large_jump:
                    self.large_jumps_count += 1
            
            self.last_x_midpoint = x_midpoint_pixels
            
            # Update counts - UNCHANGED
            self.detection_count += 1
            if not is_large_jump:
                self.consistent_detection_count += 1
            
            # Calculate variance metrics - UNCHANGED
            self.current_variance_pixels, self.current_std_dev_pixels, self.current_mean_pixels = self.calculate_x_midpoint_variance()
            
            # Calculate multi-term variance - UNCHANGED
            short_var, medium_var, long_var = self.calculate_multi_term_variance(x_midpoint_pixels)
            
            # Classify stability - UNCHANGED
            stability_class = self.classify_stability(self.current_std_dev_pixels)
            self.stability_zones[stability_class] += 1
            
            # Calculate elapsed time - UNCHANGED
            mode_elapsed = time.time() - self.mode_start_time
            
            # Get head tracking info - NEW
            head_angle_deg = self.get_head_angle_degrees()
            head_tracking_active = self.head_tracker is not None and self.head_tracking_enabled
            
            # Write to CSV with head tracking data
            with open(self.csv_log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    mode_elapsed, time.time(), self.update_counter,
                    x_midpoint_pixels, x_midpoint_normalized,
                    person_data['x_camera'], person_data['z_camera'], person_data['confidence'],
                    self.current_variance_pixels, self.current_std_dev_pixels, self.current_mean_pixels,
                    self.detection_count, self.consistent_detection_count, self.large_jumps_count,
                    stability_class, head_angle_deg, head_tracking_active
                ])
        except Exception:
            pass  # Silent fail to not affect detection
    
    def stop_robot(self):
        """Stop robot movement (wheels only, not head)"""
        try:
            robot = self.get_robot()
            
            velocity_manager = None
            if hasattr(robot, 'direct_velocity_manager') and robot.direct_velocity_manager:
                velocity_manager = robot.direct_velocity_manager
            elif hasattr(robot, 'velocity_manager') and robot.velocity_manager:
                velocity_manager = robot.velocity_manager
            
            if velocity_manager:
                stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                velocity_manager.perform_action(stop_config)
                
        except Exception:
            pass
    
    def update(self) -> Status:
        """
        Main update method - PROPER py_trees implementation
        
        CRITICAL (per project instruction #9):
        - Does NOT handle ESC or any keyboard events
        - Exit behavior checks for ESC and handles mode change
        - This behavior just runs and displays, returns RUNNING
        - The Selector with memory=False allows exit checking each tick
        """
        try:
            if not self.initialized:
                if not self.initialize_components():
                    return Status.FAILURE
            
            self.update_counter += 1
            
            # Pump events to keep pygame responsive but don't consume them
            # This prevents the "application not responding" issue
            pygame.event.pump()
            
            # Note: We do NOT use pygame.event.get() here!
            # The exit_mode_behavior (KeyPressedCondition for ESC) needs to see events
            # That behavior is in the Selector BEFORE this behavior
            # It checks each tick because memory=False on the Selector
            # When ESC is pressed, exit_mode_behavior returns SUCCESS
            # This causes the Selector to return SUCCESS (first child succeeded)
            # That ends this behavior and py_trees calls our terminate() method
            
            # Display update and get person detection
            if self.update_counter % self.display_update_rate == 0:
                try:
                    if self.screen:
                        self.screen.fill((0, 0, 0))
                        self.draw_radar_grid()
                        self.draw_robot()
                        
                        # Draw LiDAR obstacles - UNCHANGED
                        obstacle_count = 0
                        if self.lidar_system:
                            obstacles = self.lidar_system.get_display_obstacles()
                            if obstacles:
                                obstacle_count = self.draw_lidar_data(obstacles)
                        
                        # Draw person detection and get data for head tracking
                        person_data = self.draw_person_detection()
                        
                        # Update head tracking with person data - NEW
                        if person_data and self.head_tracking_enabled:
                            self.update_head_tracking(person_data)
                        elif self.head_tracker and time.time() - self.last_person_detected > self.person_lost_timeout:
                            # Person lost - center head
                            try:
                                self.angle_history.clear()
                                self.last_sent_angle = None
                                if self.head_tracker:
                                    self.head_tracker.set_manual_position(0.0)
                            except Exception:
                                pass  # Silently handle to not affect other systems
                        
                        # Draw info - Will show head tracking status
                        self.draw_info(obstacle_count)
                        
                        pygame.display.flip()
                        
                except Exception as e:
                    print(f"Display error: {e}")
            
            # ALWAYS return RUNNING
            # The exit_mode_behavior in the Selector will check for ESC
            # When ESC is pressed, it returns SUCCESS, which makes the Selector
            # return SUCCESS, ending this behavior and calling terminate()
            return Status.RUNNING
            
        except Exception as e:
            print(f"Update error: {e}")
            # Return RUNNING even on error
            return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """
        Terminate method - called when behavior stops
        This is called by py_trees when exit_mode_behavior triggers
        """
        print("🔄 LiDAR Test terminating properly via py_trees...")
        
        try:
            # Stop robot movement - UNCHANGED
            self.stop_robot()
            
            # Stop head tracking - NEW (optional cleanup)
            if self.head_tracker:
                print("🛑 Stopping head tracking...")
                try:
                    # Center head before stopping
                    self.head_tracker.set_manual_position(0.0)
                    time.sleep(0.5)
                    self.head_tracker.stop_tracking()
                except Exception as e:
                    print(f"   Head tracking cleanup warning: {e}")
                self.head_tracker = None
            elif self.head_tracking_enabled:
                # Try to center head directly if no tracker
                try:
                    robot = self.get_robot()
                    if hasattr(robot, 'servo_controller') and robot.servo_controller:
                        robot.servo_controller.center()
                    elif hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager:
                        robot.head_velocity_manager.center_head()
                except Exception:
                    pass
            
            # Stop LiDAR system - UNCHANGED
            if self.lidar_system:
                print("🛑 Stopping LiDAR system...")
                self.lidar_system.stop()
                self.lidar_system = None
            
            # Shutdown detection system - UNCHANGED
            if self.detection_system:
                print("🛑 Stopping detection system...")
                self.detection_system.shutdown()
                self.detection_system = None
            
            # Clean up blackboard - UNCHANGED
            try:
                if self.blackboard.exists("LIDAR_SYSTEM"):
                    self.blackboard.unset("LIDAR_SYSTEM")
            except Exception:
                pass
            
            # Generate final summary - ENHANCED with head tracking info
            if self.detection_count > 0:
                overall_variance = statistics.variance(list(self.x_midpoints_pixels)) if len(self.x_midpoints_pixels) > 1 else 0
                overall_std_dev = math.sqrt(overall_variance)
                consistency_rate = (self.consistent_detection_count / self.detection_count) * 100
                
                print(f"\n📊 LIDAR TEST FINAL SUMMARY:")
                print(f"   Total Detections: {self.detection_count}")
                print(f"   Consistency Rate: {consistency_rate:.1f}%")
                print(f"   Overall Std Dev: ±{overall_std_dev:.2f} pixels")
                print(f"   Target Met: {'✅ YES' if overall_std_dev < 37 else '❌ NO'} (target <37px)")
                print(f"   Head Tracking: {'Enabled' if self.head_tracking_enabled else 'Disabled'}")
                print(f"   CSV Data: {self.csv_log_filename}")
            
            # Clear pygame event queue - UNCHANGED
            if pygame.get_init():
                pygame.event.clear()
            
            self.initialized = False
            
            print("✅ LiDAR Test terminated successfully")
            
        except Exception as e:
            print(f"⚠️ Termination error: {e}")
        
        # Call parent terminate - UNCHANGED
        super().terminate(new_status)


# Export with compatibility names
OptimizedLidarTestBehavior = LidarTestBehavior
ImprovedLidarTest = LidarTestBehavior
StableLidarTest = LidarTestBehavior

__all__ = ['LidarTestBehavior', 'OptimizedLidarTestBehavior', 'ImprovedLidarTest', 'StableLidarTest']