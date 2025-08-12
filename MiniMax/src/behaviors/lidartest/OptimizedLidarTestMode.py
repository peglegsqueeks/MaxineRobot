#!/usr/bin/env python3
"""
Optimized LiDAR Test Mode - Integration of Standalone Detection System
Uses EXACT detection pipeline from detection_consistency_test.py for <37px variance
Combined with working LiDAR obstacle detection and pygame plotting
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


class OptimizedDetectionSystem:
    """EXACT detection system from detection_consistency_test.py for <37px variance"""
    
    def __init__(self):
        # EXACT settings from standalone test
        self.device = None
        self.pipeline = None
        self.detection_queue = None
        self.preview_queue = None
        self.has_detection = False
        self.camera_initialized = False
        
        # Camera specs - EXACT from standalone
        self.camera_resolution_width = 300
        self.camera_resolution_height = 300
        self.camera_hfov_degrees = 114
        self.target_fps = 25
        
        # Detection settings - EXACT from standalone
        self.confidence_threshold = 0.4  # Lower threshold like standalone
        self.detection_skip_frames = 1
        self.frame_counter = 0
        
        # Z-depth smoothing - EXACT from standalone
        self.z_depth_smoother = deque(maxlen=5)
        self.confidence_weights = deque(maxlen=5)
        self.smoothed_z_depth = 0
        self.depth_trust_threshold = 0.6
        
        # Performance tracking
        self.fps_counter = deque(maxlen=30)
        self.last_frame_time = time.time()
        self.current_fps = 0.0
    
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
            detection_nn.setConfidenceThreshold(0.4)  # EXACT threshold
            detection_nn.setBlobPath(local_blob_path)
            detection_nn.setBoundingBoxScaleFactor(0.5)  # EXACT scale factor
            detection_nn.setDepthLowerThreshold(100)    # EXACT thresholds
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
            # Check for devices
            devices = dai.Device.getAllAvailableDevices()
            if len(devices) == 0:
                print("⚠️ No OAK devices found")
                return False
            
            self.pipeline = self.create_pipeline()
            if not self.pipeline:
                return False
            
            # Connect to device
            self.device = dai.Device(self.pipeline)
            
            # Optimize device - EXACT from standalone
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
            
            # Wait for camera to stabilize
            time.sleep(2)
            
            self.camera_initialized = True
            print("✅ Optimized detection system initialized (using standalone settings)")
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
            
            # Skip frames - EXACT from standalone
            if self.frame_counter % self.detection_skip_frames != 0:
                return None
            
            detections = self.detection_queue.tryGet()
            if not detections:
                return None
            
            # Filter for people (label 15)
            person_detections = [det for det in detections.detections if det.label == 15]
            if not person_detections:
                return None
            
            # Get closest person
            closest_person = min(person_detections, key=lambda p: p.spatialCoordinates.z)
            
            x_camera = closest_person.spatialCoordinates.x
            y_camera = closest_person.spatialCoordinates.y
            raw_z_depth = closest_person.spatialCoordinates.z
            confidence = closest_person.confidence
            
            if raw_z_depth <= 0 or raw_z_depth > 15000:
                return None
            
            # Apply smoothing
            smoothed_z_depth = self.smooth_z_depth(raw_z_depth, confidence)
            
            # Calculate bounding box center
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


# Import working LiDAR components from existing file
from .LidarTestBehavior import UltraStableLidarSystem, EnhancedFacialAnimationRestorer


class OptimizedLidarTestBehavior(MaxineBehavior):
    """Optimized LiDAR Test with standalone detection system for <37px variance"""
    
    def __init__(self):
        super().__init__("Optimized LiDAR Test - Standalone Detection Integration")
        
        # Blackboard setup
        self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.READ)
        self.blackboard.register_key("HEAD_CENTER_POSITION", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.WRITE)
        
        # Core components
        self.detection_system = None  # Will use OptimizedDetectionSystem
        self.lidar_system = None
        self.screen = None
        self.head_tracker = None
        self.facial_restorer = EnhancedFacialAnimationRestorer()
        self.initialized = False
        
        # Display parameters
        self.center_x = 0
        self.center_y = 0
        self.scale = 0
        self.update_counter = 0
        self.display_update_rate = 3
        
        # Head tracking parameters - Conservative for stability
        self.current_head_angle = 0.0
        self.head_angle_lock = threading.Lock()
        self.angle_history = []
        self.max_angle_history = 5
        self.last_sent_angle = None
        self.angle_change_threshold = math.radians(8)
        self.update_counter_tracking = 0
        self.tracking_update_interval = 6
        self.last_person_detected = 0
        self.person_lost_timeout = 2.0
        
        # CSV LOGGING
        self.csv_log_filename = "LIDARTEST_OPTIMIZED.csv"
        self.csv_initialized = False
        self.mode_start_time = time.time()
        
        # X-midpoint consistency tracking
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
        self.short_term_variance = deque(maxlen=75)
        self.medium_term_variance = deque(maxlen=250)
        self.long_term_variance = deque(maxlen=1500)
        
        # Stability classification
        self.stability_zones = {
            'stable': 0, 'moderate': 0, 'unstable': 0, 'very_unstable': 0
        }
        
        if not pygame.get_init():
            pygame.init()
        pygame.font.init()
    
    def setup(self, **kwargs):
        """Setup behavior"""
        return True
    
    def initialise(self):
        """Initialize when behavior starts"""
        if not self.initialized:
            self.initialize_components()
        self.stop_robot()
        
        # Reset CSV logging for new session
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
            # Initialize display
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
            pygame.display.set_caption("MAXINE OPTIMIZED LIDAR TEST - <37px VARIANCE")
            
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            self.scale = min(display_info.current_w, display_info.current_h) // 7
            
            # Draw initial interface
            self.draw_clean_interface()
            pygame.display.flip()
            
            # Initialize OPTIMIZED detection system (from standalone)
            print("🎯 Initializing optimized detection system...")
            self.detection_system = OptimizedDetectionSystem()
            if self.detection_system.initialize():
                print("✅ Optimized detection system ready")
            else:
                print("⚠️ Using fallback detection")
            
            # Initialize head tracker
            robot = self.get_robot()
            if (hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager) or \
               (hasattr(robot, 'servo_controller') and robot.servo_controller):
                try:
                    from src.behaviors.lidarchase.HeadTracker import HeadTracker
                    self.head_tracker = HeadTracker(
                        robot.head_velocity_manager if hasattr(robot, 'head_velocity_manager') else None,
                        robot.servo_controller if hasattr(robot, 'servo_controller') else None
                    )
                    self.head_tracker.start_tracking()
                    self.head_tracker.set_manual_position(0.0)
                except Exception:
                    self.head_tracker = None
            
            # Initialize LiDAR system
            self.start_stable_lidar()
            
            # Initialize CSV logging
            self.initialize_csv_log()
            
            self.initialized = True
            return True
            
        except Exception as e:
            print(f"Initialization error: {e}")
            self.initialized = False
            return False
    
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
                    'bbox_xmin', 'bbox_ymin', 'bbox_xmax', 'bbox_ymax', 'bbox_width', 'bbox_height',
                    'x_jump_from_previous', 'is_large_jump',
                    'rolling_variance_pixels', 'rolling_std_dev_pixels', 'rolling_mean_pixels',
                    'short_term_variance', 'medium_term_variance', 'long_term_variance',
                    'detection_count', 'consistent_detections', 'large_jumps_count',
                    'stability_classification', 'head_angle_deg', 'head_tracking_active'
                ])
            self.csv_initialized = True
            print(f"✅ Optimized consistency logging to: {self.csv_log_filename}")
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
                    print("✅ LiDAR system started successfully")
                    time.sleep(3)
                else:
                    print("❌ Failed to start LiDAR system")
                    self.lidar_system = None
        except Exception as e:
            print(f"❌ LiDAR initialization error: {e}")
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
        
        # Fallback to robot's camera sensor if available
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
            
            if closest_person.confidence < 0.4:  # Use same threshold as optimized
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
        """Draw person detection with consistency information"""
        person_data = self.get_person_detection()
        
        if not person_data:
            return
        
        try:
            x_camera = person_data['x_camera']
            z_camera = person_data['z_camera']
            bbox_center = person_data['bbox_center']
            
            if z_camera <= 0:
                return
            
            # Log consistency data
            self.log_detection_consistency_to_csv(person_data)
            
            # Calculate angle and position for radar display
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
                color = (0, 255, 0)
            elif stability_class == 'moderate':
                color = (255, 255, 0)
            elif stability_class == 'unstable':
                color = (255, 165, 0)
            else:
                color = (255, 0, 0)
            
            # Draw detection
            if 0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height():
                pygame.draw.circle(self.screen, color, (x, y), 8)
                pygame.draw.circle(self.screen, (255, 255, 255), (x, y), 9, 2)
                
                # Show variance info
                font = pygame.font.Font(None, 20)
                x_pixels = bbox_center['x_pixels']
                info_text = f"X:{x_pixels}px ±{self.current_std_dev_pixels:.1f}"
                text_surface = font.render(info_text, True, color)
                self.screen.blit(text_surface, (x + 10, y - 10))
            
            # Draw large x-coordinate information
            self.draw_person_x_coordinate_info(bbox_center)
            
        except Exception:
            pass
    
    def draw_person_x_coordinate_info(self, bbox_center):
        """Draw person's x-coordinate center with consistency info"""
        try:
            screen_width = self.screen.get_width()
            
            x_norm = bbox_center['x_normalized']
            x_pixels = bbox_center['x_pixels']
            
            large_font = pygame.font.Font(None, 72)
            medium_font = pygame.font.Font(None, 48)
            
            main_text = f"Person X-Center: {x_pixels}px"
            detail_text = f"Normalized: {x_norm:.3f} | Variance: ±{self.current_std_dev_pixels:.1f}px"
            consistency_text = f"Detections: {self.detection_count} | Consistent: {self.consistent_detection_count} | Jumps: {self.large_jumps_count}"
            
            # Color based on consistency
            if self.current_std_dev_pixels <= 10:
                main_color = (0, 255, 0)
            elif self.current_std_dev_pixels <= 25:
                main_color = (255, 255, 0)
            elif self.current_std_dev_pixels <= 50:
                main_color = (255, 165, 0)
            else:
                main_color = (255, 0, 0)
            
            # Render text
            main_surface = large_font.render(main_text, True, main_color)
            detail_surface = medium_font.render(detail_text, True, (255, 255, 255))
            consistency_surface = medium_font.render(consistency_text, True, (0, 255, 255))
            
            # Position text
            main_x = (screen_width - main_surface.get_width()) // 2
            main_y = 50
            
            detail_x = (screen_width - detail_surface.get_width()) // 2
            detail_y = main_y + main_surface.get_height() + 10
            
            consistency_x = (screen_width - consistency_surface.get_width()) // 2
            consistency_y = detail_y + detail_surface.get_height() + 10
            
            # Draw background
            total_height = main_surface.get_height() + detail_surface.get_height() + consistency_surface.get_height() + 30
            max_width = max(main_surface.get_width(), detail_surface.get_width(), consistency_surface.get_width())
            status_bg = pygame.Rect(main_x - 10, main_y - 5, max_width + 20, total_height)
            
            pygame.draw.rect(self.screen, (0, 0, 0), status_bg)
            pygame.draw.rect(self.screen, main_color, status_bg, 3)
            
            # Draw text
            self.screen.blit(main_surface, (main_x, main_y))
            self.screen.blit(detail_surface, (detail_x, detail_y))
            self.screen.blit(consistency_surface, (consistency_x, consistency_y))
            
        except Exception:
            pass
    
    def update_head_tracking(self, person_data):
        """Conservative head tracking for stability"""
        if not self.head_tracker or not person_data:
            return
        
        try:
            bbox_center = person_data['bbox_center']
            x_pixels = bbox_center['x_pixels']
            z_camera = person_data['z_camera']
            
            if z_camera <= 0:
                return
            
            # Only process every N frames
            self.update_counter_tracking += 1
            if self.update_counter_tracking % self.tracking_update_interval != 0:
                return
            
            # Calculate pixel-based angle
            screen_center_x = self.screen.get_width() // 2 if self.screen else 960
            pixel_offset = x_pixels - screen_center_x
            pixel_offset_normalized = pixel_offset / screen_center_x
            
            # Convert to angle
            camera_hfov_rad = math.radians(108)
            raw_angle_rad = -pixel_offset_normalized * (camera_hfov_rad / 2.0)
            
            # Smooth angle
            self.angle_history.append(raw_angle_rad)
            if len(self.angle_history) > self.max_angle_history:
                self.angle_history.pop(0)
            
            if len(self.angle_history) >= 2:
                smoothed_angle_rad = sum(self.angle_history) / len(self.angle_history)
            else:
                smoothed_angle_rad = raw_angle_rad
            
            # Dead zone
            dead_zone_rad = math.radians(12)
            is_in_dead_zone = abs(smoothed_angle_rad) <= dead_zone_rad
            
            # Significant change check
            significant_change = (self.last_sent_angle is None or 
                                abs(smoothed_angle_rad - self.last_sent_angle) > self.angle_change_threshold)
            
            # Send command if needed
            if (not is_in_dead_zone) and significant_change:
                self.head_tracker.set_person_tracking(smoothed_angle_rad)
                self.last_sent_angle = smoothed_angle_rad
            
            self.last_person_detected = time.time()
            
        except Exception:
            pass
    
    def draw_info(self, obstacle_count):
        """Draw test information with consistency metrics"""
        try:
            tracking_status = "ON" if self.head_tracker else "OFF"
            lidar_status = "ACTIVE" if self.lidar_system else "INACTIVE"
            detection_status = "OPTIMIZED" if (self.detection_system and self.detection_system.camera_initialized) else "FALLBACK"
            
            current_head_angle = math.degrees(self.last_sent_angle) if self.last_sent_angle else 0.0
            consistency_rate = (self.consistent_detection_count / max(1, self.detection_count)) * 100
            
            info_lines = [
                f"OPTIMIZED LIDAR TEST - Detection: {detection_status} | LiDAR: {lidar_status} | Head: {tracking_status}",
                f"LiDAR Obstacles: {obstacle_count} | Head: {current_head_angle:.1f}° | Target: <37px variance",
                f"Detection Consistency: {consistency_rate:.1f}% | Variance: ±{self.current_std_dev_pixels:.1f}px",
                f"Detections: {self.detection_count} | Large Jumps: {self.large_jumps_count}",
                f"CSV Logging: {self.csv_log_filename} | ESC: Exit to Idle Mode"
            ]
            
            y_offset = self.screen.get_height() - 150
            font = pygame.font.Font(None, 36)
            
            for i, line in enumerate(info_lines):
                if i == 0:
                    color = (0, 255, 255)
                elif "Consistency" in line:
                    if consistency_rate > 90:
                        color = (0, 255, 0)
                    elif consistency_rate > 80:
                        color = (255, 255, 0)
                    else:
                        color = (255, 0, 0)
                elif "CSV Logging" in line:
                    color = (255, 165, 0)
                else:
                    color = (255, 255, 255)
                
                text_surface = font.render(line, True, color)
                self.screen.blit(text_surface, (20, y_offset + i * 30))
            
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
        """Log detection consistency data"""
        try:
            if not self.csv_initialized:
                self.initialize_csv_log()
            
            bbox_center = person_data['bbox_center']
            x_midpoint_pixels = bbox_center['x_pixels']
            x_midpoint_normalized = bbox_center['x_normalized']
            
            # Track midpoints
            self.x_midpoints_pixels.append(x_midpoint_pixels)
            self.x_midpoints_normalized.append(x_midpoint_normalized)
            self.variance_window.append(x_midpoint_pixels)
            
            # Calculate jump
            x_jump = 0
            is_large_jump = False
            if self.last_x_midpoint is not None:
                x_jump = abs(x_midpoint_pixels - self.last_x_midpoint)
                is_large_jump = x_jump > self.jump_threshold_pixels
                if is_large_jump:
                    self.large_jumps_count += 1
            
            self.last_x_midpoint = x_midpoint_pixels
            
            # Update counts
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
            
            # Get head tracking info
            head_angle_deg = math.degrees(self.last_sent_angle) if self.last_sent_angle else 0.0
            head_tracking_active = self.head_tracker is not None
            
            # Calculate elapsed time
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
                    stability_class, head_angle_deg, head_tracking_active
                ])
        except Exception:
            pass
    
    def stop_robot(self):
        """Stop robot movement"""
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
    
    def center_head_for_idle_mode(self):
        """Center head for idle mode"""
        try:
            robot = self.get_robot()
            
            if hasattr(robot, 'servo_controller') and robot.servo_controller:
                robot.servo_controller.center()
                time.sleep(0.5)
            elif hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager:
                robot.head_velocity_manager.center_head()
                time.sleep(0.5)
                    
        except Exception:
            pass
    
    def perform_idle_mode_transition(self):
        """Transition to IDLE mode with final summary"""
        try:
            # Generate final summary
            if self.detection_count > 0:
                overall_variance = statistics.variance(list(self.x_midpoints_pixels)) if len(self.x_midpoints_pixels) > 1 else 0
                overall_std_dev = math.sqrt(overall_variance)
                consistency_rate = (self.consistent_detection_count / self.detection_count) * 100
                
                print(f"\n📊 OPTIMIZED LIDAR TEST SUMMARY:")
                print(f"   Total Detections: {self.detection_count}")
                print(f"   Consistency Rate: {consistency_rate:.1f}%")
                print(f"   Overall Std Dev: ±{overall_std_dev:.2f} pixels")
                print(f"   Large Jumps: {self.large_jumps_count}")
                print(f"   CSV Data: {self.csv_log_filename}")
            
            robot = self.get_robot()
            
            self.stop_robot()
            self.center_head_for_idle_mode()
            
            if self.lidar_system:
                print("🛑 Stopping LiDAR system...")
                self.lidar_system.stop()
                self.lidar_system = None
                print("✅ LiDAR system stopped")
            
            if self.detection_system:
                print("🛑 Stopping detection system...")
                self.detection_system.shutdown()
                self.detection_system = None
                print("✅ Detection system stopped")
            
            if pygame.get_init():
                pygame.event.clear()
            
            self.facial_restorer.restore_resting_face_immediately(robot)
            
            return True
            
        except Exception:
            return False
    
    def update(self) -> Status:
        """Main update method"""
        try:
            if not self.initialized:
                if not self.initialize_components():
                    return Status.FAILURE
            
            self.update_counter += 1
            
            # Handle person detection and head tracking
            person_data = self.get_person_detection()
            if person_data:
                self.update_head_tracking(person_data)
            elif time.time() - self.last_person_detected > self.person_lost_timeout and self.head_tracker:
                # Clear tracking when person is lost
                self.angle_history.clear()
                self.last_sent_angle = None
                self.head_tracker.set_manual_position(0.0)
            
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.perform_idle_mode_transition()
                        robot = self.get_robot()
                        robot.set_mode(RobotMode.IDLE)
                        return Status.SUCCESS
                elif event.type == pygame.QUIT:
                    self.perform_idle_mode_transition()
                    robot = self.get_robot()
                    robot.set_mode(RobotMode.IDLE)
                    return Status.SUCCESS
            
            # Display update
            if self.update_counter % self.display_update_rate == 0:
                try:
                    if self.screen:
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
                        self.draw_person_detection()
                        
                        # Draw info
                        self.draw_info(obstacle_count)
                        
                        pygame.display.flip()
                        
                except Exception as e:
                    print(f"Display error: {e}")
            
            return Status.RUNNING
            
        except Exception as e:
            print(f"Update error: {e}")
            try:
                self.stop_robot()
                self.perform_idle_mode_transition()
            except Exception:
                pass
            
            return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """Terminate method with summary"""
        try:
            self.stop_robot()
            
            if self.head_tracker:
                self.head_tracker.stop_tracking()
                self.head_tracker = None
            
            if self.lidar_system:
                self.lidar_system.stop()
                self.lidar_system = None
            
            if self.detection_system:
                self.detection_system.shutdown()
                self.detection_system = None
            
            self.perform_idle_mode_transition()
            
            try:
                if self.blackboard.exists("LIDAR_SYSTEM"):
                    self.blackboard.unset("LIDAR_SYSTEM")
            except Exception:
                pass
            
            self.initialized = False
            
        except Exception:
            try:
                self.center_head_for_idle_mode()
                robot = self.get_robot()
                if hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                    robot.facial_animation_manager.bring_to_front()
            except:
                pass
        
        super().terminate(new_status)