#!/usr/bin/env python3
"""
Fixed Potential Field Visualization Test - FOLLOWS EXISTING WORKING PATTERN
Uses EXACT same camera pipeline from detection_consistency_test.py
ONLY overrides detection system creation - everything else uses working parent methods

File: src/behaviors/lidar_test_mode/PotentialFieldVisualizationTest.py
"""
from __future__ import annotations

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
from collections import deque
from datetime import datetime
from py_trees.common import Status
from pyrplidar import PyRPlidar

from src.behaviors.MaxineBehavior import MaxineBehavior
from src.types.RobotModes import RobotMode
from src.types.MovementDirection import MovementDirection
from src.action_managers.VelocityManager import VelocityConfig

# Import existing working coordinate verification system
from .CoordinateVerificationLidarTest import CoordinateVerificationLidarTest

# Import potential field navigation
from .PotentialFieldNavigation import PotentialFieldNavigator, PotentialFieldConfig, Vector2D

# EXACT IMPORTS from detection_consistency_test.py
try:
    import depthai as dai
    import cv2
    import numpy as np
    DEPTHAI_AVAILABLE = True
except ImportError as e:
    print(f"❌ Missing required libraries: {e}")
    DEPTHAI_AVAILABLE = False


class ExactStandaloneDetectionSystem:
    """
    EXACT COPY of detection_consistency_test.py camera and detection system
    This REPLACES the OptimizedDetectionSystem but keeps same interface
    """
    
    def __init__(self):
        # EXACT COPY: All variables from detection_consistency_test.py
        self.device = None
        self.pipeline = None
        self.detection_queue = None
        self.preview_queue = None
        self.has_detection = False
        self.camera_initialized = False
        self.camera_error_message = ""
        
        # EXACT COPY: Camera settings from detection_consistency_test.py
        self.camera_resolution_width = 300
        self.camera_resolution_height = 300
        self.camera_hfov_degrees = 114
        self.target_fps = 25
        
        # EXACT COPY: Detection settings from detection_consistency_test.py
        self.confidence_threshold = 0.4  # Add this for compatibility
        self.detection_skip_frames = 1
        self.frame_counter = 0
        
        # EXACT COPY: Z-depth smoothing from detection_consistency_test.py
        self.z_depth_smoother = deque(maxlen=5)
        self.confidence_weights = deque(maxlen=5)
        self.smoothed_z_depth = 0
        self.depth_trust_threshold = 0.6
    
    def check_camera_connection(self):
        """EXACT COPY from detection_consistency_test.py"""
        try:
            devices = dai.Device.getAllAvailableDevices()
            return len(devices) > 0, f"Found {len(devices)} device(s)"
        except Exception as e:
            return False, f"Device detection error: {str(e)}"
    
    def create_pipeline(self):
        """EXACT COPY from detection_consistency_test.py"""
        try:
            pipeline = dai.Pipeline()
            
            local_blob_path = "./mobilenet-ssd_openvino_2021.4_5shave.blob"
            if not os.path.exists(local_blob_path):
                raise Exception(f"Blob file not found: {local_blob_path}")
            
            # EXACT COPY: Working camera setup from detection_consistency_test.py
            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            mono_left.setFps(self.target_fps)
            
            mono_right = pipeline.create(dai.node.MonoCamera)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            mono_right.setFps(self.target_fps)
            
            # EXACT COPY: Working ImageManip setup from detection_consistency_test.py
            manip_nn = pipeline.create(dai.node.ImageManip)
            manip_nn.initialConfig.setResize(300, 300)
            manip_nn.initialConfig.setKeepAspectRatio(False)
            manip_nn.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
            mono_right.out.link(manip_nn.inputImage)
            
            # EXACT COPY: Working stereo depth settings from detection_consistency_test.py
            depth = pipeline.create(dai.node.StereoDepth)
            depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
            depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
            depth.initialConfig.setConfidenceThreshold(180)
            depth.setLeftRightCheck(True)
            depth.setSubpixel(False)
            depth.setDepthAlign(dai.CameraBoardSocket.CAM_C)
            
            mono_left.out.link(depth.left)
            mono_right.out.link(depth.right)
            
            # EXACT COPY: Working detection network settings from detection_consistency_test.py
            detection_nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
            detection_nn.setConfidenceThreshold(0.4)  # EXACT same threshold
            detection_nn.setBlobPath(local_blob_path)
            detection_nn.setBoundingBoxScaleFactor(0.5)  # EXACT same scale
            detection_nn.setDepthLowerThreshold(100)    # EXACT same threshold
            detection_nn.setDepthUpperThreshold(8000)   # EXACT same threshold
            
            manip_nn.out.link(detection_nn.input)
            depth.depth.link(detection_nn.inputDepth)
            
            # Outputs
            detection_out = pipeline.create(dai.node.XLinkOut)
            detection_out.setStreamName("detections")
            detection_nn.out.link(detection_out.input)
            
            return pipeline
        except Exception:
            return None
    
    def initialize(self):
        """EXACT COPY from detection_consistency_test.py initialize_camera - SAME INTERFACE as OptimizedDetectionSystem"""
        if not DEPTHAI_AVAILABLE:
            print("⚠️ DepthAI not available - using fallback")
            return False
            
        try:
            connected, message = self.check_camera_connection()
            if not connected:
                self.camera_error_message = message
                print(f"⚠️ {message}")
                return False
            
            self.pipeline = self.create_pipeline()
            if not self.pipeline:
                self.camera_error_message = "Pipeline creation failed"
                return False
            
            try:
                self.device = dai.Device(self.pipeline)
            except Exception as e:
                self.camera_error_message = f"Device connection failed: {str(e)}"
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
            except Exception:
                self.detection_queue = None
                self.has_detection = False
            
            # Test camera frames (simplified)
            time.sleep(2)
            self.camera_initialized = True
            print("✅ EXACT standalone detection system initialized")
            return True
            
        except Exception as e:
            self.camera_error_message = f"Camera initialization error: {str(e)}"
            print(f"❌ Camera initialization error: {e}")
            return False
    
    def smooth_z_depth(self, raw_z_depth, confidence):
        """EXACT COPY from detection_consistency_test.py"""
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
        """EXACT COPY from detection_consistency_test.py process_detections logic - SAME INTERFACE as OptimizedDetectionSystem"""
        self.frame_counter += 1
        
        if self.frame_counter % self.detection_skip_frames != 0:
            return None
        
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
                    'x_pixels': 0,  # Will be calculated by calling code
                    'y_pixels': 0   # Will be calculated by calling code
                },
                'bounding_box': {
                    'xmin': bbox_xmin,
                    'ymin': bbox_ymin,
                    'xmax': bbox_xmax,
                    'ymax': bbox_ymax
                },
                'detection_method': 'EXACT_STANDALONE_PIPELINE'
            }
            
        except Exception:
            return None
    
    def shutdown(self):
        """Add shutdown method to match parent's expected interface"""
        try:
            if self.device:
                self.device.close()
        except Exception:
            pass


class PotentialFieldVisualizationTest(CoordinateVerificationLidarTest):
    """
    Potential Field Visualization Test - Uses EXACT standalone camera pipeline
    ONLY replaces detection system creation - everything else uses working parent methods
    """
    
    def __init__(self):
        # py_trees Lifecycle: __init__ - Just setup variables, NO hardware initialization
        super().__init__()
        
        # Override behavior name and CSV filename
        self.name = "Potential Field Visualization Test"
        self.csv_log_filename = "POTENTIAL_FIELD_NAVIGATION.csv"
        
        # Potential field system 
        self.potential_field_config = PotentialFieldConfig(
            attractive_strength=2.0,
            repulsive_strength=1.5,
            target_distance=1500.0,
            repulsive_max_distance=800.0,
            max_linear_speed=0.6,
            max_angular_speed=0.4,
            movement_threshold=0.1,
            stop_at_target=True
        )
        self.navigator = PotentialFieldNavigator(self.potential_field_config)
        
        # Grid visualization
        self.grid_size = 300 
        self.grid_extent = 2400 
        self.potential_field_cache = {}
        self.last_field_calculation = 0.0
        self.field_calculation_interval = 0.3
        
        # Navigation state
        self.navigation_enabled = True
        self.last_navigation_metrics = {}
        
        # Calibration system 
        self.calibration_mode = True
        self.adjustment_x = 0 
        self.adjustment_y = 0 
        self.adjustment_step = 10 
        self.offset_file_counter = 1 
        
        # Calibration tracking
        self.original_person_x = 0 
        self.original_person_y = 0
        self.last_save_message = "" 
        self.last_save_time = 0 
        
        # Applied offset
        self.applied_offset_x = 0 
        self.applied_offset_y = 0 
        self.offset_calculated = False 
    
    def setup(self):
        """py_trees Lifecycle: setup() - Prepare for potential execution, NO hardware initialization"""
        # Call parent setup 
        super().setup()
        return True
    
    def initialize_components(self):
        """Override ONLY the detection system creation - use parent for everything else"""
        if self.initialized:
            return True
        
        try:
            # Call parent's display setup
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
            pygame.display.set_caption("POTENTIAL FIELD VISUALIZATION - EXACT STANDALONE PIPELINE")
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            self.scale = min(display_info.current_w, display_info.current_h) // 7
            self.draw_clean_interface()
            pygame.display.flip()
            
            # REPLACE detection system with EXACT standalone system (ONLY CHANGE)
            print("🎯 Initializing EXACT standalone detection system...")
            self.detection_system = ExactStandaloneDetectionSystem()
            if self.detection_system.initialize():
                print("✅ EXACT standalone detection system ready")
            else:
                print("❌ EXACT standalone detection system failed - no fallback")
                return False
            
            # Use parent's working methods for everything else
            self.start_stable_lidar()
            self.initialize_csv_log()
            if self.head_tracking_enabled:
                self.initialize_head_tracker()
            
            self.initialized = True
            return True
            
        except Exception as e:
            print(f"❌ Component initialization failed: {e}")
            return False
    
    def update(self) -> Status:
        """Enhanced update - adds calibration input handling to parent's working update logic"""
        try:
            if not self.initialized:
                if not self.initialize_components():
                    return Status.FAILURE
            
            self.update_counter += 1
            
            # Handle calibration input events BEFORE calling parent update
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        # ALWAYS RETURN to IDLE mode when exiting
                        robot = self.get_robot()
                        robot.set_mode(RobotMode.IDLE)
                        return Status.SUCCESS
                    else:
                        self.handle_calibration_input(event)
                elif event.type == pygame.QUIT:
                    robot = self.get_robot()
                    robot.set_mode(RobotMode.IDLE)
                    return Status.SUCCESS
            
            # Call parent's WORKING update logic - preserves ALL functionality
            return super().update()
            
        except Exception as e:
            print(f"⚠️ Update error: {e}")
            robot = self.get_robot()
            robot.set_mode(RobotMode.IDLE)
            return Status.FAILURE
    
    def draw_person_detection(self, person_data=None):
        """Enhanced person detection drawing - adds calibration markers to parent's working drawing"""
        # Call parent's WORKING person detection drawing first
        result = super().draw_person_detection(person_data)
        
        # Add potential field visualization if person detected
        if person_data is None:
            person_data = self.get_person_detection()
        
        if person_data and self.navigation_enabled:
            self.draw_potential_field_overlay(person_data)
            self.draw_potential_field_grid()
        
        # Add calibration markers on top of existing working display
        try:
            if not person_data:
                return result
            
            # Display current applied offset at top of screen
            font = pygame.font.Font(None, 36)
            potential_status = "ON" if self.navigation_enabled else "OFF"
            offset_text = font.render(f"EXACT STANDALONE PIPELINE | Potential Field: {potential_status} | Offset: X:{self.applied_offset_x} Y:{self.applied_offset_y}", True, (255, 255, 255))
            offset_rect = offset_text.get_rect(center=(self.screen.get_width() // 2, 50))
            self.screen.blit(offset_text, offset_rect)
            
            # Get person position in screen coordinates
            bbox_center = person_data['bbox_center']
            original_x = bbox_center['x_pixels']
            original_y = bbox_center['y_pixels']
            
            # Store original position for calibration
            self.original_person_x = original_x
            self.original_person_y = original_y
            
            # Calculate calibration marker positions
            # Orange circle: Original camera detection
            orange_x = original_x
            orange_y = original_y
            
            # Blue circle: Applied offset correction
            blue_x = original_x + self.applied_offset_x
            blue_y = original_y + self.applied_offset_y
            
            # Green circle: Manual adjustment (for calibration)
            green_x = original_x + self.adjustment_x
            green_y = original_y + self.adjustment_y
            
            # Draw calibration circles (small so they don't interfere)
            pygame.draw.circle(self.screen, (255, 165, 0), (orange_x, orange_y), 8, 2)  # Orange: Original
            pygame.draw.circle(self.screen, (0, 100, 255), (blue_x, blue_y), 6, 2)      # Blue: Corrected
            pygame.draw.circle(self.screen, (0, 255, 0), (green_x, green_y), 4, 2)      # Green: Manual adjustment
            
            # Show save message if recent
            if self.last_save_message and (time.time() - self.last_save_time) < 3.0:
                save_font = pygame.font.Font(None, 32)
                save_text = save_font.render(self.last_save_message, True, (0, 255, 0))
                save_rect = save_text.get_rect(center=(self.screen.get_width() // 2, 120))
                self.screen.blit(save_text, save_rect)
            
            # Draw calibration instructions 
            self.draw_calibration_instructions()
            
        except Exception:
            pass
        
        return result
    
    def draw_calibration_instructions(self):
        """Draw calibration instructions at bottom of screen"""
        try:
            font = pygame.font.Font(None, 24)
            instructions = [
                "CALIBRATION: WASD=Move Green, X=Save, C=Apply | Orange=Original, Blue=Corrected, Green=Manual"
            ]
            
            y_start = self.screen.get_height() - 30
            for i, instruction in enumerate(instructions):
                text = font.render(instruction, True, (150, 150, 150))
                text_rect = text.get_rect(center=(self.screen.get_width() // 2, y_start + i * 25))
                self.screen.blit(text, text_rect)
        except Exception:
            pass
    
    def handle_calibration_input(self, event):
        """Handle calibration keyboard input"""
        if event.type != pygame.KEYDOWN:
            return
        
        if event.key == pygame.K_w:  # W - Move adjustment up
            self.adjustment_y -= self.adjustment_step
        elif event.key == pygame.K_s:  # S - Move adjustment down
            self.adjustment_y += self.adjustment_step
        elif event.key == pygame.K_a:  # A - Move adjustment left
            self.adjustment_x -= self.adjustment_step
        elif event.key == pygame.K_d:  # D - Move adjustment right
            self.adjustment_x += self.adjustment_step
        elif event.key == pygame.K_x:  # X - Save current offset
            self.save_calibration_offset()
        elif event.key == pygame.K_c:  # C - Calculate and apply offset
            self.calculate_and_apply_offset()
        elif event.key == pygame.K_f:  # F - Toggle potential field display
            self.navigation_enabled = not self.navigation_enabled
            self.last_save_message = f"Potential Field: {'ON' if self.navigation_enabled else 'OFF'}"
            self.last_save_time = time.time()
    
    def save_calibration_offset(self):
        """Save current calibration offset to CSV file"""
        try:
            # Get current head angle if available
            head_angle = self.get_head_angle_degrees()
            
            filename = f"offset{self.offset_file_counter}.csv"
            
            # Create CSV if it doesn't exist
            if not os.path.exists(filename):
                with open(filename, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(['timestamp', 'head_angle_deg', 'adjustment_x_pixels', 'adjustment_y_pixels', 
                                   'original_x', 'original_y', 'notes'])
            
            # Append calibration data
            with open(filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    time.time(),
                    head_angle,
                    self.adjustment_x,
                    self.adjustment_y,
                    self.original_person_x,
                    self.original_person_y,
                    f"EXACT pipeline calibration at head angle {head_angle:.1f}°"
                ])
            
            self.last_save_message = f"Saved to {filename}"
            self.last_save_time = time.time()
            self.offset_file_counter += 1
            
        except Exception as e:
            self.last_save_message = f"Save failed: {e}"
            self.last_save_time = time.time()
    
    def calculate_and_apply_offset(self):
        """Calculate offset from saved calibration files and apply"""
        try:
            all_x_offsets = []
            all_y_offsets = []
            
            # Read all offset CSV files
            for i in range(1, self.offset_file_counter):
                filename = f"offset{i}.csv"
                if os.path.exists(filename):
                    with open(filename, 'r') as csvfile:
                        reader = csv.DictReader(csvfile)
                        for row in reader:
                            try:
                                x_offset = float(row['adjustment_x_pixels'])
                                y_offset = float(row['adjustment_y_pixels'])
                                all_x_offsets.append(x_offset)
                                all_y_offsets.append(y_offset)
                            except (ValueError, KeyError):
                                continue
            
            if all_x_offsets and all_y_offsets:
                # Calculate average offset
                self.applied_offset_x = int(statistics.mean(all_x_offsets))
                self.applied_offset_y = int(statistics.mean(all_y_offsets))
                self.offset_calculated = True
                
                self.last_save_message = f"Applied offset: X:{self.applied_offset_x}, Y:{self.applied_offset_y}"
                self.last_save_time = time.time()
            else:
                self.last_save_message = "No calibration data found"
                self.last_save_time = time.time()
                
        except Exception as e:
            self.last_save_message = f"Offset calculation failed: {e}"
            self.last_save_time = time.time()
    
    def draw_potential_field_overlay(self, person_data):
        """Draw potential field visualization overlay"""
        try:
            current_time = time.time()
            
            # Only update field calculation every 300ms for performance
            if current_time - self.last_field_calculation > self.field_calculation_interval:
                self.calculate_potential_field_cache(person_data)
                self.last_field_calculation = current_time
            
            # Draw cached field
            self.draw_cached_potential_field()
            
        except Exception as e:
            print(f"⚠️ Potential field drawing error: {e}")
    
    def calculate_potential_field_cache(self, person_data):
        """Calculate and cache potential field for performance"""
        try:
            self.potential_field_cache.clear()
            
            # Get robot position (center of display)
            robot_x = self.center_x
            robot_y = self.center_y
            
            # Get person position (target)
            person_x = person_data['bbox_center']['x_pixels']
            person_y = person_data['bbox_center']['y_pixels']
            
            # Get obstacles from LiDAR
            obstacles = self.lidar_system.get_display_obstacles() if self.lidar_system else []
            
            # Calculate field for grid points
            for x in range(robot_x - self.grid_extent, robot_x + self.grid_extent, self.grid_size):
                for y in range(robot_y - self.grid_extent, robot_y + self.grid_extent, self.grid_size):
                    if 0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height():
                        # Calculate forces at this grid point
                        attractive_force = self.calculate_attractive_force(x, y, person_x, person_y)
                        repulsive_force = self.calculate_repulsive_force(x, y, obstacles)
                        
                        total_force = Vector2D(
                            attractive_force.x + repulsive_force.x,
                            attractive_force.y + repulsive_force.y
                        )
                        
                        self.potential_field_cache[(x, y)] = total_force
        except Exception as e:
            print(f"⚠️ Potential field calculation error: {e}")
    
    def calculate_attractive_force(self, x, y, target_x, target_y):
        """Calculate attractive force towards target"""
        try:
            dx = target_x - x
            dy = target_y - y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 1:
                return Vector2D(0, 0)
            
            # Simple attractive force - scales with distance
            force_magnitude = min(self.potential_field_config.attractive_strength, distance * 0.01)
            return Vector2D(dx/distance * force_magnitude, dy/distance * force_magnitude)
        except Exception:
            return Vector2D(0, 0)
    
    def calculate_repulsive_force(self, x, y, obstacles):
        """Calculate repulsive force from obstacles"""
        try:
            total_force = Vector2D(0, 0)
            
            for angle_deg, distance_mm in obstacles:
                # Convert obstacle to screen coordinates
                obstacle_x, obstacle_y = self.polar_to_screen(angle_deg, distance_mm)
                
                dx = x - obstacle_x
                dy = y - obstacle_y
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance < self.potential_field_config.repulsive_max_distance and distance > 1:
                    # Inverse square law for repulsive force
                    force_magnitude = self.potential_field_config.repulsive_strength / (distance * distance)
                    force_magnitude = min(force_magnitude, 10.0)  # Cap maximum force
                    
                    total_force.x += dx/distance * force_magnitude
                    total_force.y += dy/distance * force_magnitude
            
            return total_force
        except Exception:
            return Vector2D(0, 0)
    
    def polar_to_screen(self, angle_deg, distance_mm):
        """Convert polar coordinates (LiDAR) to screen coordinates"""
        try:
            # Convert to radians
            angle_rad = math.radians(angle_deg)
            
            # Convert distance to pixels
            pixel_distance = distance_mm * self.scale / 1000
            
            # Calculate screen position relative to center (robot position)
            x = self.center_x + pixel_distance * math.sin(angle_rad)
            y = self.center_y - pixel_distance * math.cos(angle_rad)  # Y axis is flipped
            
            return int(x), int(y)
        except Exception:
            return self.center_x, self.center_y
    
    def draw_cached_potential_field(self):
        """Draw the cached potential field as force vectors"""
        try:
            for (x, y), force in self.potential_field_cache.items():
                # Calculate force magnitude
                force_magnitude = math.sqrt(force.x*force.x + force.y*force.y)
                
                if force_magnitude > 0.1:
                    # Scale force vector for display
                    scale_factor = 30
                    end_x = x + force.x * scale_factor
                    end_y = y + force.y * scale_factor
                    
                    # Enhanced color coding based on force type and magnitude
                    # Determine if force is more attractive (towards person) or repulsive (away from obstacles)
                    if force_magnitude < 1.0:
                        color = (0, 255, 0)      # Green for low force
                    elif force_magnitude < 3.0:
                        color = (100, 255, 100)  # Light green for low-medium force
                    elif force_magnitude < 6.0:
                        color = (255, 255, 0)    # Yellow for medium force
                    elif force_magnitude < 10.0:
                        color = (255, 165, 0)    # Orange for high force
                    else:
                        color = (255, 0, 0)      # Red for very high force
                    
                    # Draw force vector as line with arrowhead
                    pygame.draw.line(self.screen, color, (x, y), (end_x, end_y), 2)
                    
                    # Draw small arrowhead for direction clarity
                    if force_magnitude > 0.5:
                        self.draw_arrow_head(x, y, end_x, end_y, color)
        except Exception as e:
            print(f"⚠️ Force vector drawing error: {e}")
    
    def draw_arrow_head(self, start_x, start_y, end_x, end_y, color):
        """Draw small arrowhead at end of force vector"""
        try:
            # Calculate arrow direction
            dx = end_x - start_x
            dy = end_y - start_y
            length = math.sqrt(dx*dx + dy*dy)
            
            if length > 0:
                # Normalize direction
                dx /= length
                dy /= length
                
                # Calculate arrowhead points
                arrow_length = 8
                arrow_angle = math.pi / 6  # 30 degrees
                
                # Left arrow point
                left_x = end_x - arrow_length * (dx * math.cos(arrow_angle) + dy * math.sin(arrow_angle))
                left_y = end_y - arrow_length * (dy * math.cos(arrow_angle) - dx * math.sin(arrow_angle))
                
                # Right arrow point
                right_x = end_x - arrow_length * (dx * math.cos(-arrow_angle) + dy * math.sin(-arrow_angle))
                right_y = end_y - arrow_length * (dy * math.cos(-arrow_angle) - dx * math.sin(-arrow_angle))
                
                # Draw arrowhead lines
                pygame.draw.line(self.screen, color, (end_x, end_y), (left_x, left_y), 2)
                pygame.draw.line(self.screen, color, (end_x, end_y), (right_x, right_y), 2)
        except Exception:
            pass
    
    def draw_potential_field_grid(self):
        """Draw potential field grid overlay"""
        try:
            # Draw grid lines to show force calculation points
            grid_color = (100, 100, 100)  # Dark gray
            
            # Draw vertical grid lines
            for x in range(self.center_x - self.grid_extent, self.center_x + self.grid_extent, self.grid_size):
                if 0 <= x < self.screen.get_width():
                    pygame.draw.line(self.screen, grid_color, 
                                   (x, self.center_y - self.grid_extent), 
                                   (x, self.center_y + self.grid_extent), 1)
            
            # Draw horizontal grid lines
            for y in range(self.center_y - self.grid_extent, self.center_y + self.grid_extent, self.grid_size):
                if 0 <= y < self.screen.get_height():
                    pygame.draw.line(self.screen, grid_color, 
                                   (self.center_x - self.grid_extent, y), 
                                   (self.center_x + self.grid_extent, y), 1)
            
            # Draw grid boundary
            boundary_color = (150, 150, 150)
            boundary_rect = pygame.Rect(
                self.center_x - self.grid_extent, 
                self.center_y - self.grid_extent,
                self.grid_extent * 2, 
                self.grid_extent * 2
            )
            pygame.draw.rect(self.screen, boundary_color, boundary_rect, 2)
            
            # Add grid info text
            font = pygame.font.Font(None, 28)
            grid_text = font.render(f"Potential Field Grid: {self.grid_size}mm spacing", True, (200, 200, 200))
            text_rect = grid_text.get_rect(center=(self.screen.get_width() // 2, 90))
            self.screen.blit(grid_text, text_rect)
            
        except Exception as e:
            print(f"⚠️ Grid drawing error: {e}")
    
    def draw_calibration_instructions(self):
        """Draw calibration instructions at bottom of screen"""
        try:
            font = pygame.font.Font(None, 24)
            instructions = [
                "POTENTIAL FIELD: Green=Low Force, Yellow=Medium, Orange=High, Red=Very High | F=Toggle ON/OFF",
                "CALIBRATION: WASD=Move Green, X=Save, C=Apply | Orange=Original, Blue=Corrected, Green=Manual"
            ]
            
            y_start = self.screen.get_height() - 60
            for i, instruction in enumerate(instructions):
                text = font.render(instruction, True, (150, 150, 150))
                text_rect = text.get_rect(center=(self.screen.get_width() // 2, y_start + i * 25))
                self.screen.blit(text, text_rect)
        except Exception:
            pass
    
    def terminate(self, new_status: Status):
        """py_trees Lifecycle: terminate() - Called when behavior STOPS running"""
        try:
            # Call parent termination first (preserves all cleanup)
            super().terminate(new_status)
        except Exception as e:
            print(f"⚠️ Termination error: {e}")