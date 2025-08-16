#!/usr/bin/env python3
"""
Optimized Potential Field Visualization Test - EXTENDS CoordinateVerificationLidarTest
High-performance version with minimal overhead and preserved head tracking

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
import cv2
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


class PotentialFieldVisualizationTest(CoordinateVerificationLidarTest):
    """
    OPTIMIZED potential field visualization with Camera-LiDAR Calibration System
    
    CALIBRATION MODE:
    - Orange circle: Original camera-detected person position  
    - Blue circle: Offset-corrected position (should align with lidar)
    - Green circle: Manual adjustment marker (use WASD to move)
    - White dots: LiDAR obstacle detections
    
    CONTROLS:
    - WASD: Move green marker to align with lidar detection
    - X: Save calibration offset to offsetN.csv  
    - C: Calculate and apply offset from saved calibration data
    - ESC: Exit to IDLE mode
    
    CALIBRATION PROCEDURE:
    1. Person stands at fixed distance (2-3 meters)
    2. Test at multiple head angles: 0°, +30°, -30°, +45°, -45°
    3. Use WASD to align green circle with lidar detection (white dots)
    4. Press X to save offset for each head angle
    5. Press C to calculate average offset and apply correction
    6. Blue circle should now align with lidar detections
    
    The system will display the current applied offset at the top of the screen.
    """
    
    def __init__(self):
        # Initialize the working base system first
        super().__init__()
        
        # Override behavior name and CSV filename
        self.name = "Potential Field Visualization Test"
        self.csv_log_filename = "POTENTIAL_FIELD_NAVIGATION.csv"
        
        # EXACT COPY: Camera system from detection_consistency_test.py (OPTIONAL enhancement)
        self.exact_device = None
        self.exact_pipeline = None
        self.exact_detection_queue = None
        self.exact_preview_queue = None
        self.exact_has_detection = False
        self.exact_camera_initialized = False
        self.exact_camera_error_message = ""
        
        # EXACT COPY: Camera settings from detection_consistency_test.py
        self.exact_camera_resolution_width = 300
        self.exact_camera_resolution_height = 300
        self.exact_camera_hfov_degrees = 114
        self.exact_target_fps = 25
        
        # EXACT COPY: Detection settings from detection_consistency_test.py
        self.exact_detection_skip_frames = 1
        self.exact_frame_counter = 0
        
        # EXACT COPY: Z-depth smoothing from detection_consistency_test.py
        self.exact_z_depth_smoother = deque(maxlen=5)
        self.exact_confidence_weights = deque(maxlen=5)
        self.exact_smoothed_z_depth = 0
        self.exact_depth_trust_threshold = 0.6
        
        # PERFORMANCE OPTIMIZATIONS
        self.display_update_rate = 6  # Update display less frequently (was 3)
        
        # Simplified potential field system (NEW)
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
        
        # OPTIMIZED Grid visualization (3x3 = 300mm spacing, not 100mm)
        self.grid_size = 300  # 300mm grid cells (much less dense)
        self.grid_extent = 2400  # Smaller extent for performance
        self.potential_field_cache = {}
        self.last_field_calculation = 0.0
        self.field_calculation_interval = 0.3  # Update field every 300ms for responsiveness
        
        # Minimal navigation state tracking
        self.navigation_enabled = True
        self.last_navigation_metrics = {}
        
        # Manual calibration system for camera-lidar alignment
        self.calibration_mode = True
        self.adjustment_x = 0  # Pixel offset from original camera position
        self.adjustment_y = 0  # Pixel offset from original camera position
        self.adjustment_step = 10  # Pixels per keypress
        self.offset_file_counter = 1  # For offset1.csv, offset2.csv, etc.
        self.original_person_x = 0  # Store original camera detection
        self.original_person_y = 0
        self.last_save_message = ""  # Display save confirmation
        self.last_save_time = 0  # For timed display of save message
        
        # Applied offset for camera-lidar alignment (calculated from calibration data)
        self.applied_offset_x = 0  # Current offset being applied to camera detections
        self.applied_offset_y = 0  # Current offset being applied to camera detections
        self.offset_calculated = False  # Whether we have calculated an offset from data
    
    def check_exact_camera_connection(self):
        """EXACT COPY from detection_consistency_test.py"""
        try:
            devices = dai.Device.getAllAvailableDevices()
            return len(devices) > 0, f"Found {len(devices)} device(s)"
        except Exception as e:
            return False, f"Device detection error: {str(e)}"
    
    def create_exact_pipeline(self):
        """EXACT COPY from detection_consistency_test.py"""
        try:
            pipeline = dai.Pipeline()
            
            local_blob_path = "./mobilenet-ssd_openvino_2021.4_5shave.blob"
            if not os.path.exists(local_blob_path):
                raise Exception(f"Blob file not found: {local_blob_path}")
            
            # EXACT COPY: Working camera setup
            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            mono_left.setFps(self.exact_target_fps)
            
            mono_right = pipeline.create(dai.node.MonoCamera)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            mono_right.setFps(self.exact_target_fps)
            
            # EXACT COPY: Working ImageManip setup
            manip_nn = pipeline.create(dai.node.ImageManip)
            manip_nn.initialConfig.setResize(300, 300)
            manip_nn.initialConfig.setKeepAspectRatio(False)
            manip_nn.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
            mono_right.out.link(manip_nn.inputImage)
            
            # EXACT COPY: Working stereo depth settings
            depth = pipeline.create(dai.node.StereoDepth)
            depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
            depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
            depth.initialConfig.setConfidenceThreshold(180)
            depth.setLeftRightCheck(True)
            depth.setSubpixel(False)
            depth.setDepthAlign(dai.CameraBoardSocket.CAM_C)
            
            mono_left.out.link(depth.left)
            mono_right.out.link(depth.right)
            
            # EXACT COPY: Working detection network settings
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
    
    def initialize_exact_camera(self):
        """EXACT COPY from detection_consistency_test.py"""
        if not DEPTHAI_AVAILABLE:
            self.exact_camera_error_message = "DepthAI not available"
            return False
            
        try:
            connected, message = self.check_exact_camera_connection()
            if not connected:
                self.exact_camera_error_message = message
                return False
            
            self.exact_pipeline = self.create_exact_pipeline()
            if not self.exact_pipeline:
                self.exact_camera_error_message = "Pipeline creation failed"
                return False
            
            try:
                self.exact_device = dai.Device(self.exact_pipeline)
            except Exception as e:
                self.exact_camera_error_message = f"Device connection failed: {str(e)}"
                return False
            
            # Optimize device
            try:
                if hasattr(self.exact_device, 'setLogLevel'):
                    self.exact_device.setLogLevel(dai.LogLevel.WARN)
                self.exact_device.setIrLaserDotProjectorIntensity(900)
            except Exception:
                pass
            
            # Get queues
            try:
                self.exact_detection_queue = self.exact_device.getOutputQueue("detections", maxSize=4, blocking=False)
                self.exact_has_detection = True
            except Exception:
                self.exact_detection_queue = None
                self.exact_has_detection = False
            
            time.sleep(2)
            self.exact_camera_initialized = True
            print("✅ Exact camera system initialized")
            return True
        except Exception as e:
            self.exact_camera_error_message = f"Camera initialization error: {str(e)}"
            print(f"⚠️ Exact camera initialization failed: {e}")
            return False
    
    def exact_smooth_z_depth(self, raw_z_depth, confidence):
        """EXACT COPY from detection_consistency_test.py"""
        try:
            self.exact_z_depth_smoother.append(raw_z_depth)
            self.exact_confidence_weights.append(confidence)
            
            if len(self.exact_z_depth_smoother) < 2:
                self.exact_smoothed_z_depth = raw_z_depth
                return raw_z_depth
            
            total_weight = 0
            weighted_sum = 0
            
            for i, (z_val, conf) in enumerate(zip(self.exact_z_depth_smoother, self.exact_confidence_weights)):
                recency_weight = (i + 1) / len(self.exact_z_depth_smoother)
                confidence_weight = max(0.1, conf)
                combined_weight = recency_weight * confidence_weight
                
                weighted_sum += z_val * combined_weight
                total_weight += combined_weight
            
            smoothed = weighted_sum / total_weight if total_weight > 0 else raw_z_depth
            
            if confidence < self.exact_depth_trust_threshold and len(self.exact_z_depth_smoother) > 1:
                prev_z = self.exact_z_depth_smoother[-2]
                max_jump = 500
                if abs(smoothed - prev_z) > max_jump:
                    blend_factor = confidence / self.exact_depth_trust_threshold
                    smoothed = prev_z + (smoothed - prev_z) * blend_factor
            
            self.exact_smoothed_z_depth = smoothed
            return smoothed
        except Exception:
            return raw_z_depth

    def get_person_detection(self):
        """OVERRIDE parent to use EXACT detection_consistency_test.py system when available"""
        # Try exact camera system first if available
        if self.exact_has_detection and self.exact_detection_queue and self.exact_camera_initialized:
            try:
                self.exact_frame_counter += 1
                if self.exact_frame_counter % self.exact_detection_skip_frames == 0:
                    
                    detections = self.exact_detection_queue.tryGet()
                    if detections:
                        person_detections = [det for det in detections.detections if det.label == 15]
                        if person_detections:
                            closest_person = min(person_detections, key=lambda p: p.spatialCoordinates.z)
                            
                            x_camera = closest_person.spatialCoordinates.x
                            y_camera = closest_person.spatialCoordinates.y
                            raw_z_depth = closest_person.spatialCoordinates.z
                            confidence = closest_person.confidence
                            
                            if raw_z_depth > 0 and raw_z_depth <= 15000:
                                smoothed_z_depth = self.exact_smooth_z_depth(raw_z_depth, confidence)
                                
                                # Calculate X midpoint for consistency tracking
                                bbox_xmin = closest_person.xmin
                                bbox_xmax = closest_person.xmax
                                bbox_ymin = closest_person.ymin
                                bbox_ymax = closest_person.ymax
                                
                                x_midpoint_normalized = (bbox_xmin + bbox_xmax) / 2.0
                                y_midpoint_normalized = (bbox_ymin + bbox_ymax) / 2.0
                                x_midpoint_pixels = int(x_midpoint_normalized * self.screen.get_width()) if self.screen else 0
                                y_midpoint_pixels = int(y_midpoint_normalized * self.screen.get_height()) if self.screen else 0
                                
                                # Format to match parent's expected format
                                return {
                                    'x_camera': x_camera,
                                    'y_camera': y_camera,
                                    'z_camera': smoothed_z_depth,
                                    'confidence': confidence,
                                    'bbox_center': {
                                        'x_normalized': x_midpoint_normalized,
                                        'y_normalized': y_midpoint_normalized,
                                        'x_pixels': x_midpoint_pixels,
                                        'y_pixels': y_midpoint_pixels
                                    },
                                    'bbox_xmin': bbox_xmin,
                                    'bbox_ymin': bbox_ymin,
                                    'bbox_xmax': bbox_xmax,
                                    'bbox_ymax': bbox_ymax,
                                    'detection_method': 'EXACT_CAMERA'
                                }
            except Exception as e:
                print(f"⚠️ Exact camera detection failed: {e}")
        
        # Fallback to parent's detection system
        detection = super().get_person_detection()
        if detection:
            detection['detection_method'] = 'FALLBACK'
        return detection

    def save_calibration_offset(self):
        """Save REAL calibration offset data to CSV file"""
        print(f"💾 FUNCTION CALLED: save_calibration_offset() - counter: {self.offset_file_counter}")
        
        try:
            # Get current working directory and create filename
            import os
            current_dir = os.getcwd()
            filename = f"offset{self.offset_file_counter}.csv"
            full_path = os.path.join(current_dir, filename)
            
            print(f"💾 Creating REAL calibration file: {full_path}")
            
            # Get current person detection data
            person_data = self._cached_person_data
            if not person_data:
                print("❌ No person data available for calibration")
                self.last_save_message = "ERROR: No person data"
                self.last_save_time = time.time()
                return
            
            # Calculate offset between original camera position and adjusted position
            offset_x = self.adjustment_x
            offset_y = self.adjustment_y
            
            # Get head angle
            try:
                head_angle_deg = self.get_head_angle_degrees()
                print(f"📐 Head angle: {head_angle_deg}")
            except Exception as e:
                print(f"⚠️ Head angle error: {e}")
                head_angle_deg = 0.0
            
            # Get person distance and position
            x_camera = person_data['x_camera']
            z_camera = person_data['z_camera']
            confidence = person_data.get('confidence', 0.0)
            
            # Get closest lidar obstacle for comparison
            closest_lidar_distance = 0
            closest_lidar_angle = 0
            if self.lidar_system:
                obstacles = self.lidar_system.get_display_obstacles()
                if obstacles:
                    # Find closest obstacle
                    closest = min(obstacles, key=lambda obs: obs[1])
                    closest_lidar_angle, closest_lidar_distance = closest
                    print(f"📐 Closest lidar: angle={closest_lidar_angle}, dist={closest_lidar_distance}")
            
            # Write REAL calibration data to CSV
            with open(full_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'timestamp', 'head_angle_deg', 'offset_x_pixels', 'offset_y_pixels',
                    'original_camera_x_pixels', 'original_camera_y_pixels',
                    'adjusted_camera_x_pixels', 'adjusted_camera_y_pixels',
                    'camera_x_mm', 'camera_z_mm', 'camera_confidence',
                    'closest_lidar_angle_deg', 'closest_lidar_distance_mm',
                    'person_distance_mm', 'screen_center_x', 'screen_center_y'
                ])
                writer.writerow([
                    time.time(), head_angle_deg, offset_x, offset_y,
                    self.original_person_x, self.original_person_y,
                    self.original_person_x + offset_x, self.original_person_y + offset_y,
                    x_camera, z_camera, confidence,
                    closest_lidar_angle, closest_lidar_distance,
                    z_camera, self.center_x, self.center_y
                ])
            
            print(f"💾 REAL calibration data written to file")
            
            # Check if file exists and verify content
            if os.path.exists(full_path):
                file_size = os.path.getsize(full_path)
                print(f"✅ SUCCESS: Real calibration file created - {filename} ({file_size} bytes)")
                
                # Display offset information
                offset_magnitude = math.sqrt(offset_x**2 + offset_y**2)
                print(f"✅ Calibration data saved to {filename}")
                print(f"   Offset: ({offset_x:+d}, {offset_y:+d}) pixels, magnitude: {offset_magnitude:.1f}px")
                print(f"   Head angle: {head_angle_deg:.1f}°, Person distance: {z_camera:.0f}mm")
                print(f"   Camera pos: ({self.original_person_x}, {self.original_person_y})")
                print(f"   Lidar: {closest_lidar_angle:.1f}° at {closest_lidar_distance:.0f}mm")
                
                self.last_save_message = f"SAVED: {filename}"
                self.offset_file_counter += 1
            else:
                print(f"❌ FAILED: File not found - {full_path}")
                self.last_save_message = "ERROR: File not created"
            
            self.last_save_time = time.time()
            
        except Exception as e:
            print(f"❌ EXCEPTION in save_calibration_offset(): {e}")
            import traceback
            traceback.print_exc()
            self.last_save_message = f"ERROR: {str(e)}"
            self.last_save_time = time.time()
    
    def calculate_offset_from_data(self):
        """Calculate camera-lidar offset from saved calibration data"""
        try:
            print("🔍 Analyzing calibration data to calculate offset...")
            
            # Look for offset CSV files
            import os
            import glob
            
            csv_files = glob.glob("offset*.csv")
            if not csv_files:
                print("❌ No offset CSV files found")
                self.last_save_message = "ERROR: No offset files found"
                self.last_save_time = time.time()
                return False
            
            total_offset_x = 0
            total_offset_y = 0
            valid_samples = 0
            
            for csv_file in csv_files:
                try:
                    print(f"📄 Reading {csv_file}")
                    with open(csv_file, 'r') as f:
                        reader = csv.reader(f)
                        header = next(reader)  # Skip header
                        row = next(reader)     # Read data row
                        
                        # Check if this is real calibration data (has offset columns)
                        if 'offset_x_pixels' in header and 'offset_y_pixels' in header:
                            offset_x_idx = header.index('offset_x_pixels')
                            offset_y_idx = header.index('offset_y_pixels')
                            
                            offset_x = float(row[offset_x_idx])
                            offset_y = float(row[offset_y_idx])
                            
                            print(f"   Offset: ({offset_x:+.0f}, {offset_y:+.0f}) pixels")
                            
                            total_offset_x += offset_x
                            total_offset_y += offset_y
                            valid_samples += 1
                        else:
                            print(f"   Skipping {csv_file} - test data format")
                            
                except Exception as e:
                    print(f"⚠️ Error reading {csv_file}: {e}")
            
            if valid_samples > 0:
                # Calculate average offset
                avg_offset_x = total_offset_x / valid_samples
                avg_offset_y = total_offset_y / valid_samples
                
                # Apply the calculated offset
                self.applied_offset_x = int(round(avg_offset_x))
                self.applied_offset_y = int(round(avg_offset_y))
                self.offset_calculated = True
                
                print(f"✅ Calculated offset from {valid_samples} samples:")
                print(f"   Average offset: ({avg_offset_x:+.1f}, {avg_offset_y:+.1f}) pixels")
                print(f"   Applied offset: ({self.applied_offset_x:+d}, {self.applied_offset_y:+d}) pixels")
                
                self.last_save_message = f"OFFSET CALCULATED: ({self.applied_offset_x:+d}, {self.applied_offset_y:+d})"
                self.last_save_time = time.time()
                
                return True
            else:
                print("❌ No valid calibration data found")
                self.last_save_message = "ERROR: No valid calibration data"
                self.last_save_time = time.time()
                return False
                
        except Exception as e:
            print(f"❌ Error calculating offset: {e}")
            self.last_save_message = f"ERROR: {str(e)}"
            self.last_save_time = time.time()
            return False

    def set_manual_offset(self, offset_x, offset_y):
        """Manually set the camera-lidar offset (for testing purposes)"""
        self.applied_offset_x = offset_x
        self.applied_offset_y = offset_y
        self.offset_calculated = True
        print(f"🔧 Manual offset set: ({offset_x:+d}, {offset_y:+d}) pixels")
        self.last_save_message = f"MANUAL OFFSET: ({offset_x:+d}, {offset_y:+d})"
        self.last_save_time = time.time()

    def initialize_csv_log(self):
        """Enhanced CSV with navigation metrics - SAME columns as parent plus nav data"""
        try:
            if os.path.exists(self.csv_log_filename):
                os.remove(self.csv_log_filename)
                
            with open(self.csv_log_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    # EXISTING columns from parent
                    'timestamp', 'mode_time_elapsed',
                    'person_detected', 'person_x_camera', 'person_z_camera', 'person_confidence',
                    'person_robot_angle_deg', 'person_robot_distance', 'person_transform_confidence',
                    'head_angle_deg', 'head_tracking_active',
                    'lidar_obstacles_count', 'closest_lidar_angle_deg', 'closest_lidar_distance',
                    'person_lidar_angle_diff_deg', 'person_lidar_distance_diff', 'coordinates_aligned',
                    'alignment_score', 'detection_quality',
                    # NEW navigation columns (minimal)
                    'nav_total_force', 'nav_attractive_force', 'nav_repulsive_force',
                    'nav_movement_command', 'nav_decision', 'person_in_target_zone', 'obstacles_filtered'
                ])
            self.csv_initialized = True
            
            # Show user where calibration files will be saved
            import os
            current_dir = os.getcwd()
            print(f"📁 Calibration files will be saved to: {current_dir}")
            print(f"   Files: offset1.csv, offset2.csv, etc.")
            print(f"")
            print(f"🎯 CALIBRATION INSTRUCTIONS:")
            print(f"   1. Position person at 2-3 meters distance")
            print(f"   2. Start with head at 0° (centered)")
            print(f"   3. Use WASD to align GREEN circle with white LiDAR dots")
            print(f"   4. Press X to save calibration data")
            print(f"   5. Repeat at different head angles: ±30°, ±45°")
            print(f"   6. Press C to calculate and apply offset")
            print(f"   7. Blue circle should now align with LiDAR detections")
            print(f"")
            
        except Exception as e:
            print(f"⚠️ CSV log initialization failed: {e}")
    
    def initialise(self):
        """RESTORE working initialization + try exact camera"""
        # Call parent initialization (keeps all working functionality)
        super().initialise()
        
        # Try to initialize EXACT camera system as OPTIONAL enhancement
        try:
            if DEPTHAI_AVAILABLE:
                self.initialize_exact_camera()
                print("✅ Optional exact camera system available")
            else:
                print("⚠️ Exact camera system not available - using fallback")
        except Exception as e:
            print(f"⚠️ Exact camera system failed: {e} - using fallback")
        
        print("✅ Potential Field Visualization Test initialized")

    def draw_person_detection_no_text(self, person_data=None):
        """Draw person detection with calibration markers (no text to avoid clutter)"""
        if person_data is None:
            person_data = self.get_person_detection()
        if not person_data:
            return None
        
        try:
            # Display current applied offset at top of screen
            font = pygame.font.Font(None, 36)
            offset_text = font.render(f"Applied Offset: X:{self.applied_offset_x} Y:{self.applied_offset_y}", True, (255, 255, 255))
            offset_rect = offset_text.get_rect(center=(self.screen.get_width() // 2, 50))
            self.screen.blit(offset_text, offset_rect)
                
            x_camera = person_data['x_camera']
            z_camera = person_data['z_camera']
            
            if z_camera <= 0:
                return
            
            # Calculate person position on radar (same calculation as parent)
            person_angle_rad = math.atan2(x_camera, z_camera)
            person_angle_deg = math.degrees(person_angle_rad)
            
            # Convert to display coordinates
            display_angle_deg = 360 - person_angle_deg
            while display_angle_deg < 0:
                display_angle_deg += 360
            while display_angle_deg >= 360:
                display_angle_deg -= 360
            
            distance_m = z_camera / 1000.0
            display_angle_rad = math.radians(90 - display_angle_deg)
            
            person_x = self.center_x + int(distance_m * self.scale * math.cos(display_angle_rad))
            person_y = self.center_y - int(distance_m * self.scale * math.sin(display_angle_rad))
            
            # Store original position for calibration
            self.original_person_x = person_x
            self.original_person_y = person_y
            
            # Calculate offset-corrected position (this is what should align with lidar)
            corrected_x = person_x + self.applied_offset_x
            corrected_y = person_y + self.applied_offset_y
            
            # Draw original camera detection (orange circle - smaller)
            if (0 <= person_x < self.screen.get_width() and 0 <= person_y < self.screen.get_height()):
                pygame.draw.circle(self.screen, (255, 165, 0), (person_x, person_y), 6, 2)  # Orange outline
                pygame.draw.circle(self.screen, (255, 165, 0), (person_x, person_y), 2)      # Orange center
            
            # Draw offset-corrected position (BLUE circle - this should align with lidar)
            if (0 <= corrected_x < self.screen.get_width() and 0 <= corrected_y < self.screen.get_height()):
                pygame.draw.circle(self.screen, (0, 0, 255), (corrected_x, corrected_y), 8, 3)  # Blue outline (corrected)
                pygame.draw.circle(self.screen, (0, 0, 255), (corrected_x, corrected_y), 3)      # Blue center
            
            # Draw adjustable calibration marker (GREEN filled circle - for manual adjustment)
            adjusted_x = person_x + self.adjustment_x
            adjusted_y = person_y + self.adjustment_y
            
            if (0 <= adjusted_x < self.screen.get_width() and 0 <= adjusted_y < self.screen.get_height()):
                pygame.draw.circle(self.screen, (0, 255, 0), (adjusted_x, adjusted_y), 5)  # Green filled circle
                
        except Exception:
            pass
    
    def calculate_potential_field_grid(self):
        """OPTIMIZE potential field calculation - force numbers on grid"""
        try:
            current_time = time.time()
            if current_time - self.last_field_calculation < self.field_calculation_interval:
                return
            
            self.last_field_calculation = current_time
            self.potential_field_cache = {}
            
            # Get LiDAR obstacles
            obstacles = []
            if self.lidar_system:
                obstacles = self.lidar_system.get_display_obstacles()
            
            # Get person position (use corrected position for field calculation)
            person_screen_x = None
            person_screen_y = None
            if self._cached_person_data:
                x_camera = self._cached_person_data['x_camera']
                z_camera = self._cached_person_data['z_camera']
                
                # Calculate base person position
                person_angle_rad = math.atan2(x_camera, z_camera)
                person_angle_deg = math.degrees(person_angle_rad)
                display_angle_deg = 360 - person_angle_deg
                while display_angle_deg < 0:
                    display_angle_deg += 360
                while display_angle_deg >= 360:
                    display_angle_deg -= 360
                
                distance_m = z_camera / 1000.0
                display_angle_rad = math.radians(90 - display_angle_deg)
                
                # Use CORRECTED position for field calculation
                base_x = self.center_x + int(distance_m * self.scale * math.cos(display_angle_rad))
                base_y = self.center_y - int(distance_m * self.scale * math.sin(display_angle_rad))
                
                person_screen_x = base_x + self.applied_offset_x
                person_screen_y = base_y + self.applied_offset_y
            
            # Grid setup
            grid_spacing = self.grid_size * self.scale // 1000
            if grid_spacing <= 0:
                grid_spacing = 50  # Fallback
            
            # 1. CREATE REPULSIVE FORCES AROUND OBSTACLES 
            for angle, distance in obstacles:
                if 100 <= distance <= 8000:  # Valid range
                    angle_rad = math.radians(angle)
                    
                    # Convert to cartesian screen coordinates
                    obs_x = distance * math.sin(angle_rad)
                    obs_y = distance * math.cos(angle_rad)
                    
                    obs_screen_x = self.center_x + int(obs_x * self.scale / 1000)
                    obs_screen_y = self.center_y - int(obs_y * self.scale / 1000)
                    
                    # Convert to grid coordinates
                    obs_grid_x = int(round((obs_screen_x - self.center_x) / grid_spacing))
                    obs_grid_y = int(round((self.center_y - obs_screen_y) / grid_spacing))
                    
                    # Add repulsive forces in 3x3 around obstacle - MINIMAL VALUES
                    for dx in range(-1, 2):
                        for dy in range(-1, 2):
                            gx = obs_grid_x + dx
                            gy = obs_grid_y + dy
                            
                            distance_from_obs = math.sqrt(dx*dx + dy*dy)
                            if distance_from_obs == 0:
                                repulsive_value = 3   # Reduced from 5
                            elif distance_from_obs <= 1:
                                repulsive_value = 2   # Reduced from 3
                            else:
                                repulsive_value = 1   # Same
                            
                            # Add to grid (sum with existing values)
                            if (gx, gy) in self.potential_field_cache:
                                self.potential_field_cache[(gx, gy)] += repulsive_value
                            else:
                                self.potential_field_cache[(gx, gy)] = repulsive_value
            
            # 2. CREATE ATTRACTIVE FORCES AROUND PERSON (minimal values)
            if person_screen_x is not None:
                # Convert person screen position to grid coordinates
                person_grid_x = int(round((person_screen_x - self.center_x) / grid_spacing))
                person_grid_y = int(round((self.center_y - person_screen_y) / grid_spacing))
                
                # Create spreading attractive force (1 cell radius only) - MINIMAL VALUES
                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        gx = person_grid_x + dx
                        gy = person_grid_y + dy
                        
                        distance_from_person = math.sqrt(dx*dx + dy*dy)
                        if distance_from_person == 0:
                            attractive_value = -5   # Increased to max cap
                        elif distance_from_person <= 1:
                            attractive_value = -4   # Increased 
                        else:
                            attractive_value = -3   # Increased
                        
                        # Add to grid (sum with existing values)
                        if (gx, gy) in self.potential_field_cache:
                            self.potential_field_cache[(gx, gy)] += attractive_value
                        else:
                            self.potential_field_cache[(gx, gy)] = attractive_value
            
            # 4. CAP VALUES TO PREVENT EXTREME ACCUMULATION
            # Clamp all values to reasonable range to prevent 50+ numbers
            for key in self.potential_field_cache:
                value = self.potential_field_cache[key]
                self.potential_field_cache[key] = max(-5, min(5, value))  # Cap between -5 and +5
            
        except Exception:
            pass  # Fail silently for performance
    
    def draw_grid_overlay(self):
        """Draw simple grid overlay lines - FIXED VERSION"""
        try:
            if not self.screen:
                return
                
            # Use a simple fixed grid spacing in pixels
            grid_spacing_pixels = 100  # 100 pixel grid spacing
            grid_color = (0, 40, 0)  # Very dark green so it doesn't interfere
            
            screen_width = self.screen.get_width()
            screen_height = self.screen.get_height()
            
            # Draw vertical grid lines
            for x in range(0, screen_width, grid_spacing_pixels):
                pygame.draw.line(self.screen, grid_color, (x, 0), (x, screen_height), 1)
            
            # Draw horizontal grid lines
            for y in range(0, screen_height, grid_spacing_pixels):
                pygame.draw.line(self.screen, grid_color, (0, y), (screen_width, y), 1)
                
        except Exception:
            pass  # Fail silently for performance
    
    def draw_potential_field_grid(self):
        """Draw potential field force numbers on grid points"""
        try:
            if not self.screen:
                return
                
            font = pygame.font.Font(None, 24)
            grid_spacing = self.grid_size * self.scale // 1000
            
            for (gx, gy), value in self.potential_field_cache.items():
                # Calculate screen position from grid position
                screen_x = self.center_x + gx * grid_spacing
                screen_y = self.center_y - gy * grid_spacing
                
                # Check bounds
                if (0 <= screen_x < self.screen.get_width() and 0 <= screen_y < self.screen.get_height()):
                    
                    # Convert to display value
                    display_val = int(abs(value))
                    
                    # Color coding
                    if value < 0:
                        color = (0, 255, 0)  # Bright green for net attractive forces
                    else:
                        color = (255, 0, 0)  # Bright red for net repulsive forces
                    
                    text = str(display_val)
                    
                    # Only display significant values (adjusted for reduced force values)
                    if display_val >= 1:
                        # Draw text with black outline for better visibility
                        outline_color = (0, 0, 0)
                        
                        # Draw text outline
                        for dx in [-1, 0, 1]:
                            for dy in [-1, 0, 1]:
                                if dx != 0 or dy != 0:
                                    outline_surface = font.render(text, True, outline_color)
                                    outline_rect = outline_surface.get_rect(center=(screen_x + dx, screen_y + dy))
                                    self.screen.blit(outline_surface, outline_rect)
                        
                        # Draw main text
                        text_surface = font.render(text, True, color)
                        text_rect = text_surface.get_rect(center=(screen_x, screen_y))
                        self.screen.blit(text_surface, text_rect)
                        
        except Exception:
            pass  # Fail silently for performance

    def update(self) -> Status:
        """OPTIMIZED update method - minimal overhead"""
        try:
            if not self.initialized:
                if not self.initialize_components():
                    return Status.FAILURE
            
            self.update_counter += 1
            
            # Handle calibration keys (WASD for movement, X for saving offset, C for calculating offset)
            keys = pygame.key.get_pressed()
            if keys[pygame.K_w]:
                self.adjustment_y -= self.adjustment_step  # Move up (decrease Y)
            if keys[pygame.K_s]:
                self.adjustment_y += self.adjustment_step  # Move down (increase Y)
            if keys[pygame.K_a]:
                self.adjustment_x -= self.adjustment_step  # Move left (decrease X)
            if keys[pygame.K_d]:
                self.adjustment_x += self.adjustment_step  # Move right (increase X)
            
            # Check for X key press to save offset (doesn't interfere with ESC handling)
            if keys[pygame.K_x]:
                # Use a simple debounce mechanism to avoid multiple saves
                current_time = time.time()
                if not hasattr(self, '_last_x_press') or current_time - self._last_x_press > 1.0:
                    print("🔑 X key pressed - attempting to save calibration offset...")
                    if self._cached_person_data:
                        print("✅ Person data available, saving...")
                        print(f"📁 About to call save_calibration_offset() function...")
                        try:
                            self.save_calibration_offset()
                            print(f"📁 save_calibration_offset() completed")
                        except Exception as e:
                            print(f"❌ Exception in save_calibration_offset(): {e}")
                            import traceback
                            traceback.print_exc()
                    else:
                        print("❌ No person detected - cannot save offset")
                        self.last_save_message = "ERROR: No person detected"
                        self.last_save_time = current_time
                    self._last_x_press = current_time
            
            # Check for C key press to calculate and apply offset from saved data
            if keys[pygame.K_c]:
                current_time = time.time()
                if not hasattr(self, '_last_c_press') or current_time - self._last_c_press > 1.0:
                    print("🔑 C key pressed - calculating offset from calibration data...")
                    self.calculate_offset_from_data()
                    self._last_c_press = current_time
            
            # Test offset keys (for demonstration)
            if keys[pygame.K_1]:
                current_time = time.time()
                if not hasattr(self, '_last_1_press') or current_time - self._last_1_press > 1.0:
                    self.set_manual_offset(0, 0)  # No offset
                    self._last_1_press = current_time
                    
            if keys[pygame.K_2]:
                current_time = time.time()
                if not hasattr(self, '_last_2_press') or current_time - self._last_2_press > 1.0:
                    self.set_manual_offset(50, 0)  # 50 pixels right
                    self._last_2_press = current_time
                    
            if keys[pygame.K_3]:
                current_time = time.time()
                if not hasattr(self, '_last_3_press') or current_time - self._last_3_press > 1.0:
                    self.set_manual_offset(-50, 0)  # 50 pixels left
                    self._last_3_press = current_time
            
            pygame.event.pump()

            # Always poll detection + update head every tick (UNCHANGED from working version)
            person_data = self.get_person_detection()
            if person_data:
                self._cached_person_data = person_data
                self._cache_t = time.time()
                if self.head_tracking_enabled:
                    self.update_head_tracking(person_data)
            else:
                if self._cached_person_data and (time.time() - self._cache_t) <= 0.25 and self.head_tracking_enabled:
                    self.update_head_tracking(self._cached_person_data)
                elif self.head_tracker and time.time() - self.last_person_detected > self.person_lost_timeout:
                    try:
                        self.angle_history.clear()
                        self.last_sent_angle = None
                        self.head_tracker.set_manual_position(0.0)
                    except Exception:
                        pass
                # Reset navigation
                self.navigator.reset_navigation_state()

            # OPTIMIZED display update (less frequent)
            if self.update_counter % self.display_update_rate == 0:
                try:
                    if self.screen:
                        self.screen.fill((0, 0, 0))
                        
                        # Draw radar grid 
                        self.draw_radar_grid()  # Base radar circles and lines
                        self.draw_grid_overlay()  # Potential field grid overlay
                        self.draw_robot()  # From parent
                        
                        # LiDAR obstacles (from parent)
                        obstacle_count = 0
                        if self.lidar_system:
                            obstacles = self.lidar_system.get_display_obstacles()
                            if obstacles:
                                obstacle_count = self.draw_lidar_data(obstacles)
                        
                        # Person detection with calibration markers
                        if self._cached_person_data:
                            self.draw_person_detection_no_text(self._cached_person_data)
                        
                        # Calculate and draw potential field
                        self.calculate_potential_field_grid()
                        self.draw_potential_field_grid()
                        
                        # Draw info
                        self.draw_info(obstacle_count)
                        
                        # Display save message if recent
                        if self.last_save_message and time.time() - self.last_save_time < 3.0:
                            font = pygame.font.Font(None, 36)
                            text = font.render(self.last_save_message, True, (255, 255, 255))
                            self.screen.blit(text, (50, 100))  # Below offset display
                        
                        pygame.display.flip()
                except Exception:
                    pass  # Fail silently
            
            return Status.RUNNING
        except Exception:
            return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """Clean termination with exact camera cleanup"""
        # Reset navigation system
        if hasattr(self, 'navigator'):
            self.navigator.reset_navigation_state()
        
        try:
            # Stop exact camera system if initialized
            if self.exact_device:
                print("🛑 Stopping exact camera system...")
                self.exact_device.close()
                self.exact_device = None
                print("✅ Exact camera system stopped")
        except Exception as e:
            print(f"⚠️ Error stopping exact camera: {e}")
        
        # Call parent termination (handles everything else)
        super().terminate(new_status)

# Export for use
__all__ = ['PotentialFieldVisualizationTest']