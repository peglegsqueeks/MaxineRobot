#!/usr/bin/env python3
"""
DETECTION CONSISTENCY TEST - X Midpoint Variance Analysis
- Measures x midpoint variance of detected person for stability analysis
- Large FPS and Variance displays
- Press ENTER to start 30-second consistency test  
- Outputs detailed variance analysis to TEST.csv
- Compares standalone detection vs integrated robot stability
"""
import pygame
import math
import time
import csv
import os
import statistics
from collections import deque
from datetime import datetime

try:
    import depthai as dai
    import cv2
    import numpy as np
except ImportError as e:
    print(f"❌ Missing required libraries: {e}")
    exit(1)


class DetectionConsistencyTester:
    """Detection consistency tester focused on X midpoint variance analysis"""
    
    def __init__(self):
        # Initialize pygame for display
        pygame.init()
        pygame.font.init()
        
        # Get fullscreen resolution
        display_info = pygame.display.Info()
        self.screen_width = display_info.current_w
        self.screen_height = display_info.current_h
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.FULLSCREEN)
        pygame.display.set_caption("Detection Consistency Test - X Midpoint Variance")
        
        # Hide mouse cursor
        pygame.mouse.set_visible(False)
        
        # Fonts for display
        self.huge_font = pygame.font.Font(None, 120)  # Large FPS and Variance
        self.large_font = pygame.font.Font(None, 64)  # For "Press ENTER"
        self.medium_font = pygame.font.Font(None, 48)  # For variance display
        self.small_font = pygame.font.Font(None, 32)  # For details
        
        # Test states
        self.test_state = "waiting"
        self.test_duration = 30.0  # 30-second test
        self.countdown_duration = 3.0
        self.test_start_time = 0
        self.countdown_start_time = 0
        
        # Camera system
        self.device = None
        self.pipeline = None
        self.detection_queue = None
        self.preview_queue = None
        self.has_detection = False
        self.camera_initialized = False
        self.camera_error_message = ""
        
        # FPS Monitoring
        self.fps_counter = deque(maxlen=30)
        self.last_frame_time = time.time()
        self.current_fps = 0.0
        self.target_fps = 25
        
        # CONSISTENCY ANALYSIS - X Midpoint Tracking
        self.x_midpoints = deque(maxlen=1000)  # Store up to 1000 x midpoints
        self.x_midpoints_pixels = deque(maxlen=1000)  # Pixel coordinates
        self.x_midpoints_normalized = deque(maxlen=1000)  # Normalized coordinates
        
        # Real-time variance calculation
        self.variance_window = deque(maxlen=100)  # Rolling window for variance
        self.current_variance_pixels = 0.0
        self.current_std_dev_pixels = 0.0
        self.current_mean_pixels = 0.0
        
        # Consistency metrics
        self.detection_count = 0
        self.consistent_detection_count = 0
        self.large_jumps_count = 0
        self.jump_threshold_pixels = 50  # Consider >50px a "large jump"
        self.last_x_midpoint = None
        
        # Variance analysis periods
        self.short_term_variance = deque(maxlen=25)   # 1-second variance (at 25fps)
        self.medium_term_variance = deque(maxlen=125) # 5-second variance  
        self.long_term_variance = deque(maxlen=750)   # 30-second variance
        
        # Detection system (preserved settings)
        self.detection_rate_history = deque(maxlen=100)
        self.last_detection_time = time.time()
        self.current_detection_rate = 0.0
        
        # Z-depth tracking (for completeness)
        self.z_depth_smoother = deque(maxlen=5)
        self.confidence_weights = deque(maxlen=5)
        self.smoothed_z_depth = 0
        self.depth_trust_threshold = 0.6
        
        # Data tracking
        self.detection_history = deque(maxlen=2000)
        self.current_detection = None
        
        # Camera specs (preserved)
        self.camera_resolution_width = 300
        self.camera_resolution_height = 300
        self.camera_hfov_degrees = 114
        
        # Performance settings
        self.detection_skip_frames = 1
        self.frame_counter = 0
        
        # Logging - OUTPUT TO TEST.csv
        self.log_filename = "TESTVARIENCE.csv"
        self.csv_initialized = False
        
        # Control flags
        self.running = True
        self.paused = False
        
        # Camera display settings
        self.camera_display_size = (600, 450)
        self.camera_surface = None
        self.show_camera_debug = True
        
        # Stability analysis
        self.stability_zones = {
            'stable': 0,      # <10px variance
            'moderate': 0,    # 10-25px variance
            'unstable': 0,    # 25-50px variance  
            'very_unstable': 0  # >50px variance
        }
    
    def initialize_csv_log(self):
        """Initialize CSV log for consistency analysis"""
        if self.csv_initialized:
            return
        
        try:
            with open(self.log_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # CSV Header for consistency analysis
                writer.writerow([
                    'test_time_elapsed', 'timestamp', 'frame_number', 'fps',
                    'x_midpoint_pixels', 'x_midpoint_normalized', 'x_camera_mm',
                    'z_depth_mm', 'confidence',
                    'bbox_xmin', 'bbox_ymin', 'bbox_xmax', 'bbox_ymax', 'bbox_width', 'bbox_height',
                    'x_jump_from_previous', 'is_large_jump', 
                    'rolling_variance_pixels', 'rolling_std_dev_pixels', 'rolling_mean_pixels',
                    'short_term_variance', 'medium_term_variance', 'long_term_variance',
                    'detection_count', 'consistent_detections', 'large_jumps_count',
                    'stability_classification', 'time_remaining', 'camera_mode'
                ])
            self.csv_initialized = True
            print(f"✅ Consistency analysis logging to: {self.log_filename}")
        except Exception as e:
            print(f"⚠️ CSV initialization failed: {e}")
    
    def check_camera_connection(self):
        """Check camera connection"""
        try:
            devices = dai.Device.getAllAvailableDevices()
            return len(devices) > 0, f"Found {len(devices)} device(s)"
        except Exception as e:
            return False, f"Device detection error: {str(e)}"
    
    def create_pipeline(self):
        """Create pipeline - PRESERVED settings"""
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
    
    def initialize_camera(self):
        """Initialize camera"""
        try:
            connected, message = self.check_camera_connection()
            if not connected:
                self.camera_error_message = message
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
            
            try:
                self.preview_queue = self.device.getOutputQueue("preview", maxSize=4, blocking=False)
            except Exception:
                self.preview_queue = None
            
            # Test camera frames
            if self.preview_queue:
                for i in range(20):
                    test_frame = self.preview_queue.tryGet()
                    if test_frame:
                        self.camera_initialized = True
                        break
                    time.sleep(0.1)
            
            time.sleep(2)
            return True
        except Exception as e:
            self.camera_error_message = f"Camera initialization error: {str(e)}"
            return False
    
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
            if self.show_camera_debug:
                self.show_camera_debug = False
    
    def calculate_fps(self):
        """Calculate FPS"""
        current_time = time.time()
        frame_time = current_time - self.last_frame_time
        self.last_frame_time = current_time
        
        if frame_time > 0:
            fps = 1.0 / frame_time
            self.fps_counter.append(fps)
            if len(self.fps_counter) > 0:
                self.current_fps = sum(self.fps_counter) / len(self.fps_counter)
    
    def calculate_x_midpoint_variance(self):
        """Calculate real-time X midpoint variance for consistency analysis"""
        try:
            if len(self.x_midpoints_pixels) < 2:
                return 0.0, 0.0, 0.0
            
            # Use rolling window for real-time calculation
            window_data = list(self.variance_window)
            
            if len(window_data) < 2:
                return 0.0, 0.0, 0.0
            
            # Calculate variance, standard deviation, and mean
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
        """Classify detection stability based on standard deviation"""
        if std_dev_pixels < 10:
            return 'stable'
        elif std_dev_pixels < 25:
            return 'moderate'
        elif std_dev_pixels < 50:
            return 'unstable'
        else:
            return 'very_unstable'
    
    def get_test_time_remaining(self):
        """Get remaining test time"""
        if self.test_state == "running":
            elapsed = time.time() - self.test_start_time
            remaining = max(0, self.test_duration - elapsed)
            return remaining
        return self.test_duration
    
    def get_test_time_elapsed(self):
        """Get elapsed test time"""
        if self.test_state == "running":
            return time.time() - self.test_start_time
        return 0
    
    def smooth_z_depth(self, raw_z_depth, confidence):
        """Apply confidence-weighted temporal smoothing - PRESERVED"""
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
    
    def process_detections(self):
        """Process detections with X MIDPOINT CONSISTENCY ANALYSIS"""
        if self.test_state != "running":
            return None
        
        start_time = time.time()
        self.frame_counter += 1
        
        if self.frame_counter % self.detection_skip_frames != 0:
            return None
        
        camera_mode = "real" if (self.has_detection and self.detection_queue and self.camera_initialized) else "simulation"
        
        if camera_mode == "simulation":
            return self.simulate_detection_for_consistency_test(camera_mode)
        
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
            
            # CONSISTENCY ANALYSIS - Calculate X midpoint
            bbox_xmin = closest_person.xmin
            bbox_xmax = closest_person.xmax
            bbox_ymin = closest_person.ymin
            bbox_ymax = closest_person.ymax
            
            # X midpoint calculations
            x_midpoint_normalized = (bbox_xmin + bbox_xmax) / 2.0
            x_midpoint_pixels = int(x_midpoint_normalized * self.screen_width)  # Convert to screen pixels
            
            # Track midpoints for variance analysis
            self.x_midpoints.append(x_camera)  # mm coordinates
            self.x_midpoints_pixels.append(x_midpoint_pixels)  # pixel coordinates
            self.x_midpoints_normalized.append(x_midpoint_normalized)  # normalized coordinates
            self.variance_window.append(x_midpoint_pixels)  # rolling variance window
            
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
            
            # Calculate real-time variance metrics
            self.current_variance_pixels, self.current_std_dev_pixels, self.current_mean_pixels = self.calculate_x_midpoint_variance()
            
            # Calculate multi-term variance
            short_var, medium_var, long_var = self.calculate_multi_term_variance(x_midpoint_pixels)
            
            # Classify stability
            stability_class = self.classify_stability(self.current_std_dev_pixels)
            self.stability_zones[stability_class] += 1
            
            processing_time = (time.time() - start_time) * 1000
            test_elapsed = self.get_test_time_elapsed()
            time_remaining = self.get_test_time_remaining()
            
            analysis_data = {
                'test_time_elapsed': test_elapsed,
                'timestamp': time.time(),
                'frame_number': self.frame_counter,
                'fps': self.current_fps,
                'x_midpoint_pixels': x_midpoint_pixels,
                'x_midpoint_normalized': x_midpoint_normalized,
                'x_camera_mm': x_camera,
                'y_camera': y_camera,
                'z_depth': smoothed_z_depth,
                'raw_z_depth': raw_z_depth,
                'confidence': confidence,
                'bbox_xmin': bbox_xmin,
                'bbox_ymin': bbox_ymin,
                'bbox_xmax': bbox_xmax,
                'bbox_ymax': bbox_ymax,
                'bbox_width': bbox_xmax - bbox_xmin,
                'bbox_height': bbox_ymax - bbox_ymin,
                'x_jump_from_previous': x_jump,
                'is_large_jump': is_large_jump,
                'rolling_variance_pixels': self.current_variance_pixels,
                'rolling_std_dev_pixels': self.current_std_dev_pixels,
                'rolling_mean_pixels': self.current_mean_pixels,
                'short_term_variance': short_var,
                'medium_term_variance': medium_var,
                'long_term_variance': long_var,
                'detection_count': self.detection_count,
                'consistent_detections': self.consistent_detection_count,
                'large_jumps_count': self.large_jumps_count,
                'stability_classification': stability_class,
                'processing_time_ms': processing_time,
                'time_remaining': time_remaining,
                'camera_mode': camera_mode
            }
            
            self.log_consistency_to_csv(analysis_data)
            self.current_detection = analysis_data
            self.detection_history.append(analysis_data)
            
            return analysis_data
        except Exception:
            return None
    
    def simulate_detection_for_consistency_test(self, camera_mode="simulation"):
        """Simulation mode with controlled variance patterns"""
        if self.test_state != "running":
            return None
        
        try:
            current_time = time.time()
            test_elapsed = self.get_test_time_elapsed()
            
            # Simulate different stability patterns during test
            if test_elapsed < 10:
                # First 10 seconds: Very stable (person sitting still)
                base_x = 0.5  # Center of frame
                noise = 0.01 * math.sin(current_time * 2)  # Very small oscillation
            elif test_elapsed < 20:
                # Next 10 seconds: Moderate variance (slight movement)
                base_x = 0.5 + 0.1 * math.sin(current_time * 0.5)
                noise = 0.03 * math.sin(current_time * 5)
            else:
                # Last 10 seconds: Higher variance (more movement)
                base_x = 0.5 + 0.2 * math.sin(current_time * 0.3)
                noise = 0.05 * math.sin(current_time * 8)
            
            x_midpoint_normalized = base_x + noise
            x_midpoint_normalized = max(0.1, min(0.9, x_midpoint_normalized))  # Keep in bounds
            x_midpoint_pixels = int(x_midpoint_normalized * self.screen_width)
            
            # Simulated camera coordinates
            x_camera = (x_midpoint_normalized - 0.5) * 2000  # -1000 to +1000 mm
            y_camera = -200
            z_depth = 2500 + 50 * math.sin(current_time)
            confidence = 0.85
            
            # Simulate bounding box
            bbox_width = 0.3
            bbox_height = 0.4
            bbox_xmin = x_midpoint_normalized - bbox_width/2
            bbox_xmax = x_midpoint_normalized + bbox_width/2
            bbox_ymin = 0.3
            bbox_ymax = 0.7
            
            # Track midpoints for variance analysis
            self.x_midpoints.append(x_camera)
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
            
            time_remaining = self.get_test_time_remaining()
            
            analysis_data = {
                'test_time_elapsed': test_elapsed,
                'timestamp': current_time,
                'frame_number': self.frame_counter,
                'fps': self.current_fps,
                'x_midpoint_pixels': x_midpoint_pixels,
                'x_midpoint_normalized': x_midpoint_normalized,
                'x_camera_mm': x_camera,
                'y_camera': y_camera,
                'z_depth': z_depth,
                'raw_z_depth': z_depth,
                'confidence': confidence,
                'bbox_xmin': bbox_xmin,
                'bbox_ymin': bbox_ymin,
                'bbox_xmax': bbox_xmax,
                'bbox_ymax': bbox_ymax,
                'bbox_width': bbox_width,
                'bbox_height': bbox_height,
                'x_jump_from_previous': x_jump,
                'is_large_jump': is_large_jump,
                'rolling_variance_pixels': self.current_variance_pixels,
                'rolling_std_dev_pixels': self.current_std_dev_pixels,
                'rolling_mean_pixels': self.current_mean_pixels,
                'short_term_variance': short_var,
                'medium_term_variance': medium_var,
                'long_term_variance': long_var,
                'detection_count': self.detection_count,
                'consistent_detections': self.consistent_detection_count,
                'large_jumps_count': self.large_jumps_count,
                'stability_classification': stability_class,
                'processing_time_ms': 5.0,
                'time_remaining': time_remaining,
                'camera_mode': camera_mode
            }
            
            self.log_consistency_to_csv(analysis_data)
            self.current_detection = analysis_data
            self.detection_history.append(analysis_data)
            
            return analysis_data
        except Exception:
            return None
    
    def log_consistency_to_csv(self, data):
        """Log consistency analysis data to TEST.csv"""
        try:
            with open(self.log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    data.get('test_time_elapsed', 0),
                    data['timestamp'],
                    data.get('frame_number', 0),
                    data.get('fps', 0),
                    data['x_midpoint_pixels'],
                    data['x_midpoint_normalized'],
                    data['x_camera_mm'],
                    data['z_depth'],
                    data['confidence'],
                    data['bbox_xmin'],
                    data['bbox_ymin'],
                    data['bbox_xmax'],
                    data['bbox_ymax'],
                    data['bbox_width'],
                    data['bbox_height'],
                    data['x_jump_from_previous'],
                    data['is_large_jump'],
                    data['rolling_variance_pixels'],
                    data['rolling_std_dev_pixels'],
                    data['rolling_mean_pixels'],
                    data['short_term_variance'],
                    data['medium_term_variance'],
                    data['long_term_variance'],
                    data['detection_count'],
                    data['consistent_detections'],
                    data['large_jumps_count'],
                    data['stability_classification'],
                    data.get('time_remaining', 0),
                    data.get('camera_mode', 'unknown')
                ])
        except Exception:
            pass
    
    def draw_detection_bounding_box(self, camera_x, camera_y):
        """Draw green bounding box with variance information"""
        try:
            if not self.current_detection:
                return
            
            bbox_xmin = self.current_detection.get('bbox_xmin', 0)
            bbox_ymin = self.current_detection.get('bbox_ymin', 0) 
            bbox_xmax = self.current_detection.get('bbox_xmax', 1)
            bbox_ymax = self.current_detection.get('bbox_ymax', 1)
            
            display_width, display_height = self.camera_display_size
            
            box_left = int(bbox_xmin * display_width)
            box_top = int(bbox_ymin * display_height)
            box_right = int(bbox_xmax * display_width)
            box_bottom = int(bbox_ymax * display_height)
            
            box_width = box_right - box_left
            box_height = box_bottom - box_top
            
            box_left = max(0, min(box_left, display_width - 1))
            box_top = max(0, min(box_top, display_height - 1))
            box_width = max(1, min(box_width, display_width - box_left))
            box_height = max(1, min(box_height, display_height - box_top))
            
            abs_x = camera_x + box_left
            abs_y = camera_y + box_top
            
            # Color-code bounding box by stability
            stability_class = self.current_detection.get('stability_classification', 'stable')
            if stability_class == 'stable':
                box_color = (0, 255, 0)  # Green - stable
            elif stability_class == 'moderate':
                box_color = (255, 255, 0)  # Yellow - moderate
            elif stability_class == 'unstable':
                box_color = (255, 165, 0)  # Orange - unstable
            else:
                box_color = (255, 0, 0)  # Red - very unstable
            
            box_thickness = 3
            pygame.draw.rect(self.screen, box_color, 
                           (abs_x, abs_y, box_width, box_height), box_thickness)
            
            # Add midpoint marker
            midpoint_x = abs_x + box_width // 2
            midpoint_y = abs_y + box_height // 2
            pygame.draw.circle(self.screen, (255, 255, 255), (midpoint_x, midpoint_y), 5, 2)
            
            # Add variance information
            std_dev = self.current_detection.get('rolling_std_dev_pixels', 0)
            x_pixel = self.current_detection.get('x_midpoint_pixels', 0)
            
            var_text = self.small_font.render(f"X:{x_pixel}px", True, box_color)
            var_x = abs_x
            var_y = max(abs_y - 45, camera_y)
            self.screen.blit(var_text, (var_x, var_y))
            
            std_text = self.small_font.render(f"±{std_dev:.1f}px", True, box_color)
            std_x = abs_x
            std_y = max(abs_y - 25, camera_y)
            self.screen.blit(std_text, (std_x, std_y))
            
        except Exception:
            pass
    
    def draw_waiting_screen(self):
        """Draw waiting screen"""
        self.screen.fill((0, 0, 0))
        
        # Title
        title_text = self.large_font.render("DETECTION CONSISTENCY TEST", True, (0, 255, 255))
        title_rect = title_text.get_rect(center=(self.screen_width // 2, self.screen_height // 2 - 100))
        self.screen.blit(title_text, title_rect)
        
        # Subtitle
        subtitle_text = self.medium_font.render("X Midpoint Variance Analysis", True, (255, 255, 255))
        subtitle_rect = subtitle_text.get_rect(center=(self.screen_width // 2, self.screen_height // 2 - 50))
        self.screen.blit(subtitle_text, subtitle_rect)
        
        # Instructions
        start_text = self.large_font.render("Press ENTER to start 30-second test", True, (0, 255, 0))
        start_rect = start_text.get_rect(center=(self.screen_width // 2, self.screen_height // 2 + 20))
        self.screen.blit(start_text, start_rect)
        
        # ESC to exit
        exit_text = self.small_font.render("Press ESC to exit", True, (255, 255, 255))
        exit_rect = exit_text.get_rect(center=(self.screen_width // 2, self.screen_height // 2 + 100))
        self.screen.blit(exit_text, exit_rect)
    
    def draw_countdown_screen(self):
        """Draw countdown screen"""
        self.screen.fill((0, 0, 0))
        
        elapsed = time.time() - self.countdown_start_time
        remaining = max(0, self.countdown_duration - elapsed)
        countdown_num = int(remaining) + 1
        
        if countdown_num > 0:
            countdown_text = self.huge_font.render(str(countdown_num), True, (255, 255, 0))
            countdown_rect = countdown_text.get_rect(center=(self.screen_width // 2, self.screen_height // 2))
            self.screen.blit(countdown_text, countdown_rect)
        else:
            start_text = self.huge_font.render("START!", True, (0, 255, 0))
            start_rect = start_text.get_rect(center=(self.screen_width // 2, self.screen_height // 2))
            self.screen.blit(start_text, start_rect)
    
    def draw_test_screen(self):
        """Draw test screen with FPS and VARIANCE displays"""
        self.screen.fill((0, 0, 0))
        
        # 1. Large FPS display in top-left
        if self.current_fps >= 22:
            fps_color = (0, 255, 0)      # Green
        elif self.current_fps >= 18:
            fps_color = (255, 255, 0)    # Yellow  
        else:
            fps_color = (255, 0, 0)      # Red
        
        fps_text = self.huge_font.render(f"FPS: {self.current_fps:.1f}", True, fps_color)
        self.screen.blit(fps_text, (50, 50))
        
        # 2. Large VARIANCE display below FPS
        if self.current_std_dev_pixels <= 10:
            var_color = (0, 255, 0)      # Green - stable
        elif self.current_std_dev_pixels <= 25:
            var_color = (255, 255, 0)    # Yellow - moderate
        elif self.current_std_dev_pixels <= 50:
            var_color = (255, 165, 0)    # Orange - unstable
        else:
            var_color = (255, 0, 0)      # Red - very unstable
        
        var_text = self.huge_font.render(f"VAR: ±{self.current_std_dev_pixels:.1f}px", True, var_color)
        self.screen.blit(var_text, (50, 180))
        
        # 3. Detection statistics
        if self.detection_count > 0:
            consistency_rate = (self.consistent_detection_count / self.detection_count) * 100
        else:
            consistency_rate = 0
        
        stats_y = 310
        stats_text = [
            f"Detections: {self.detection_count}",
            f"Consistent: {consistency_rate:.1f}%",
            f"Large Jumps: {self.large_jumps_count}",
            f"Mean X: {self.current_mean_pixels:.0f}px"
        ]
        
        for i, stat in enumerate(stats_text):
            stat_color = (255, 255, 255)
            if "Consistent" in stat and consistency_rate < 80:
                stat_color = (255, 165, 0)
            elif "Large Jumps" in stat and self.large_jumps_count > 10:
                stat_color = (255, 0, 0)
            
            text_surface = self.medium_font.render(stat, True, stat_color)
            self.screen.blit(text_surface, (50, stats_y + i * 40))
        
        # 4. Time remaining
        time_remaining = self.get_test_time_remaining()
        time_text = self.medium_font.render(f"Time: {time_remaining:.1f}s", True, (0, 255, 255))
        self.screen.blit(time_text, (50, stats_y + 180))
        
        # 5. Camera display in center-right
        camera_x = self.screen_width - self.camera_display_size[0] - 50
        camera_y = (self.screen_height - self.camera_display_size[1]) // 2
        
        # Camera background
        pygame.draw.rect(self.screen, (50, 50, 50), 
                        (camera_x - 5, camera_y - 5, 
                         self.camera_display_size[0] + 10, 
                         self.camera_display_size[1] + 10))
        
        if self.camera_surface and self.camera_initialized:
            # Display live camera
            self.screen.blit(self.camera_surface, (camera_x, camera_y))
            
            # Draw bounding boxes with variance coloring
            if self.current_detection:
                self.draw_detection_bounding_box(camera_x, camera_y)
        else:
            # Show camera status
            status_text = self.small_font.render("SIMULATION MODE", True, (255, 255, 0))
            status_rect = status_text.get_rect(center=(camera_x + self.camera_display_size[0]//2, 
                                                     camera_y + self.camera_display_size[1]//2))
            self.screen.blit(status_text, status_rect)
        
        # Camera border (color-coded by variance)
        border_color = var_color
        pygame.draw.rect(self.screen, border_color, 
                        (camera_x - 2, camera_y - 2, 
                         self.camera_display_size[0] + 4, 
                         self.camera_display_size[1] + 4), 3)
    
    def draw_finished_screen(self):
        """Draw finished screen with summary"""
        self.screen.fill((0, 0, 0))
        
        # Title
        complete_text = self.huge_font.render("TEST COMPLETE", True, (0, 255, 0))
        complete_rect = complete_text.get_rect(center=(self.screen_width // 2, self.screen_height // 2 - 200))
        self.screen.blit(complete_text, complete_rect)
        
        # Summary statistics
        if self.detection_count > 0:
            consistency_rate = (self.consistent_detection_count / self.detection_count) * 100
            overall_variance = statistics.variance(list(self.x_midpoints_pixels)) if len(self.x_midpoints_pixels) > 1 else 0
            overall_std_dev = math.sqrt(overall_variance)
        else:
            consistency_rate = 0
            overall_variance = 0
            overall_std_dev = 0
        
        summary_y = self.screen_height // 2 - 100
        summary_stats = [
            f"Total Detections: {self.detection_count}",
            f"Consistency Rate: {consistency_rate:.1f}%",
            f"Overall Std Dev: ±{overall_std_dev:.1f}px",
            f"Large Jumps: {self.large_jumps_count}",
            f"Data saved to: {self.log_filename}"
        ]
        
        for i, stat in enumerate(summary_stats):
            stat_color = (255, 255, 255)
            if "Consistency" in stat:
                stat_color = (0, 255, 0) if consistency_rate > 90 else (255, 255, 0) if consistency_rate > 80 else (255, 0, 0)
            elif "Std Dev" in stat:
                stat_color = (0, 255, 0) if overall_std_dev < 10 else (255, 255, 0) if overall_std_dev < 25 else (255, 0, 0)
            elif "saved to" in stat:
                stat_color = (0, 255, 255)
            
            text_surface = self.medium_font.render(stat, True, stat_color)
            text_rect = text_surface.get_rect(center=(self.screen_width // 2, summary_y + i * 50))
            self.screen.blit(text_surface, text_rect)
        
        # ESC to exit
        exit_text = self.large_font.render("Press ESC to exit", True, (255, 255, 255))
        exit_rect = exit_text.get_rect(center=(self.screen_width // 2, self.screen_height // 2 + 200))
        self.screen.blit(exit_text, exit_rect)
    
    def update_test_state(self):
        """Update test state"""
        current_time = time.time()
        
        if self.test_state == "countdown":
            if current_time - self.countdown_start_time >= self.countdown_duration:
                self.test_state = "running"
                self.test_start_time = current_time
                self.initialize_csv_log()
        
        elif self.test_state == "running":
            if current_time - self.test_start_time >= self.test_duration:
                self.test_state = "finished"
                self.generate_consistency_summary()
    
    def generate_consistency_summary(self):
        """Generate CONSISTENCY summary"""
        try:
            if self.detection_count > 0:
                consistency_rate = (self.consistent_detection_count / self.detection_count) * 100
                overall_variance = statistics.variance(list(self.x_midpoints_pixels)) if len(self.x_midpoints_pixels) > 1 else 0
                overall_std_dev = math.sqrt(overall_variance)
                
                x_range = max(self.x_midpoints_pixels) - min(self.x_midpoints_pixels) if self.x_midpoints_pixels else 0
                
                # Calculate stability percentages
                total_classifications = sum(self.stability_zones.values())
                stability_percentages = {}
                if total_classifications > 0:
                    for zone, count in self.stability_zones.items():
                        stability_percentages[zone] = (count / total_classifications) * 100
                else:
                    stability_percentages = {zone: 0 for zone in self.stability_zones}
            else:
                consistency_rate = overall_variance = overall_std_dev = x_range = 0
                stability_percentages = {zone: 0 for zone in self.stability_zones}
            
            # CONSISTENCY SUMMARY OUTPUT
            print(f"\n{'='*60}")
            print(f"DETECTION CONSISTENCY TEST RESULTS")
            print(f"{'='*60}")
            print(f"Total Test Duration: {self.test_duration:.1f} seconds")
            print(f"Total Detections: {self.detection_count}")
            print(f"Consistent Detections: {self.consistent_detection_count} ({consistency_rate:.1f}%)")
            print(f"Large Jumps (>{self.jump_threshold_pixels}px): {self.large_jumps_count}")
            print(f"")
            print(f"X MIDPOINT VARIANCE ANALYSIS:")
            print(f"Overall Standard Deviation: ±{overall_std_dev:.2f} pixels")
            print(f"Overall Variance: {overall_variance:.2f} pixels²")
            print(f"X Position Range: {x_range:.0f} pixels")
            if self.x_midpoints_pixels:
                print(f"Mean X Position: {statistics.mean(self.x_midpoints_pixels):.1f} pixels")
            print(f"")
            print(f"STABILITY CLASSIFICATION BREAKDOWN:")
            print(f"Stable (<10px StdDev): {stability_percentages['stable']:.1f}%")
            print(f"Moderate (10-25px): {stability_percentages['moderate']:.1f}%")
            print(f"Unstable (25-50px): {stability_percentages['unstable']:.1f}%")
            print(f"Very Unstable (>50px): {stability_percentages['very_unstable']:.1f}%")
            print(f"")
            print(f"DATA OUTPUT: {self.log_filename}")
            print(f"{'='*60}")
            
        except Exception as e:
            print(f"Summary generation error: {e}")
    
    def run(self):
        """Run the CONSISTENCY test"""
        # Startup
        camera_ready = self.initialize_camera()
        if not camera_ready and not self.camera_initialized:
            print("Camera initialization failed - running in simulation mode")
        
        clock = pygame.time.Clock()
        
        try:
            while self.running:
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.running = False
                        elif event.key == pygame.K_RETURN and self.test_state == "waiting":
                            self.test_state = "countdown"
                            self.countdown_start_time = time.time()
                        elif event.key == pygame.K_SPACE and self.test_state == "running":
                            self.paused = not self.paused
                    elif event.type == pygame.QUIT:
                        self.running = False
                
                self.update_test_state()
                self.calculate_fps()
                
                if self.test_state == "running":
                    self.update_camera_frame()
                
                if not self.paused:
                    self.process_detections()
                
                # Draw appropriate screen
                if self.test_state == "waiting":
                    self.draw_waiting_screen()
                elif self.test_state == "countdown":
                    self.draw_countdown_screen()
                elif self.test_state == "running":
                    self.draw_test_screen()
                elif self.test_state == "finished":
                    self.draw_finished_screen()
                
                pygame.display.flip()
                clock.tick(self.target_fps)
                
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()
        
        return True
    
    def cleanup(self):
        """Cleanup"""
        try:
            if self.device:
                self.device.close()
            pygame.quit()
        except Exception:
            pass


def main():
    """Main function"""
    tester = DetectionConsistencyTester()
    
    try:
        tester.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        tester.cleanup()


if __name__ == "__main__":
    main()