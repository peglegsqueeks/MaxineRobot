#!/usr/bin/env python3
"""
MINIMAL Detection Test - Camera + FPS Only
- Camera display with green bounding boxes
- Large FPS display
- Press ENTER to start functionality  
- NO other text/console output (except final summary)
- Preserves all working detection settings
"""
import pygame
import math
import time
import csv
import os
from collections import deque
from datetime import datetime

try:
    import depthai as dai
    import cv2
    import numpy as np
except ImportError as e:
    print(f"❌ Missing required libraries: {e}")
    exit(1)


class MinimalCameraDetectionTester:
    """MINIMAL camera tester - Camera + FPS display only"""
    
    def __init__(self):
        # Initialize pygame for display
        pygame.init()
        pygame.font.init()
        
        # Get fullscreen resolution
        display_info = pygame.display.Info()
        self.screen_width = display_info.current_w
        self.screen_height = display_info.current_h
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.FULLSCREEN)
        pygame.display.set_caption("Minimal Detection Test")
        
        # Hide mouse cursor
        pygame.mouse.set_visible(False)
        
        # Fonts - MINIMAL
        self.huge_font = pygame.font.Font(None, 120)  # Large FPS
        self.large_font = pygame.font.Font(None, 64)  # For "Press ENTER"
        self.small_font = pygame.font.Font(None, 32)  # For bounding box labels
        
        # Test states
        self.test_state = "waiting"
        self.test_duration = 30.0
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
        
        # FPS tracking for summary
        self.all_fps_values = []
        
        # Detection system (preserved settings)
        self.detection_rate_history = deque(maxlen=100)
        self.last_detection_time = time.time()
        self.current_detection_rate = 0.0
        
        # Z-depth smoothing (preserved)
        self.z_depth_smoother = deque(maxlen=5)
        self.confidence_weights = deque(maxlen=5)
        self.smoothed_z_depth = 0
        self.depth_trust_threshold = 0.6
        
        # Data tracking
        self.detection_history = deque(maxlen=2000)
        self.z_distance_history = deque(maxlen=50)
        self.flicker_events = []
        self.current_detection = None
        
        # Analysis parameters (preserved)
        self.flicker_threshold = 300
        self.edge_threshold = 0.7
        
        # Camera specs (preserved)
        self.camera_resolution_width = 300
        self.camera_resolution_height = 300
        self.camera_hfov_degrees = 114
        
        # Performance settings
        self.detection_skip_frames = 1
        self.frame_counter = 0
        
        # Logging
        self.log_filename = self.create_log_filename()
        self.csv_initialized = False
        
        # Control flags
        self.running = True
        self.paused = False
        
        # Camera display settings
        self.camera_display_size = (600, 450)  # Larger camera display
        self.camera_surface = None
        self.show_camera_debug = True
    
    def create_log_filename(self):
        """Create log filename"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return f"minimal_detection_test_{timestamp}.csv"
    
    def initialize_csv_log(self):
        """Initialize CSV log - SILENT"""
        if self.csv_initialized:
            return
        
        try:
            with open(self.log_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'test_time_elapsed', 'timestamp', 'frame_number', 'fps', 
                    'x_camera', 'y_camera', 'z_depth', 'raw_z_depth', 'confidence',
                    'bbox_xmin', 'bbox_ymin', 'bbox_xmax', 'bbox_ymax',
                    'y_bottom_distance', 'estimated_distance_from_y',
                    'x_position_ratio', 'is_at_edge', 'z_flicker_detected', 'z_change_amount',
                    'distance_from_center', 'processing_time_ms', 'time_remaining',
                    'camera_mode'
                ])
            self.csv_initialized = True
        except Exception:
            pass  # SILENT
    
    def check_camera_connection(self):
        """Check camera connection - SILENT"""
        try:
            devices = dai.Device.getAllAvailableDevices()
            return len(devices) > 0, f"Found {len(devices)} device(s)"
        except Exception as e:
            return False, f"Device detection error: {str(e)}"
    
    def create_pipeline(self):
        """Create pipeline - PRESERVED settings, SILENT"""
        try:
            pipeline = dai.Pipeline()
            
            local_blob_path = "./mobilenet-ssd_openvino_2022.1_6shave.blob"
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
        """Initialize camera - SILENT"""
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
            
            # Optimize device - SILENT
            try:
                if hasattr(self.device, 'setLogLevel'):
                    self.device.setLogLevel(dai.LogLevel.WARN)
                self.device.setIrLaserDotProjectorIntensity(900)
            except Exception:
                pass
            
            # Get queues - SILENT
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
            
            # Test camera frames - SILENT
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
                
                if self.test_state == "running":
                    self.all_fps_values.append(self.current_fps)
    
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
    
    def calculate_y_distance_method(self, detection):
        """Calculate distance using y-coordinate method - PRESERVED"""
        try:
            ymin = detection.ymin
            ymax = detection.ymax
            
            bbox_bottom_pixel = ymax * self.camera_resolution_height
            screen_bottom_pixel = self.camera_resolution_height
            y_bottom_distance = screen_bottom_pixel - bbox_bottom_pixel
            
            max_distance = 8000
            min_distance = 500
            distance_ratio = y_bottom_distance / self.camera_resolution_height
            distance_ratio = max(0, min(1, distance_ratio))
            
            estimated_distance = min_distance + (distance_ratio ** 0.8) * (max_distance - min_distance)
            
            return {
                'bbox_bottom_pixel': bbox_bottom_pixel,
                'y_bottom_distance': y_bottom_distance,
                'estimated_distance': estimated_distance
            }
        except Exception:
            return None
    
    def analyze_fov_position(self, x_camera):
        """Analyze FOV position - PRESERVED"""
        try:
            max_x_at_1m = 1000 * math.tan(math.radians(self.camera_hfov_degrees / 2))
            x_position_ratio = x_camera / max_x_at_1m if max_x_at_1m > 0 else 0
            x_position_ratio = max(-1.0, min(1.0, x_position_ratio))
            
            is_at_edge = abs(x_position_ratio) > self.edge_threshold
            distance_from_center = abs(x_position_ratio)
            
            return {
                'x_position_ratio': x_position_ratio,
                'is_at_edge': is_at_edge,
                'distance_from_center': distance_from_center
            }
        except Exception:
            return {'x_position_ratio': 0, 'is_at_edge': False, 'distance_from_center': 0}
    
    def detect_z_flickering(self, current_z):
        """Detect z distance flickering - PRESERVED"""
        self.z_distance_history.append(current_z)
        
        if len(self.z_distance_history) < 5:
            return False, 0
        
        recent_distances = list(self.z_distance_history)[-5:]
        recent_avg = sum(recent_distances[:-1]) / len(recent_distances[:-1])
        z_change = abs(current_z - recent_avg)
        
        adaptive_threshold = self.flicker_threshold + (current_z * 0.05)
        is_flickering = z_change > adaptive_threshold
        
        return is_flickering, z_change
    
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
    
    def calculate_detection_rate(self):
        """Calculate detection rate - MINIMAL"""
        current_time = time.time()
        time_since_last = current_time - self.last_detection_time
        
        if time_since_last > 0:
            detection_rate = 1.0 / time_since_last
            self.detection_rate_history.append(detection_rate)
            
            if len(self.detection_rate_history) > 0:
                self.current_detection_rate = sum(self.detection_rate_history) / len(self.detection_rate_history)
        
        self.last_detection_time = current_time
    
    def process_detections(self):
        """Process detections - PRESERVED logic, SILENT"""
        if self.test_state != "running":
            return None
        
        start_time = time.time()
        self.frame_counter += 1
        
        if self.frame_counter % self.detection_skip_frames != 0:
            return None
        
        camera_mode = "real" if (self.has_detection and self.detection_queue and self.camera_initialized) else "simulation"
        
        if camera_mode == "simulation":
            return self.simulate_detection_for_testing(camera_mode)
        
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
            
            y_method_data = self.calculate_y_distance_method(closest_person)
            fov_analysis = self.analyze_fov_position(x_camera)
            is_flickering, z_change = self.detect_z_flickering(smoothed_z_depth)
            
            processing_time = (time.time() - start_time) * 1000
            test_elapsed = self.get_test_time_elapsed()
            time_remaining = self.get_test_time_remaining()
            
            analysis_data = {
                'test_time_elapsed': test_elapsed,
                'timestamp': time.time(),
                'frame_number': self.frame_counter,
                'fps': self.current_fps,
                'x_camera': x_camera,
                'y_camera': y_camera,
                'z_depth': smoothed_z_depth,
                'raw_z_depth': raw_z_depth,
                'confidence': confidence,
                'bbox_xmin': closest_person.xmin,
                'bbox_ymin': closest_person.ymin,
                'bbox_xmax': closest_person.xmax,
                'bbox_ymax': closest_person.ymax,
                'y_method_data': y_method_data,
                'fov_analysis': fov_analysis,
                'is_flickering': is_flickering,
                'z_change': z_change,
                'processing_time_ms': processing_time,
                'time_remaining': time_remaining,
                'camera_mode': camera_mode
            }
            
            if is_flickering:
                flicker_event = {
                    'timestamp': analysis_data['timestamp'],
                    'test_time_elapsed': test_elapsed,
                    'frame_number': self.frame_counter,
                    'x_camera': x_camera,
                    'z_change': z_change,
                    'is_at_edge': fov_analysis['is_at_edge'],
                    'fps': self.current_fps
                }
                self.flicker_events.append(flicker_event)
            
            self.log_to_csv(analysis_data)
            self.current_detection = analysis_data
            self.detection_history.append(analysis_data)
            self.calculate_detection_rate()
            
            return analysis_data
        except Exception:
            return None  # SILENT
    
    def simulate_detection_for_testing(self, camera_mode="simulation"):
        """Simulation mode fallback - SILENT"""
        if self.test_state != "running":
            return None
        
        try:
            current_time = time.time()
            test_elapsed = self.get_test_time_elapsed()
            
            cycle_time = test_elapsed % 12.0
            x_camera = -900 + (cycle_time / 12.0) * 1800
            y_camera = -200
            
            base_z = 2500
            if abs(x_camera) > 700:
                flicker_amount = 500 * math.sin(current_time * 8)
                z_depth = base_z + flicker_amount
            else:
                z_depth = base_z + 50 * math.sin(current_time * 2)
            
            confidence = 0.85
            
            class MockDetection:
                def __init__(self, x, y, z, conf):
                    self.spatialCoordinates = type('SpatialCoords', (), {'x': x, 'y': y, 'z': z})()
                    self.confidence = conf
                    self.xmin = 0.3
                    self.ymin = 0.2
                    self.xmax = 0.7
                    self.ymax = 0.8
            
            mock_person = MockDetection(x_camera, y_camera, z_depth, confidence)
            
            y_method_data = self.calculate_y_distance_method(mock_person)
            fov_analysis = self.analyze_fov_position(x_camera)
            is_flickering, z_change = self.detect_z_flickering(z_depth)
            
            time_remaining = self.get_test_time_remaining()
            
            analysis_data = {
                'test_time_elapsed': test_elapsed,
                'timestamp': current_time,
                'frame_number': self.frame_counter,
                'fps': self.current_fps,
                'x_camera': x_camera,
                'y_camera': y_camera,
                'z_depth': z_depth,
                'raw_z_depth': z_depth,
                'confidence': confidence,
                'bbox_xmin': mock_person.xmin,
                'bbox_ymin': mock_person.ymin,
                'bbox_xmax': mock_person.xmax,
                'bbox_ymax': mock_person.ymax,
                'y_method_data': y_method_data,
                'fov_analysis': fov_analysis,
                'is_flickering': is_flickering,
                'z_change': z_change,
                'processing_time_ms': 5.0,
                'time_remaining': time_remaining,
                'camera_mode': camera_mode
            }
            
            self.log_to_csv(analysis_data)
            self.current_detection = analysis_data
            self.detection_history.append(analysis_data)
            
            return analysis_data
        except Exception:
            return None
    
    def log_to_csv(self, data):
        """Log to CSV - SILENT"""
        try:
            y_method = data.get('y_method_data', {})
            fov_analysis = data.get('fov_analysis', {})
            
            with open(self.log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    data.get('test_time_elapsed', 0),
                    data['timestamp'],
                    data.get('frame_number', 0),
                    data.get('fps', 0),
                    data['x_camera'],
                    data['y_camera'],
                    data['z_depth'],
                    data.get('raw_z_depth', data['z_depth']),
                    data['confidence'],
                    data['bbox_xmin'],
                    data['bbox_ymin'],
                    data['bbox_xmax'],
                    data['bbox_ymax'],
                    y_method.get('y_bottom_distance', 0),
                    y_method.get('estimated_distance', 0),
                    fov_analysis.get('x_position_ratio', 0),
                    fov_analysis.get('is_at_edge', False),
                    data['is_flickering'],
                    data['z_change'],
                    fov_analysis.get('distance_from_center', 0),
                    data.get('processing_time_ms', 0),
                    data.get('time_remaining', 0),
                    data.get('camera_mode', 'unknown')
                ])
        except Exception:
            pass  # SILENT
    
    def draw_detection_bounding_box(self, camera_x, camera_y):
        """Draw green bounding box around detected person"""
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
            
            # Draw green bounding box
            box_color = (0, 255, 0)  # Bright green
            box_thickness = 3  # Thicker for visibility
            
            pygame.draw.rect(self.screen, box_color, 
                           (abs_x, abs_y, box_width, box_height), box_thickness)
            
            # Add confidence and distance labels
            confidence = self.current_detection.get('confidence', 0)
            conf_text = self.small_font.render(f"{confidence:.2f}", True, box_color)
            conf_x = abs_x
            conf_y = max(abs_y - 25, camera_y)
            self.screen.blit(conf_text, (conf_x, conf_y))
            
            z_depth = self.current_detection.get('z_depth', 0)
            dist_text = self.small_font.render(f"{z_depth:.0f}mm", True, box_color)
            dist_x = abs_x
            dist_y = min(abs_y + box_height + 5, camera_y + display_height - 20)
            self.screen.blit(dist_text, (dist_x, dist_y))
            
        except Exception:
            pass
    
    def draw_waiting_screen(self):
        """Draw waiting screen - MINIMAL"""
        self.screen.fill((0, 0, 0))  # Black background
        
        # Only show "Press ENTER to start test"
        start_text = self.large_font.render("Press ENTER to start test", True, (0, 255, 0))
        start_rect = start_text.get_rect(center=(self.screen_width // 2, self.screen_height // 2))
        self.screen.blit(start_text, start_rect)
        
        # ESC to exit
        exit_text = self.small_font.render("Press ESC to exit", True, (255, 255, 255))
        exit_rect = exit_text.get_rect(center=(self.screen_width // 2, self.screen_height // 2 + 100))
        self.screen.blit(exit_text, exit_rect)
    
    def draw_countdown_screen(self):
        """Draw countdown screen - MINIMAL"""
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
        """Draw test screen - Camera + FPS only"""
        self.screen.fill((0, 0, 0))
        
        # 1. Large FPS display in top-left
        if self.current_fps >= 22:
            fps_color = (0, 255, 0)      # Green
        elif self.current_fps >= 18:
            fps_color = (255, 255, 0)    # Yellow  
        else:
            fps_color = (255, 0, 0)      # Red
        
        fps_text = self.huge_font.render(f"{self.current_fps:.1f}", True, fps_color)
        self.screen.blit(fps_text, (50, 50))
        
        # 2. Camera display in center-right
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
            
            # Draw green bounding boxes around detected people
            if self.current_detection and self.current_detection.get('camera_mode') == 'real':
                self.draw_detection_bounding_box(camera_x, camera_y)
        else:
            # Show camera status if not working
            if not self.camera_initialized:
                status_text = self.small_font.render("SIMULATION MODE", True, (255, 255, 0))
            else:
                status_text = self.small_font.render("NO FRAMES", True, (255, 100, 0))
            
            status_rect = status_text.get_rect(center=(camera_x + self.camera_display_size[0]//2, 
                                                     camera_y + self.camera_display_size[1]//2))
            self.screen.blit(status_text, status_rect)
        
        # Camera border
        border_color = (0, 255, 0) if self.camera_initialized else (255, 255, 0)
        pygame.draw.rect(self.screen, border_color, 
                        (camera_x - 2, camera_y - 2, 
                         self.camera_display_size[0] + 4, 
                         self.camera_display_size[1] + 4), 2)
    
    def draw_finished_screen(self):
        """Draw finished screen - MINIMAL"""
        self.screen.fill((0, 0, 0))
        
        # Show "TEST COMPLETE"
        complete_text = self.huge_font.render("TEST COMPLETE", True, (0, 255, 0))
        complete_rect = complete_text.get_rect(center=(self.screen_width // 2, self.screen_height // 2))
        self.screen.blit(complete_text, complete_rect)
        
        # ESC to exit
        exit_text = self.large_font.render("Press ESC to exit", True, (255, 255, 255))
        exit_rect = exit_text.get_rect(center=(self.screen_width // 2, self.screen_height // 2 + 100))
        self.screen.blit(exit_text, exit_rect)
    
    def update_test_state(self):
        """Update test state - SILENT"""
        current_time = time.time()
        
        if self.test_state == "countdown":
            if current_time - self.countdown_start_time >= self.countdown_duration:
                self.test_state = "running"
                self.test_start_time = current_time
                self.initialize_csv_log()
        
        elif self.test_state == "running":
            if current_time - self.test_start_time >= self.test_duration:
                self.test_state = "finished"
                self.generate_summary()  # Only output summary
    
    def generate_summary(self):
        """Generate MINIMAL summary - ONLY final FPS average"""
        try:
            detection_count = len(self.detection_history)
            
            if len(self.all_fps_values) > 0:
                average_fps = sum(self.all_fps_values) / len(self.all_fps_values)
                min_fps = min(self.all_fps_values)
                max_fps = max(self.all_fps_values)
            else:
                average_fps = min_fps = max_fps = 0
            
            # ONLY OUTPUT: Essential summary
            print(f"Average FPS: {average_fps:.2f}")
            print(f"FPS Range: {min_fps:.1f} - {max_fps:.1f}")
            print(f"Detections: {detection_count}")
            print(f"Data: {self.log_filename}")
            
        except Exception:
            pass  # SILENT
    
    def run(self):
        """Run the MINIMAL test"""
        # SILENT startup - only critical error messages
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
            pass  # SILENT
        finally:
            self.cleanup()
        
        return True
    
    def cleanup(self):
        """Cleanup - SILENT"""
        try:
            if self.device:
                self.device.close()
            pygame.quit()
        except Exception:
            pass


def main():
    """Main function - MINIMAL output"""
    tester = MinimalCameraDetectionTester()
    
    try:
        tester.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        tester.cleanup()


if __name__ == "__main__":
    main()