#!/usr/bin/env python3
"""
Fixed Enhanced Camera Depth Test - Based on Working Debug Tester
Optimized for Maxine Robot - Detection Rate & Edge Optimization
Quick 10-second tests focused on gap reduction and edge performance
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
    print("✅ DepthAI and OpenCV imports successful")
except ImportError as e:
    print(f"❌ Missing required libraries: {e}")
    print("Install with: pip install depthai-python opencv-python")
    exit(1)


class FixedEnhancedCameraTester:
    """Fixed enhanced camera tester based on working debug_tester pipeline"""
    
    def __init__(self):
        # Initialize pygame for display
        pygame.init()
        pygame.font.init()
        
        # Get fullscreen resolution
        display_info = pygame.display.Info()
        self.screen_width = display_info.current_w
        self.screen_height = display_info.current_h
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.FULLSCREEN)
        pygame.display.set_caption("Fixed Enhanced Camera Test - Maxine Robot Optimization")
        
        # Hide mouse cursor
        pygame.mouse.set_visible(False)
        
        # Fonts
        self.large_font = pygame.font.Font(None, 64)
        self.medium_font = pygame.font.Font(None, 48)
        self.small_font = pygame.font.Font(None, 32)
        self.tiny_font = pygame.font.Font(None, 24)
        
        # Initialize DepthAI camera - using working debug_tester approach
        self.device = None
        self.pipeline = None
        self.detection_queue = None
        self.preview_queue = None
        self.has_detection = False
        
        # Quick test configuration (10 seconds each)
        self.test_duration = 10  # Reduced from 30 to 10 seconds
        self.quick_tests = [
            {'name': 'baseline', 'confidence': 0.5, 'bbox_scale': 0.5, 'description': 'Standard settings'},
            {'name': 'low_conf', 'confidence': 0.3, 'bbox_scale': 0.5, 'description': 'Lower confidence for more detections'},
            {'name': 'very_low_conf', 'confidence': 0.2, 'bbox_scale': 0.5, 'description': 'Very low confidence threshold'},
            {'name': 'wide_bbox', 'confidence': 0.4, 'bbox_scale': 0.7, 'description': 'Wider bounding boxes'},
            {'name': 'edge_optimized', 'confidence': 0.25, 'bbox_scale': 0.6, 'description': 'Optimized for edge detection'}
        ]
        
        self.current_test_index = 0
        self.current_test = self.quick_tests[0]
        self.test_start_time = 0
        self.test_results = []
        
        # Maxine Robot optimization data tracking
        self.gap_data = []  # For gap reduction analysis
        self.edge_performance = {'left': [], 'right': [], 'center': []}  # Edge detection tracking
        self.z_depth_stability = []  # Z-depth accuracy tracking
        
        # Real-time tracking
        self.total_frames = 0
        self.detection_frames = 0
        self.current_detection = None
        self.last_detection_time = 0
        self.gap_start_time = None
        self.current_gap_duration = 0
        self.detection_history = deque(maxlen=100)
        
        # Gap tracking for Maxine optimization
        self.gap_events = []
        self.flicker_events = []
        self.edge_loss_events = []  # When detection is lost at edges
        
        # Camera specs for Oak-d pro W (from user preferences)
        self.camera_hfov_degrees = 127
        self.z_flicker_threshold = 300  # mm
        
        # MobileNet class mapping (from working debug_tester)
        self.mobilenet_classes = {
            0: 'background', 1: 'aeroplane', 2: 'bicycle', 3: 'bird', 4: 'boat', 5: 'bottle',
            6: 'bus', 7: 'car', 8: 'cat', 9: 'chair', 10: 'cow', 11: 'diningtable',
            12: 'dog', 13: 'horse', 14: 'motorbike', 15: 'person', 16: 'pottedplant',
            17: 'sheep', 18: 'sofa', 19: 'train', 20: 'tvmonitor'
        }
        
        # File logging for Maxine optimization
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.optimization_log = f"maxine_optimization_data_{timestamp}.csv"
        self.gap_analysis_log = f"maxine_gap_analysis_{timestamp}.csv"
        self.edge_performance_log = f"maxine_edge_performance_{timestamp}.csv"
        
        self.initialize_optimization_logs()
        
        # Control flags
        self.running = True
        self.paused = False
        self.test_active = False
        self.completed_tests = []  # Track which tests have been completed
        
        print("\n🤖 MAXINE ROBOT - ENHANCED CAMERA OPTIMIZATION")
        print("=" * 55)
        print("🎯 Quick Optimization Tests (10 seconds each):")
        for i, test in enumerate(self.quick_tests, 1):
            print(f"  {i}. {test['name']}: {test['description']}")
        print("\n📊 Optimization Focus:")
        print("  • Detection gap reduction")
        print("  • Z-depth stability improvement") 
        print("  • Edge detection enhancement")
        print("  • Real-time performance metrics")
        print("=" * 55)
    
    def initialize_optimization_logs(self):
        """Initialize CSV logs for Maxine robot optimization data"""
        try:
            # Main optimization data
            with open(self.optimization_log, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'test_name', 'confidence_threshold', 'bbox_scale', 'timestamp',
                    'x_camera', 'y_camera', 'z_depth', 'detection_confidence',
                    'x_position_ratio', 'edge_zone', 'distance_from_center',
                    'z_flicker_detected', 'z_change_amount', 'gap_duration_before',
                    'frame_number', 'detection_rate_current'
                ])
            
            # Gap analysis for detection consistency
            with open(self.gap_analysis_log, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'test_name', 'gap_start_time', 'gap_duration', 'gap_end_reason',
                    'last_x_position', 'last_edge_zone', 'confidence_threshold',
                    'frames_since_last_detection'
                ])
            
            # Edge performance analysis
            with open(self.edge_performance_log, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'test_name', 'edge_zone', 'detections_count', 'flicker_count',
                    'avg_z_depth', 'z_stability_score', 'avg_confidence',
                    'detection_consistency_score'
                ])
            
            print(f"📁 Optimization logs initialized:")
            print(f"  • Main data: {self.optimization_log}")
            print(f"  • Gap analysis: {self.gap_analysis_log}")
            print(f"  • Edge performance: {self.edge_performance_log}")
            
        except Exception as e:
            print(f"⚠️ Failed to initialize optimization logs: {e}")
    
    def create_working_pipeline(self):
        """Create pipeline based on working debug_tester configuration"""
        try:
            pipeline = dai.Pipeline()
            
            # Color camera - exact same as working debug_tester
            cam_rgb = pipeline.create(dai.node.ColorCamera)
            cam_rgb.setPreviewSize(416, 416)
            cam_rgb.setInterleaved(False)
            cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            cam_rgb.setFps(15)
            
            # ImageManip - exact same as working debug_tester
            manip = pipeline.create(dai.node.ImageManip)
            manip.initialConfig.setResize(300, 300)
            manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
            cam_rgb.preview.link(manip.inputImage)
            
            # Stereo depth - exact same as working debug_tester
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
            
            # MobileNet with current test parameters
            detection_nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
            
            # Load blob - same as working debug_tester
            local_blob_path = "./mobilenet-ssd_openvino_2022.1_6shave.blob"
            if os.path.exists(local_blob_path):
                detection_nn.setBlobPath(local_blob_path)
                print(f"✅ Using working blob: {local_blob_path}")
            else:
                print("❌ Blob not found - required for optimization tests")
                return None
            
            # Set current test parameters
            detection_nn.setConfidenceThreshold(self.current_test['confidence'])
            detection_nn.setBoundingBoxScaleFactor(self.current_test['bbox_scale'])
            detection_nn.setDepthLowerThreshold(50)   # Wide range for testing
            detection_nn.setDepthUpperThreshold(10000)
            
            # Link pipeline - exact same as working debug_tester
            manip.out.link(detection_nn.input)
            depth.depth.link(detection_nn.inputDepth)
            
            # Create outputs
            detection_out = pipeline.create(dai.node.XLinkOut)
            detection_out.setStreamName("detections")
            detection_nn.out.link(detection_out.input)
            
            preview_out = pipeline.create(dai.node.XLinkOut)
            preview_out.setStreamName("preview")
            manip.out.link(preview_out.input)
            
            print(f"✅ Working pipeline created for test: {self.current_test['name']}")
            print(f"   Confidence: {self.current_test['confidence']}")
            print(f"   BBox Scale: {self.current_test['bbox_scale']}")
            
            return pipeline
            
        except Exception as e:
            print(f"❌ Failed to create working pipeline: {e}")
            return None
    
    def initialize_camera(self):
        """Initialize camera with working debug_tester approach"""
        try:
            print(f"📷 Initializing camera for test: {self.current_test['name']}...")
            
            self.pipeline = self.create_working_pipeline()
            if not self.pipeline:
                return False
            
            # Close previous device if exists
            if self.device:
                self.device.close()
                time.sleep(1)
            
            self.device = dai.Device(self.pipeline)
            
            # Get output queues - same as working debug_tester
            try:
                self.detection_queue = self.device.getOutputQueue("detections", maxSize=4, blocking=False)
                self.has_detection = True
                print("✅ Detection queue available")
            except RuntimeError:
                self.detection_queue = None
                self.has_detection = False
                print("❌ No detection queue available")
                return False
            
            try:
                self.preview_queue = self.device.getOutputQueue("preview", maxSize=4, blocking=False)
            except RuntimeError:
                self.preview_queue = None
            
            print("✅ Camera initialized successfully")
            time.sleep(1)  # Brief stabilization
            return True
            
        except Exception as e:
            print(f"❌ Failed to initialize camera: {e}")
            return False
    
    def calculate_edge_zone(self, x_camera):
        """Calculate edge zone and position analysis for Maxine robot"""
        # FOV analysis for edge detection optimization
        max_x_at_1m = 1000 * math.tan(math.radians(self.camera_hfov_degrees / 2))
        x_position_ratio = x_camera / max_x_at_1m if max_x_at_1m > 0 else 0
        x_position_ratio = max(-1.0, min(1.0, x_position_ratio))
        
        # Edge zone classification for Maxine optimization
        distance_from_center = abs(x_position_ratio)
        
        if x_position_ratio < -0.6:
            edge_zone = 'left'
        elif x_position_ratio > 0.6:
            edge_zone = 'right'
        else:
            edge_zone = 'center'
        
        return {
            'x_position_ratio': x_position_ratio,
            'edge_zone': edge_zone,
            'distance_from_center': distance_from_center,
            'is_edge': distance_from_center > 0.6
        }
    
    def detect_z_flicker(self, current_z):
        """Detect Z-depth flickering for stability analysis"""
        if len(self.detection_history) < 3:
            return False, 0
        
        recent_z_values = [d['z_depth'] for d in list(self.detection_history)[-3:]]
        avg_recent_z = sum(recent_z_values) / len(recent_z_values)
        z_change = abs(current_z - avg_recent_z)
        
        is_flickering = z_change > self.z_flicker_threshold
        return is_flickering, z_change
    
    def process_optimized_detections(self):
        """Process detections with Maxine robot optimization focus"""
        self.total_frames += 1
        current_time = time.time()
        
        if not self.has_detection or not self.detection_queue:
            return None
        
        try:
            detections = self.detection_queue.tryGet()
            if detections:
                # Find person detections (class 15) - same as working debug_tester
                person_detections = [det for det in detections.detections if det.label == 15]
                
                if person_detections:
                    # Get best person detection
                    best_person = max(person_detections, key=lambda p: p.confidence)
                    
                    # Extract spatial coordinates
                    x_camera = best_person.spatialCoordinates.x
                    y_camera = best_person.spatialCoordinates.y
                    z_depth = best_person.spatialCoordinates.z
                    confidence = best_person.confidence
                    
                    # Validate detection with current test threshold
                    if (confidence >= self.current_test['confidence'] and 
                        100 < z_depth < 15000):  # Valid depth range
                        
                        self.detection_frames += 1
                        
                        # Calculate optimization metrics
                        edge_analysis = self.calculate_edge_zone(x_camera)
                        is_flickering, z_change = self.detect_z_flicker(z_depth)
                        
                        # Calculate gap duration before this detection
                        gap_duration_before = 0
                        if self.gap_start_time:
                            gap_duration_before = current_time - self.gap_start_time
                            
                            # Log gap event for analysis
                            self.gap_events.append({
                                'test_name': self.current_test['name'],
                                'duration': gap_duration_before,
                                'end_reason': 'detection_restored',
                                'confidence_threshold': self.current_test['confidence']
                            })
                            
                            self.gap_start_time = None
                            self.current_gap_duration = 0
                        
                        # Compile optimization data
                        detection_data = {
                            'timestamp': current_time,
                            'x_camera': x_camera,
                            'y_camera': y_camera,
                            'z_depth': z_depth,
                            'confidence': confidence,
                            'edge_analysis': edge_analysis,
                            'is_flickering': is_flickering,
                            'z_change': z_change,
                            'gap_duration_before': gap_duration_before,
                            'frame_number': self.total_frames,
                            'detection_rate': self.detection_frames / self.total_frames
                        }
                        
                        # Track edge performance for Maxine optimization
                        edge_zone = edge_analysis['edge_zone']
                        self.edge_performance[edge_zone].append({
                            'z_depth': z_depth,
                            'confidence': confidence,
                            'is_flickering': is_flickering,
                            'z_change': z_change
                        })
                        
                        # Track Z-depth stability
                        self.z_depth_stability.append({
                            'z_depth': z_depth,
                            'z_change': z_change,
                            'edge_zone': edge_zone,
                            'is_stable': not is_flickering
                        })
                        
                        # Log optimization data
                        self.log_optimization_data(detection_data)
                        
                        # Update tracking
                        self.current_detection = detection_data
                        self.detection_history.append(detection_data)
                        self.last_detection_time = current_time
                        
                        return detection_data
            
            # No valid detection - track gap
            self.handle_detection_gap(current_time)
            return None
            
        except Exception as e:
            print(f"❌ Detection processing error: {e}")
            return None
    
    def handle_detection_gap(self, current_time):
        """Handle detection gaps for Maxine optimization analysis"""
        if self.gap_start_time is None:
            self.gap_start_time = current_time
        
        self.current_gap_duration = current_time - self.gap_start_time
        
        # If gap is getting long, record it as significant
        if self.current_gap_duration > 2.0:  # 2+ second gaps are problematic for Maxine
            # Record where we lost detection
            last_position = None
            last_edge_zone = 'unknown'
            
            if self.current_detection:
                last_position = self.current_detection['x_camera']
                last_edge_zone = self.current_detection['edge_analysis']['edge_zone']
            
            # Log gap for analysis
            gap_data = {
                'test_name': self.current_test['name'],
                'gap_start_time': self.gap_start_time,
                'gap_duration': self.current_gap_duration,
                'gap_end_reason': 'ongoing',
                'last_x_position': last_position,
                'last_edge_zone': last_edge_zone,
                'confidence_threshold': self.current_test['confidence'],
                'frames_since_last_detection': self.total_frames - self.detection_frames
            }
            
            self.gap_data.append(gap_data)
    
    def log_optimization_data(self, data):
        """Log optimization data for Maxine robot analysis"""
        try:
            edge_analysis = data['edge_analysis']
            
            with open(self.optimization_log, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    self.current_test['name'],
                    self.current_test['confidence'],
                    self.current_test['bbox_scale'],
                    data['timestamp'],
                    data['x_camera'],
                    data['y_camera'],
                    data['z_depth'],
                    data['confidence'],
                    edge_analysis['x_position_ratio'],
                    edge_analysis['edge_zone'],
                    edge_analysis['distance_from_center'],
                    data['is_flickering'],
                    data['z_change'],
                    data['gap_duration_before'],
                    data['frame_number'],
                    data['detection_rate']
                ])
        except Exception:
            pass  # Continue test even if logging fails
    
    def advance_to_next_test(self):
        """Advance to next optimization test"""
        # Save current test results
        self.save_current_test_results()
        
        # Move to next test
        self.current_test_index += 1
        
        if self.current_test_index >= len(self.quick_tests):
            # All tests complete
            print("✅ All optimization tests completed!")
            self.test_active = False
            self.running = False  # End the program
            return False
        
        # Setup next test
        self.current_test = self.quick_tests[self.current_test_index]
        print(f"\n🔄 Next test {self.current_test_index + 1}/{len(self.quick_tests)}: {self.current_test['name']}")
        print(f"   {self.current_test['description']}")
        print("   Press ENTER to start this test...")
        
        # Reset statistics for new test
        self.reset_test_statistics()
        
        return True
    
    def save_current_test_results(self):
        """Save current test results for analysis"""
        try:
            print(f"💾 Saving results for test: {self.current_test['name']}")
            
            # Calculate test metrics
            detection_rate = self.detection_frames / max(self.total_frames, 1)
            gap_count = len([g for g in self.gap_data if g['test_name'] == self.current_test['name']])
            
            # Save gap analysis
            gaps_saved = 0
            for gap in self.gap_data:
                if gap['test_name'] == self.current_test['name']:
                    with open(self.gap_analysis_log, 'a', newline='') as csvfile:
                        writer = csv.writer(csvfile)
                        writer.writerow([
                            gap['test_name'], gap['gap_start_time'], gap['gap_duration'],
                            gap['gap_end_reason'], gap['last_x_position'], gap['last_edge_zone'],
                            gap['confidence_threshold'], gap['frames_since_last_detection']
                        ])
                    gaps_saved += 1
            
            # Save edge performance analysis
            edges_saved = 0
            for edge_zone, performances in self.edge_performance.items():
                if performances:
                    detections_count = len(performances)
                    flicker_count = sum(1 for p in performances if p['is_flickering'])
                    avg_z_depth = sum(p['z_depth'] for p in performances) / detections_count
                    avg_confidence = sum(p['confidence'] for p in performances) / detections_count
                    z_stability_score = 100 * (1 - flicker_count / detections_count)
                    
                    with open(self.edge_performance_log, 'a', newline='') as csvfile:
                        writer = csv.writer(csvfile)
                        writer.writerow([
                            self.current_test['name'], edge_zone, detections_count, flicker_count,
                            avg_z_depth, z_stability_score, avg_confidence, detection_rate * 100
                        ])
                    edges_saved += 1
            
            print(f"📊 Test '{self.current_test['name']}' results: {detection_rate:.1%} detection rate, {gap_count} gaps")
            print(f"💾 Saved {gaps_saved} gap records, {edges_saved} edge performance records")
            
            # Mark test as completed
            if self.current_test['name'] not in self.completed_tests:
                self.completed_tests.append(self.current_test['name'])
                print(f"✅ Test '{self.current_test['name']}' marked as completed")
            
        except Exception as e:
            print(f"❌ Failed to save test results: {e}")
    
    def reset_test_statistics(self):
        """Reset statistics for new test"""
        print(f"🔄 Resetting statistics for new test")
        self.total_frames = 0
        self.detection_frames = 0
        self.current_detection = None
        self.gap_start_time = None
        self.current_gap_duration = 0
        self.detection_history.clear()
        
        # Clear per-test data (but keep cumulative data in files)
        # Note: gap_data and edge_performance contain historical data for all tests
        # We filter by test_name when analyzing, so we don't clear them here
    
    def draw_optimization_display(self):
        """Draw optimization test display"""
        self.screen.fill((0, 0, 0))
        
        # Title with test progress
        title = self.large_font.render(f"MAXINE OPTIMIZATION - TEST {self.current_test_index + 1}/{len(self.quick_tests)}", True, (255, 255, 0))
        self.screen.blit(title, (50, 20))
        
        # Current test info
        test_info = self.medium_font.render(f"{self.current_test['name'].upper()}: {self.current_test['description']}", True, (0, 255, 255))
        self.screen.blit(test_info, (50, 80))
        
        y_offset = 130
        
        # Test progress
        if self.test_active:
            elapsed = time.time() - self.test_start_time
            progress = min(elapsed / self.test_duration, 1.0)
            remaining = max(self.test_duration - elapsed, 0)
            
            progress_text = self.medium_font.render(f"Progress: {progress:.0%} ({remaining:.1f}s remaining)", True, (255, 255, 255))
            self.screen.blit(progress_text, (50, y_offset))
            y_offset += 40
            
            # Progress bar
            bar_width = 400
            bar_height = 20
            pygame.draw.rect(self.screen, (100, 100, 100), (50, y_offset, bar_width, bar_height), 2)
            pygame.draw.rect(self.screen, (0, 255, 0), (52, y_offset + 2, int((bar_width - 4) * progress), bar_height - 4))
            y_offset += 40
        
        # Current test metrics
        detection_rate = self.detection_frames / max(self.total_frames, 1)
        gap_count = len([g for g in self.gap_data if g['test_name'] == self.current_test['name']])
        
        metrics = [
            f"Detection Rate: {detection_rate:.1%}",
            f"Frames: {self.detection_frames}/{self.total_frames}",
            f"Gaps: {gap_count}",
            f"Current Gap: {self.current_gap_duration:.1f}s",
            f"Confidence Threshold: {self.current_test['confidence']}",
            f"BBox Scale: {self.current_test['bbox_scale']}"
        ]
        
        y_offset += 20
        for metric in metrics:
            color = (0, 255, 0) if "Rate:" in metric and detection_rate > 0.8 else (255, 255, 255)
            if "Gap:" in metric and self.current_gap_duration > 1:
                color = (255, 0, 0)
            
            text = self.small_font.render(metric, True, color)
            self.screen.blit(text, (50, y_offset))
            y_offset += 30
        
        # Current detection info
        if self.current_detection:
            y_offset += 20
            detection_title = self.medium_font.render("CURRENT DETECTION:", True, (255, 255, 0))
            self.screen.blit(detection_title, (50, y_offset))
            y_offset += 40
            
            data = self.current_detection
            edge_analysis = data['edge_analysis']
            
            detection_info = [
                f"Position: X={data['x_camera']:.0f}mm, Z={data['z_depth']:.0f}mm",
                f"Edge Zone: {edge_analysis['edge_zone']} ({edge_analysis['distance_from_center']:.2f})",
                f"Confidence: {data['confidence']:.3f}",
                f"Z-Flicker: {'YES' if data['is_flickering'] else 'NO'} ({data['z_change']:.0f}mm)"
            ]
            
            for info in detection_info:
                color = (255, 255, 255)
                if "Z-Flicker: YES" in info:
                    color = (255, 100, 100)
                elif "Edge Zone: left" in info or "Edge Zone: right" in info:
                    color = (255, 200, 0)
                
                text = self.small_font.render(info, True, color)
                self.screen.blit(text, (50, y_offset))
                y_offset += 25
        
        # Instructions
        if not self.test_active:
            if self.current_test_index == 0 and self.total_frames == 0:
                instruction_text = self.large_font.render("PRESS ENTER TO START FIRST TEST", True, (0, 255, 0))
            elif self.current_test_index < len(self.quick_tests) - 1:
                next_test = self.quick_tests[self.current_test_index + 1] if self.current_test_index + 1 < len(self.quick_tests) else None
                if next_test:
                    instruction_text = self.large_font.render(f"PRESS ENTER FOR TEST {self.current_test_index + 2}/5: {next_test['name'].upper()}", True, (0, 255, 0))
                else:
                    instruction_text = self.large_font.render("ALL TESTS COMPLETED!", True, (0, 255, 0))
            else:
                instruction_text = self.large_font.render("ALL TESTS COMPLETED!", True, (0, 255, 0))
            
            text_rect = instruction_text.get_rect(center=(self.screen_width // 2, self.screen_height // 2))
            self.screen.blit(instruction_text, text_rect)
        
        # Controls
        controls = [
            "ENTER - Start/Next test",
            "SPACE - Pause/Resume", 
            "ESC - Exit"
        ]
        
        control_x = self.screen_width - 300
        control_y = 50
        for control in controls:
            text = self.tiny_font.render(control, True, (255, 100, 100))
            self.screen.blit(text, (control_x, control_y))
            control_y += 25
        
        pygame.display.flip()
    
    def run_optimization_tests(self):
        """Run the Maxine robot optimization tests"""
        print("🤖 Maxine Robot - Enhanced Camera Optimization")
        print("=" * 50)
        
        # Check for blob file
        blob_path = "./mobilenet-ssd_openvino_2022.1_6shave.blob"
        if not os.path.exists(blob_path):
            print(f"❌ ERROR: Blob file not found: {blob_path}")
            return False
        
        print("🚀 Ready to start optimization tests...")
        print("📋 Each test will run for 10 seconds")
        print("🎯 Focus: Gap reduction, Z-depth stability, edge detection")
        print("\n⌨️  Press ENTER when ready to start test 1/5")
        print("⌨️  After each test completes, press ENTER for the next test")
        print("⌨️  ESC to exit, SPACE to pause/resume current test")
        
        clock = pygame.time.Clock()
        
        try:
            while self.running:
                # Handle events
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.running = False
                        elif event.key == pygame.K_RETURN:
                            if not self.test_active:
                                if self.current_test_index == 0 and self.total_frames == 0:
                                    # Start very first test
                                    if self.initialize_camera():
                                        self.test_active = True
                                        self.test_start_time = time.time()
                                        print(f"🚀 Starting test 1/5: {self.current_test['name']}")
                                    else:
                                        print("❌ Failed to initialize camera")
                                else:
                                    # Start next test after previous completed
                                    if self.advance_to_next_test():
                                        # Initialize camera with new parameters for next test
                                        if self.initialize_camera():
                                            self.test_active = True
                                            self.test_start_time = time.time()
                                            print(f"🚀 Starting test {self.current_test_index + 1}/5: {self.current_test['name']}")
                                        else:
                                            print("❌ Failed to initialize camera for next test")
                                    # If advance_to_next_test() returns False, all tests are done and self.running is False
                        elif event.key == pygame.K_SPACE:
                            self.paused = not self.paused
                            print(f"{'PAUSED' if self.paused else 'RESUMED'}")
                    elif event.type == pygame.QUIT:
                        self.running = False
                
                # Process test
                if self.test_active and not self.paused:
                    # Check if test time completed
                    elapsed = time.time() - self.test_start_time
                    if elapsed >= self.test_duration:
                        print(f"✅ Test {self.current_test_index + 1}/5 '{self.current_test['name']}' completed")
                        self.test_active = False
                        print("   Press ENTER for next test...")
                    else:
                        # Process detections
                        self.process_optimized_detections()
                
                # Update display
                self.draw_optimization_display()
                
                # 15 FPS
                clock.tick(15)
                
        except KeyboardInterrupt:
            print("\n⏹️ Optimization tests stopped")
        finally:
            self.cleanup()
            
        return True
    
    def cleanup(self):
        """Cleanup with final optimization report"""
        if hasattr(self, '_cleaned_up'):
            return  # Prevent double cleanup
        self._cleaned_up = True
        
        try:
            # Save current test results if test was active
            if self.test_active and hasattr(self, 'current_test'):
                self.save_current_test_results()
            
            if self.device:
                self.device.close()
            
            self.generate_maxine_optimization_report()
            pygame.quit()
            
        except Exception as e:
            print(f"Cleanup error: {e}")
    
    def generate_maxine_optimization_report(self):
        """Generate final optimization report for Maxine robot"""
        try:
            print("\n" + "=" * 70)
            print("🤖 MAXINE ROBOT - OPTIMIZATION ANALYSIS REPORT")
            print("=" * 70)
            
            print(f"📊 Test Summary:")
            completed_count = len(self.completed_tests)
            print(f"   Tests Completed: {completed_count}/{len(self.quick_tests)}")
            if completed_count > 0:
                print(f"   Completed Tests: {', '.join(self.completed_tests)}")
            print(f"   Total Detection Frames: {self.detection_frames}")
            print(f"   Total Test Frames: {self.total_frames}")
            
            # Gap analysis summary
            all_gaps = self.gap_data
            if all_gaps:
                avg_gap_duration = sum(g['gap_duration'] for g in all_gaps) / len(all_gaps)
                max_gap_duration = max(g['gap_duration'] for g in all_gaps)
                gap_count = len(all_gaps)
                
                print(f"\n🔄 Detection Gap Analysis:")
                print(f"   Total Gaps Recorded: {gap_count}")
                print(f"   Average Gap Duration: {avg_gap_duration:.2f}s")
                print(f"   Maximum Gap Duration: {max_gap_duration:.2f}s")
                print(f"   Gaps > 2s (problematic): {len([g for g in all_gaps if g['gap_duration'] > 2])}")
            
            # Edge performance summary
            total_edge_detections = sum(len(performances) for performances in self.edge_performance.values())
            if total_edge_detections > 0:
                print(f"\n📍 Edge Detection Performance:")
                for zone, performances in self.edge_performance.items():
                    if performances:
                        count = len(performances)
                        flickers = sum(1 for p in performances if p['is_flickering'])
                        flicker_rate = flickers / count
                        avg_z = sum(p['z_depth'] for p in performances) / count
                        
                        print(f"   {zone.upper():6}: {count:3} detections, {flicker_rate:5.1%} flicker, {avg_z:6.0f}mm avg Z")
            
            # Z-depth stability summary
            if self.z_depth_stability:
                stable_count = sum(1 for z in self.z_depth_stability if z['is_stable'])
                stability_rate = stable_count / len(self.z_depth_stability)
                avg_z_change = sum(z['z_change'] for z in self.z_depth_stability) / len(self.z_depth_stability)
                
                print(f"\n📏 Z-Depth Stability Analysis:")
                print(f"   Stability Rate: {stability_rate:.1%}")
                print(f"   Average Z-Change: {avg_z_change:.1f}mm")
                print(f"   Stable Readings: {stable_count}/{len(self.z_depth_stability)}")
            
            print(f"\n📁 Optimization Data Files:")
            print(f"   • Main data: {self.optimization_log}")
            print(f"   • Gap analysis: {self.gap_analysis_log}")
            print(f"   • Edge performance: {self.edge_performance_log}")
            
            print(f"\n💡 Next Steps for Maxine Robot:")
            print(f"   1. Analyze CSV files to find optimal confidence threshold")
            print(f"   2. Implement gap detection and recovery strategies")
            print(f"   3. Optimize edge detection for wide FOV navigation")
            print(f"   4. Use Z-depth stability data for motion planning")
            
            print("=" * 70)
            
        except Exception as e:
            print(f"Error generating optimization report: {e}")


def main():
    """Main function"""
    print("🤖 Maxine Robot - Enhanced Camera Optimization Test")
    print("Based on Working Debug Tester Configuration")
    print()
    print("🎯 This test will optimize for:")
    print("  • Reduced detection gaps for continuous tracking")
    print("  • Improved Z-depth stability for accurate navigation")
    print("  • Enhanced edge detection for wide FOV coverage")
    print()
    print("⚡ Quick 10-second tests for efficient optimization")
    print("📊 Generates data-driven recommendations for Maxine")
    print()
    
    tester = FixedEnhancedCameraTester()
    
    try:
        success = tester.run_optimization_tests()
        if success:
            print("🎉 Maxine optimization tests completed!")
        else:
            print("❌ Optimization tests failed")
    except Exception as e:
        print(f"💥 Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tester.cleanup()


if __name__ == "__main__":
    main()