#!/usr/bin/env python3
"""
Comprehensive Intel Model Spatial Detection Tester for Maxine Robot
Tests ALL Intel pre-optimized models and provides data-driven recommendations
Compares spatial detection performance, stability, and accuracy across models
"""
import pygame
import math
import time
import csv
import os
import json
from collections import deque
from datetime import datetime
import threading
import queue
import statistics

try:
    import depthai as dai
    import cv2
    import numpy as np
    print("✅ DepthAI and OpenCV imports successful")
except ImportError as e:
    print(f"❌ Missing required libraries: {e}")
    print("Install with: pip install depthai-python opencv-python")
    exit(1)

try:
    import blobconverter
    print("✅ BlobConverter available for model download")
    BLOBCONVERTER_AVAILABLE = True
except ImportError:
    print("⚠️ BlobConverter not available - install with: pip install blobconverter")
    BLOBCONVERTER_AVAILABLE = False


class IntelModelSpatialTester:
    """Comprehensive tester for Intel pre-optimized spatial detection models"""
    
    def __init__(self):
        # Initialize pygame for display
        pygame.init()
        pygame.font.init()
        
        # Get fullscreen resolution
        display_info = pygame.display.Info()
        self.screen_width = display_info.current_w
        self.screen_height = display_info.current_h
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.FULLSCREEN)
        pygame.display.set_caption("Intel Model Spatial Detection Tester")
        
        # Hide mouse cursor
        pygame.mouse.set_visible(False)
        
        # Fonts
        self.large_font = pygame.font.Font(None, 64)
        self.medium_font = pygame.font.Font(None, 48)
        self.small_font = pygame.font.Font(None, 32)
        
        # Intel Models to Test (FIXED input dimensions and class IDs)
        self.intel_models = {
            'person-detection-retail-0013': {
                'input_size': (544, 320),  # FIXED: Was (320, 544) 
                'description': 'Person detection optimized for retail/spatial applications',
                'expected_class_id': 1,  # Person class
                'confidence_threshold': 0.3,  # Lower threshold for better coverage
                'network_type': 'MobileNetSpatialDetectionNetwork'
            },
            'face-detection-retail-0004': {
                'input_size': (300, 300),
                'description': 'Face detection with proven spatial coordinate stability',
                'expected_class_id': 1,  # Face class
                'confidence_threshold': 0.4,
                'network_type': 'MobileNetSpatialDetectionNetwork'
            },
            'person-detection-0200': {
                'input_size': (256, 256),
                'description': 'Lightweight person detection',
                'expected_class_id': 1,
                'confidence_threshold': 0.3,
                'network_type': 'MobileNetSpatialDetectionNetwork'
            },
            'person-detection-0201': {
                'input_size': (384, 384),
                'description': 'Higher resolution person detection',
                'expected_class_id': 1,
                'confidence_threshold': 0.3,
                'network_type': 'MobileNetSpatialDetectionNetwork'
            },
            'person-detection-0202': {
                'input_size': (512, 512),
                'description': 'High resolution person detection',
                'expected_class_id': 1,
                'confidence_threshold': 0.3,
                'network_type': 'MobileNetSpatialDetectionNetwork'
            },
            'pedestrian-detection-adas-0002': {
                'input_size': (672, 384),  # FIXED: Was (384, 672)
                'description': 'ADAS pedestrian detection',
                'expected_class_id': 1,
                'confidence_threshold': 0.3,
                'network_type': 'MobileNetSpatialDetectionNetwork'
            },
            'mobilenet-ssd': {
                'input_size': (300, 300),
                'description': 'Original MobileNet-SSD (baseline comparison)',
                'expected_class_id': 15,  # Person class in COCO
                'confidence_threshold': 0.5,
                'network_type': 'MobileNetSpatialDetectionNetwork'
            }
        }
        
        # Test parameters
        self.test_duration_per_model = 60  # seconds per model
        self.min_detections_required = 30  # minimum detections for valid test
        
        # Results storage
        self.test_results = {}
        self.current_model = None
        self.current_test_data = []
        
        # Camera specs
        self.camera_hfov_degrees = 127  # Oak-d pro W
        
        # Test control
        self.running = True
        self.testing_active = False
        self.current_model_index = 0
        
        # Results file
        self.results_filename = f"intel_model_comparison_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        self.detailed_csv_filename = f"intel_model_detailed_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
        print("\n🧪 INTEL MODEL SPATIAL DETECTION TESTER")
        print("=" * 50)
        print(f"📊 Will test {len(self.intel_models)} Intel models")
        print(f"⏱️  {self.test_duration_per_model}s per model")
        print(f"📈 Minimum {self.min_detections_required} detections required per model")
        print(f"📁 Results: {self.results_filename}")
        print(f"📋 Detailed data: {self.detailed_csv_filename}")
        
    def download_model_blob(self, model_name, model_config):
        """Download model blob using blobconverter"""
        if not BLOBCONVERTER_AVAILABLE:
            print(f"⚠️ Cannot download {model_name} - blobconverter not available")
            return None
            
        try:
            print(f"📥 Downloading {model_name}...")
            blob_path = blobconverter.from_zoo(
                name=model_name,
                shaves=6,
                zoo_type="intel"
            )
            print(f"✅ Downloaded {model_name} to {blob_path}")
            return blob_path
        except Exception as e:
            print(f"❌ Failed to download {model_name}: {e}")
            return None
    
    def create_model_pipeline(self, model_name, model_config, blob_path):
        """Create DepthAI pipeline for specific Intel model"""
        try:
            pipeline = dai.Pipeline()
            
            # Get input size for this model
            input_width, input_height = model_config['input_size']
            
            # Color camera
            cam_rgb = pipeline.create(dai.node.ColorCamera)
            cam_rgb.setPreviewSize(640, 480)  # High quality source
            cam_rgb.setInterleaved(False)
            cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            cam_rgb.setFps(15)
            
            # ImageManip for model-specific input size
            manip = pipeline.create(dai.node.ImageManip)
            manip.initialConfig.setResize(input_width, input_height)
            manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
            cam_rgb.preview.link(manip.inputImage)
            
            # Stereo depth cameras
            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_right = pipeline.create(dai.node.MonoCamera)
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
            mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            
            # Enhanced depth configuration
            depth = pipeline.create(dai.node.StereoDepth)
            depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
            depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
            depth.initialConfig.setConfidenceThreshold(180)
            depth.setLeftRightCheck(True)
            depth.setSubpixel(True)
            depth.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            
            mono_left.out.link(depth.left)
            mono_right.out.link(depth.right)
            
            # Create spatial detection network
            detection_nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
            detection_nn.setBlobPath(blob_path)
            detection_nn.setConfidenceThreshold(model_config['confidence_threshold'])
            detection_nn.setBoundingBoxScaleFactor(0.5)
            detection_nn.setDepthLowerThreshold(100)
            detection_nn.setDepthUpperThreshold(8000)
            
            # Link inputs
            manip.out.link(detection_nn.input)
            depth.depth.link(detection_nn.inputDepth)
            
            # Create outputs
            detection_out = pipeline.create(dai.node.XLinkOut)
            detection_out.setStreamName("detections")
            detection_nn.out.link(detection_out.input)
            
            preview_out = pipeline.create(dai.node.XLinkOut)
            preview_out.setStreamName("preview")
            manip.out.link(preview_out.input)
            
            print(f"✅ Pipeline created for {model_name} ({input_width}x{input_height})")
            return pipeline
            
        except Exception as e:
            print(f"❌ Failed to create pipeline for {model_name}: {e}")
            return None
    
    def test_single_model(self, model_name, model_config):
        """Test a single Intel model and collect performance data"""
        print(f"\n🧪 TESTING: {model_name}")
        print("=" * 40)
        print(f"📝 {model_config['description']}")
        print(f"📐 Input size: {model_config['input_size']} (W×H)")
        print(f"🎯 Expected class ID: {model_config['expected_class_id']}")
        print(f"📊 Confidence threshold: {model_config['confidence_threshold']}")
        
        # Check for existing blob
        blob_filename = f"{model_name}_openvino_2021.4_6shave.blob"
        if not os.path.exists(blob_filename):
            blob_path = self.download_model_blob(model_name, model_config)
            if not blob_path:
                print(f"❌ Cannot test {model_name} - blob unavailable")
                return None
        else:
            blob_path = blob_filename
            print(f"✅ Using existing blob: {blob_path}")
        
        # Create pipeline
        pipeline = self.create_model_pipeline(model_name, model_config, blob_path)
        if not pipeline:
            return None
        
        # Initialize device
        try:
            device = dai.Device(pipeline)
            detection_queue = device.getOutputQueue("detections", maxSize=4, blocking=False)
            preview_queue = device.getOutputQueue("preview", maxSize=4, blocking=False)
            print("✅ Device initialized successfully")
        except Exception as e:
            print(f"❌ Failed to initialize device for {model_name}: {e}")
            return None
        
        # Test data collection
        test_data = []
        start_time = time.time()
        last_detection_time = start_time
        detection_count = 0
        z_history = deque(maxlen=10)
        total_inference_count = 0
        valid_detection_count = 0
        
        print(f"⏱️  Testing for {self.test_duration_per_model} seconds...")
        print("📋 Stand in front of camera and move slowly left to right")
        print("🔍 Debug: Looking for detections...")
        
        try:
            while (time.time() - start_time) < self.test_duration_per_model:
                # Update display
                self.draw_testing_display(model_name, model_config, test_data, 
                                        time.time() - start_time, detection_count)
                
                # Process detections
                detections = detection_queue.tryGet()
                if detections:
                    total_inference_count += 1
                    
                    # Debug: Show all detections
                    if total_inference_count % 30 == 1:  # Every ~1.5 seconds
                        print(f"🔍 Debug: Found {len(detections.detections)} total detections")
                        for i, det in enumerate(detections.detections[:3]):  # Show first 3
                            print(f"   Detection {i}: Label={det.label}, Conf={det.confidence:.3f}, Z={det.spatialCoordinates.z:.0f}mm")
                    
                    # Find relevant detections
                    relevant_detections = [
                        det for det in detections.detections 
                        if det.label == model_config['expected_class_id'] and det.confidence >= model_config['confidence_threshold']
                    ]
                    
                    if relevant_detections:
                        valid_detection_count += 1
                        
                        # Get closest detection
                        closest = min(relevant_detections, key=lambda p: p.spatialCoordinates.z)
                        
                        x_camera = closest.spatialCoordinates.x
                        y_camera = closest.spatialCoordinates.y
                        z_depth = closest.spatialCoordinates.z
                        confidence = closest.confidence
                        
                        # Validate detection
                        if 100 < z_depth < 15000:  # Reasonable range
                            # Calculate metrics
                            fov_analysis = self.analyze_fov_position(x_camera)
                            z_flicker, z_change = self.detect_z_flickering(z_depth, z_history)
                            y_method_data = self.calculate_y_distance_method(closest, model_config)
                            
                            # Store data point
                            data_point = {
                                'timestamp': time.time(),
                                'model_name': model_name,
                                'x_camera': x_camera,
                                'y_camera': y_camera,
                                'z_depth': z_depth,
                                'confidence': confidence,
                                'bbox_xmin': closest.xmin,
                                'bbox_ymin': closest.ymin,
                                'bbox_xmax': closest.xmax,
                                'bbox_ymax': closest.ymax,
                                'x_position_ratio': fov_analysis['x_position_ratio'],
                                'is_at_edge': fov_analysis['is_at_edge'],
                                'z_flicker_detected': z_flicker,
                                'z_change_amount': z_change,
                                'y_bottom_distance': y_method_data.get('y_bottom_distance', 0),
                                'estimated_distance_from_y': y_method_data.get('estimated_distance', 0),
                                'input_width': model_config['input_size'][0],
                                'input_height': model_config['input_size'][1]
                            }
                            
                            test_data.append(data_point)
                            detection_count += 1
                            last_detection_time = time.time()
                            
                            # Show progress every 5 detections
                            if detection_count % 5 == 0:
                                print(f"📊 Progress: {detection_count} detections collected")
                
                # Handle events
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.running = False
                            device.close()
                            return None
                        elif event.key == pygame.K_SPACE:
                            # Skip to next model
                            print("⏭️ Skipping to next model...")
                            device.close()
                            return None
                
                # Check for detection timeout
                if (time.time() - last_detection_time) > 15:  # 15 seconds without detection
                    print(f"⚠️ No valid detections for 15 seconds")
                    print(f"   Total inferences: {total_inference_count}")
                    print(f"   Valid detections: {valid_detection_count}")
                    print(f"   Looking for class ID: {model_config['expected_class_id']}")
                    print(f"   Confidence threshold: {model_config['confidence_threshold']}")
                    last_detection_time = time.time()  # Reset timeout
                
                time.sleep(0.05)  # 20 FPS
                
        except KeyboardInterrupt:
            print("\n⏹️ Test interrupted")
        finally:
            device.close()
        
        # Debug: Show test results
        print(f"\n📊 Test Summary for {model_name}:")
        print(f"   Total inferences: {total_inference_count}")
        print(f"   Valid detections: {valid_detection_count}")
        print(f"   Collected data points: {len(test_data)}")
        
        # Analyze results
        if len(test_data) < self.min_detections_required:
            print(f"❌ Insufficient data: {len(test_data)} detections (need {self.min_detections_required})")
            if len(test_data) > 0:
                print(f"💡 Got some data - consider lowering confidence threshold or adjusting class ID")
                # Show sample detection for debugging
                sample = test_data[0]
                print(f"   Sample detection: Class={model_config['expected_class_id']}, Conf={sample['confidence']:.3f}, Z={sample['z_depth']:.0f}mm")
            return None
        
        # Store test data for CSV export
        self.current_test_data = test_data
        
        results = self.analyze_model_performance(model_name, model_config, test_data)
        print(f"✅ Test completed: {len(test_data)} detections analyzed")
        
        return results
    
    def analyze_fov_position(self, x_camera):
        """Analyze field of view position"""
        max_x_at_1m = 1000 * math.tan(math.radians(self.camera_hfov_degrees / 2))
        x_position_ratio = x_camera / max_x_at_1m if max_x_at_1m > 0 else 0
        x_position_ratio = max(-1.0, min(1.0, x_position_ratio))
        is_at_edge = abs(x_position_ratio) > 0.7
        
        return {
            'x_position_ratio': x_position_ratio,
            'is_at_edge': is_at_edge,
            'distance_from_center': abs(x_position_ratio)
        }
    
    def detect_z_flickering(self, current_z, z_history):
        """Detect z-depth flickering"""
        z_history.append(current_z)
        
        if len(z_history) < 5:
            return False, 0
        
        recent_avg = sum(list(z_history)[-4:-1]) / 3
        z_change = abs(current_z - recent_avg)
        is_flickering = z_change > 300  # 30cm threshold
        
        return is_flickering, z_change
    
    def calculate_y_distance_method(self, detection, model_config):
        """Calculate distance using y-coordinate method"""
        try:
            input_height = model_config['input_size'][1]
            
            ymin = detection.ymin
            ymax = detection.ymax
            
            bbox_bottom_pixel = ymax * input_height
            screen_bottom_pixel = input_height
            
            y_bottom_distance = screen_bottom_pixel - bbox_bottom_pixel
            
            # Estimate distance
            max_distance = 8000  # mm
            min_distance = 500   # mm
            
            distance_ratio = y_bottom_distance / input_height
            estimated_distance = min_distance + (distance_ratio * (max_distance - min_distance))
            
            return {
                'bbox_bottom_pixel': bbox_bottom_pixel,
                'y_bottom_distance': y_bottom_distance,
                'estimated_distance': estimated_distance
            }
        except Exception:
            return {}
    
    def analyze_model_performance(self, model_name, model_config, test_data):
        """Analyze performance metrics for a model"""
        if not test_data:
            return None
        
        # Extract metrics
        confidences = [d['confidence'] for d in test_data]
        z_depths = [d['z_depth'] for d in test_data]
        z_changes = [d['z_change_amount'] for d in test_data if d['z_change_amount'] > 0]
        flickers = [d for d in test_data if d['z_flicker_detected']]
        edge_detections = [d for d in test_data if d['is_at_edge']]
        center_detections = [d for d in test_data if not d['is_at_edge']]
        
        # Calculate statistics
        results = {
            'model_name': model_name,
            'description': model_config['description'],
            'input_size': model_config['input_size'],
            'test_duration': self.test_duration_per_model,
            'total_detections': len(test_data),
            'detection_rate': len(test_data) / self.test_duration_per_model,
            
            # Confidence metrics
            'avg_confidence': statistics.mean(confidences),
            'min_confidence': min(confidences),
            'max_confidence': max(confidences),
            'confidence_std': statistics.stdev(confidences) if len(confidences) > 1 else 0,
            
            # Z-depth stability metrics
            'avg_z_depth': statistics.mean(z_depths),
            'z_depth_std': statistics.stdev(z_depths) if len(z_depths) > 1 else 0,
            'avg_z_change': statistics.mean(z_changes) if z_changes else 0,
            'max_z_change': max(z_changes) if z_changes else 0,
            'flicker_count': len(flickers),
            'flicker_rate': len(flickers) / len(test_data),
            'large_changes_count': len([c for c in z_changes if c > 500]),
            'large_changes_rate': len([c for c in z_changes if c > 500]) / len(z_changes) if z_changes else 0,
            
            # FOV performance
            'edge_detection_count': len(edge_detections),
            'edge_detection_rate': len(edge_detections) / len(test_data),
            'center_detection_count': len(center_detections),
            'center_detection_rate': len(center_detections) / len(test_data),
            
            # Edge vs center flicker rates
            'edge_flicker_rate': len([d for d in edge_detections if d['z_flicker_detected']]) / len(edge_detections) if edge_detections else 0,
            'center_flicker_rate': len([d for d in center_detections if d['z_flicker_detected']]) / len(center_detections) if center_detections else 0,
            
            # Y-method comparison
            'y_method_distances': [d['estimated_distance_from_y'] for d in test_data if d['estimated_distance_from_y'] > 0],
            'avg_y_method_distance': statistics.mean([d['estimated_distance_from_y'] for d in test_data if d['estimated_distance_from_y'] > 0]) if any(d['estimated_distance_from_y'] > 0 for d in test_data) else 0,
            'y_z_discrepancy': 0,  # Will calculate after
            
            # Quality score (custom metric)
            'stability_score': 0,  # Will calculate
            'overall_score': 0     # Will calculate
        }
        
        # Calculate Y-method vs Z-depth discrepancy
        if results['avg_y_method_distance'] > 0:
            results['y_z_discrepancy'] = abs(results['avg_y_method_distance'] - results['avg_z_depth'])
        
        # Calculate stability score (0-100, higher is better)
        stability_score = 100
        stability_score -= min(results['flicker_rate'] * 100, 50)  # Penalty for flicker
        stability_score -= min(results['avg_z_change'] / 10, 30)   # Penalty for large changes
        stability_score -= min(results['z_depth_std'] / 100, 20)   # Penalty for variation
        results['stability_score'] = max(0, stability_score)
        
        # Calculate overall score (0-100, higher is better)
        overall_score = 0
        overall_score += results['avg_confidence'] * 20  # 20 points for confidence
        overall_score += results['stability_score'] * 0.6  # 60 points for stability
        overall_score += results['detection_rate'] * 2  # 20 points for detection rate (max 10 fps)
        results['overall_score'] = min(100, overall_score)
        
        return results
    
    def draw_testing_display(self, model_name, model_config, test_data, elapsed_time, detection_count):
        """Draw testing display"""
        self.screen.fill((0, 0, 0))
        
        # Title
        title = self.large_font.render(f"TESTING: {model_name}", True, (255, 255, 0))
        self.screen.blit(title, (50, 50))
        
        # Model info
        desc = self.medium_font.render(model_config['description'], True, (255, 255, 255))
        self.screen.blit(desc, (50, 120))
        
        input_size = self.medium_font.render(f"Input: {model_config['input_size'][0]}x{model_config['input_size'][1]}", True, (255, 255, 255))
        self.screen.blit(input_size, (50, 170))
        
        # Progress
        progress = elapsed_time / self.test_duration_per_model
        progress_width = int(600 * progress)
        pygame.draw.rect(self.screen, (100, 100, 100), (50, 220, 600, 30), 2)
        pygame.draw.rect(self.screen, (0, 255, 0), (52, 222, progress_width-4, 26))
        
        time_text = self.medium_font.render(f"Time: {elapsed_time:.1f}s / {self.test_duration_per_model}s", True, (255, 255, 255))
        self.screen.blit(time_text, (50, 270))
        
        # Detection count
        det_text = self.medium_font.render(f"Detections: {detection_count} (need {self.min_detections_required})", True, (255, 255, 255))
        color = (0, 255, 0) if detection_count >= self.min_detections_required else (255, 100, 100)
        det_text = self.medium_font.render(f"Detections: {detection_count} (need {self.min_detections_required})", True, color)
        self.screen.blit(det_text, (50, 320))
        
        # Recent performance
        if test_data:
            recent_data = test_data[-10:]  # Last 10 detections
            avg_conf = sum(d['confidence'] for d in recent_data) / len(recent_data)
            flicker_count = sum(1 for d in recent_data if d['z_flicker_detected'])
            
            conf_text = self.small_font.render(f"Recent avg confidence: {avg_conf:.3f}", True, (255, 255, 255))
            self.screen.blit(conf_text, (50, 370))
            
            flicker_text = self.small_font.render(f"Recent flickers: {flicker_count}/10", True, (255, 255, 255))
            self.screen.blit(flicker_text, (50, 400))
        
        # Instructions
        instructions = [
            "Stand in front of camera",
            "Move slowly left to right", 
            "SPACE - Skip to next model",
            "ESC - Exit testing"
        ]
        
        y_offset = 500
        for instruction in instructions:
            color = (255, 100, 100) if "ESC" in instruction or "SPACE" in instruction else (255, 255, 255)
            text = self.small_font.render(instruction, True, color)
            self.screen.blit(text, (50, y_offset))
            y_offset += 30
        
        pygame.display.flip()
    
    def save_detailed_csv(self, all_test_data):
        """Save detailed CSV with all test data"""
        try:
            with open(self.detailed_csv_filename, 'w', newline='') as csvfile:
                fieldnames = [
                    'timestamp', 'model_name', 'x_camera', 'y_camera', 'z_depth', 'confidence',
                    'bbox_xmin', 'bbox_ymin', 'bbox_xmax', 'bbox_ymax',
                    'x_position_ratio', 'is_at_edge', 'z_flicker_detected', 'z_change_amount',
                    'y_bottom_distance', 'estimated_distance_from_y', 'input_width', 'input_height'
                ]
                
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
                for test_data in all_test_data.values():
                    if test_data:
                        for data_point in test_data:
                            writer.writerow(data_point)
            
            print(f"📁 Detailed data saved to: {self.detailed_csv_filename}")
        except Exception as e:
            print(f"❌ Failed to save detailed CSV: {e}")
    
    def generate_comparison_report(self, results):
        """Generate comprehensive comparison report"""
        if not results:
            print("❌ No results to compare")
            return
        
        # Sort by overall score
        sorted_results = sorted(results.values(), key=lambda r: r['overall_score'], reverse=True)
        
        print("\n" + "=" * 80)
        print("📊 INTEL MODEL SPATIAL DETECTION COMPARISON REPORT")
        print("=" * 80)
        
        # Summary table
        print(f"\n🏆 RANKING (by Overall Score):")
        print("-" * 80)
        print(f"{'Rank':<4} {'Model':<30} {'Score':<8} {'Stability':<10} {'Flicker%':<10} {'Detections':<12}")
        print("-" * 80)
        
        for i, result in enumerate(sorted_results, 1):
            print(f"{i:<4} {result['model_name']:<30} {result['overall_score']:<8.1f} "
                  f"{result['stability_score']:<10.1f} {result['flicker_rate']*100:<10.1f} "
                  f"{result['total_detections']:<12}")
        
        # Detailed analysis
        print(f"\n📈 DETAILED PERFORMANCE ANALYSIS:")
        print("-" * 80)
        
        best_model = sorted_results[0]
        worst_model = sorted_results[-1]
        
        print(f"🥇 BEST OVERALL: {best_model['model_name']}")
        print(f"   • Overall Score: {best_model['overall_score']:.1f}/100")
        print(f"   • Stability Score: {best_model['stability_score']:.1f}/100")
        print(f"   • Flicker Rate: {best_model['flicker_rate']*100:.1f}%")
        print(f"   • Avg Z-depth Change: {best_model['avg_z_change']:.0f}mm")
        print(f"   • Detection Rate: {best_model['detection_rate']:.1f} fps")
        print(f"   • Input Resolution: {best_model['input_size'][0]}x{best_model['input_size'][1]}")
        
        # Find best in specific categories
        most_stable = min(results.values(), key=lambda r: r['flicker_rate'])
        highest_confidence = max(results.values(), key=lambda r: r['avg_confidence'])
        best_edge_performance = min(results.values(), key=lambda r: r['edge_flicker_rate'] if r['edge_detection_count'] > 5 else 1)
        
        print(f"\n🎯 CATEGORY WINNERS:")
        print(f"   📐 Most Stable Z-depth: {most_stable['model_name']} ({most_stable['flicker_rate']*100:.1f}% flicker)")
        print(f"   🎯 Highest Confidence: {highest_confidence['model_name']} ({highest_confidence['avg_confidence']:.3f} avg)")
        print(f"   🔲 Best Edge Performance: {best_edge_performance['model_name']} ({best_edge_performance['edge_flicker_rate']*100:.1f}% edge flicker)")
        
        # Recommendations
        print(f"\n💡 RECOMMENDATIONS FOR MAXINE ROBOT:")
        print("-" * 50)
        
        if best_model['flicker_rate'] < 0.1:  # Less than 10% flicker
            print(f"✅ RECOMMENDED: {best_model['model_name']}")
            print(f"   Excellent stability for navigation use case")
        elif most_stable['flicker_rate'] < 0.15:  # Less than 15% flicker
            print(f"⚠️ RECOMMENDED: {most_stable['model_name']}")
            print(f"   Best stability available, acceptable for navigation")
        else:
            print(f"❌ WARNING: All models show high flicker rates")
            print(f"   Consider additional filtering or parameter tuning")
        
        # Save results to JSON
        try:
            with open(self.results_filename, 'w') as f:
                json.dump(results, f, indent=2)
            print(f"\n📁 Complete results saved to: {self.results_filename}")
        except Exception as e:
            print(f"❌ Failed to save results: {e}")
    
    def run_quick_test(self):
        """Run a quick test of just the most promising models"""
        print("\n🚀 RUNNING QUICK TEST - Top 3 Models Only")
        print("=" * 50)
        
        # Select most promising models
        quick_models = {
            'face-detection-retail-0004': self.intel_models['face-detection-retail-0004'],
            'person-detection-0200': self.intel_models['person-detection-0200'],
            'mobilenet-ssd': self.intel_models['mobilenet-ssd']
        }
        
        print(f"📊 Testing {len(quick_models)} most promising models")
        print("📋 Instructions:")
        print("  • Stand in front of camera during each test")
        print("  • Move slowly left to right to test edge performance")
        print("  • Press SPACE to skip a model, ESC to exit")
        print()
        
        input("Press ENTER to begin quick testing...")
        
        results = {}
        all_test_data = {}
        
        for i, (model_name, model_config) in enumerate(quick_models.items()):
            if not self.running:
                break
                
            print(f"\n📊 Progress: {i+1}/{len(quick_models)}")
            
            # Test the model
            result = self.test_single_model(model_name, model_config)
            
            if result:
                results[model_name] = result
                # Store test data for detailed CSV
                if hasattr(self, 'current_test_data'):
                    all_test_data[model_name] = self.current_test_data
                print(f"✅ {model_name} completed - Score: {result['overall_score']:.1f}")
            else:
                print(f"❌ {model_name} failed or insufficient data")
            
            # Brief pause between models
            if i < len(quick_models) - 1:
                print("\n⏸️  3 second pause before next model...")
                time.sleep(3)
        
        # Save detailed data
        self.save_detailed_csv(all_test_data)
        
        # Generate comparison report
        self.generate_comparison_report(results)
        
        return results

    def run_comprehensive_test(self):
        """Run comprehensive test of all Intel models"""
        print("\n🚀 STARTING COMPREHENSIVE INTEL MODEL TEST")
        print("=" * 50)
        print("📋 Instructions:")
        print("  • Stand in front of camera during each test")
        print("  • Move slowly left to right to test edge performance")
        print("  • Stay in view for full test duration")
        print("  • Press SPACE to skip a model, ESC to exit")
        print()
        
        input("Press ENTER to begin testing...")
        
        results = {}
        all_test_data = {}
        
        for i, (model_name, model_config) in enumerate(self.intel_models.items()):
            if not self.running:
                break
                
            print(f"\n📊 Progress: {i+1}/{len(self.intel_models)}")
            
            # Test the model
            result = self.test_single_model(model_name, model_config)
            
            if result:
                results[model_name] = result
                # Store test data for detailed CSV
                if hasattr(self, 'current_test_data'):
                    all_test_data[model_name] = self.current_test_data
                print(f"✅ {model_name} completed - Score: {result['overall_score']:.1f}")
            else:
                print(f"❌ {model_name} failed or insufficient data")
            
            # Brief pause between models
            if i < len(self.intel_models) - 1:
                print("\n⏸️  5 second pause before next model...")
                time.sleep(5)
        
        # Save detailed data
        self.save_detailed_csv(all_test_data)
        
        # Generate comparison report
        self.generate_comparison_report(results)
        
        return results
    
    def cleanup(self):
        """Cleanup resources"""
        try:
            pygame.quit()
        except Exception as e:
            print(f"Cleanup error: {e}")


def main():
    """Main function"""
    print("🧪 Maxine Robot - Comprehensive Intel Model Spatial Detection Tester")
    print("Testing Intel pre-optimized models for empirical performance comparison")
    print()
    print("📋 Choose your testing mode:")
    print("  1. 🚀 Quick Test (3 models, ~5 minutes)")
    print("  2. 🔬 Comprehensive Test (7 models, ~10 minutes)")
    print()
    
    while True:
        choice = input("Enter 1 for Quick Test or 2 for Comprehensive Test: ").strip()
        if choice in ['1', '2']:
            break
        print("Please enter 1 or 2")
    
    if not BLOBCONVERTER_AVAILABLE:
        print("\n⚠️ WARNING: BlobConverter not available")
        print("Some models may not be downloadable automatically")
        print("Install with: pip install blobconverter")
        print()
    
    tester = IntelModelSpatialTester()
    
    try:
        if choice == '1':
            print("\n🚀 RUNNING QUICK TEST")
            print("Testing: face-detection-retail-0004, person-detection-0200, mobilenet-ssd")
            results = tester.run_quick_test()
        else:
            print("\n🔬 RUNNING COMPREHENSIVE TEST")
            print("Testing all 7 Intel models")
            results = tester.run_comprehensive_test()
        
        if results:
            print(f"\n🎉 TEST COMPLETED!")
            print(f"📊 {len(results)} models successfully tested")
            print("📁 Check the generated files for detailed analysis")
        else:
            print("\n❌ Test failed or no valid results")
            print("💡 Try the Quick Test mode or check camera positioning")
            
    except Exception as e:
        print(f"💥 Error during testing: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tester.cleanup()


if __name__ == "__main__":
    main()