#!/usr/bin/env python3
"""
Diagnostic Camera Tester - Debug Detection Pipeline
Identifies why 0% detection rate is occurring
Shows exactly what the MobileNet model is detecting
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
    exit(1)


class DiagnosticCameraTester:
    """Diagnostic tester to identify detection pipeline issues"""
    
    def __init__(self):
        # Initialize pygame for display
        pygame.init()
        pygame.font.init()
        
        # Get fullscreen resolution
        display_info = pygame.display.Info()
        self.screen_width = display_info.current_w
        self.screen_height = display_info.current_h
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.FULLSCREEN)
        pygame.display.set_caption("Diagnostic Camera Test - Detection Debug")
        
        # Hide mouse cursor
        pygame.mouse.set_visible(False)
        
        # Fonts
        self.large_font = pygame.font.Font(None, 64)
        self.medium_font = pygame.font.Font(None, 48)
        self.small_font = pygame.font.Font(None, 32)
        self.tiny_font = pygame.font.Font(None, 24)
        
        # Initialize DepthAI camera
        self.device = None
        self.pipeline = None
        self.detection_queue = None
        self.preview_queue = None
        self.has_detection = False
        
        # Diagnostic tracking
        self.total_frames = 0
        self.inference_frames = 0
        self.raw_detections = []
        self.person_detections = []
        self.all_class_detections = {}
        self.confidence_histogram = {}
        self.spatial_coord_issues = []
        
        # COCO class names for debugging
        self.coco_classes = {
            0: 'background', 1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 5: 'airplane',
            6: 'bus', 7: 'train', 8: 'truck', 9: 'boat', 10: 'traffic light', 11: 'fire hydrant',
            12: 'stop sign', 13: 'parking meter', 14: 'bench', 15: 'bird', 16: 'cat', 17: 'dog',
            18: 'horse', 19: 'sheep', 20: 'cow', 21: 'elephant', 22: 'bear', 23: 'zebra',
            24: 'giraffe', 25: 'backpack', 26: 'umbrella', 27: 'handbag', 28: 'tie', 29: 'suitcase',
            30: 'frisbee', 31: 'skis', 32: 'snowboard', 33: 'sports ball', 34: 'kite', 35: 'baseball bat',
            36: 'baseball glove', 37: 'skateboard', 38: 'surfboard', 39: 'tennis racket', 40: 'bottle',
            41: 'wine glass', 42: 'cup', 43: 'fork', 44: 'knife', 45: 'spoon', 46: 'bowl',
            47: 'banana', 48: 'apple', 49: 'sandwich', 50: 'orange', 51: 'broccoli', 52: 'carrot',
            53: 'hot dog', 54: 'pizza', 55: 'donut', 56: 'cake', 57: 'chair', 58: 'couch',
            59: 'potted plant', 60: 'bed', 61: 'dining table', 62: 'toilet', 63: 'tv', 64: 'laptop',
            65: 'mouse', 66: 'remote', 67: 'keyboard', 68: 'cell phone', 69: 'microwave', 70: 'oven',
            71: 'toaster', 72: 'sink', 73: 'refrigerator', 74: 'book', 75: 'clock', 76: 'vase',
            77: 'scissors', 78: 'teddy bear', 79: 'hair drier', 80: 'toothbrush'
        }
        
        # Note: MobileNet-SSD uses different class mapping - person is class 15
        self.mobilenet_classes = {
            0: 'background', 1: 'aeroplane', 2: 'bicycle', 3: 'bird', 4: 'boat', 5: 'bottle',
            6: 'bus', 7: 'car', 8: 'cat', 9: 'chair', 10: 'cow', 11: 'diningtable',
            12: 'dog', 13: 'horse', 14: 'motorbike', 15: 'person', 16: 'pottedplant',
            17: 'sheep', 18: 'sofa', 19: 'train', 20: 'tvmonitor'
        }
        
        # Diagnostic parameters
        self.min_confidence_for_analysis = 0.1  # Very low to catch everything
        self.show_all_detections = True
        self.detailed_logging = True
        
        # Control flags
        self.running = True
        self.paused = False
        
        print("\n🔍 DIAGNOSTIC CAMERA TESTER")
        print("=" * 50)
        print("🎯 Diagnostic Goals:")
        print("  • Identify why 0% detection rate occurred")
        print("  • Show ALL objects being detected")
        print("  • Analyze confidence distributions")
        print("  • Check spatial coordinate validity")
        print("  • Verify person class ID (should be 15)")
        print("=" * 50)
        
    def create_diagnostic_pipeline(self):
        """Create diagnostic pipeline with maximum detection sensitivity"""
        try:
            pipeline = dai.Pipeline()
            
            # Color camera - same as working version
            cam_rgb = pipeline.create(dai.node.ColorCamera)
            cam_rgb.setPreviewSize(416, 416)
            cam_rgb.setInterleaved(False)
            cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            cam_rgb.setFps(15)
            
            # ImageManip - exact same as working version
            manip = pipeline.create(dai.node.ImageManip)
            manip.initialConfig.setResize(300, 300)
            manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
            cam_rgb.preview.link(manip.inputImage)
            
            # Stereo depth - same configuration
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
            
            # MobileNet with MAXIMUM sensitivity for diagnosis
            detection_nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
            
            # Load blob
            local_blob_path = "./mobilenet-ssd_openvino_2022.1_6shave.blob"
            if os.path.exists(local_blob_path):
                detection_nn.setBlobPath(local_blob_path)
                print(f"✅ Using blob: {local_blob_path}")
            else:
                print("❌ Blob not found for diagnostics")
                return None
            
            # VERY LOW thresholds to catch everything
            detection_nn.setConfidenceThreshold(0.1)  # Extremely low
            detection_nn.setBoundingBoxScaleFactor(0.8)  # Large boxes
            detection_nn.setDepthLowerThreshold(50)   # Very close
            detection_nn.setDepthUpperThreshold(10000)  # Very far
            
            # Link pipeline
            manip.out.link(detection_nn.input)
            depth.depth.link(detection_nn.inputDepth)
            
            # Create outputs
            detection_out = pipeline.create(dai.node.XLinkOut)
            detection_out.setStreamName("detections")
            detection_nn.out.link(detection_out.input)
            
            preview_out = pipeline.create(dai.node.XLinkOut)
            preview_out.setStreamName("preview")
            manip.out.link(preview_out.input)
            
            print("✅ Diagnostic pipeline created with maximum sensitivity")
            print("🔍 Confidence threshold: 0.1 (very low)")
            print("🔍 Depth range: 50-10000mm (very wide)")
            print("🔍 BBox scale: 0.8 (large boxes)")
            
            return pipeline
            
        except Exception as e:
            print(f"❌ Failed to create diagnostic pipeline: {e}")
            return None
    
    def initialize_camera(self):
        """Initialize camera for diagnostic testing"""
        try:
            print("📷 Initializing camera for diagnostic testing...")
            
            self.pipeline = self.create_diagnostic_pipeline()
            if not self.pipeline:
                return False
            
            self.device = dai.Device(self.pipeline)
            
            # Get output queues
            try:
                self.detection_queue = self.device.getOutputQueue("detections", maxSize=4, blocking=False)
                self.has_detection = True
                print("✅ Diagnostic detection queue available")
            except RuntimeError:
                self.detection_queue = None
                self.has_detection = False
                print("❌ No detection queue - cannot run diagnostics")
                return False
            
            try:
                self.preview_queue = self.device.getOutputQueue("preview", maxSize=4, blocking=False)
            except RuntimeError:
                self.preview_queue = None
            
            print("✅ Camera initialized for diagnostic testing")
            time.sleep(2)
            return True
            
        except Exception as e:
            print(f"❌ Failed to initialize camera: {e}")
            return False
    
    def process_diagnostic_detections(self):
        """Process detections with full diagnostic analysis"""
        self.total_frames += 1
        
        if not self.has_detection or not self.detection_queue:
            print("❌ No detection queue available")
            return None
        
        try:
            detections = self.detection_queue.tryGet()
            if detections:
                self.inference_frames += 1
                
                # Log ALL detections for analysis
                if len(detections.detections) > 0:
                    print(f"\n🔍 Frame {self.total_frames}: Found {len(detections.detections)} detections")
                    
                    for i, det in enumerate(detections.detections):
                        class_id = det.label
                        confidence = det.confidence
                        
                        # Get class name
                        class_name = self.mobilenet_classes.get(class_id, f"Unknown({class_id})")
                        
                        # Get spatial coordinates
                        try:
                            x = det.spatialCoordinates.x
                            y = det.spatialCoordinates.y
                            z = det.spatialCoordinates.z
                            spatial_valid = True
                        except:
                            x = y = z = 0
                            spatial_valid = False
                        
                        print(f"  Detection {i}: {class_name} (ID:{class_id}) conf={confidence:.3f}")
                        print(f"    Spatial: x={x:.0f}mm, y={y:.0f}mm, z={z:.0f}mm, valid={spatial_valid}")
                        print(f"    BBox: ({det.xmin:.3f}, {det.ymin:.3f}) to ({det.xmax:.3f}, {det.ymax:.3f})")
                        
                        # Track statistics
                        if class_id not in self.all_class_detections:
                            self.all_class_detections[class_id] = {
                                'count': 0, 
                                'name': class_name,
                                'confidences': [],
                                'spatial_issues': 0
                            }
                        
                        self.all_class_detections[class_id]['count'] += 1
                        self.all_class_detections[class_id]['confidences'].append(confidence)
                        
                        if not spatial_valid or z <= 0 or z > 15000:
                            self.all_class_detections[class_id]['spatial_issues'] += 1
                            self.spatial_coord_issues.append({
                                'frame': self.total_frames,
                                'class': class_name,
                                'confidence': confidence,
                                'x': x, 'y': y, 'z': z,
                                'valid': spatial_valid
                            })
                        
                        # Track confidence histogram
                        conf_bucket = int(confidence * 10) / 10  # 0.1 buckets
                        if conf_bucket not in self.confidence_histogram:
                            self.confidence_histogram[conf_bucket] = 0
                        self.confidence_histogram[conf_bucket] += 1
                        
                        # Special handling for person detections (class 15)
                        if class_id == 15:  # Person
                            self.person_detections.append({
                                'frame': self.total_frames,
                                'confidence': confidence,
                                'x': x, 'y': y, 'z': z,
                                'spatial_valid': spatial_valid,
                                'bbox': (det.xmin, det.ymin, det.xmax, det.ymax)
                            })
                            
                            print(f"    ✅ PERSON DETECTED! Confidence: {confidence:.3f}")
                            
                            # Check if this would pass normal filtering
                            if confidence >= 0.5 and spatial_valid and 100 < z < 15000:
                                print(f"    ✅ Would PASS normal filtering")
                                return {
                                    'class': 'person',
                                    'confidence': confidence,
                                    'x': x, 'y': y, 'z': z,
                                    'frame': self.total_frames
                                }
                            else:
                                print(f"    ❌ Would FAIL normal filtering:")
                                if confidence < 0.5:
                                    print(f"      - Low confidence: {confidence:.3f} < 0.5")
                                if not spatial_valid:
                                    print(f"      - Invalid spatial coordinates")
                                if not (100 < z < 15000):
                                    print(f"      - Invalid depth: {z:.0f}mm (need 100-15000)")
                else:
                    # No detections this frame
                    if self.total_frames % 30 == 0:  # Every 2 seconds at 15fps
                        print(f"🔍 Frame {self.total_frames}: No detections found")
            
            return None
            
        except Exception as e:
            print(f"❌ Detection processing error: {e}")
            return None
    
    def draw_diagnostic_display(self):
        """Draw diagnostic information"""
        self.screen.fill((0, 0, 0))
        
        # Title
        title = self.large_font.render("DIAGNOSTIC MODE - DETECTION ANALYSIS", True, (255, 255, 0))
        self.screen.blit(title, (50, 20))
        
        y_offset = 80
        
        # Basic statistics
        detection_rate = self.inference_frames / max(self.total_frames, 1)
        person_count = len(self.person_detections)
        
        stats = [
            f"Total Frames: {self.total_frames}",
            f"Inference Frames: {self.inference_frames} ({detection_rate:.1%})",
            f"Person Detections: {person_count}",
            f"Unique Classes: {len(self.all_class_detections)}"
        ]
        
        for stat in stats:
            color = (0, 255, 0) if "Person" in stat and person_count > 0 else (255, 255, 255)
            text = self.medium_font.render(stat, True, color)
            self.screen.blit(text, (50, y_offset))
            y_offset += 40
        
        # Show detected classes
        y_offset += 20
        class_title = self.medium_font.render("DETECTED CLASSES:", True, (255, 255, 0))
        self.screen.blit(class_title, (50, y_offset))
        y_offset += 40
        
        if self.all_class_detections:
            for class_id, data in sorted(self.all_class_detections.items(), key=lambda x: x[1]['count'], reverse=True):
                count = data['count']
                name = data['name']
                avg_conf = sum(data['confidences']) / len(data['confidences'])
                spatial_issues = data['spatial_issues']
                
                class_text = f"  {name} (ID:{class_id}): {count} det, {avg_conf:.2f} conf"
                if spatial_issues > 0:
                    class_text += f", {spatial_issues} spatial issues"
                
                color = (255, 0, 0) if name == "person" else (255, 255, 255)
                text = self.small_font.render(class_text, True, color)
                self.screen.blit(text, (50, y_offset))
                y_offset += 25
                
                if y_offset > self.screen_height - 100:
                    break
        else:
            no_detections = self.small_font.render("  No detections found yet", True, (255, 100, 100))
            self.screen.blit(no_detections, (50, y_offset))
        
        # Recent person detections
        if self.person_detections:
            recent_persons = self.person_detections[-3:]  # Last 3
            person_y = self.screen_height - 200
            
            person_title = self.medium_font.render("RECENT PERSON DETECTIONS:", True, (0, 255, 0))
            self.screen.blit(person_title, (50, person_y))
            person_y += 40
            
            for person in recent_persons:
                person_text = f"Frame {person['frame']}: conf={person['confidence']:.3f}, z={person['z']:.0f}mm"
                text = self.small_font.render(person_text, True, (0, 255, 0))
                self.screen.blit(text, (50, person_y))
                person_y += 25
        
        # Controls
        controls = ["ESC - Exit diagnostic", "SPACE - Pause"]
        control_x = self.screen_width - 300
        control_y = 50
        
        for control in controls:
            text = self.small_font.render(control, True, (255, 100, 100))
            self.screen.blit(text, (control_x, control_y))
            control_y += 30
        
        pygame.display.flip()
    
    def generate_diagnostic_report(self):
        """Generate comprehensive diagnostic report"""
        try:
            print("\n" + "=" * 70)
            print("🔍 DIAGNOSTIC REPORT - DETECTION PIPELINE ANALYSIS")
            print("=" * 70)
            
            print(f"📊 Overall Statistics:")
            print(f"   Total Frames Processed: {self.total_frames}")
            print(f"   Frames with Inferences: {self.inference_frames}")
            print(f"   Person Detections Found: {len(self.person_detections)}")
            print(f"   Unique Classes Detected: {len(self.all_class_detections)}")
            print(f"   Spatial Coordinate Issues: {len(self.spatial_coord_issues)}")
            
            print(f"\n🎯 Class Detection Breakdown:")
            if self.all_class_detections:
                for class_id, data in sorted(self.all_class_detections.items(), key=lambda x: x[1]['count'], reverse=True):
                    count = data['count']
                    name = data['name']
                    avg_conf = sum(data['confidences']) / len(data['confidences'])
                    max_conf = max(data['confidences'])
                    spatial_issues = data['spatial_issues']
                    
                    print(f"   {name} (ID:{class_id}):")
                    print(f"     Count: {count}")
                    print(f"     Avg Confidence: {avg_conf:.3f}")
                    print(f"     Max Confidence: {max_conf:.3f}")
                    print(f"     Spatial Issues: {spatial_issues}/{count}")
            else:
                print("   ❌ NO OBJECTS DETECTED AT ALL")
                print("   This indicates a fundamental pipeline issue:")
                print("     - Camera may not be working")
                print("     - Blob file may be corrupted")
                print("     - Detection network may not be running")
            
            print(f"\n👤 Person Detection Analysis:")
            if self.person_detections:
                valid_persons = [p for p in self.person_detections if p['spatial_valid'] and 100 < p['z'] < 15000]
                high_conf_persons = [p for p in valid_persons if p['confidence'] >= 0.5]
                
                print(f"   Total Person Detections: {len(self.person_detections)}")
                print(f"   Spatially Valid: {len(valid_persons)}")
                print(f"   High Confidence (≥0.5): {len(high_conf_persons)}")
                
                if high_conf_persons:
                    avg_conf = sum(p['confidence'] for p in high_conf_persons) / len(high_conf_persons)
                    print(f"   Average Valid Confidence: {avg_conf:.3f}")
                    print(f"   ✅ SOLUTION: Person detection IS working!")
                    print(f"   The previous 0% rate was due to no person in camera view")
                else:
                    print(f"   ❌ No valid high-confidence person detections")
                    if valid_persons:
                        max_conf = max(p['confidence'] for p in valid_persons)
                        print(f"   Best confidence found: {max_conf:.3f}")
                        print(f"   💡 SOLUTION: Lower confidence threshold to {max_conf - 0.05:.2f}")
                    else:
                        print(f"   💡 SOLUTION: Check spatial coordinate validation")
            else:
                print("   ❌ NO PERSON DETECTIONS FOUND")
                print("   Possible causes:")
                print("     - No person in camera view during test")
                print("     - Person class ID wrong (expected 15)")
                print("     - All person detections filtered out")
            
            print(f"\n🔧 RECOMMENDATIONS:")
            if len(self.person_detections) > 0:
                print("   ✅ Detection pipeline is working correctly")
                print("   ✅ Person class ID (15) is correct")
                print("   💡 Previous 0% rate was likely due to:")
                print("     - No person standing in front of camera")
                print("     - Person moved out of view during automated tests")
                print("     - Test duration too short for person positioning")
            else:
                print("   🔧 For next test:")
                print("     - Ensure person stands directly in front of camera")
                print("     - Stay in view for entire test duration")
                print("     - Check camera is not obstructed")
                print("     - Verify adequate lighting")
            
        except Exception as e:
            print(f"Error generating diagnostic report: {e}")
    
    def run_diagnostic_test(self):
        """Run the diagnostic test"""
        print("🔍 Diagnostic Camera Test - Detection Pipeline Analysis")
        print("=" * 60)
        
        if not self.initialize_camera():
            return False
        
        print("🚀 Diagnostic test starting...")
        print("📋 Stand in front of camera to test person detection")
        print("🔍 Will show ALL detected objects and classes")
        
        clock = pygame.time.Clock()
        
        try:
            while self.running:
                # Handle events
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.running = False
                        elif event.key == pygame.K_SPACE:
                            self.paused = not self.paused
                            print(f"{'PAUSED' if self.paused else 'RESUMED'}")
                    elif event.type == pygame.QUIT:
                        self.running = False
                
                # Process detections
                if not self.paused:
                    detection_data = self.process_diagnostic_detections()
                
                # Update display
                self.draw_diagnostic_display()
                
                # 15 FPS
                clock.tick(15)
                
        except KeyboardInterrupt:
            print("\n⏹️ Diagnostic test stopped")
        finally:
            self.cleanup()
            
        return True
    
    def cleanup(self):
        """Cleanup with diagnostic report"""
        try:
            if self.device:
                self.device.close()
            
            self.generate_diagnostic_report()
            pygame.quit()
            
        except Exception as e:
            print(f"Cleanup error: {e}")


def main():
    """Main function"""
    print("🔍 Maxine Robot - Diagnostic Camera Test")
    print("Detection Pipeline Analysis & Debug")
    print()
    print("🎯 This diagnostic will:")
    print("  1. Show ALL objects being detected by MobileNet")
    print("  2. Verify person class ID and confidence levels")
    print("  3. Check spatial coordinate validity")
    print("  4. Identify why 0% detection rate occurred")
    print("  5. Provide specific recommendations")
    print()
    print("📋 For best results:")
    print("  • Stand directly in front of camera")
    print("  • Move slowly to test different positions")
    print("  • Stay in view for at least 30 seconds")
    print("  • Ensure good lighting conditions")
    print()
    
    # Check for blob file
    blob_path = "./mobilenet-ssd_openvino_2022.1_6shave.blob"
    if not os.path.exists(blob_path):
        print(f"❌ ERROR: Blob file not found: {blob_path}")
        return
    else:
        print(f"✅ Blob file found: {blob_path}")
        print()
    
    tester = DiagnosticCameraTester()
    
    try:
        success = tester.run_diagnostic_test()
        if success:
            print("🎉 Diagnostic test completed!")
        else:
            print("❌ Diagnostic test failed")
    except Exception as e:
        print(f"💥 Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tester.cleanup()


if __name__ == "__main__":
    main()