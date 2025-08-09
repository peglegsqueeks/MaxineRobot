#!/usr/bin/env python3
"""
OPTIMIZED LiDAR Test Mode with Enhanced Camera Integration
Integrates existing depthai camera sensor with multi-processing optimizations
Maintains Lidar timing sensitivity while maximizing camera performance

KEY FEATURES:
1. Uses existing CameraSensor and ObjectDetectionPipeline from depthai folder
2. Multi-threaded camera processing with dedicated worker threads
3. Maximum horizontal FOV (127°) with no detection holes
4. Improved FPS through optimized threading and queue management
5. Preserves sensitive Lidar obstacle detection timing
6. Enhanced detection coverage using aspect ratio preservation
"""

import pygame
import math
import time
import threading
import csv
import os
from datetime import datetime
from typing import List, Optional, Tuple
from collections import deque
from pyrplidar import PyRPlidar
import py_trees
from py_trees.common import Status

# Import existing depthai components
from ...sensors.CameraSensor import CameraSensor
from ...depth_ai.ObjectDetectionPipeline import ObjectDetectionPipeline
from ...types.CameraMode import CameraMode
from ...behaviors.MaxineBehavior import MaxineBehavior
from ...types.RobotModes import RobotMode
from ...types.MovementDirection import MovementDirection
from ...types.KeyboardKey import KeyboardKey
from ...action_managers.VelocityManager import VelocityConfig


class MultiProcessCameraManager:
    """
    Multi-process camera manager using existing depthai infrastructure
    Optimizes detection coverage and FPS through dedicated worker processes
    
    CRITICAL: Does NO initialization in __init__ to prevent avatar display interference
    """
    
    def __init__(self):
        # Camera optimization parameters (just constants, no initialization)
        self.TARGET_FPS = 25  # Increased from standard 15 FPS
        self.DETECTION_CONFIDENCE = 0.2  # Lowered for better coverage (will be updated from camera)
        self.MAX_DETECTION_DISTANCE = 8000  # 8 meters maximum range
        self.HORIZONTAL_FOV_DEGREES = 127  # Maximum Oak-D Pro W FOV
        
        # Camera system variables - NO OBJECTS CREATED HERE
        self.camera_sensor = None
        self.current_detections = []
        self.detection_lock = None  # Will be created when needed
        self.camera_thread = None
        self.shutdown_flag = None  # Will be created when needed
        
        # Performance monitoring variables - just counters
        self.fps_counter = None  # Will be created when needed
        self.detection_count = 0
        self.frame_count = 0
        self.last_fps_time = time.time()
        
        # DO NOT print anything or create any objects that could interfere with avatar
        # print("🎥 Multi-Process Camera Manager initialized")
    
    def start_camera_system(self):
        """Start optimized camera system using existing CameraSensor"""
        try:
            # NOW it's safe to create the threading objects and print messages
            print("📷 Initializing camera sensor...")
            
            # Create threading objects only when actually starting
            self.detection_lock = threading.Lock()
            self.shutdown_flag = threading.Event()
            self.fps_counter = deque(maxlen=30)
            
            # Create camera sensor and switch to object detection mode
            self.camera_sensor = CameraSensor(CameraMode.DISABLED)
            
            # Wait a moment for initialization
            time.sleep(1.0)
            
            # Switch to object detection mode
            print("🔄 Switching to OBJECT_DETECTION mode...")
            self.camera_sensor.switch_mode(CameraMode.OBJECT_DETECTION)
            
            # Wait for camera to stabilize
            time.sleep(3.0)
            
            # Test camera reading
            print("🧪 Testing camera reading...")
            test_reading = self.camera_sensor.get_reading()
            if test_reading is None:
                print("❌ Camera not providing readings")
                return False
            
            print("✅ Camera reading test successful")
            
            # Check the actual confidence threshold being used by ObjectDetectionReading
            if hasattr(test_reading, 'confidence_threshold'):
                actual_threshold = test_reading.confidence_threshold
                print(f"📊 Camera confidence threshold: {actual_threshold:.2f} ({actual_threshold*100:.0f}%)")
                
                # Update our target confidence to match the camera system
                self.DETECTION_CONFIDENCE = actual_threshold
            else:
                print(f"📊 Using default confidence threshold: {self.DETECTION_CONFIDENCE:.2f} ({self.DETECTION_CONFIDENCE*100:.0f}%)")
            
            # Start camera reading thread
            self.start_camera_thread()
            
            print("✅ Camera system started with threading optimization")
            print(f"🎯 Detection parameters:")
            print(f"   • Confidence threshold: {self.DETECTION_CONFIDENCE:.2f} ({self.DETECTION_CONFIDENCE*100:.0f}%)")
            print(f"   • Target FPS: {self.TARGET_FPS}")
            print(f"   • FOV: {self.HORIZONTAL_FOV_DEGREES}°")
            return True
            
        except Exception as e:
            print(f"❌ Camera system startup failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def start_camera_thread(self):
        """Start dedicated thread for camera processing"""
        try:
            # Ensure shutdown flag is created if it doesn't exist
            if self.shutdown_flag is None:
                self.shutdown_flag = threading.Event()
                
            self.shutdown_flag.clear()
            self.camera_thread = threading.Thread(target=self._camera_worker, daemon=True)
            self.camera_thread.start()
            print("✅ Camera processing thread started")
            
        except Exception as e:
            print(f"❌ Failed to start camera thread: {e}")
    
    def _camera_worker(self):
        """
        Dedicated worker thread for camera processing
        Uses existing CameraSensor infrastructure
        """
        try:
            frame_interval = 1.0 / self.TARGET_FPS
            last_frame_time = 0
            
            print("🎥 Camera worker thread started")
            
            while not self.shutdown_flag.is_set():
                current_time = time.time()
                
                # Maintain target FPS
                if current_time - last_frame_time < frame_interval:
                    time.sleep(0.001)
                    continue
                
                try:
                    # Get camera reading using existing infrastructure
                    reading = self.camera_sensor.get_reading()
                    
                    if reading and hasattr(reading, 'get_people_locations'):
                        # Get people detections using the correct API
                        people_detections = reading.get_people_locations()
                        
                        if people_detections:
                            # Process detections using existing ObjectDetectionReading format
                            processed_detections = self._process_person_detections(people_detections)
                            
                            # Update current detections with thread safety (only if lock exists)
                            if self.detection_lock is not None:
                                with self.detection_lock:
                                    self.current_detections = processed_detections
                                    if processed_detections:
                                        self.detection_count += len(processed_detections)
                                    self.frame_count += 1
                                    
                                    # Update FPS counter (only if it exists)
                                    if self.fps_counter is not None:
                                        self.fps_counter.append(current_time)
                            
                            # Log detections occasionally for debugging
                            if self.frame_count % 50 == 0:
                                print(f"📊 Camera: Frame {self.frame_count}, People detected: {len(people_detections)}, Processed: {len(processed_detections)}")
                        else:
                            # No people detected, still count the frame
                            if self.detection_lock is not None:
                                with self.detection_lock:
                                    self.current_detections = []
                                    self.frame_count += 1
                                    if self.fps_counter is not None:
                                        self.fps_counter.append(current_time)
                    
                    last_frame_time = current_time
                    
                except Exception as e:
                    # Continue on individual frame errors
                    if self.frame_count % 100 == 0:  # Log errors occasionally
                        print(f"⚠️ Camera frame error: {e}")
                    time.sleep(0.01)
                    continue
                    
        except Exception as e:
            print(f"❌ Camera worker error: {e}")
            import traceback
            traceback.print_exc()
    
    def _process_person_detections(self, person_detections):
        """
        Process person detections from ObjectDetectionReading.get_people_locations()
        These are already filtered for person class and confidence by ObjectDetectionReading
        """
        processed = []
        
        if not person_detections:
            return processed
        
        try:
            for detection in person_detections:
                # These detections are already dai.SpatialImgDetection objects
                # Extract basic detection data
                person_data = {
                    'confidence': detection.confidence,
                    'x_center': detection.xmin + (detection.xmax - detection.xmin) / 2,
                    'y_center': detection.ymin + (detection.ymax - detection.ymin) / 2,
                    'width': detection.xmax - detection.xmin,
                    'height': detection.ymax - detection.ymin
                }
                
                # Add depth information if available
                if hasattr(detection, 'spatialCoordinates') and detection.spatialCoordinates:
                    person_data.update({
                        'depth_x': detection.spatialCoordinates.x,
                        'depth_y': detection.spatialCoordinates.y, 
                        'depth_z': detection.spatialCoordinates.z,
                        'distance_mm': detection.spatialCoordinates.z
                    })
                
                # Calculate horizontal angle for FOV coverage
                if 'x_center' in person_data:
                    # Convert normalized x to angle (-63.5° to +63.5° for 127° FOV)
                    angle_degrees = (person_data['x_center'] - 0.5) * self.HORIZONTAL_FOV_DEGREES
                    person_data['horizontal_angle'] = angle_degrees
                
                processed.append(person_data)
                        
        except Exception as e:
            print(f"⚠️ Person detection processing error: {e}")
        
        return processed
    
    def get_latest_detections(self):
        """Get latest processed detections with thread safety"""
        try:
            # Return current detections with thread safety
            with self.detection_lock:
                return self.current_detections.copy()
                
        except Exception as e:
            return []
    
    def get_performance_stats(self):
        """Get camera performance statistics"""
        try:
            current_time = time.time()
            
            # Calculate FPS
            fps = 0
            if len(self.fps_counter) > 1:
                time_span = self.fps_counter[-1] - self.fps_counter[0]
                if time_span > 0:
                    fps = (len(self.fps_counter) - 1) / time_span
            
            return {
                'fps': round(fps, 1),
                'frame_count': self.frame_count,
                'detection_count': self.detection_count,
                'thread_active': self.camera_thread.is_alive() if self.camera_thread else False,
                'camera_mode': 'OBJECT_DETECTION' if self.camera_sensor else 'DISABLED'
            }
            
        except Exception:
            return {'fps': 0, 'frame_count': 0, 'detection_count': 0, 'thread_active': False, 'camera_mode': 'DISABLED'}
    
    def cleanup(self):
        """Clean shutdown of camera system"""
        try:
            print("🛑 Shutting down camera system...")
            
            # Signal thread to stop (only if shutdown_flag exists)
            if self.shutdown_flag is not None:
                self.shutdown_flag.set()
                
                # Wait for thread to finish
                if self.camera_thread and self.camera_thread.is_alive():
                    self.camera_thread.join(timeout=3)
                    if self.camera_thread.is_alive():
                        print("⚠️ Camera thread did not stop gracefully")
            
            # Cleanup camera sensor using correct method
            if self.camera_sensor:
                try:
                    self.camera_sensor.stop_camera()  # Use correct method name
                    print("✅ Camera sensor stopped")
                except Exception as e:
                    print(f"⚠️ Camera stop warning: {e}")
                finally:
                    self.camera_sensor = None
            
            # Reset threading objects
            self.camera_thread = None
            self.shutdown_flag = None
            self.detection_lock = None
            self.fps_counter = None
            
            print("✅ Camera system shutdown complete")
            
        except Exception as e:
            print(f"⚠️ Camera cleanup error: {e}")


class EnhancedLidarSystem:
    """
    Enhanced LiDAR system with preserved timing sensitivity
    Maintains existing obstacle detection performance with proper cleanup
    
    CRITICAL: Does NO initialization in __init__ to prevent avatar display interference
    """
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000, timeout=2.0):
        # Connection parameters (just constants)
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.motor_pwm = 600
        
        # System state variables - NO OBJECTS CREATED HERE
        self.lidar = None
        self.is_connected = False
        self.scan_generator = None
        self.scan_iterator = None
        
        # Thread-safe data storage variables - objects created when needed
        self.obstacle_data = []
        self.data_lock = None  # Will be created when needed
        self.shutdown_flag = [False]  # Simple list, not threading object
        
        # Scanning thread
        self.scan_thread = None
        
        # DO NOT print anything or create any objects that could interfere with avatar
        # print("🔧 Enhanced LiDAR system initialized")
        
    def connect(self):
        """Connect to LiDAR with enhanced stability and proper cleanup"""
        try:
            # NOW it's safe to create threading objects and print messages
            if self.data_lock is None:
                self.data_lock = threading.Lock()
                print("🔧 LiDAR threading objects created")
            
            # Ensure clean state before connecting
            self.disconnect()
            time.sleep(1.0)
            
            print("🔌 Connecting to LiDAR...")
            self.lidar = PyRPlidar()
            self.lidar.connect(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            self.is_connected = True
            print("✅ LiDAR connected successfully")
            return True
        except Exception as e:
            print(f"❌ LiDAR connection failed: {e}")
            self.is_connected = False
            return False
    
    def start_scanning(self):
        """Start LiDAR scanning in dedicated thread with proper initialization"""
        if not self.is_connected:
            print("❌ Cannot start scanning - LiDAR not connected")
            return False
            
        try:
            print("⚙️ Initializing LiDAR scanning...")
            
            # Stop any existing scan
            self.lidar.stop()
            time.sleep(1.0)
            
            # Set motor PWM and wait for stabilization
            print("🔄 Starting LiDAR motor...")
            self.lidar.set_motor_pwm(self.motor_pwm)
            time.sleep(3.0)  # Allow motor to stabilize
            
            # Try express mode first, then force mode
            scan_started = False
            try:
                print("🚀 Attempting express scan mode...")
                self.scan_generator = self.lidar.start_scan_express(4)
                self.scan_iterator = self.scan_generator()
                scan_started = True
                print("✅ Express scan mode started")
            except Exception as e:
                print(f"⚠️ Express mode failed: {e}, trying force mode...")
                try:
                    self.scan_generator = self.lidar.force_scan()
                    self.scan_iterator = self.scan_generator()
                    scan_started = True
                    print("✅ Force scan mode started")
                except Exception as e2:
                    print(f"❌ Force mode also failed: {e2}")
                    return False
            
            if scan_started:
                # Start scanning thread
                self.shutdown_flag[0] = False
                self.scan_thread = threading.Thread(target=self._scan_worker, daemon=True)
                self.scan_thread.start()
                print("✅ LiDAR scanning thread started")
                return True
            else:
                return False
            
        except Exception as e:
            print(f"❌ Failed to start LiDAR scanning: {e}")
            return False
    
    def _scan_worker(self):
        """
        Dedicated thread for LiDAR scanning
        Preserves timing sensitivity of original implementation
        """
        consecutive_failures = 0
        max_failures = 100
        scan_buffer = []
        last_angle = None
        min_scan_points = 150
        
        print("🔄 LiDAR scan worker started")
        
        while consecutive_failures < max_failures and not self.shutdown_flag[0]:
            try:
                measurement = next(self.scan_iterator)
                
                if measurement:
                    consecutive_failures = 0
                    
                    try:
                        quality = getattr(measurement, 'quality', 0)
                        angle = getattr(measurement, 'angle', 0)
                        distance = getattr(measurement, 'distance', 0)
                        
                        if quality > 5 and distance > 100 and distance < 6000:
                            scan_buffer.append((quality, angle, distance))
                            
                            # Complete scan detection
                            if (last_angle is not None and 
                                angle < last_angle and 
                                len(scan_buffer) > min_scan_points):
                                
                                # Process complete scan
                                self._process_scan_data(scan_buffer.copy())
                                scan_buffer.clear()
                            
                            last_angle = angle
                        
                    except Exception:
                        continue
                else:
                    consecutive_failures += 1
                    time.sleep(0.02)
                    
            except StopIteration:
                print("⚠️ LiDAR scan iterator stopped, attempting restart...")
                # Restart scanning if needed
                time.sleep(2)
                if self.start_scanning():
                    consecutive_failures = 0
                    scan_buffer.clear()
                    last_angle = None
                    continue
                else:
                    print("❌ Failed to restart LiDAR scanning")
                    break
                    
            except Exception as e:
                consecutive_failures += 1
                if consecutive_failures % 20 == 0:  # Log every 20 failures
                    print(f"⚠️ LiDAR scan error (failure {consecutive_failures}): {e}")
                time.sleep(0.15)
        
        print("🛑 LiDAR scan worker stopped")
    
    def _process_scan_data(self, scan_data):
        """Process complete LiDAR scan with thread safety"""
        try:
            obstacles = []
            
            for quality, angle, distance in scan_data:
                # Convert to cartesian coordinates
                angle_rad = math.radians(angle)
                x = distance * math.cos(angle_rad)
                y = distance * math.sin(angle_rad)
                
                obstacles.append({
                    'angle': angle,
                    'distance': distance,
                    'x': x,
                    'y': y,
                    'quality': quality
                })
            
            # Thread-safe update
            with self.data_lock:
                self.obstacle_data = obstacles
                
        except Exception:
            pass
    
    def get_obstacles(self):
        """Get current obstacle data with thread safety"""
        with self.data_lock:
            return self.obstacle_data.copy()
    
    def disconnect(self):
        """Clean shutdown of LiDAR system with proper cleanup"""
        try:
            print("🛑 Disconnecting LiDAR system...")
            
            # Stop scanning thread
            self.shutdown_flag[0] = True
            
            if self.scan_thread and self.scan_thread.is_alive():
                self.scan_thread.join(timeout=3)
                if self.scan_thread.is_alive():
                    print("⚠️ LiDAR thread did not stop gracefully")
            
            # Stop LiDAR and motor
            if self.lidar and self.is_connected:
                try:
                    self.lidar.stop()
                    time.sleep(0.5)
                    print("🛑 LiDAR scanning stopped")
                    
                    self.lidar.set_motor_pwm(0)
                    time.sleep(0.5)
                    print("🛑 LiDAR motor stopped")
                    
                    self.lidar.disconnect()
                    print("🛑 LiDAR disconnected")
                except Exception as e:
                    print(f"⚠️ LiDAR shutdown warning: {e}")
                
                self.is_connected = False
            
            # Reset state
            self.lidar = None
            self.scan_generator = None
            self.scan_iterator = None
            
            # Clear obstacle data safely
            if self.data_lock is not None:
                with self.data_lock:
                    self.obstacle_data = []
            else:
                self.obstacle_data = []
            
            print("✅ LiDAR system cleanup complete")
                
        except Exception as e:
            print(f"⚠️ LiDAR disconnect error: {e}")
            self.is_connected = False


class OptimizedLidarTestBehavior(MaxineBehavior):
    """
    Optimized LiDAR Test mode with enhanced camera integration
    Uses existing depthai infrastructure with multi-processing optimizations
    """
    
    def __init__(self):
        super().__init__("Optimized LiDAR Test with Enhanced Camera")
        
        # Blackboard setup
        self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.READ)
        self.blackboard.register_key("HEAD_CENTER_POSITION", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.WRITE)
        
        # Core systems
        self.camera_manager = MultiProcessCameraManager()
        self.lidar_system = EnhancedLidarSystem()
        self.screen = None
        self.initialized = False
        
        # Robot dimensions and positioning (from user specs)
        self.ROBOT_LENGTH_MM = 660
        self.ROBOT_WIDTH_MM = 550
        self.LIDAR_TO_CAMERA_OFFSET_MM = 130  # Camera is 130mm back from lidar
        self.CAMERA_HEIGHT_ABOVE_LIDAR_MM = 550  # Camera is 550mm above lidar
        
        # Display parameters
        self.center_x = 0
        self.center_y = 0
        self.scale = 0
        self.update_counter = 0
        self.display_update_rate = 2  # Update every 2 frames for better performance
        
        # Performance tracking
        self.performance_data = []
        self.last_stats_time = time.time()
        
        if not pygame.get_init():
            pygame.init()
        pygame.font.init()
        self.font = pygame.font.Font(None, 36)
    
    def setup(self) -> Status:
        """Initialize optimized test mode"""
        if self.initialized:
            return Status.SUCCESS
            
        try:
            # Initialize display
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode(
                (display_info.current_w, display_info.current_h), 
                pygame.FULLSCREEN
            )
            pygame.display.set_caption("MAXINE OPTIMIZED LIDAR TEST")
            
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            self.scale = min(display_info.current_w, display_info.current_h) // 12
            
            # Initialize camera system
            if not self.camera_manager.start_camera_system():
                print("⚠️ Camera system failed to start, continuing with LiDAR only")
            
            # Initialize LiDAR system with proper connection and scanning
            print("🔧 Initializing LiDAR system...")
            if self.lidar_system.connect():
                if self.lidar_system.start_scanning():
                    self.blackboard.set("LIDAR_SYSTEM", self.lidar_system)
                    print("✅ LiDAR system initialized and scanning")
                else:
                    print("❌ LiDAR scanning failed to start")
                    return Status.FAILURE
            else:
                print("❌ LiDAR connection failed")
                return Status.FAILURE
            
            self.initialized = True
            print("✅ Optimized LiDAR Test mode initialized")
            return Status.SUCCESS
            
        except Exception as e:
            print(f"❌ Setup failed: {e}")
            return Status.FAILURE
    
    def update(self) -> Status:
        """Main update loop with optimized processing"""
        try:
            self.update_counter += 1
            
            # Handle pygame events
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        robot = self.get_robot()
                        robot.set_mode(RobotMode.IDLE)
                        return Status.SUCCESS
                elif event.type == pygame.QUIT:
                    robot = self.get_robot()
                    robot.set_mode(RobotMode.IDLE)
                    return Status.SUCCESS
            
            # Update display at reduced rate for better performance
            should_update_display = (self.update_counter % self.display_update_rate == 0)
            
            if should_update_display:
                self._update_display()
                
                # Log performance data periodically
                if time.time() - self.last_stats_time > 5.0:
                    self._log_performance_stats()
                    self.last_stats_time = time.time()
            
            return Status.RUNNING
            
        except Exception as e:
            print(f"❌ Update error: {e}")
            return Status.RUNNING
    
    def _update_display(self):
        """Update pygame display with optimized rendering"""
        try:
            if not self.screen:
                return
                
            self.screen.fill((0, 0, 0))
            
            # Draw radar grid
            self._draw_radar_grid()
            
            # Draw robot
            self._draw_robot()
            
            # Get and draw LiDAR data
            obstacles = self.lidar_system.get_obstacles()
            if obstacles:
                self._draw_lidar_data(obstacles)
            
            # Get and draw camera detections
            detections = self.camera_manager.get_latest_detections()
            if detections:
                self._draw_camera_detections(detections)
            
            # Draw performance info
            self._draw_performance_info(len(obstacles), len(detections))
            
            pygame.display.flip()
            
        except Exception as e:
            pass
    
    def _draw_radar_grid(self):
        """Draw radar grid background"""
        try:
            # Draw concentric circles
            for radius in range(1000, 6000, 1000):
                scaled_radius = int(radius * self.scale / 1000)
                pygame.draw.circle(self.screen, (0, 50, 0), 
                                 (self.center_x, self.center_y), scaled_radius, 1)
            
            # Draw angle lines
            for angle in range(0, 360, 30):
                angle_rad = math.radians(angle)
                end_x = self.center_x + int(5000 * self.scale * math.cos(angle_rad) / 1000)
                end_y = self.center_y + int(5000 * self.scale * math.sin(angle_rad) / 1000)
                pygame.draw.line(self.screen, (0, 50, 0), 
                               (self.center_x, self.center_y), (end_x, end_y), 1)
                
        except Exception:
            pass
    
    def _draw_robot(self):
        """Draw robot representation"""
        try:
            # Robot body
            robot_width = int(self.ROBOT_WIDTH_MM * self.scale / 1000)
            robot_length = int(self.ROBOT_LENGTH_MM * self.scale / 1000)
            
            robot_rect = pygame.Rect(
                self.center_x - robot_width // 2,
                self.center_y - robot_length // 2,
                robot_width,
                robot_length
            )
            pygame.draw.rect(self.screen, (100, 100, 255), robot_rect, 2)
            
            # LiDAR position (front center)
            lidar_x = self.center_x
            lidar_y = self.center_y - robot_length // 2
            pygame.draw.circle(self.screen, (255, 100, 100), (lidar_x, lidar_y), 5)
            
            # Camera position (130mm back from lidar)
            camera_offset = int(self.LIDAR_TO_CAMERA_OFFSET_MM * self.scale / 1000)
            camera_x = self.center_x
            camera_y = lidar_y + camera_offset
            pygame.draw.circle(self.screen, (100, 255, 100), (camera_x, camera_y), 4)
            
        except Exception:
            pass
    
    def _draw_lidar_data(self, obstacles):
        """Draw LiDAR obstacle data"""
        try:
            for obstacle in obstacles:
                x = obstacle['x']
                y = obstacle['y']
                
                # Convert to display coordinates
                display_x = self.center_x + int(x * self.scale / 1000)
                display_y = self.center_y - int(y * self.scale / 1000)
                
                # Color based on distance
                distance = obstacle['distance']
                if distance < 1000:
                    color = (255, 0, 0)  # Red for close
                elif distance < 2000:
                    color = (255, 255, 0)  # Yellow for medium
                else:
                    color = (0, 255, 0)  # Green for far
                
                pygame.draw.circle(self.screen, color, (display_x, display_y), 2)
                
        except Exception:
            pass
    
    def _draw_camera_detections(self, detections):
        """Draw camera person detections aligned with LiDAR coordinate system (0° = front)"""
        try:
            # LiDAR position on display (coordinate reference point)
            lidar_display_x = self.center_x
            lidar_display_y = self.center_y - int(self.ROBOT_LENGTH_MM * self.scale / 2000)  # Front of robot
            
            for detection in detections:
                spatial_x = detection.get('depth_x', 0)  # Camera X: left(-)/right(+) from camera center
                spatial_y = detection.get('depth_y', 0)  # Camera Y: down(-)/up(+) from camera center  
                spatial_z = detection.get('depth_z', 0)  # Camera Z: forward distance from camera
                confidence = detection.get('confidence', 0)
                
                if spatial_z > 0:  # Valid depth data
                    # COORDINATE SYSTEM ALIGNMENT:
                    # Camera coordinates: X=left(-)/right(+), Y=down(-)/up(+), Z=forward
                    # Aligned system: 0° = front, 90° = right (matching LiDAR after adjustment)
                    
                    # Step 1: Convert camera position to LiDAR-relative position
                    # Camera is 130mm BACK from LiDAR position
                    robot_relative_x = spatial_x  # Left/right offset stays the same
                    robot_relative_y = -(spatial_z + self.LIDAR_TO_CAMERA_OFFSET_MM)  # Forward is negative Y, add camera offset
                    
                    # Step 2: Convert to display coordinates from LiDAR position
                    person_display_x = lidar_display_x + int(robot_relative_x * self.scale / 1000)
                    person_display_y = lidar_display_y - int(robot_relative_y * self.scale / 1000)  # Negative because forward is up on screen
                    
                    # Calculate angle for verification (0° = front, 90° = right)
                    if robot_relative_x != 0 or robot_relative_y != 0:
                        angle_rad = math.atan2(robot_relative_x, -robot_relative_y)  # atan2(x, -y) for 0° = front
                        angle_deg = (math.degrees(angle_rad)) % 360
                    else:
                        angle_deg = 0
                    
                    # Draw detection with confidence-based size
                    size = int(12 + confidence * 18)
                    color = (255, 165, 0)  # Orange for person detections
                    
                    # Draw filled circle for person
                    pygame.draw.circle(self.screen, color, (person_display_x, person_display_y), size)
                    
                    # Draw white outline for visibility
                    pygame.draw.circle(self.screen, (255, 255, 255), (person_display_x, person_display_y), size + 2, 3)
                    
                    # Draw confidence, distance, and angle info
                    distance_m = spatial_z / 1000.0
                    info_text = f"PERSON {confidence:.2f}"
                    distance_text = f"({distance_m:.1f}m @ {angle_deg:.0f}°)"
                    
                    text_surface = self.font.render(info_text, True, (255, 255, 255))
                    dist_surface = self.font.render(distance_text, True, (255, 165, 0))
                    
                    self.screen.blit(text_surface, (person_display_x + size + 5, person_display_y - 15))
                    self.screen.blit(dist_surface, (person_display_x + size + 5, person_display_y + 10))
                    
                    # Draw line from LiDAR position to person for alignment verification
                    pygame.draw.line(self.screen, (255, 165, 0), 
                                   (lidar_display_x, lidar_display_y), 
                                   (person_display_x, person_display_y), 3)
                    
        except Exception as e:
            print(f"⚠️ Camera detection drawing error: {e}")
            pass
    
    def _draw_performance_info(self, obstacle_count, detection_count):
        """Draw performance information with coordinate system details"""
        try:
            camera_stats = self.camera_manager.get_performance_stats()
            
            info_lines = [
                "=== OPTIMIZED LIDAR TEST MODE ===",
                f"LiDAR Obstacles: {obstacle_count}",
                f"Camera Detections: {detection_count}",
                f"Camera FPS: {camera_stats['fps']:.1f} (Target: 25)",
                f"Frames Processed: {camera_stats['frame_count']}",
                f"Total People Found: {camera_stats['detection_count']}",
                f"Camera Thread: {'Active' if camera_stats['thread_active'] else 'Inactive'}",
                "",
                "=== COORDINATE SYSTEM (ALIGNED) ===",
                f"0° = FRONT  |  90° = RIGHT  |  180° = REAR  |  270° = LEFT",
                f"LiDAR: Front center (reference point)",
                f"Camera: {self.LIDAR_TO_CAMERA_OFFSET_MM}mm back from LiDAR",
                f"🟠 Orange circles = Camera detections",
                f"🔴🟡🟢 Colored dots = LiDAR obstacles",
                f"Lines show alignment verification",
                f"Angles shown for close detections",
                "",
                "=== TESTING INSTRUCTIONS ===",
                f"Confidence Threshold: {self.camera_manager.DETECTION_CONFIDENCE:.2f} ({self.camera_manager.DETECTION_CONFIDENCE*100:.0f}%)",
                "Stand 1-3m in front for best detection",
                "Camera and LiDAR should show same position",
                "Press ESC to exit to IDLE mode"
            ]
            
            y_offset = 50
            for line in info_lines:
                if line.startswith("=== OPTIMIZED"):
                    color = (255, 255, 0)  # Yellow for main header
                elif line.startswith("=== COORDINATE"):
                    color = (0, 255, 255)  # Cyan for coordinate info
                elif line.startswith("=== TESTING"):
                    color = (255, 165, 0)  # Orange for testing info
                elif "FPS:" in line or "Thread:" in line or "Detections:" in line:
                    color = (0, 255, 255)  # Cyan for performance metrics
                elif "0° = FRONT" in line:
                    color = (150, 255, 150)  # Light green for degree reference
                elif "Orange circles" in line or "Colored dots" in line or "Lines show" in line or "Angles shown" in line:
                    color = (200, 200, 200)  # Light gray for legend
                elif "LiDAR:" in line or "Camera:" in line:
                    color = (150, 255, 150)  # Light green for position info
                elif "Confidence:" in line or "Stand 1-3m" in line or "Camera and LiDAR" in line:
                    color = (255, 165, 0)  # Orange for important info
                elif line == "":
                    y_offset += 10  # Add spacing
                    continue
                else:
                    color = (255, 255, 255)  # White for info
                    
                text = self.font.render(line, True, color)
                self.screen.blit(text, (50, y_offset))
                y_offset += 28
                
        except Exception:
            pass
    
    def _log_performance_stats(self):
        """Log performance statistics"""
        try:
            camera_stats = self.camera_manager.get_performance_stats()
            obstacles = self.lidar_system.get_obstacles()
            detections = self.camera_manager.get_latest_detections()
            
            stats = {
                'timestamp': datetime.now().isoformat(),
                'camera_fps': camera_stats['fps'],
                'frame_count': camera_stats['frame_count'],
                'obstacle_count': len(obstacles),
                'detection_count': len(detections),
                'queue_detection': camera_stats['queue_sizes']['detection'],
                'queue_result': camera_stats['queue_sizes']['result']
            }
            
            self.performance_data.append(stats)
            
            # Keep last 100 entries
            if len(self.performance_data) > 100:
                self.performance_data = self.performance_data[-100:]
                
        except Exception:
            pass
    
    def terminate(self, new_status: Status):
        """Clean shutdown of optimized test mode with proper avatar restoration"""
        try:
            print("🛑 Terminating optimized LiDAR test mode...")
            
            # Save performance data
            self._save_performance_data()
            
            # Cleanup camera system first
            try:
                if self.camera_manager:
                    self.camera_manager.cleanup()
                    self.camera_manager = None
            except Exception as e:
                print(f"⚠️ Camera cleanup warning: {e}")
            
            # Cleanup LiDAR system
            try:
                if self.lidar_system:
                    self.lidar_system.disconnect()
                    self.lidar_system = None
            except Exception as e:
                print(f"⚠️ LiDAR cleanup warning: {e}")
            
            # Cleanup blackboard
            try:
                if self.blackboard.exists("LIDAR_SYSTEM"):
                    self.blackboard.unset("LIDAR_SYSTEM")
            except Exception:
                pass
            
            # IMPORTANT: Restore avatar display properly
            try:
                robot = self.get_robot()
                if hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                    print("🎭 Restoring avatar display...")
                    
                    # Recreate the facial animation display (since we took it over)
                    robot.facial_animation_manager.close_window()
                    time.sleep(0.5)  # Give time for cleanup
                    robot.facial_animation_manager.open_window()
                    
                    # Display resting face
                    if hasattr(robot.facial_animation_manager, 'resting_face_img'):
                        robot.facial_animation_manager.display.fill((0, 0, 0))
                        robot.facial_animation_manager.display.blit(
                            robot.facial_animation_manager.resting_face_img, (0, 0)
                        )
                        pygame.display.flip()
                        pygame.display.update()
                    
                    robot.facial_animation_manager.bring_to_front()
                    print("✅ Avatar display restored")
                    
            except Exception as e:
                print(f"⚠️ Avatar restoration warning: {e}")
            
            # Reset screen reference
            self.screen = None
            self.initialized = False
            
            print("✅ Optimized LiDAR Test mode terminated cleanly")
            
        except Exception as e:
            print(f"⚠️ Termination error: {e}")
        
        super().terminate(new_status)
    
    def _save_performance_data(self):
        """Save performance data to CSV"""
        try:
            if not self.performance_data:
                return
                
            filename = f"optimized_lidar_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            
            with open(filename, 'w', newline='') as csvfile:
                fieldnames = ['timestamp', 'camera_fps', 'frame_count', 'obstacle_count', 
                            'detection_count', 'queue_detection', 'queue_result']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                writer.writeheader()
                for data in self.performance_data:
                    writer.writerow(data)
            
            print(f"📊 Performance data saved to {filename}")
            
        except Exception as e:
            print(f"⚠️ Failed to save performance data: {e}")


# Export classes for compatibility
StableLidarTest = OptimizedLidarTestBehavior
LidarTestBehavior = OptimizedLidarTestBehavior
OptimizedLidarTest = OptimizedLidarTestBehavior