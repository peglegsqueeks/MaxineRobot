#!/usr/bin/env python3
"""
LiDAR Test Mode - FIXED with CSV Consistency Logging
Combines working head tracking with full LiDAR obstacle detection and display
ADDED: X-midpoint consistency tracking and LIDARTEST.csv logging
FIXED: Removed overly aggressive detection filtering for better stability
"""

import pygame
import math
import time
import threading
import queue
import py_trees
import csv
import statistics
from collections import deque
from datetime import datetime
from py_trees.common import Status
from pyrplidar import PyRPlidar

from src.behaviors.MaxineBehavior import MaxineBehavior
from src.types.RobotModes import RobotMode
from src.types.CameraMode import CameraMode
from src.types.MovementDirection import MovementDirection
from src.action_managers.VelocityManager import VelocityConfig


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
        
        # Conservative obstacle mapping for moving robot
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
    
    def get_obstacles(self):
        """Get obstacles (alias for compatibility)"""
        return self.get_display_obstacles()
    
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


class EnhancedFacialAnimationRestorer:
    """Enhanced facial animation restorer"""
    
    def restore_resting_face_immediately(self, robot):
        """Restore resting face"""
        try:
            if hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                for _ in range(3):
                    try:
                        robot.facial_animation_manager.display.fill((0, 0, 0))
                        robot.facial_animation_manager.display.blit(
                            robot.facial_animation_manager.resting_face_img, (0, 0)
                        )
                        pygame.display.flip()
                        pygame.display.update()
                        time.sleep(0.1)
                    except Exception:
                        continue
        except Exception:
            pass


class StableLidarTest(MaxineBehavior):
    """Stable LiDAR Test with FIXED detection consistency and CSV logging"""
    
    def __init__(self):
        super().__init__("Stable LiDAR Test - FIXED with CSV Logging")
        
        # Blackboard setup
        self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.READ)
        self.blackboard.register_key("HEAD_CENTER_POSITION", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.WRITE)
        
        # Core components
        self.lidar_system = None
        self.screen = None
        self.head_tracker = None
        self.facial_restorer = EnhancedFacialAnimationRestorer()
        self.initialized = False
        
        # Speech tracking
        self.idle_mode_announced = False
        
        # Head tracking
        self.current_head_angle = 0.0
        self.head_angle_lock = threading.Lock()
        
        # Display parameters
        self.center_x = 0
        self.center_y = 0
        self.scale = 0
        self.update_counter = 0
        self.display_update_rate = 3
        
        # Person tracking with stability improvements
        self.last_person_detected = 0
        self.person_lost_timeout = 2.0
        
        # FIXED: Much more lenient head tracking parameters
        self.angle_history = []
        self.max_angle_history = 5           # FIXED: Reduced from 8 to 5
        self.last_sent_angle = None
        self.angle_change_threshold = math.radians(8)  # FIXED: Increased from 2° to 8°
        self.update_counter_tracking = 0
        self.tracking_update_interval = 6    # FIXED: Increased from 3 to 6 frames
        
        # ADDED: X-MIDPOINT CONSISTENCY TRACKING AND CSV LOGGING
        self.csv_log_filename = "LIDARTEST.csv"
        self.csv_initialized = False
        self.mode_start_time = time.time()
        
        # X-midpoint tracking for consistency analysis
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
        self.short_term_variance = deque(maxlen=75)   # 3-second variance at 25fps
        self.medium_term_variance = deque(maxlen=250) # 10-second variance
        self.long_term_variance = deque(maxlen=1500)  # 60-second variance
        
        # Stability classification
        self.stability_zones = {
            'stable': 0,      # <10px variance
            'moderate': 0,    # 10-25px variance
            'unstable': 0,    # 25-50px variance  
            'very_unstable': 0  # >50px variance
        }
        
        if not pygame.get_init():
            pygame.init()
        pygame.font.init()
    
    def initialize_csv_log(self):
        """Initialize CSV log for Lidar Test consistency analysis"""
        if self.csv_initialized:
            return
        
        try:
            # Always overwrite existing file
            with open(self.csv_log_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # CSV Header for consistency analysis
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
            print(f"✅ Lidar Test consistency logging to: {self.csv_log_filename}")
        except Exception as e:
            print(f"⚠️ CSV initialization failed: {e}")
    
    def calculate_x_midpoint_variance(self):
        """Calculate real-time X midpoint variance for consistency analysis"""
        try:
            if len(self.variance_window) < 2:
                return 0.0, 0.0, 0.0
            
            window_data = list(self.variance_window)
            
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
    
    def log_detection_consistency_to_csv(self, person_data):
        """Log detection consistency data to LIDARTEST.csv"""
        try:
            if not self.csv_initialized:
                self.initialize_csv_log()
            
            # Extract data from person_data
            bbox_center = person_data['bbox_center']
            x_midpoint_pixels = bbox_center['x_pixels']
            x_midpoint_normalized = bbox_center['x_normalized']
            
            # Track midpoints for variance analysis
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
            
            # Get current head tracking info
            head_angle_deg = math.degrees(self.last_sent_angle) if self.last_sent_angle else 0.0
            head_tracking_active = self.head_tracker is not None
            
            # Calculate mode elapsed time
            mode_elapsed = time.time() - self.mode_start_time
            
            # Write to CSV
            with open(self.csv_log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    mode_elapsed,
                    time.time(),
                    self.update_counter,
                    x_midpoint_pixels,
                    x_midpoint_normalized,
                    person_data['x_camera'],
                    person_data['z_camera'],
                    person_data['confidence'],
                    person_data['bounding_box']['xmin'],
                    person_data['bounding_box']['ymin'],
                    person_data['bounding_box']['xmax'],
                    person_data['bounding_box']['ymax'],
                    person_data['bounding_box']['xmax'] - person_data['bounding_box']['xmin'],
                    person_data['bounding_box']['ymax'] - person_data['bounding_box']['ymin'],
                    x_jump,
                    is_large_jump,
                    self.current_variance_pixels,
                    self.current_std_dev_pixels,
                    self.current_mean_pixels,
                    short_var,
                    medium_var,
                    long_var,
                    self.detection_count,
                    self.consistent_detection_count,
                    self.large_jumps_count,
                    stability_class,
                    head_angle_deg,
                    head_tracking_active
                ])
        except Exception as e:
            # Silent error handling to avoid disrupting main functionality
            pass
    
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
            pygame.display.set_caption("MAXINE LIDAR TEST - FIXED WITH CSV LOGGING")
            
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            self.scale = min(display_info.current_w, display_info.current_h) // 7
            
            # Draw initial interface
            self.draw_clean_interface()
            pygame.display.flip()
            
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
            else:
                self.head_tracker = None
            
            # Initialize LiDAR system
            self.start_stable_lidar()
            
            self.initialized = True
            return True
            
        except Exception as e:
            print(f"Initialization error: {e}")
            self.initialized = False
            return False
    
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
                    time.sleep(3)  # Allow stabilization
                else:
                    print("❌ Failed to start LiDAR system")
                    self.lidar_system = None
        except Exception as e:
            print(f"❌ LiDAR initialization error: {e}")
            self.lidar_system = None
    
    def draw_clean_interface(self):
        """Draw clean radar interface"""
        self.screen.fill((0, 0, 0))
        self.draw_radar_grid()
        self.draw_robot()
    
    def draw_radar_grid(self):
        """Draw radar-style grid"""
        # Range circles
        for distance in [1000, 2000, 3000, 4000, 5000, 6000]:
            radius = distance * self.scale // 1000
            if radius < min(self.center_x, self.center_y) - 50:
                line_width = 3 if distance == 6000 else 2
                color = (0, 150, 0) if distance < 6000 else (255, 255, 0)
                pygame.draw.circle(self.screen, color, (self.center_x, self.center_y), radius, line_width)
        
        # Angle lines
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
            # Fix coordinate system: LiDAR 0° = front, Display should show front as up
            # Rotate by 90° to align LiDAR front (0°) with display front (up)
            corrected_angle_rad = math.radians(90 - angle)
            
            x = self.center_x + math.cos(corrected_angle_rad) * (distance * self.scale // 1000)
            y = self.center_y - math.sin(corrected_angle_rad) * (distance * self.scale // 1000)  # Negative for pygame Y-axis
            
            if (0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height()):
                pygame.draw.circle(self.screen, (255, 255, 255), (int(x), int(y)), 2)
                obstacle_count += 1
        
        return obstacle_count
    
    def get_person_detection(self):
        """FIXED: Simplified person detection similar to standalone test for better consistency"""
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
            
            # FIXED: Use first/closest person with much lower confidence threshold (like standalone test)
            closest_person = people[0]  # Already sorted by distance in ObjectDetectionReading
            
            # FIXED: Use lower confidence threshold like standalone test (0.4 vs 0.5)
            if closest_person.confidence < 0.4:
                return None
            
            # FIXED: Simple bounding box calculation without complex filtering
            x_center_normalized = (closest_person.xmax + closest_person.xmin) / 2.0
            y_center_normalized = (closest_person.ymax + closest_person.ymin) / 2.0
            
            # Convert to screen coordinates for display
            screen_width = self.screen.get_width() if self.screen else 1920
            screen_height = self.screen.get_height() if self.screen else 1080
            
            x_center_pixels = int(x_center_normalized * screen_width)
            y_center_pixels = int(y_center_normalized * screen_height)
            
            return {
                'x_camera': closest_person.spatialCoordinates.x,  # Keep original for head tracking
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
    
    def draw_person_detection(self):
        """Draw person detection with consistency information"""
        person_data = self.get_person_detection()
        
        if not person_data:
            return
        
        try:
            x_camera = person_data['x_camera']
            z_camera = person_data['z_camera']
            confidence = person_data['confidence']
            bbox_center = person_data['bbox_center']
            
            if z_camera <= 0:
                return
            
            # ADDED: Log detection consistency data to CSV
            self.log_detection_consistency_to_csv(person_data)
            
            # Calculate angle and position for radar display
            person_angle_rad = math.atan2(x_camera, z_camera)
            person_angle_deg = math.degrees(person_angle_rad)
            
            # Convert to display coordinates (same as LiDAR)
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
                color = (0, 255, 0)      # Green - stable
            elif stability_class == 'moderate':
                color = (255, 255, 0)    # Yellow - moderate
            elif stability_class == 'unstable':
                color = (255, 165, 0)    # Orange - unstable
            else:
                color = (255, 0, 0)      # Red - very unstable
            
            # Draw detection with stability color coding
            if 0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height():
                pygame.draw.circle(self.screen, color, (x, y), 8)
                pygame.draw.circle(self.screen, (255, 255, 255), (x, y), 9, 2)
                
                # Show x-pixel position and variance
                font = pygame.font.Font(None, 20)
                x_pixels = bbox_center['x_pixels']
                info_text = f"X:{x_pixels}px ±{self.current_std_dev_pixels:.1f}"
                text_surface = font.render(info_text, True, color)
                self.screen.blit(text_surface, (x + 10, y - 10))
            
            # Draw large x-coordinate information at top of screen (like original)
            self.draw_person_x_coordinate_info(bbox_center)
            
        except Exception:
            pass
    
    def draw_person_x_coordinate_info(self, bbox_center):
        """Draw person's x-coordinate center in large font with consistency info"""
        try:
            screen_width = self.screen.get_width()
            screen_height = self.screen.get_height()
            
            # Get coordinates and consistency data
            x_norm = bbox_center['x_normalized']
            x_pixels = bbox_center['x_pixels']
            
            # Create large font for display
            large_font = pygame.font.Font(None, 72)
            medium_font = pygame.font.Font(None, 48)
            
            # Format the display text with consistency info
            main_text = f"Person X-Center: {x_pixels}px"
            detail_text = f"Normalized: {x_norm:.3f} | Variance: ±{self.current_std_dev_pixels:.1f}px"
            consistency_text = f"Detections: {self.detection_count} | Consistent: {self.consistent_detection_count} | Jumps: {self.large_jumps_count}"
            
            # Color based on consistency
            if self.current_std_dev_pixels <= 10:
                main_color = (0, 255, 0)      # Green - stable
            elif self.current_std_dev_pixels <= 25:
                main_color = (255, 255, 0)    # Yellow - moderate
            elif self.current_std_dev_pixels <= 50:
                main_color = (255, 165, 0)    # Orange - unstable
            else:
                main_color = (255, 0, 0)      # Red - very unstable
            
            # Render text surfaces
            main_surface = large_font.render(main_text, True, main_color)
            detail_surface = medium_font.render(detail_text, True, (255, 255, 255))
            consistency_surface = medium_font.render(consistency_text, True, (0, 255, 255))
            
            # Position at top center of screen
            main_x = (screen_width - main_surface.get_width()) // 2
            main_y = 50
            
            detail_x = (screen_width - detail_surface.get_width()) // 2
            detail_y = main_y + main_surface.get_height() + 10
            
            consistency_x = (screen_width - consistency_surface.get_width()) // 2
            consistency_y = detail_y + detail_surface.get_height() + 10
            
            # Draw background rectangles for readability
            total_height = main_surface.get_height() + detail_surface.get_height() + consistency_surface.get_height() + 30
            max_width = max(main_surface.get_width(), detail_surface.get_width(), consistency_surface.get_width())
            status_bg = pygame.Rect(main_x - 10, main_y - 5, max_width + 20, total_height)
            
            pygame.draw.rect(self.screen, (0, 0, 0), status_bg)
            pygame.draw.rect(self.screen, main_color, status_bg, 3)
            
            # Blit text to screen
            self.screen.blit(main_surface, (main_x, main_y))
            self.screen.blit(detail_surface, (detail_x, detail_y))
            self.screen.blit(consistency_surface, (consistency_x, consistency_y))
            
        except Exception:
            pass

    def update_head_tracking(self, person_data):
        """FIXED: More stable head tracking with larger dead zone and less sensitivity"""
        if not self.head_tracker or not person_data:
            return
        
        try:
            bbox_center = person_data['bbox_center']
            x_pixels = bbox_center['x_pixels']
            z_camera = person_data['z_camera']
            
            if z_camera <= 0:
                return
            
            # FIXED: Only process every 6 frames to reduce jitter (increased from 3)
            self.update_counter_tracking += 1
            if self.update_counter_tracking % self.tracking_update_interval != 0:
                return
            
            # Calculate pixel-based angle
            screen_center_x = self.screen.get_width() // 2 if self.screen else 960
            pixel_offset = x_pixels - screen_center_x
            pixel_offset_normalized = pixel_offset / screen_center_x
            
            # Convert to angle with correct polarity
            camera_hfov_rad = math.radians(108)
            raw_angle_rad = -pixel_offset_normalized * (camera_hfov_rad / 2.0)
            
            # FIXED: Reduced smoothing history for faster response
            self.angle_history.append(raw_angle_rad)
            if len(self.angle_history) > self.max_angle_history:
                self.angle_history.pop(0)
            
            # FIXED: Simple average instead of complex weighted average
            if len(self.angle_history) >= 2:
                smoothed_angle_rad = sum(self.angle_history) / len(self.angle_history)
            else:
                smoothed_angle_rad = raw_angle_rad
            
            smoothed_angle_deg = math.degrees(smoothed_angle_rad)
            
            # FIXED: Much larger dead zone for stability (12° instead of 6°)
            dead_zone_rad = math.radians(12)  # FIXED: Doubled dead zone
            is_in_dead_zone = abs(smoothed_angle_rad) <= dead_zone_rad
            
            # FIXED: Less sensitive change threshold
            significant_change = (self.last_sent_angle is None or 
                                abs(smoothed_angle_rad - self.last_sent_angle) > self.angle_change_threshold)
            
            # Decision logic - FIXED: Much more restrictive movement
            will_send_command = (not is_in_dead_zone) and significant_change
            
            # Execute head movement decision
            if will_send_command:
                self.head_tracker.set_person_tracking(smoothed_angle_rad)
                self.last_sent_angle = smoothed_angle_rad
            
            self.last_person_detected = time.time()
            
        except Exception as e:
            pass

    def draw_info(self, obstacle_count):
        """Draw test information with consistency metrics"""
        try:
            tracking_status = "ON" if self.head_tracker else "OFF"
            lidar_status = "ACTIVE" if self.lidar_system else "INACTIVE"
            
            # Get current head position for debug
            current_head_angle = math.degrees(self.last_sent_angle) if self.last_sent_angle else 0.0
            
            # Calculate consistency rate
            consistency_rate = (self.consistent_detection_count / max(1, self.detection_count)) * 100
            
            info_lines = [
                f"MAXINE LIDAR TEST - LiDAR: {lidar_status} | Head: {tracking_status}",
                f"LiDAR Obstacles: {obstacle_count} | Head: {current_head_angle:.1f}° | DeadZone: ±12°",
                f"Detection Consistency: {consistency_rate:.1f}% | Variance: ±{self.current_std_dev_pixels:.1f}px",
                f"Detections: {self.detection_count} | Large Jumps: {self.large_jumps_count}",
                f"CSV Logging: {self.csv_log_filename} | ESC: Exit to Idle Mode"
            ]
            
            y_offset = self.screen.get_height() - 150
            font = pygame.font.Font(None, 36)
            
            for i, line in enumerate(info_lines):
                if i == 0:
                    color = (0, 255, 255)  # Cyan for header
                elif "Consistency" in line:
                    # Color-code consistency line
                    if consistency_rate > 90:
                        color = (0, 255, 0)      # Green
                    elif consistency_rate > 80:
                        color = (255, 255, 0)    # Yellow
                    else:
                        color = (255, 0, 0)      # Red
                elif "CSV Logging" in line:
                    color = (255, 165, 0)    # Orange for CSV info
                else:
                    color = (255, 255, 255)  # White
                
                text_surface = font.render(line, True, color)
                self.screen.blit(text_surface, (20, y_offset + i * 30))
            
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
    
    def perform_enhanced_idle_mode_transition(self):
        """Enhanced transition to IDLE mode with final CSV summary"""
        try:
            # Generate final CSV summary
            if self.detection_count > 0:
                overall_variance = statistics.variance(list(self.x_midpoints_pixels)) if len(self.x_midpoints_pixels) > 1 else 0
                overall_std_dev = math.sqrt(overall_variance)
                consistency_rate = (self.consistent_detection_count / self.detection_count) * 100
                
                print(f"\n📊 LIDAR TEST CONSISTENCY SUMMARY:")
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
            
            if pygame.get_init():
                pygame.event.clear()
            
            self.facial_restorer.restore_resting_face_immediately(robot)
            
            return True
            
        except Exception:
            return False
    
    def update(self) -> Status:
        """Main update method with FIXED detection consistency and CSV logging"""
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
                # Clear tracking history when person is lost
                self.angle_history.clear()
                self.last_sent_angle = None
                self.head_tracker.set_manual_position(0.0)
            
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.perform_enhanced_idle_mode_transition()
                        robot = self.get_robot()
                        robot.set_mode(RobotMode.IDLE)
                        return Status.SUCCESS
                elif event.type == pygame.QUIT:
                    self.perform_enhanced_idle_mode_transition()
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
                        
                        # Draw person detection with consistency tracking
                        self.draw_person_detection()
                        
                        # Draw info with consistency metrics
                        self.draw_info(obstacle_count)
                        
                        pygame.display.flip()
                        
                except Exception as e:
                    print(f"Display error: {e}")
            
            return Status.RUNNING
            
        except Exception as e:
            print(f"Update error: {e}")
            try:
                self.stop_robot()
                self.perform_enhanced_idle_mode_transition()
            except Exception:
                pass
            
            return Status.RUNNING

    def terminate(self, new_status: Status):
        """Terminate method with CSV summary"""
        try:
            self.stop_robot()
            
            if self.head_tracker:
                self.head_tracker.stop_tracking()
                self.head_tracker = None
            
            if self.lidar_system:
                self.lidar_system.stop()
                self.lidar_system = None
            
            self.perform_enhanced_idle_mode_transition()
            
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


# Compatibility aliases
LidarTestBehavior = StableLidarTest
OptimizedStableLidarTest = StableLidarTest 
OptimizedLidarTestWithHybridDistance = StableLidarTest
FixedStableLidarTest = StableLidarTest

# Legacy classes
class HybridDistanceCalculator:
    pass

class FixedUltraStablePyRPLidarA3:
    pass

class FixedUltraStableLidarSystem:
    pass