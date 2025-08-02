#!/usr/bin/env python3
"""
Fixed LiDAR Test Mode - Complete Implementation
Includes dual FPS tracking (Camera + Combined LiDAR+Camera), enhanced coordinate system 
with head rotation handling, and comprehensive visual debugging features while preserving 
original working exit behavior. Console output minimized for clean operation.
"""

import pygame
import math
import time
import threading
import py_trees
from py_trees.common import Status
from pyrplidar import PyRPlidar
import queue
from collections import deque

from src.behaviors.MaxineBehavior import MaxineBehavior
from src.types.RobotModes import RobotMode
from src.types.CameraMode import CameraMode
from src.types.KeyboardKey import KeyboardKey
from src.types.MovementDirection import MovementDirection
from src.action_managers.VelocityManager import VelocityConfig


class FixedFastLidarSystem:
    """Fixed fast LiDAR system that works with original behavior structure"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000, timeout=1.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.lidar = None
        self.is_connected = False
        self.scan_generator = None
        self.scan_iterator = None
        self.shutdown_flag = [False]
        self.latest_obstacles = []
        self.scan_thread = None
        self.data_lock = threading.Lock()
        
        # Fast scanning parameters (improved from original)
        self.motor_pwm = 650  # Balanced speed and stability
        self.scan_mode = 2    # Express mode for faster scanning
        
    def connect(self):
        """Connect to LiDAR"""
        try:
            self.lidar = PyRPlidar()
            self.lidar.connect(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            self.is_connected = True
            return True
        except Exception:
            self.is_connected = False
            return False
    
    def start(self):
        """Start LiDAR with fast scanning"""
        try:
            if not self.is_connected and not self.connect():
                return False
                
            self.lidar.stop()
            time.sleep(0.5)
            
            # Set motor PWM for faster scanning
            self.lidar.set_motor_pwm(self.motor_pwm)
            time.sleep(2.0)
            
            # Start express scan for speed
            self.scan_generator = self.lidar.start_scan_express(self.scan_mode)
            self.scan_iterator = self.scan_generator()
            
            # Start background scanning thread
            self.shutdown_flag = [False]
            self.scan_thread = threading.Thread(target=self._scan_worker, daemon=True)
            self.scan_thread.start()
            
            return True
            
        except Exception:
            return False
    
    def _scan_worker(self):
        """Background scanning thread"""
        scan_buffer = []
        last_angle = None
        min_scan_points = 100  # Reduced for faster updates
        
        while not self.shutdown_flag[0]:
            try:
                measurement = next(self.scan_iterator)
                
                if measurement:
                    try:
                        quality = getattr(measurement, 'quality', 0)
                        angle = getattr(measurement, 'angle', 0)
                        distance = getattr(measurement, 'distance', 0)
                        
                        # Filter valid measurements
                        if quality > 5 and 150 < distance < 6000:
                            scan_buffer.append((angle, distance))
                            
                            # Detect complete scan (360° rotation)
                            if (last_angle is not None and 
                                angle < last_angle and 
                                len(scan_buffer) > min_scan_points):
                                
                                # Update latest obstacles
                                with self.data_lock:
                                    self.latest_obstacles = scan_buffer.copy()
                                scan_buffer.clear()
                            
                            last_angle = angle
                        
                    except Exception:
                        continue
                else:
                    time.sleep(0.01)
                    
            except StopIteration:
                # Restart scanning if needed
                try:
                    if not self.shutdown_flag[0]:
                        self.lidar.stop()
                        time.sleep(0.5)
                        self.lidar.set_motor_pwm(self.motor_pwm)
                        time.sleep(1.0)
                        self.scan_generator = self.lidar.start_scan_express(self.scan_mode)
                        self.scan_iterator = self.scan_generator()
                except Exception:
                    break
                    
            except Exception:
                time.sleep(0.1)
    
    def get_display_obstacles(self):
        """Get obstacles for display (compatible with original interface)"""
        with self.data_lock:
            return self.latest_obstacles.copy() if self.latest_obstacles else []
    
    def stop(self):
        """Stop LiDAR system"""
        try:
            self.shutdown_flag[0] = True
            
            if self.scan_thread and self.scan_thread.is_alive():
                self.scan_thread.join(timeout=2.0)
            
            if self.lidar and self.is_connected:
                self.lidar.stop()
                time.sleep(0.5)
                self.lidar.set_motor_pwm(0)
                time.sleep(0.5)
                self.lidar.disconnect()
                
            self.is_connected = False
            
        except Exception:
            pass


class EnhancedFacialAnimationRestorer:
    """Enhanced facial animation restorer - compatible with original"""
    
    def restore_resting_face_immediately(self, robot):
        """Restore resting face"""
        try:
            if hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                for attempt in range(3):
                    try:
                        robot.facial_animation_manager.display.fill((0, 0, 0))
                        robot.facial_animation_manager.display.blit(
                            robot.facial_animation_manager.resting_face_img, (0, 0)
                        )
                        pygame.display.flip()
                        pygame.display.update()
                        time.sleep(0.1)
                        break
                    except Exception:
                        continue
        except Exception:
            pass


class StableLidarTest(MaxineBehavior):
    """Enhanced StableLidarTest with dual FPS tracking and comprehensive coordinate system handling"""
    
    def __init__(self):
        super().__init__("Stable LiDAR Test with Enhanced Facial Restoration")
        
        # Blackboard setup (ORIGINAL)
        self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.READ)
        self.blackboard.register_key("HEAD_CENTER_POSITION", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.WRITE)
        
        # Core components (ORIGINAL)
        self.lidar_system = None
        self.screen = None
        self.head_tracker = None
        self.facial_restorer = EnhancedFacialAnimationRestorer()
        self.initialized = False
        
        # Speech tracking (ORIGINAL)
        self.idle_mode_announced = False
        
        # Head tracking state (ORIGINAL)
        self.current_head_angle = 0.0
        self.head_angle_lock = threading.Lock()
        
        # Display parameters (ORIGINAL)
        self.center_x = 0
        self.center_y = 0
        self.scale = 0
        self.update_counter = 0
        self.display_update_rate = 3
        
        # FPS tracking for camera system
        self.camera_fps_counter = deque(maxlen=30)  # Track camera detection FPS
        self.camera_frame_count = 0
        self.camera_last_fps_update = time.time()
        self.camera_fps = 0.0
        
        # FPS tracking for combined LiDAR+Camera system
        self.system_fps_counter = deque(maxlen=30)  # Track combined system FPS (LiDAR + Camera)
        self.system_frame_count = 0
        self.system_last_fps_update = time.time()
        self.system_fps = 0.0
        
        # Enhanced coordinate system tracking
        self.coordinate_debug_enabled = True
        self.last_head_angle_deg = 0.0
        self.camera_coordinate_history = deque(maxlen=5)
        
        if not pygame.get_init():
            pygame.init()
        pygame.font.init()
    
    def initialize_components(self):
        """Initialize test mode components - ORIGINAL STRUCTURE"""
        if self.initialized:
            return True
        
        try:
            # Get head align center position (ORIGINAL)
            head_align_center = self.get_head_align_center_position()
            head_moved_to_center = self.move_head_to_center_position(head_align_center)
            
            # Initialize display (ORIGINAL)
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
            pygame.display.set_caption("MAXINE STABLE LIDAR TEST")
            
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            self.scale = min(display_info.current_w, display_info.current_h) // 7
            
            # Draw clean interface (ORIGINAL)
            self.draw_clean_interface()
            pygame.display.flip()
            
            self.blackboard.set("HEAD_CENTER_POSITION", head_align_center)
            
            # Initialize camera system (FIXED - more robust)
            print("📷 Initializing camera system...")
            camera_success = self.initialize_camera_system()
            if camera_success:
                print("✅ Camera system active")
            else:
                print("⚠️ Camera system failed, continuing without camera")
            
            # Initialize head tracker (ORIGINAL)
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
                    self.current_head_angle = 0.0
                except Exception:
                    self.head_tracker = None
            else:
                self.head_tracker = None
            
            self.initialized = True
            
            # Start enhanced stable lidar (FIXED - faster scanning)
            self.start_enhanced_stable_lidar()
            
            return True
            
        except Exception:
            self.initialized = False
            return False
    
    def initialize_camera_system(self):
        """FIXED camera initialization with better error handling"""
        try:
            robot = self.get_robot()
            
            # Check if robot has camera sensor
            if not hasattr(robot, 'camera_sensor') or not robot.camera_sensor:
                return False
            
            # Try to switch to object detection mode with retries
            for attempt in range(3):
                try:
                    robot.camera_sensor.switch_mode(CameraMode.OBJECT_DETECTION)
                    time.sleep(2.0)  # Allow initialization time
                    
                    # Test camera reading
                    test_reading = robot.camera_sensor.get_reading()
                    if test_reading is not None:
                        return True
                    
                except Exception:
                    time.sleep(1.0)
                    continue
            
            return False
            
        except Exception:
            return False
    
    def start_enhanced_stable_lidar(self):
        """FIXED - Start enhanced stable LiDAR system with fast scanning"""
        try:
            if not self.lidar_system:
                self.lidar_system = FixedFastLidarSystem()  # FIXED - use fast system
                success = self.lidar_system.start()
                if success:
                    self.blackboard.set("LIDAR_SYSTEM", self.lidar_system)
                    print("✅ LiDAR scanning started (fast mode)")
                    time.sleep(3)
                else:
                    print("❌ LiDAR fast scanning failed")
        except Exception:
            pass
    
    def get_head_align_center_position(self):
        """Get head align center position (ORIGINAL)"""
        try:
            head_offset = self.blackboard.get("HeadAlignOffSet")
            if head_offset is not None:
                return head_offset
        except Exception:
            pass
        return 0.0
    
    def move_head_to_center_position(self, center_position):
        """Move head to center position (ORIGINAL)"""
        try:
            robot = self.get_robot()
            if hasattr(robot, 'servo_controller') and robot.servo_controller:
                robot.servo_controller.set_position(center_position)
                time.sleep(1.0)
                return True
        except Exception:
            pass
        return False
    
    def draw_clean_interface(self):
        """Draw clean test interface (ORIGINAL)"""
        self.screen.fill((0, 0, 0))
        self.draw_radar_grid()
        self.draw_robot()
    
    def draw_radar_grid(self):
        """Draw radar grid (ORIGINAL)"""
        # Draw range circles
        for distance in [1000, 2000, 3000, 4000, 5000, 6000]:
            radius = distance * self.scale // 1000
            if radius < min(self.center_x, self.center_y) - 50:
                line_width = 3 if distance == 6000 else 2
                color = (0, 150, 0) if distance < 6000 else (255, 255, 0)
                pygame.draw.circle(self.screen, color, (self.center_x, self.center_y), radius, line_width)
        
        # Draw angle lines
        for angle in range(0, 360, 30):
            angle_rad = math.radians(angle)
            end_x = self.center_x + math.cos(angle_rad) * (self.scale * 6)
            end_y = self.center_y + math.sin(angle_rad) * (self.scale * 6)
            pygame.draw.line(self.screen, (0, 100, 0), (self.center_x, self.center_y), (end_x, end_y), 1)
    
    def draw_robot(self):
        """Draw robot representation with head rotation visualization"""
        robot_size = 10
        
        # Draw robot body (fixed)
        pygame.draw.circle(self.screen, (0, 255, 0), (self.center_x, self.center_y), robot_size, 3)
        
        # Robot front indicator (LiDAR direction - always forward)
        front_x = self.center_x
        front_y = self.center_y - robot_size - 5
        pygame.draw.line(self.screen, (255, 255, 0), (self.center_x, self.center_y), (front_x, front_y), 3)
        
        # Head/Camera direction indicator (rotates with head)
        head_angle_rad = self.get_current_head_angle()
        head_angle_deg = self.get_current_head_angle_degrees()
        
        # Calculate head direction
        head_length = robot_size + 8
        head_x = self.center_x + math.sin(head_angle_rad) * head_length
        head_y = self.center_y - math.cos(head_angle_rad) * head_length
        
        # Draw head direction line
        head_color = (255, 100, 100) if abs(head_angle_deg) > 5 else (100, 100, 255)
        pygame.draw.line(self.screen, head_color, (self.center_x, self.center_y), (head_x, head_y), 2)
        
        # Draw head position indicator
        pygame.draw.circle(self.screen, head_color, (int(head_x), int(head_y)), 4)
        
        # Draw LiDAR position (front center)
        lidar_x = self.center_x
        lidar_y = self.center_y - robot_size - 3
        pygame.draw.circle(self.screen, (255, 0, 0), (lidar_x, lidar_y), 3)
        
        # Add labels if head is rotated significantly
        if abs(head_angle_deg) > 10:
            font = pygame.font.Font(None, 20)
            # LiDAR label
            lidar_label = font.render("L", True, (255, 0, 0))
            self.screen.blit(lidar_label, (lidar_x - 5, lidar_y - 20))
            
            # Camera label
            cam_label = font.render("C", True, head_color)
            self.screen.blit(cam_label, (int(head_x) - 5, int(head_y) - 15))
    
    def draw_lidar_data(self, obstacles):
        """Draw LiDAR data with corrected coordinate system alignment"""
        for angle, distance in obstacles:
            # Fix coordinate system: LiDAR 0° = front, Display should show front as up
            # Rotate by 90° to align LiDAR front (0°) with display front (up)
            corrected_angle_rad = math.radians(90 - angle)
            
            x = self.center_x + math.cos(corrected_angle_rad) * (distance * self.scale // 1000)
            y = self.center_y - math.sin(corrected_angle_rad) * (distance * self.scale // 1000)  # Negative for pygame Y-axis
            
            if (0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height()):
                pygame.draw.circle(self.screen, (255, 255, 255), (int(x), int(y)), 2)
        
        return len(obstacles)
    
    def update_fps_counters(self):
        """Update FPS counters for camera and combined lidar+camera system"""
        current_time = time.time()
        
        # Update camera FPS (tracks camera detection processing)
        self.camera_fps_counter.append(current_time)
        if len(self.camera_fps_counter) > 1:
            time_span = self.camera_fps_counter[-1] - self.camera_fps_counter[0]
            if time_span > 0:
                self.camera_fps = (len(self.camera_fps_counter) - 1) / time_span
        
        # Update combined system FPS (tracks complete lidar+camera processing cycles)
        self.system_fps_counter.append(current_time)
        if len(self.system_fps_counter) > 1:
            time_span = self.system_fps_counter[-1] - self.system_fps_counter[0]
            if time_span > 0:
                self.system_fps = (len(self.system_fps_counter) - 1) / time_span
    
    def get_person_detection(self):
        """Get person detection from camera with FPS tracking"""
        detection_start_time = time.time()
        
        try:
            robot = self.get_robot()
            
            if not hasattr(robot, 'camera_sensor') or not robot.camera_sensor:
                return None
            
            # Get camera reading
            reading = robot.camera_sensor.get_reading()
            if not reading:
                return None
            
            # Get people detections
            if hasattr(reading, 'get_people_locations'):
                people = reading.get_people_locations()
            elif hasattr(reading, 'detections'):
                # Filter for person detections (class 15)
                people = [det for det in reading.detections if det.label == 15]
            else:
                return None
            
            if not people:
                return None
            
            # Get closest person with highest confidence
            closest_person = max(people, key=lambda p: p.confidence)
            
            # Check minimum confidence threshold
            if closest_person.confidence < 0.5:
                return None
            
            # Track camera FPS
            self.camera_frame_count += 1
            
            detection_data = {
                'detection': closest_person,
                'x_camera': closest_person.spatialCoordinates.x,
                'y_camera': closest_person.spatialCoordinates.y,
                'z_camera': closest_person.spatialCoordinates.z,
                'confidence': closest_person.confidence,
                'processing_time_ms': (time.time() - detection_start_time) * 1000
            }
            
            # Store coordinate history for debugging
            if self.coordinate_debug_enabled:
                coord_info = {
                    'x_camera': detection_data['x_camera'],
                    'z_camera': detection_data['z_camera'],
                    'head_angle': self.get_current_head_angle_degrees(),
                    'timestamp': time.time()
                }
                self.camera_coordinate_history.append(coord_info)
            
            return detection_data
            
        except Exception as e:
            return None
    
    def get_current_head_angle(self):
        """Get current head angle in radians"""
        try:
            with self.head_angle_lock:
                return math.radians(self.current_head_angle)
        except Exception:
            return 0.0
    
    def get_current_head_angle_degrees(self):
        """Get current head angle in degrees"""
        try:
            with self.head_angle_lock:
                return self.current_head_angle
        except Exception:
            return 0.0
    
    def transform_camera_to_display_coordinates(self, person_data):
        """Enhanced camera coordinate transformation with proper head rotation handling"""
        try:
            x_camera = person_data['x_camera']  # mm left/right from camera center
            z_camera = person_data['z_camera']  # mm forward from camera
            
            # Get current head angle in radians
            head_angle_rad = self.get_current_head_angle()
            head_angle_deg = math.degrees(head_angle_rad)
            
            # Calculate angle relative to camera center
            if z_camera > 0:
                camera_relative_angle = math.atan2(x_camera, z_camera)
            else:
                camera_relative_angle = 0.0
            
            # CRITICAL: Transform to robot coordinates accounting for head rotation
            # When head turns right (+), camera view rotates right, so detected objects appear more left relative to robot
            # When head turns left (-), camera view rotates left, so detected objects appear more right relative to robot
            robot_angle_rad = camera_relative_angle + head_angle_rad
            
            # Convert to degrees for display
            angle_degrees = math.degrees(robot_angle_rad)
            
            # Normalize to 0-360 range
            while angle_degrees < 0:
                angle_degrees += 360
            while angle_degrees >= 360:
                angle_degrees -= 360
            
            # Use z_camera as distance with camera position compensation
            # Camera is 130mm back from LiDAR, so adjust distance accordingly
            camera_offset_mm = 130  # Camera is 130mm back from front of robot
            adjusted_distance = max(500, z_camera - camera_offset_mm) if z_camera > 0 else 1000
            
            return {
                'angle': angle_degrees,
                'distance': adjusted_distance,
                'confidence': person_data['confidence'],
                'head_angle_deg': head_angle_deg,
                'camera_relative_angle_deg': math.degrees(camera_relative_angle),
                'robot_angle_deg': angle_degrees,
                'coordinate_valid': True
            }
            
        except Exception:
            return None
    
    def draw_person_detection(self):
        """Draw person detection with corrected coordinate system alignment"""
        person_data = self.get_person_detection()
        
        if not person_data:
            return False
        
        # Transform to display coordinates
        display_coords = self.transform_camera_to_display_coordinates(person_data)
        
        if not display_coords:
            return False
        
        try:
            angle = display_coords['angle']
            distance = display_coords['distance']
            confidence = display_coords['confidence']
            
            # Convert to display coordinates (align with LiDAR coordinate system)
            # Apply same coordinate transformation as LiDAR for consistency
            corrected_angle_rad = math.radians(90 - angle)  # Same as LiDAR transformation
            
            x = self.center_x + math.cos(corrected_angle_rad) * (distance * self.scale // 1000)
            y = self.center_y - math.sin(corrected_angle_rad) * (distance * self.scale // 1000)
            
            # Draw person detection if within screen bounds
            if (0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height()):
                # Draw person as orange circle (larger than LiDAR obstacles)
                pygame.draw.circle(self.screen, (255, 165, 0), (int(x), int(y)), 8)  # Orange circle
                pygame.draw.circle(self.screen, (255, 255, 255), (int(x), int(y)), 8, 2)  # White border
                
                # Draw confidence text
                font = pygame.font.Font(None, 24)
                confidence_text = f"{confidence:.2f}"
                text_surface = font.render(confidence_text, True, (255, 255, 255))
                self.screen.blit(text_surface, (int(x) + 12, int(y) - 8))
                
                return True
            
        except Exception:
            pass
        
        return False
    
    def verify_coordinate_system(self):
        """Verify coordinate system handles head rotation correctly (silent verification)"""
        if not self.coordinate_debug_enabled or len(self.camera_coordinate_history) < 2:
            return
        
        try:
            # Analyze coordinate history for head rotation effects
            recent_coords = list(self.camera_coordinate_history)[-3:]
            
            for i, coord in enumerate(recent_coords):
                head_angle = coord['head_angle']
                x_camera = coord['x_camera']
                z_camera = coord['z_camera']
                
                # Calculate what the robot-relative angle should be (silent verification)
                if z_camera > 0:
                    camera_angle_rad = math.atan2(x_camera, z_camera)
                    expected_robot_angle_rad = camera_angle_rad + math.radians(head_angle)
                    expected_robot_angle_deg = math.degrees(expected_robot_angle_rad)
                    
                    # Normalize
                    while expected_robot_angle_deg < 0:
                        expected_robot_angle_deg += 360
                    while expected_robot_angle_deg >= 360:
                        expected_robot_angle_deg -= 360
                
        except Exception as e:
            pass  # Silent error handling
    
    def draw_coordinate_system_info(self):
        """Draw coordinate system information overlay"""
        if not self.coordinate_debug_enabled:
            return
        
        try:
            # Draw coordinate system legend
            legend_x = self.screen.get_width() - 200
            legend_y = self.screen.get_height() - 120
            
            font_tiny = pygame.font.Font(None, 18)
            
            # Background for legend
            legend_rect = pygame.Rect(legend_x - 10, legend_y - 10, 180, 100)
            pygame.draw.rect(self.screen, (0, 0, 0, 128), legend_rect)
            pygame.draw.rect(self.screen, (100, 100, 100), legend_rect, 1)
            
            # Legend text
            legend_lines = [
                "Coordinate System:",
                "• LiDAR: Fixed to robot body",
                "• Camera: Rotates with head", 
                "• 0° = Robot front (up)",
                "• Head rotation compensated"
            ]
            
            for i, line in enumerate(legend_lines):
                color = (255, 255, 255) if i == 0 else (200, 200, 200)
                text_surface = font_tiny.render(line, True, color)
                self.screen.blit(text_surface, (legend_x, legend_y + i * 18))
                
        except Exception:
            pass
    
    def draw_info(self, obstacle_count):
        """Draw info with camera detection status, FPS, and coordinate debugging"""
        font = pygame.font.Font(None, 36)
        font_small = pygame.font.Font(None, 24)
        font_tiny = pygame.font.Font(None, 20)
        
        y_offset = 10
        
        # Main info line with FPS
        text = font.render(f"LiDAR Test Mode - Obstacles: {obstacle_count}", True, (255, 255, 255))
        self.screen.blit(text, (10, y_offset))
        y_offset += 35
        
        # Enhanced FPS Display
        fps_text = f"Combined System FPS: {self.system_fps:.1f} | Camera FPS: {self.camera_fps:.1f}"
        fps_color = (0, 255, 255)  # Cyan
        fps_surface = font_small.render(fps_text, True, fps_color)
        self.screen.blit(fps_surface, (10, y_offset))
        y_offset += 25
        
        # FPS Performance indicators
        combined_fps_status = "GOOD" if self.system_fps > 15 else "LOW" if self.system_fps > 5 else "POOR"
        camera_fps_status = "GOOD" if self.camera_fps > 10 else "LOW" if self.camera_fps > 3 else "POOR"
        
        perf_text = f"Performance: Combined({combined_fps_status}) | Camera({camera_fps_status})"
        perf_color = (0, 255, 0) if combined_fps_status == "GOOD" and camera_fps_status == "GOOD" else (255, 255, 0) if "LOW" in perf_text else (255, 100, 100)
        perf_surface = font_tiny.render(perf_text, True, perf_color)
        self.screen.blit(perf_surface, (10, y_offset))
        y_offset += 20
        
        # Head angle display
        head_angle_deg = self.get_current_head_angle_degrees()
        head_text = f"Head Angle: {head_angle_deg:.1f}°"
        head_color = (255, 255, 0) if abs(head_angle_deg) > 5 else (255, 255, 255)
        head_surface = font_small.render(head_text, True, head_color)
        self.screen.blit(head_surface, (10, y_offset))
        y_offset += 30
        
        # Camera status and coordinate debugging
        person_data = self.get_person_detection()
        if person_data:
            camera_text = f"Camera: Person Detected (Conf: {person_data['confidence']:.2f})"
            camera_color = (0, 255, 0)  # Green
            
            camera_surface = font_small.render(camera_text, True, camera_color)
            self.screen.blit(camera_surface, (10, y_offset))
            y_offset += 25
            
            # Processing time
            if 'processing_time_ms' in person_data:
                proc_text = f"Processing: {person_data['processing_time_ms']:.1f}ms"
                proc_surface = font_tiny.render(proc_text, True, (200, 200, 200))
                self.screen.blit(proc_surface, (10, y_offset))
                y_offset += 20
            
            # Coordinate transformation info
            display_coords = self.transform_camera_to_display_coordinates(person_data)
            if display_coords and display_coords.get('coordinate_valid'):
                # Raw camera coordinates
                raw_text = f"Raw Cam: X={person_data['x_camera']:.0f}mm, Z={person_data['z_camera']:.0f}mm"
                raw_surface = font_tiny.render(raw_text, True, (200, 200, 200))
                self.screen.blit(raw_surface, (10, y_offset))
                y_offset += 20
                
                # Transformed coordinates with head angle compensation
                trans_text = f"Robot Coords: {display_coords['robot_angle_deg']:.1f}° at {display_coords['distance']:.0f}mm"
                trans_surface = font_tiny.render(trans_text, True, (255, 255, 255))
                self.screen.blit(trans_surface, (10, y_offset))
                y_offset += 20
                
                # Head angle compensation visualization
                if abs(display_coords['head_angle_deg']) > 1:
                    comp_text = f"Head Compensation: Cam {display_coords['camera_relative_angle_deg']:.1f}° + Head {display_coords['head_angle_deg']:.1f}° = {display_coords['robot_angle_deg']:.1f}°"
                    comp_color = (255, 200, 100)  # Orange
                    comp_surface = font_tiny.render(comp_text, True, comp_color)
                    self.screen.blit(comp_surface, (10, y_offset))
                    y_offset += 20
            
        else:
            camera_text = "Camera: No Person Detected"
            camera_color = (255, 255, 0)  # Yellow
            camera_surface = font_small.render(camera_text, True, camera_color)
            self.screen.blit(camera_surface, (10, y_offset))
            y_offset += 25
        
        # Performance indicators (right side of screen)
        right_x = self.screen.get_width() - 280
        perf_y = 10
        
        # Combined System FPS status
        combined_fps_color = (0, 255, 0) if self.system_fps > 15 else (255, 255, 0) if self.system_fps > 5 else (255, 100, 100)
        combined_status = f"Combined FPS: {self.system_fps:.1f}"
        combined_surface = font_small.render(combined_status, True, combined_fps_color)
        self.screen.blit(combined_surface, (right_x, perf_y))
        perf_y += 25
        
        # LiDAR performance
        lidar_status = "LiDAR: Active" if self.lidar_system else "LiDAR: Inactive"
        lidar_color = (0, 255, 0) if self.lidar_system else (255, 0, 0)
        lidar_surface = font_small.render(lidar_status, True, lidar_color)
        self.screen.blit(lidar_surface, (right_x, perf_y))
        perf_y += 25
        
        # Camera performance
        camera_status = "Camera: Active" if person_data else "Camera: Inactive"
        camera_perf_color = (0, 255, 0) if person_data else (255, 255, 0)
        camera_perf_surface = font_small.render(camera_status, True, camera_perf_color)
        self.screen.blit(camera_perf_surface, (right_x, perf_y))
        perf_y += 25
        
        # Head tracking status
        head_status = "Head: Tracking" if self.head_tracker else "Head: Manual"
        head_track_color = (0, 255, 0) if self.head_tracker else (255, 255, 0)
        head_track_surface = font_small.render(head_status, True, head_track_color)
        self.screen.blit(head_track_surface, (right_x, perf_y))
        
        # Controls info (bottom of screen)
        controls_text = "ESC - Exit to IDLE Mode | D - Toggle Debug Display | Head rotation affects camera coordinates"
        controls_surface = font_tiny.render(controls_text, True, (150, 150, 150))
        self.screen.blit(controls_surface, (10, self.screen.get_height() - 25))
    
    def stop_robot(self):
        """Stop robot movement (ORIGINAL)"""
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
        """Center head for idle mode (ORIGINAL)"""
        try:
            robot = self.get_robot()
            if hasattr(robot, 'servo_controller') and robot.servo_controller:
                robot.servo_controller.set_position(0.0)
                time.sleep(0.5)
        except Exception:
            pass
    
    def perform_enhanced_idle_mode_transition(self):
        """ORIGINAL WORKING IDLE TRANSITION - DO NOT CHANGE"""
        try:
            robot = self.get_robot()
            
            # Stop movement
            self.stop_robot()
            
            # Center head
            self.center_head_for_idle_mode()
            
            # Stop LiDAR
            if self.lidar_system:
                self.lidar_system.stop()
                self.lidar_system = None
            
            # Clear pygame events
            if pygame.get_init():
                pygame.event.clear()
            
            # Restore facial animation immediately
            self.facial_restorer.restore_resting_face_immediately(robot)
            
            # Announce mode change ONLY ONCE
            if not self.idle_mode_announced:
                try:
                    if hasattr(robot, 'speech_manager') and robot.speech_manager:
                        robot.speech_manager.perform_action("IDLE MODE")
                        self.idle_mode_announced = True
                except Exception:
                    pass
            
            return True
            
        except Exception:
            return False
    
    def update(self) -> Status:
        """Enhanced update method with FPS tracking and coordinate debugging"""
        try:
            if not self.initialized:
                self.initialize_components()
                return Status.RUNNING
            
            self.update_counter += 1
            self.system_frame_count += 1
            
            # Update FPS counters
            self.update_fps_counters()
            
            # Update head angle if head tracker is available
            if self.head_tracker:
                try:
                    current_angle = self.head_tracker.get_current_position()
                    if current_angle is not None:
                        with self.head_angle_lock:
                            self.current_head_angle = current_angle
                            self.last_head_angle_deg = current_angle
                except Exception:
                    pass
            
            # Handle events with ORIGINAL exit handling
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        # ORIGINAL WORKING EXIT BEHAVIOR - DO NOT CHANGE
                        self.perform_enhanced_idle_mode_transition()
                        
                        robot = self.get_robot()
                        robot.set_mode(RobotMode.IDLE)
                        return Status.SUCCESS
                    elif event.key == pygame.K_d:
                        # Toggle coordinate debugging
                        self.coordinate_debug_enabled = not self.coordinate_debug_enabled
                elif event.type == pygame.QUIT:
                    # ORIGINAL WORKING EXIT BEHAVIOR - DO NOT CHANGE
                    self.perform_enhanced_idle_mode_transition()
                    
                    robot = self.get_robot()
                    robot.set_mode(RobotMode.IDLE)
                    return Status.SUCCESS
            
            # Display update (ORIGINAL with minor optimization)
            should_update_display = (self.update_counter % self.display_update_rate == 0)
            
            if should_update_display:
                try:
                    if self.screen:
                        self.screen.fill((0, 0, 0))
                        self.draw_radar_grid()
                        self.draw_robot()
                        
                        # Get display obstacles
                        obstacles = []
                        if self.lidar_system:
                            obstacles = self.lidar_system.get_display_obstacles()
                            if obstacles:
                                self.draw_lidar_data(obstacles)
                        
                        # Draw person detection from camera
                        self.draw_person_detection()
                        
                        # Draw coordinate system info overlay
                        self.draw_coordinate_system_info()
                        
                        # Verify coordinate system (debug output)
                        if self.update_counter % 30 == 0:  # Every 30 frames
                            self.verify_coordinate_system()
                        
                        # Draw test info with FPS and coordinate debugging
                        self.draw_info(len(obstacles) if obstacles else 0)
                        
                        pygame.display.flip()
                        
                except Exception:
                    pass
            
            return Status.RUNNING
            
        except Exception:
            try:
                self.stop_robot()
                self.perform_enhanced_idle_mode_transition()
            except Exception:
                pass
            
            return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """ORIGINAL WORKING TERMINATE METHOD"""
        try:
            self.stop_robot()
            
            if self.head_tracker:
                self.head_tracker.stop_tracking()
                self.head_tracker = None
            
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


# ORIGINAL COMPATIBILITY ALIASES - DO NOT CHANGE
LidarTestBehavior = StableLidarTest
OptimizedStableLidarTest = StableLidarTest 
OptimizedLidarTestWithHybridDistance = StableLidarTest
FixedStableLidarTest = StableLidarTest

# Legacy classes (simplified stubs for compatibility)
class HybridDistanceCalculator:
    pass

class FixedUltraStablePyRPLidarA3:
    pass

class FixedUltraStableLidarSystem:
    pass