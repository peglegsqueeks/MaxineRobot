import pygame
import math
import time
import threading
import queue
from typing import List, Optional, Tuple
from pyrplidar import PyRPlidar
import depthai as dai
import py_trees
from py_trees.common import Status

from src.behaviors.MaxineBehavior import MaxineBehavior
from src.path_finding.Position import Position
from src.path_finding.new_a_star import a_star
from src.types.RobotModes import RobotMode
from src.types.MovementDirection import MovementDirection
from src.types.KeyboardKey import KeyboardKey
from src.action_managers.VelocityManager import VelocityConfig
from src.behaviors.lidarchase.ObstacleMapper import ObstacleMapper
from src.behaviors.lidarchase.HeadTracker import HeadTracker


class UltraStablePyRPLidarA3:
    """Ultra-stable LiDAR class optimized for moving robot operations"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000, timeout=2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.lidar = None
        self.is_connected = False
        self.scan_generator = None
        self.scan_iterator = None
        
        # Ultra-Stable Parameters for Moving Robot
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
                    pass
            
            elif mode == 'force_only':
                self.scan_generator = self.lidar.force_scan()
                self.scan_iterator = self.scan_generator()
                self.current_mode = 'force'
                return True
            
            return False
            
        except Exception:
            return False

    def get_scan_data_generator(self, shutdown_flag):
        """Ultra-stable generator optimized for moving robot"""
        if not self.is_connected or not self.lidar or not self.scan_iterator:
            return
            
        consecutive_failures = 0
        max_failures = 100
        scan_buffer = []
        last_angle = None
        min_scan_points = 150
        restart_count = 0
        max_restarts = 2
        
        while consecutive_failures < max_failures:
            if shutdown_flag[0]:
                break
                
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
                            
                            if (last_angle is not None and 
                                angle < last_angle and 
                                len(scan_buffer) > min_scan_points):
                                yield scan_buffer.copy()
                                scan_buffer.clear()
                            
                            last_angle = angle
                        
                    except Exception:
                        continue
                else:
                    consecutive_failures += 1
                    time.sleep(0.02)
                    
            except StopIteration:
                restart_count += 1
                if restart_count >= max_restarts:
                    break
                
                time.sleep(2)
                
                if self.start_scanning('ultra_stable'):
                    consecutive_failures = 0
                    scan_buffer.clear()
                    last_angle = None
                    continue
                elif self.start_scanning('force_only'):
                    consecutive_failures = 0
                    scan_buffer.clear()
                    last_angle = None
                    continue
                else:
                    break
                    
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
                                    while not self.scan_queue.empty():
                                        self.scan_queue.get_nowait()
                                    self.scan_queue.put_nowait(stable_obstacles)
                                except (queue.Full, queue.Empty):
                                    pass
                        
                    except Exception:
                        if not self.shutdown_flag[0]:
                            continue
                        
            except Exception:
                pass
                
        except Exception:
            pass
        finally:
            if self.lidar:
                self.lidar.disconnect()
    
    def update_obstacle_confidence_stable(self, scan_data):
        """Conservative confidence update optimized for moving robot"""
        self.scan_cycle_count += 1
        seen_obstacles = set()
        
        for quality, angle, distance in scan_data:
            if distance > 200 and quality > 5 and distance < 5000:
                normalized_angle = ((angle + 180) % 360) - 180
                angle_bin = round(normalized_angle / self.angle_resolution) * self.angle_resolution
                distance_bin = round(distance / self.distance_resolution) * self.distance_resolution
                
                key = (angle_bin, distance_bin)
                seen_obstacles.add(key)
                
                current_confidence = self.obstacle_confidence.get(key, 0)
                self.obstacle_confidence[key] = min(
                    self.max_confidence, 
                    current_confidence + self.confidence_increment
                )
        
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
        """Stable obstacle retrieval for moving robot"""
        return [(angle, distance) for (angle, distance), confidence 
                in self.obstacle_confidence.items() 
                if confidence >= self.confidence_threshold]
    
    def get_latest_obstacles(self):
        """Get latest obstacles for pathfinding"""
        try:
            return self.scan_queue.get_nowait()
        except queue.Empty:
            with self.data_lock:
                return self.latest_obstacles.copy()
    
    def get_display_obstacles(self):
        """Get obstacles for display"""
        with self.data_lock:
            return self.latest_obstacles.copy()
    
    def start(self):
        """Start the ultra-stable LiDAR system"""
        self.running = True
        self.shutdown_flag[0] = False
        
        data_thread = threading.Thread(target=self.data_acquisition_thread, daemon=True)
        data_thread.start()
        self.threads.append(data_thread)
        
        return True
    
    def stop(self):
        """Stop the system gracefully"""
        self.running = False
        self.shutdown_flag[0] = True
        
        for thread in self.threads:
            thread.join(timeout=2.0)


class CoordinateTransformer:
    """Coordinate transformer for proper sensor fusion with correct robot dimensions"""
    
    def __init__(self):
        # Robot dimensions as specified: 660mm Long x 550mm Wide
        self.robot_length = 660  # mm
        self.robot_width = 550   # mm
        
        # Sensor positions relative to robot center
        # LiDAR: center front of robot body
        self.lidar_x_offset = self.robot_length / 2  # 330mm from center
        self.lidar_y_offset = 0
        
        # Camera: 130mm back from lidar, 510mm above lidar
        self.camera_x_offset = (self.robot_length / 2) - 130  # 200mm from center
        self.camera_y_offset = 0
        self.camera_z_offset = 510  # Above base
        
        # Head rotation limits: ±90 degrees as specified
        self.max_head_angle = math.radians(90)  # ±90 degrees
    
    def camera_to_robot_coordinates(self, x_camera: float, z_camera: float, 
                                   head_angle_rad: float) -> Position:
        """Transform camera coordinates to robot coordinates accounting for head rotation"""
        try:
            # Clamp head angle to physical limits
            head_angle_rad = max(-self.max_head_angle, min(self.max_head_angle, head_angle_rad))
            
            # Camera's view angle relative to camera
            camera_angle = math.atan2(x_camera, z_camera) if z_camera > 0 else 0.0
            camera_distance = math.sqrt(x_camera**2 + z_camera**2)
            
            # Transform to robot coordinates accounting for head angle and camera position
            robot_angle = camera_angle + head_angle_rad
            
            # Account for camera position offset (130mm back from lidar)
            adjusted_distance = max(100, camera_distance - self.camera_x_offset)
            
            return Position(angle=robot_angle, distance=adjusted_distance)
            
        except Exception:
            return Position(angle=0.0, distance=1000.0)
    
    def lidar_to_robot_coordinates(self, lidar_position: Position) -> Position:
        """Transform LiDAR coordinates to robot coordinates"""
        try:
            # LiDAR is at front-center, 330mm forward from robot center
            x_lidar, y_lidar = lidar_position.to_cartesian()
            
            # Account for LiDAR position offset
            x_robot = x_lidar 
            y_robot = y_lidar - self.lidar_x_offset
            
            # Convert back to polar coordinates
            distance = math.sqrt(x_robot**2 + y_robot**2)
            angle = math.atan2(x_robot, y_robot) if y_robot != 0 else 0.0
            
            return Position(angle=angle, distance=distance)
            
        except Exception:
            return lidar_position


class SmartWaypointFilter:
    """Smart waypoint filtering for smooth navigation"""
    
    def __init__(self, min_angle_change_deg=15, min_distance_m=2.0):
        self.min_angle_change = math.radians(min_angle_change_deg)
        self.min_distance = min_distance_m * 1000
    
    def filter_waypoints(self, path):
        if not path or len(path) <= 2:
            return path
        
        filtered_waypoints = [path[0]]
        last_waypoint = path[0]
        
        for i in range(1, len(path) - 1):
            current_point = path[i]
            
            distance_from_last = self.calculate_distance(last_waypoint, current_point)
            angle_change = self.calculate_angle_change(last_waypoint, current_point, path[i + 1] if i + 1 < len(path) else current_point)
            
            if angle_change > self.min_angle_change or distance_from_last > self.min_distance:
                filtered_waypoints.append(current_point)
                last_waypoint = current_point
        
        if len(path) > 1:
            filtered_waypoints.append(path[-1])
        
        return filtered_waypoints
    
    def calculate_distance(self, pos1, pos2):
        try:
            x1, y1 = pos1.to_cartesian()
            x2, y2 = pos2.to_cartesian()
            return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        except:
            return 0
    
    def calculate_angle_change(self, prev_point, current_point, next_point):
        try:
            x1, y1 = prev_point.to_cartesian()
            x2, y2 = current_point.to_cartesian()
            x3, y3 = next_point.to_cartesian()
            
            v1 = (x2 - x1, y2 - y1)
            v2 = (x3 - x2, y3 - y2)
            
            dot_product = v1[0] * v2[0] + v1[1] * v2[1]
            mag1 = math.sqrt(v1[0]**2 + v1[1]**2)
            mag2 = math.sqrt(v2[0]**2 + v2[1]**2)
            
            if mag1 == 0 or mag2 == 0:
                return 0
            
            cos_angle = dot_product / (mag1 * mag2)
            cos_angle = max(-1, min(1, cos_angle))
            
            angle_change = math.acos(cos_angle)
            return angle_change
            
        except:
            return 0


class EnhancedFacialAnimationRestorer:
    """Enhanced facial animation restoration for proper IDLE mode transition"""
    
    def __init__(self):
        self.restoration_attempts = 0
        self.max_restoration_attempts = 5
        
    def restore_facial_animation_immediately(self, robot):
        """Immediately restore facial animation with resting face display"""
        restoration_success = False
        
        for attempt in range(self.max_restoration_attempts):
            try:
                # Method 1: Direct facial animation manager restoration
                if hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                    # Bring facial animation to front
                    robot.facial_animation_manager.bring_to_front()
                    
                    # Force immediate display of resting face
                    if hasattr(robot.facial_animation_manager, 'resting_face_img'):
                        robot.facial_animation_manager.display.blit(
                            robot.facial_animation_manager.resting_face_img, (0, 0)
                        )
                        pygame.display.flip()
                        restoration_success = True
                        break
                
                # Method 2: Reinitialize facial animation if needed
                if not restoration_success and hasattr(robot, 'facial_animation_manager'):
                    try:
                        robot.facial_animation_manager.open_window()
                        robot.facial_animation_manager.display.blit(
                            robot.facial_animation_manager.resting_face_img, (0, 0)
                        )
                        pygame.display.flip()
                        restoration_success = True
                        break
                    except Exception:
                        continue
                
                # Short delay between attempts
                time.sleep(0.2)
                
            except Exception:
                continue
        
        return restoration_success


class UltraStableLidarChase(MaxineBehavior):
    """Ultra-stable LiDAR Chase with enhanced facial animation restoration for IDLE mode"""
    
    def __init__(self):
        super().__init__("Ultra Stable LiDAR Chase with Enhanced Facial Restoration")
        
        # Blackboard setup
        self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.READ)
        self.blackboard.register_key("HEAD_CENTER_POSITION", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("HEAD_CALIBRATED_CENTER", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.WRITE)
        
        # Core components
        self.lidar_system = None
        self.screen = None
        self.head_tracker = None
        self.obstacle_mapper = None
        self.coordinate_transformer = None
        self.facial_restorer = EnhancedFacialAnimationRestorer()
        self.initialized = False
        
        # Speech tracking to prevent multiple announcements
        self.idle_mode_announced = False
        
        # Head calibration - automatic for immediate operation
        self.calibration_mode = False
        self.calibration_complete = True
        self.calibrated_center_position = 0.0
        self.lidar_data_received = True
        self.calibration_instructions_spoken = True
        
        # Head tracking state
        self.current_head_angle = 0.0
        self.head_angle_lock = threading.Lock()
        
        # Navigation components
        self.waypoint_filter = SmartWaypointFilter(min_angle_change_deg=15, min_distance_m=2.0)
        
        # Pathfinding state
        self.current_path = []
        self.filtered_waypoints = []
        self.last_pathfinding_time = 0
        self.pathfinding_interval = 1.0
        
        # Movement state
        self.current_waypoint_index = 1
        self.waypoint_reached_threshold = 600
        self.movement_active = False
        self.pathfinding_in_progress = False
        
        # Target tracking
        self.stable_target_position = None
        self.target_stability_threshold = 1.0
        self.last_stable_target_update = 0
        
        # Display parameters
        self.center_x = 0
        self.center_y = 0
        self.scale = 0
        self.update_counter = 0
        self.display_update_rate = 3
        
        # Initialize pygame if needed
        if not pygame.get_init():
            pygame.init()
        
        pygame.font.init()
    
    def initialize_components(self):
        """Initialize components with ultra-stable LiDAR and correct robot dimensions"""
        if self.initialized:
            return True
        
        try:
            # Setup display
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
            pygame.display.set_caption("MAXINE ULTRA STABLE LIDAR CHASE")
            
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            self.scale = min(display_info.current_w, display_info.current_h) // 7
            
            self.draw_clean_interface()
            pygame.display.flip()
            
            self.blackboard.set("HEAD_CENTER_POSITION", 0.0)
            
            # Initialize coordinate transformer with correct robot dimensions
            self.coordinate_transformer = CoordinateTransformer()
            
            # Initialize head tracker
            robot = self.get_robot()
            if (hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager) or \
               (hasattr(robot, 'servo_controller') and robot.servo_controller):
                try:
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
            
            # Initialize navigation components
            self.stable_target_position = None
            self.last_stable_target_update = 0
            
            self.obstacle_mapper = ObstacleMapper(
                grid_size_mm=100,
                map_radius_mm=6000
            )
            
            self.initialized = True
            self.start_stable_lidar()
            
            return True
            
        except Exception:
            self.initialized = False
            return False
    
    def start_stable_lidar(self):
        """Start ultra-stable LiDAR system and register in blackboard"""
        try:
            if not self.lidar_system:
                self.lidar_system = UltraStableLidarSystem()
                success = self.lidar_system.start()
                if success:
                    # Register in blackboard for proper cleanup
                    self.blackboard.set("LIDAR_SYSTEM", self.lidar_system)
                    time.sleep(3)
        except Exception:
            pass
    
    def get_current_head_angle(self):
        """Get current head angle in radians relative to calibrated center"""
        try:
            if self.head_tracker:
                status = self.head_tracker.get_status()
                head_position = status.get('current_position', 0.0)
                head_angle_rad = head_position * math.radians(90)
                
                with self.head_angle_lock:
                    self.current_head_angle = head_angle_rad
                
                return head_angle_rad
            else:
                return 0.0
        except Exception:
            return 0.0

    def update_head_tracking(self, person_angle_rad):
        """Update head tracking to follow person"""
        try:
            if self.head_tracker:
                self.head_tracker.set_person_tracking(person_angle_rad)
        except Exception:
            pass
    
    def center_head_for_idle_mode(self):
        """Return head to center position for proper IDLE mode transition"""
        try:
            robot = self.get_robot()
            
            # Multiple centering attempts for reliability
            centering_methods = [
                lambda: robot.servo_controller.center() if hasattr(robot, 'servo_controller') and robot.servo_controller else None,
                lambda: robot.head_velocity_manager.center_head() if hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager else None,
                lambda: robot.head_manager.center_head() if hasattr(robot, 'head_manager') and robot.head_manager else None,
                lambda: robot.center_head() if hasattr(robot, 'center_head') else None
            ]
            
            for method in centering_methods:
                try:
                    result = method()
                    if result is not None:
                        time.sleep(0.5)  # Allow servo time to move
                        break
                except Exception:
                    continue
                    
        except Exception:
            pass
    
    def draw_clean_interface(self):
        """Draw clean radar interface"""
        self.screen.fill((0, 0, 0))
        self.draw_radar_grid()
        self.draw_robot()
    
    def draw_radar_grid(self):
        """Draw radar-style grid with proper range circles"""
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
        """Draw robot representation with correct dimensions"""
        pygame.draw.circle(self.screen, (0, 255, 0), (self.center_x, self.center_y), 15, 3)
        arrow_end_x = self.center_x
        arrow_end_y = self.center_y - 30
        pygame.draw.line(self.screen, (0, 255, 0), (self.center_x, self.center_y), (arrow_end_x, arrow_end_y), 5)
    
    def draw_lidar_data(self, obstacles):
        """Draw LiDAR obstacles"""
        if not obstacles:
            return 0
        
        drawn_count = 0
        for angle, distance in obstacles:
            if 100 < distance < 6000:
                distance_m = distance / 1000.0
                display_angle_rad = math.radians(90 - angle)
                
                x = self.center_x + int(distance_m * self.scale * math.cos(display_angle_rad))
                y = self.center_y - int(distance_m * self.scale * math.sin(display_angle_rad))
                
                if 0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height():
                    pygame.draw.circle(self.screen, (255, 0, 0), (x, y), 4)
                    drawn_count += 1
        
        return drawn_count

    def get_person_position(self):
        """Get person position with coordinate transformation accounting for robot geometry"""
        try:
            robot = self.get_robot()
            camera_reading = robot.camera_sensor.get_reading()
            
            if not camera_reading:
                return None
            
            people = camera_reading.get_people_locations()
            
            if not people:
                return None
            
            closest_person = people[0]
            bbox_data = camera_reading.get_stabilized_position(closest_person)
            
            if not bbox_data:
                return None
            
            camera_angle_offset = bbox_data['camera_angle_offset']
            z_depth = bbox_data['z_depth']
            
            if z_depth <= 0 or z_depth > 12000:
                return None
            
            # Get current head angle for coordinate transformation
            current_head_angle = self.get_current_head_angle()
            
            # Coordinate system alignment accounting for robot geometry
            x_camera = closest_person.spatialCoordinates.x
            z_camera = closest_person.spatialCoordinates.z
            
            # Calculate person's angle relative to the camera center
            camera_relative_angle = math.atan2(x_camera, z_camera) if z_camera > 0 else 0.0
            
            # Transform to robot coordinates accounting for camera position
            robot_absolute_angle = current_head_angle + camera_relative_angle
            
            # Normalize to standard robot coordinate system
            while robot_absolute_angle > math.pi:
                robot_absolute_angle -= 2 * math.pi
            while robot_absolute_angle < -math.pi:
                robot_absolute_angle += 2 * math.pi
            
            current_time = time.time()
            
            # Calculate safe approach distance accounting for robot size
            robot_radius = 350  # Approximate radius from robot dimensions
            safe_approach_distance = z_depth - robot_radius
            target_distance = max(800, safe_approach_distance)
            
            if z_depth < 1500:
                target_distance = max(1000, safe_approach_distance)
            
            # Create position relative to robot body
            new_target_position = Position(angle=robot_absolute_angle, distance=target_distance)
            
            # Update head tracking to follow person
            self.update_head_tracking(robot_absolute_angle)
            
            # Target stability logic
            should_update_target = False
            
            if (self.stable_target_position is None or 
                current_time - self.last_stable_target_update > self.target_stability_threshold):
                
                if self.stable_target_position is None:
                    should_update_target = True
                else:
                    try:
                        stable_x, stable_y = self.stable_target_position.to_cartesian()
                        new_x, new_y = new_target_position.to_cartesian()
                        distance_moved = math.sqrt((new_x - stable_x)**2 + (new_y - stable_y)**2)
                        
                        if distance_moved > 500:
                            should_update_target = True
                    except:
                        should_update_target = True
            
            if should_update_target:
                self.stable_target_position = new_target_position
                self.last_stable_target_update = current_time
            
            # Display position
            display_angle_deg = math.degrees(robot_absolute_angle)
            display_position = self.calculate_display_position(display_angle_deg, target_distance)
            
            person_data = {
                'angle': math.degrees(robot_absolute_angle),
                'distance': target_distance,
                'original_distance': z_depth,
                'z_depth': z_depth,
                'position': self.stable_target_position,
                'display_position': display_position,
                'is_off_screen': target_distance > 6000,
                'confidence': closest_person.confidence,
                'bbox_center': bbox_data,
                'camera_angle_offset': camera_angle_offset,
                'head_angle_deg': math.degrees(current_head_angle),
                'camera_relative_angle_deg': math.degrees(camera_relative_angle),
                'robot_absolute_angle': robot_absolute_angle,
                'target_updated': should_update_target,
                'safe_approach_distance': safe_approach_distance,
                'robot_radius': robot_radius
            }
            
            return person_data
            
        except Exception:
            return None

    def calculate_display_position(self, angle_degrees, distance_mm):
        """Calculate display position for visualization"""
        distance_m = distance_mm * 0.001
        display_angle_rad = math.radians(90 - angle_degrees)
        scaled_distance = distance_m * self.scale
        x = self.center_x + int(scaled_distance * math.cos(display_angle_rad))
        y = self.center_y - int(scaled_distance * math.sin(display_angle_rad))
        return (x, y)

    def draw_person_detection(self, person_data):
        """Draw person detection visualization"""
        if not person_data:
            return
        
        try:
            x, y = person_data['display_position']
            
            if 0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height():
                pygame.draw.circle(self.screen, (255, 255, 255), (x, y), 15)
                pygame.draw.circle(self.screen, (0, 255, 255), (x, y), 10)
        except Exception:
            pass

    def stop_robot_completely(self):
        """Stop robot movement completely"""
        try:
            robot = self.get_robot()
            
            # Try multiple velocity manager variants
            velocity_managers = []
            if hasattr(robot, 'direct_velocity_manager') and robot.direct_velocity_manager:
                velocity_managers.append(robot.direct_velocity_manager)
            if hasattr(robot, 'velocity_manager') and robot.velocity_manager:
                velocity_managers.append(robot.velocity_manager)
            
            # Send stop command to all available managers
            for velocity_manager in velocity_managers:
                try:
                    stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                    velocity_manager.perform_action(stop_config)
                except Exception:
                    continue
            
            self.movement_active = False
            
        except Exception:
            self.movement_active = False

    def perform_enhanced_idle_mode_transition(self):
        """Perform enhanced transition to IDLE mode with immediate facial animation restoration"""
        try:
            robot = self.get_robot()
            
            # Step 1: Stop all robot movement immediately
            self.stop_robot_completely()
            
            # Step 2: Center head for proper IDLE mode positioning
            self.center_head_for_idle_mode()
            
            # Step 3: Stop and cleanup LiDAR system
            if self.lidar_system:
                self.lidar_system.stop()
                self.lidar_system = None
            
            # Step 4: Clear pygame events to prevent interference
            if pygame.get_init():
                pygame.event.clear()
            
            # Step 5: IMMEDIATELY restore facial animation with resting face - AGGRESSIVE APPROACH
            restoration_success = self.facial_restorer.restore_resting_face_immediately(robot)
            
            # Step 6: Force facial animation to front and display resting face
            try:
                if hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                    robot.facial_animation_manager.bring_to_front()
                    time.sleep(0.1)
                    
                    # Force display of resting face
                    if hasattr(robot.facial_animation_manager, 'resting_face_img'):
                        robot.facial_animation_manager.display.blit(
                            robot.facial_animation_manager.resting_face_img, (0, 0)
                        )
                        pygame.display.flip()
                        
                    # Second attempt with window restoration
                    if not restoration_success:
                        robot.facial_animation_manager.open_window()
                        time.sleep(0.2)
                        robot.facial_animation_manager.display.blit(
                            robot.facial_animation_manager.resting_face_img, (0, 0)
                        )
                        pygame.display.flip()
                        restoration_success = True
                        
            except Exception:
                pass
            
            # Step 7: Announce mode transition ONLY ONCE
            if not self.idle_mode_announced:
                try:
                    if hasattr(robot, 'speech_manager') and robot.speech_manager:
                        robot.speech_manager.perform_action("IDLE MODE")
                        self.idle_mode_announced = True
                except Exception:
                    pass
            
            # Step 8: Final verification and forced display
            time.sleep(0.5)
            
            # Final aggressive restoration
            try:
                if hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                    robot.facial_animation_manager.bring_to_front()
                    if hasattr(robot.facial_animation_manager, 'resting_face_img'):
                        robot.facial_animation_manager.display.blit(
                            robot.facial_animation_manager.resting_face_img, (0, 0)
                        )
                        pygame.display.flip()
            except Exception:
                pass
            
            return True
            
        except Exception:
            # Emergency fallback
            try:
                robot = self.get_robot()
                self.stop_robot_completely()
                self.center_head_for_idle_mode()
                if hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                    robot.facial_animation_manager.bring_to_front()
                    if hasattr(robot.facial_animation_manager, 'resting_face_img'):
                        robot.facial_animation_manager.display.blit(
                            robot.facial_animation_manager.resting_face_img, (0, 0)
                        )
                        pygame.display.flip()
            except Exception:
                pass
            return False

    def update(self) -> Status:
        """Main update loop with enhanced facial animation restoration on exit"""
        try:
            if not self.initialized:
                self.initialize_components()
                return Status.RUNNING
            
            self.update_counter += 1
            
            # Handle pygame events with enhanced exit handling
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        # Enhanced IDLE mode transition with immediate facial restoration
                        self.perform_enhanced_idle_mode_transition()
                        
                        robot = self.get_robot()
                        robot.set_mode(RobotMode.IDLE)
                        return Status.SUCCESS
                elif event.type == pygame.QUIT:
                    # Enhanced IDLE mode transition with immediate facial restoration
                    self.perform_enhanced_idle_mode_transition()
                    
                    robot = self.get_robot()
                    robot.set_mode(RobotMode.IDLE)
                    return Status.SUCCESS
            
            # Display and tracking logic
            should_update_display = (self.update_counter % self.display_update_rate == 0)
            
            if should_update_display:
                try:
                    if self.screen:
                        self.screen.fill((0, 0, 0))
                        self.draw_radar_grid()
                        self.draw_robot()
                        
                        # Get and display obstacles
                        obstacles = []
                        if self.lidar_system:
                            obstacles = self.lidar_system.get_display_obstacles()
                            if obstacles:
                                self.draw_lidar_data(obstacles)
                        
                        # Person tracking and navigation
                        person_data = self.get_person_position()
                        
                        if person_data:
                            self.draw_person_detection(person_data)
                            
                            try:
                                robot = self.get_robot()
                                camera_reading = robot.camera_sensor.get_reading()
                                if camera_reading:
                                    people = camera_reading.get_people_locations()
                                    if people:
                                        self.blackboard.set("TARGET_PERSON", people[0])
                            except Exception:
                                pass
                        
                        pygame.display.flip()
                        
                except Exception:
                    pass
            
            return Status.RUNNING
            
        except Exception:
            # Emergency stop and safe transition
            try:
                self.stop_robot_completely()
                self.perform_enhanced_idle_mode_transition()
            except Exception:
                pass
            
            return Status.RUNNING

    def terminate(self, new_status: Status):
        """Enhanced termination with immediate facial animation restoration"""
        try:
            # Step 1: Stop all movement and tracking
            self.stop_robot_completely()
            self.pathfinding_in_progress = False
            self.current_path = []
            self.filtered_waypoints = []
            
            # Step 2: Stop head tracking
            if self.head_tracker:
                self.head_tracker.stop_tracking()
                self.head_tracker = None
            
            # Step 3: Clear blackboard
            try:
                if self.blackboard.exists("LIDAR_SYSTEM"):
                    self.blackboard.unset("LIDAR_SYSTEM")
            except Exception:
                pass
            
            self.initialized = False
            
        except Exception:
            pass
        
        # Call parent termination
        super().terminate(new_status)
        
        # CRITICAL: Final facial animation restoration - DO NOT REMOVE THIS
        try:
            robot = self.get_robot()
            
            # Center head first
            self.center_head_for_idle_mode()
            time.sleep(0.5)
            
            # Aggressive facial animation restoration
            if hasattr(robot, 'facial_animation_manager') and robot.facial_animation_manager:
                # Bring to front aggressively
                robot.facial_animation_manager.bring_to_front()
                time.sleep(0.2)
                
                # Ensure window is open
                if not hasattr(robot.facial_animation_manager, 'display') or robot.facial_animation_manager.display is None:
                    robot.facial_animation_manager.open_window()
                    time.sleep(0.3)
                
                # Display resting face multiple times for reliability
                if hasattr(robot.facial_animation_manager, 'resting_face_img'):
                    for _ in range(3):  # Multiple attempts
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
                
                # Final announcement if not already done
                if not self.idle_mode_announced:
                    try:
                        if hasattr(robot, 'speech_manager') and robot.speech_manager:
                            robot.speech_manager.perform_action("IDLE MODE")
                            self.idle_mode_announced = True
                    except Exception:
                        pass
                        
        except Exception:
            pass


class StableLidarTest(MaxineBehavior):
    """Stable LiDAR Test with enhanced facial animation restoration - simplified version of LidarChase"""
    
    def __init__(self):
        super().__init__("Stable LiDAR Test with Enhanced Facial Restoration")
        
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
        
        # Speech tracking to prevent multiple announcements
        self.idle_mode_announced = False
        
        # Head tracking state
        self.current_head_angle = 0.0
        self.head_angle_lock = threading.Lock()
        
        # Display parameters
        self.center_x = 0
        self.center_y = 0
        self.scale = 0
        self.update_counter = 0
        self.display_update_rate = 3
        
        if not pygame.get_init():
            pygame.init()
        
        pygame.font.init()
    
    def initialize_components(self):
        """Initialize test mode components"""
        if self.initialized:
            return True
        
        try:
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
            pygame.display.set_caption("MAXINE STABLE LIDAR TEST")
            
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            self.scale = min(display_info.current_w, display_info.current_h) // 7
            
            self.draw_clean_interface()
            pygame.display.flip()
            
            self.blackboard.set("HEAD_CENTER_POSITION", 0.0)
            
            # Initialize head tracker
            robot = self.get_robot()
            if (hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager) or \
               (hasattr(robot, 'servo_controller') and robot.servo_controller):
                try:
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
            self.start_stable_lidar()
            
            return True
            
        except Exception:
            self.initialized = False
            return False
    
    def start_stable_lidar(self):
        """Start stable LiDAR system for testing"""
        try:
            if not self.lidar_system:
                self.lidar_system = UltraStableLidarSystem()
                success = self.lidar_system.start()
                if success:
                    self.blackboard.set("LIDAR_SYSTEM", self.lidar_system)
                    time.sleep(3)
        except Exception:
            pass
    
    def draw_clean_interface(self):
        """Draw clean test interface"""
        self.screen.fill((0, 0, 0))
        self.draw_radar_grid()
        self.draw_robot()
    
    def draw_radar_grid(self):
        """Draw radar grid for test visualization"""
        for distance in [1000, 2000, 3000, 4000, 5000, 6000]:
            radius = distance * self.scale // 1000
            if radius < min(self.center_x, self.center_y) - 50:
                line_width = 3 if distance == 6000 else 2
                color = (0, 150, 0) if distance < 6000 else (255, 255, 0)
                pygame.draw.circle(self.screen, color, (self.center_x, self.center_y), radius, line_width)
        
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
        """Draw LiDAR obstacles for testing"""
        if not obstacles:
            return 0
        
        drawn_count = 0
        for angle, distance in obstacles:
            if 100 < distance < 6000:
                distance_m = distance / 1000.0
                display_angle_rad = math.radians(90 - angle)
                
                x = self.center_x + int(distance_m * self.scale * math.cos(display_angle_rad))
                y = self.center_y - int(distance_m * self.scale * math.sin(display_angle_rad))
                
                if 0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height():
                    pygame.draw.circle(self.screen, (255, 0, 0), (x, y), 4)
                    drawn_count += 1
        
        return drawn_count
    
    def draw_info(self, scan_count):
        """Draw test mode information"""
        info_lines = [
            "LIDAR TEST MODE",
            f"Obstacles: {scan_count}",
            "ESC - Return to IDLE MODE"
        ]
        
        font = pygame.font.Font(None, 48)
        y_offset = 50
        
        for line in info_lines:
            if "LIDAR TEST MODE" in line:
                color = (255, 255, 0)
                text_surface = pygame.font.Font(None, 64).render(line, True, color)
            elif "ESC" in line:
                color = (255, 0, 0)
                text_surface = font.render(line, True, color)
            else:
                color = (255, 255, 255)
                text_surface = font.render(line, True, color)
            
            self.screen.blit(text_surface, (50, y_offset))
            y_offset += 60

    def center_head_for_idle_mode(self):
        """Return head to center position for IDLE mode"""
        try:
            robot = self.get_robot()
            
            centering_methods = [
                lambda: robot.servo_controller.center() if hasattr(robot, 'servo_controller') and robot.servo_controller else None,
                lambda: robot.head_velocity_manager.center_head() if hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager else None,
            ]
            
            for method in centering_methods:
                try:
                    result = method()
                    if result is not None:
                        time.sleep(0.5)
                        break
                except Exception:
                    continue
                    
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

    def perform_enhanced_idle_mode_transition(self):
        """Perform enhanced transition to IDLE mode for test mode"""
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
        """Test mode update with enhanced exit handling"""
        try:
            if not self.initialized:
                self.initialize_components()
                return Status.RUNNING
            
            self.update_counter += 1
            
            # Handle events with enhanced exit
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
                        
                        # Draw test info
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
        """Enhanced termination for test mode"""
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


# Export classes for compatibility
IsolatedLidarChase = UltraStableLidarChase
SimpleLidarChase = UltraStableLidarChase
StableLidarChase = UltraStableLidarChase
LidarTestBehavior = StableLidarTest