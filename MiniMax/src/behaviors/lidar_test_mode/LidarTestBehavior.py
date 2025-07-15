import time
import math
import pygame
import depthai as dai
import numpy as np
import threading
import queue
import signal
import sys
from py_trees.common import Status
from ...types.RobotModes import RobotMode
from ...types.CameraMode import CameraMode
from ...types.MovementDirection import MovementDirection
from ..MaxineBehavior import MaxineBehavior
from ...action_managers.VelocityManager import VelocityConfig

# Import the proven stable lidar system
try:
    from pyrplidar import PyRPlidar
    PYRPLIDAR_AVAILABLE = True
except ImportError:
    print("⚠️ PyRPlidar not available - will use simulated data")
    PYRPLIDAR_AVAILABLE = False


class BalancedPerformancePyRPLidarA3:
    """Proven stable lidar acquisition system from working-optimised-lidar2.py"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000, timeout=1.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.lidar = None
        self.is_connected = False
        self.scan_generator = None
        self.scan_iterator = None
        
        # A3 Balanced Performance Parameters
        self.motor_pwm = 650      # Proven stable setting
        self.scan_mode = 2        # Mode 2 for optimal A3 performance

    def connect(self):
        """Connect to the LiDAR device"""
        try:
            if not PYRPLIDAR_AVAILABLE:
                return False
            self.lidar = PyRPlidar()
            self.lidar.connect(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            self.is_connected = True
            print("✅ PyRPlidar connected successfully")
            return True
        except Exception as e:
            print(f"❌ Connection error: {e}")
            self.is_connected = False
            return False

    def disconnect(self):
        """Disconnect from the LiDAR device"""
        try:
            if self.lidar and self.is_connected:
                self.lidar.stop()
                self.lidar.set_motor_pwm(0)
                self.lidar.disconnect()
                self.is_connected = False
                print("✅ PyRPlidar disconnected successfully")
        except Exception as e:
            print(f"❌ Disconnect error: {e}")

    def start_scanning(self, mode='high_performance'):
        """Start scanning using proven stable settings"""
        try:
            if not self.is_connected:
                return False
                
            self.lidar.stop()
            
            # Set balanced motor speed
            print(f"📡 Setting motor PWM to {self.motor_pwm} for balanced performance...")
            self.lidar.set_motor_pwm(self.motor_pwm)
            time.sleep(2)  # Proper stabilization time
            
            if mode == 'high_performance':
                print(f"📡 Starting A3 in balanced express mode {self.scan_mode}...")
                self.scan_generator = self.lidar.start_scan_express(self.scan_mode)
                self.scan_iterator = self.scan_generator()
                print("✅ Balanced express mode scanning started successfully")
            elif mode == 'stability':
                print("📡 Starting A3 in stability mode...")
                self.scan_generator = self.lidar.start_scan_express(1)
                self.scan_iterator = self.scan_generator()
                print("✅ Stability mode scanning started successfully")
            elif mode == 'force':
                self.scan_generator = self.lidar.force_scan()
                self.scan_iterator = self.scan_generator()
                print("✅ Force scanning started successfully")
            
            return True
            
        except Exception as e:
            print(f"❌ Start scanning error: {e}")
            return False

    def get_scan_data_generator(self):
        """Proven stable generator for performance and stability"""
        if not self.is_connected or not self.lidar or not self.scan_iterator:
            print("❌ Scanner not properly initialized")
            return
            
        consecutive_failures = 0
        max_failures = 50
        scan_buffer = []
        last_angle = None
        min_scan_points = 200
        restart_count = 0
        max_restarts = 3
        
        print("📡 Starting proven stable scan data collection...")
        
        while consecutive_failures < max_failures:
            try:
                measurement = next(self.scan_iterator)
                
                if measurement:
                    consecutive_failures = 0
                    
                    try:
                        quality = getattr(measurement, 'quality', 0)
                        angle = getattr(measurement, 'angle', 0)
                        distance = getattr(measurement, 'distance', 0)
                        
                        if quality > 0 and distance > 0 and distance < 8000:
                            scan_buffer.append((quality, angle, distance))
                            
                            # Stable scan completion detection
                            if last_angle is not None and angle < last_angle and len(scan_buffer) > min_scan_points:
                                yield scan_buffer.copy()
                                scan_buffer.clear()
                            
                            last_angle = angle
                        
                    except Exception:
                        continue
                else:
                    consecutive_failures += 1
                    time.sleep(0.01)
                    
            except StopIteration:
                restart_count += 1
                if restart_count >= max_restarts:
                    print(f"❌ Maximum restarts ({max_restarts}) reached")
                    break
                
                print(f"📡 Scanner stopped, attempting restart {restart_count}/{max_restarts}...")
                time.sleep(1)
                
                if self.start_scanning('high_performance'):
                    consecutive_failures = 0
                    scan_buffer.clear()
                    last_angle = None
                    continue
                else:
                    break
            except Exception as e:
                consecutive_failures += 1
                time.sleep(0.1)
        
        print("📡 Scan data generator exiting...")


class ProvenStableLidarSystem:
    """Integrated proven stable lidar system for navigation"""
    
    def __init__(self):
        self.lidar = None
        self.running = False
        self.shutdown_requested = False
        
        # High-performance data structures
        self.scan_queue = queue.Queue(maxsize=1)
        self.latest_obstacles = []
        self.data_lock = threading.Lock()
        
        # Proven obstacle mapping parameters
        self.obstacle_confidence = {}
        self.scan_cycle_count = 0
        self.angle_resolution = 0.5
        self.distance_resolution = 20
        
        # Proven confidence parameters
        self.confidence_threshold = 0.2
        self.confidence_increment = 0.3
        self.confidence_decay = 0.08
        self.max_confidence = 1.0
        
        # Performance monitoring
        self.scan_rate = 0
        self.last_scan_time = time.time()
        self.scan_count = 0
        self.processing_time = 0
        
        # Control
        self.acquisition_thread = None

    def start(self):
        """Start the proven stable LiDAR system"""
        print("📡 Starting Proven Stable LiDAR System...")
        self.running = True
        self.shutdown_requested = False
        
        # Start acquisition thread
        self.acquisition_thread = threading.Thread(target=self.data_acquisition_thread, daemon=True)
        self.acquisition_thread.start()
        
        print("✅ Proven stable LiDAR system started successfully!")
        return True

    def stop(self):
        """Stop the system safely"""
        print("📡 Stopping Proven Stable LiDAR System...")
        self.running = False
        self.shutdown_requested = True
        
        if self.acquisition_thread:
            self.acquisition_thread.join(timeout=2.0)
        
        print("✅ Proven stable LiDAR system stopped.")

    def data_acquisition_thread(self):
        """Proven stable data acquisition thread"""
        print("📡 Starting proven stable LiDAR data acquisition thread...")
        
        try:
            self.lidar = BalancedPerformancePyRPLidarA3('/dev/ttyUSB0', 256000, timeout=1.5)
            
            if not self.lidar.connect():
                print("❌ Failed to connect to LiDAR")
                return
            
            if not self.lidar.start_scanning('high_performance'):
                print("⚠️ Balanced mode failed, trying stability mode...")
                if not self.lidar.start_scanning('stability'):
                    print("⚠️ Stability mode failed, trying force scan...")
                    if not self.lidar.start_scanning('force'):
                        print("❌ All scanning modes failed")
                        return
            
            print("✅ LiDAR proven stable data acquisition active")
            
            try:
                for scan_data in self.lidar.get_scan_data_generator():
                    if not self.running or self.shutdown_requested:
                        break
                    
                    start_time = time.time()
                    
                    try:
                        if scan_data and len(scan_data) > 0:
                            # Performance monitoring
                            self.scan_count += 1
                            current_time = time.time()
                            if current_time - self.last_scan_time >= 5.0:  # Report every 5 seconds
                                self.scan_rate = self.scan_count / (current_time - self.last_scan_time)
                                avg_processing = self.processing_time / self.scan_count if self.scan_count > 0 else 0
                                print(f"📊 Performance: {self.scan_rate:.1f} scans/sec, {avg_processing*1000:.1f}ms processing")
                                self.scan_count = 0
                                self.last_scan_time = current_time
                                self.processing_time = 0
                            
                            # Fast obstacle confidence update
                            self.update_obstacle_confidence_fast(scan_data)
                            
                            # Get confident obstacles
                            stable_obstacles = self.get_confident_obstacles()
                            
                            if len(stable_obstacles) > 0:
                                with self.data_lock:
                                    self.latest_obstacles = stable_obstacles
                                
                                # Non-blocking queue update
                                try:
                                    if not self.scan_queue.empty():
                                        self.scan_queue.get_nowait()
                                    self.scan_queue.put_nowait(stable_obstacles)
                                except (queue.Full, queue.Empty):
                                    pass
                            
                            # Track processing time
                            self.processing_time += time.time() - start_time
                        
                    except Exception as e:
                        if not self.shutdown_requested:
                            print(f"❌ Scan processing error: {e}")
                        continue
                        
            except Exception as loop_error:
                if not self.shutdown_requested:
                    print(f"❌ Data acquisition loop error: {loop_error}")
                
        except Exception as e:
            print(f"❌ LiDAR acquisition error: {e}")
        finally:
            if self.lidar:
                self.lidar.disconnect()

    def update_obstacle_confidence_fast(self, scan_data):
        """Proven stable confidence update system"""
        self.scan_cycle_count += 1
        seen_obstacles = set()
        
        # Fast processing of scan data
        for quality, angle, distance in scan_data:
            if distance > 50 and quality > 0 and distance < 4000:  # Filter very close noise
                # Normalize angle
                normalized_angle = ((angle + 180) % 360) - 180
                
                # Bin coordinates
                angle_bin = round(normalized_angle / self.angle_resolution) * self.angle_resolution
                distance_bin = round(distance / self.distance_resolution) * self.distance_resolution
                
                key = (angle_bin, distance_bin)
                seen_obstacles.add(key)
                
                # Fast confidence update
                current_confidence = self.obstacle_confidence.get(key, 0)
                self.obstacle_confidence[key] = min(
                    self.max_confidence, 
                    current_confidence + self.confidence_increment
                )
        
        # Aggressive decay for unseen obstacles
        obstacles_to_remove = []
        for key, confidence in list(self.obstacle_confidence.items()):
            if key not in seen_obstacles:
                new_confidence = confidence - self.confidence_decay
                if new_confidence <= 0:
                    obstacles_to_remove.append(key)
                else:
                    self.obstacle_confidence[key] = new_confidence
        
        # Batch remove low-confidence obstacles
        for key in obstacles_to_remove:
            del self.obstacle_confidence[key]

    def get_confident_obstacles(self):
        """Fast proven stable obstacle retrieval"""
        return [(angle, distance) for (angle, distance), confidence 
                in self.obstacle_confidence.items() 
                if confidence >= self.confidence_threshold]

    def get_latest_obstacles(self):
        """Get latest stable obstacles for navigation"""
        try:
            obstacles = self.scan_queue.get_nowait()
            return obstacles
        except queue.Empty:
            with self.data_lock:
                return self.latest_obstacles.copy()

    def get_navigation_obstacles(self):
        """Convert proven stable obstacles to navigation format"""
        obstacles = self.get_latest_obstacles()
        navigation_data = []
        
        for angle, distance in obstacles:
            # Convert from proven stable format to navigation format
            navigation_data.append({
                'angle': angle,
                'distance': distance,
                'quality': 80  # High quality since these are confidence-filtered
            })
        
        return navigation_data


class LidarObstacleDetector:
    """
    Simplified lidar obstacle detection using proven stable system directly
    """
    def __init__(self, robot):
        self.robot = robot
        self.max_detection_distance = 8000  # 8m in millimeters
        self.obstacle_buffer_distance = 500  # 50cm safety buffer
        
        # Initialize proven stable lidar system
        self.proven_lidar = ProvenStableLidarSystem()
        self.lidar_started = False
        
        # Store raw obstacles directly from proven stable system
        self.latest_raw_obstacles = []
        self.last_scan_time = 0
        self.scan_interval = 0.1  # 10Hz processing
    
    def start_lidar(self):
        """Start the proven stable lidar system"""
        if not self.lidar_started:
            success = self.proven_lidar.start()
            if success:
                self.lidar_started = True
                # Give it time to initialize
                time.sleep(3)
                print("✅ Proven stable LiDAR started successfully")
            return success
        return True
    
    def stop_lidar(self):
        """Stop the proven stable lidar system"""
        if self.lidar_started:
            self.proven_lidar.stop()
            self.lidar_started = False
    
    def get_latest_obstacles_direct(self):
        """Get obstacles directly from proven stable system"""
        if not self.lidar_started:
            if not self.start_lidar():
                return self.get_simulated_obstacles()
        
        try:
            # Get obstacles directly from proven stable system
            obstacles = self.proven_lidar.get_latest_obstacles()
            if obstacles and len(obstacles) > 0:
                # Debug: Print sample data to understand format
                if len(obstacles) > 0 and not hasattr(self, '_debug_printed'):
                    print(f"📡 Raw proven stable data sample: {obstacles[:5]}")
                    self._debug_printed = True
                
                # Convert to our format if needed
                converted_obstacles = []
                for item in obstacles:
                    try:
                        if isinstance(item, tuple) and len(item) >= 2:
                            angle, distance = item[0], item[1]
                            converted_obstacles.append({
                                'angle': float(angle),
                                'distance': float(distance),
                                'quality': 90  # High quality from proven stable system
                            })
                        elif hasattr(item, 'angle') and hasattr(item, 'distance'):
                            converted_obstacles.append({
                                'angle': float(item.angle),
                                'distance': float(item.distance),
                                'quality': getattr(item, 'quality', 90)
                            })
                    except Exception as e:
                        continue
                
                if len(converted_obstacles) > 0:
                    self.latest_raw_obstacles = converted_obstacles
                    print(f"📡 Converted {len(converted_obstacles)} proven stable obstacles")
                    return converted_obstacles
                else:
                    print(f"📡 No valid obstacles after conversion")
                    return self.get_simulated_obstacles()
            else:
                return self.get_simulated_obstacles()
                
        except Exception as e:
            print(f"📡 Error getting proven stable data: {e}")
            return self.get_simulated_obstacles()
    
    def get_simulated_obstacles(self):
        """Generate realistic room layout simulation"""
        simulated_data = []
        
        # Simulate a rectangular room with walls
        for angle in range(0, 360, 5):  # Every 5 degrees
            distance = 4000  # Default 4m walls
            
            # Front wall (350° to 10°)
            if angle >= 350 or angle <= 10:
                distance = 3000  # 3m front wall
            # Right wall (80° to 100°)
            elif 80 <= angle <= 100:
                distance = 2500  # 2.5m right wall  
            # Back wall (170° to 190°)
            elif 170 <= angle <= 190:
                distance = 3500  # 3.5m back wall
            # Left wall (260° to 280°)
            elif 260 <= angle <= 280:
                distance = 2000  # 2m left wall
            # Add some furniture/obstacles
            elif 30 <= angle <= 50:
                distance = 1500  # Table/furniture
            elif 200 <= angle <= 220:
                distance = 1800  # Couch/furniture
            else:
                # Skip empty space to make walls more obvious
                continue
            
            simulated_data.append({
                'angle': angle,
                'distance': distance,
                'quality': 80
            })
        
        if not hasattr(self, '_sim_warned'):
            print(f"📡 Using realistic room simulation: {len(simulated_data)} points")
            self._sim_warned = True
        return simulated_data
    
    def process_lidar_scan(self):
        """Process lidar data - simplified to just get latest data"""
        current_time = time.time()
        if current_time - self.last_scan_time < self.scan_interval:
            return False
        
        # Get fresh data directly
        self.latest_raw_obstacles = self.get_latest_obstacles_direct()
        self.last_scan_time = current_time
        
        return len(self.latest_raw_obstacles) > 0
    
    def get_obstacles_in_sector(self, center_angle, sector_width):
        """Get closest obstacle in a sector using raw obstacle data"""
        if not self.latest_raw_obstacles:
            return self.max_detection_distance
        
        start_angle = (center_angle - sector_width/2) % 360
        end_angle = (center_angle + sector_width/2) % 360
        
        min_distance = self.max_detection_distance
        
        for obstacle in self.latest_raw_obstacles:
            try:
                angle = obstacle['angle'] % 360
                distance = obstacle['distance']
                
                # Check if angle is in sector
                in_sector = False
                if start_angle > end_angle:  # Crosses 0°
                    in_sector = (angle >= start_angle) or (angle <= end_angle)
                else:
                    in_sector = (start_angle <= angle <= end_angle)
                
                if in_sector and distance < min_distance:
                    min_distance = distance
                    
            except Exception as e:
                continue
        
        return min_distance
    
    def is_path_clear(self, direction_angle, robot_width=600):
        """Check if path is clear for robot movement"""
        sector_width = math.degrees(math.atan2(robot_width, 1000))
        closest_obstacle = self.get_obstacles_in_sector(direction_angle, sector_width)
        return closest_obstacle > self.obstacle_buffer_distance
    
    def get_safe_direction(self, target_angle, search_width=60):
        """Find the safest direction toward target, avoiding obstacles"""
        if self.is_path_clear(target_angle):
            return target_angle, True
        
        best_angle = target_angle
        best_clearance = 0
        
        for offset in range(0, search_width, 5):
            for direction in [-1, 1]:
                test_angle = (target_angle + direction * offset) % 360
                clearance = self.get_obstacles_in_sector(test_angle, 30)
                
                if clearance > self.obstacle_buffer_distance and clearance > best_clearance:
                    best_angle = test_angle
                    best_clearance = clearance
                    
                if clearance > 2000:
                    return test_angle, True
        
        return best_angle, best_clearance > self.obstacle_buffer_distance


class LidarDisplay:
    """
    Enhanced fullscreen visual display system for proven stable lidar data
    """
    def __init__(self):
        # Get display info for fullscreen
        display_info = pygame.display.Info()
        self.screen_width = display_info.current_w
        self.screen_height = display_info.current_h
        self.center_x = self.screen_width // 2
        self.center_y = self.screen_height // 2
        
        # Calculate display scale based on screen size
        self.display_scale = min(self.screen_width, self.screen_height) // 20  # Adaptive scale
        self.max_display_range = 8  # 8 meters
        
        # Colors
        self.colors = {
            'background': (0, 0, 0),
            'grid': (0, 100, 0),
            'distance_rings': (0, 150, 0),
            'angle_marks': (0, 200, 0),
            'obstacles': (255, 0, 0),
            'robot': (0, 0, 255),
            'target': (255, 255, 0),
            'safe_path': (0, 255, 255),
            'text': (255, 255, 255),
            'status_good': (0, 255, 0),
            'status_warning': (255, 255, 0),
            'status_danger': (255, 0, 0)
        }
        
        print(f"📺 Fullscreen display: {self.screen_width}x{self.screen_height}, scale: {self.display_scale}")
    
    def draw_distance_rings(self, screen):
        """Draw distance rings and grid"""
        # Distance rings at 1m, 2m, 4m, 6m, 8m
        for distance in [1, 2, 4, 6, 8]:
            radius = distance * self.display_scale
            max_radius = min(self.center_x, self.center_y) - 50
            if radius < max_radius:
                pygame.draw.circle(screen, self.colors['distance_rings'], 
                                 (self.center_x, self.center_y), radius, 2)
                
                # Distance labels
                font = pygame.font.Font(None, 36)
                text = font.render(f"{distance}m", True, self.colors['distance_rings'])
                screen.blit(text, (self.center_x + radius - 30, self.center_y - 15))
    
    def draw_angle_markings(self, screen):
        """Draw angle markings with directional labels"""
        font = pygame.font.Font(None, 30)
        max_radius = min(self.center_x, self.center_y) - 100
        
        for angle in range(0, 360, 30):
            # Convert to display coordinates
            display_angle_rad = math.radians(90 - angle)
            
            # Calculate line endpoints
            start_x = self.center_x + int((max_radius - 30) * math.cos(display_angle_rad))
            start_y = self.center_y - int((max_radius - 30) * math.sin(display_angle_rad))
            end_x = self.center_x + int(max_radius * math.cos(display_angle_rad))
            end_y = self.center_y - int(max_radius * math.sin(display_angle_rad))
            
            # Draw angle line
            line_width = 3 if angle % 90 == 0 else 2
            pygame.draw.line(screen, self.colors['angle_marks'], 
                           (start_x, start_y), (end_x, end_y), line_width)
            
            # Angle labels
            text_x = self.center_x + int((max_radius + 40) * math.cos(display_angle_rad))
            text_y = self.center_y - int((max_radius + 40) * math.sin(display_angle_rad))
            
            text = font.render(f"{angle}°", True, self.colors['angle_marks'])
            text_rect = text.get_rect(center=(text_x, text_y))
            screen.blit(text, text_rect)
    
    def draw_obstacles(self, screen, obstacle_detector):
        """Draw obstacles directly from proven stable lidar data"""
        # Get raw obstacles directly
        if not hasattr(obstacle_detector, 'latest_raw_obstacles') or not obstacle_detector.latest_raw_obstacles:
            return
        
        obstacles = obstacle_detector.latest_raw_obstacles
        drawn_obstacles = 0
        debug_points = []
        
        print(f"📊 Drawing {len(obstacles)} raw obstacles")
        
        for obstacle in obstacles:
            try:
                angle = obstacle.get('angle', 0)
                distance = obstacle.get('distance', 0)
                
                # Debug: Print first few obstacles
                if len(debug_points) < 3:
                    debug_points.append(f"  {angle:.1f}° {distance:.0f}mm")
                
                # Convert to display coordinates
                distance_m = distance / 1000.0  # Convert mm to meters
                
                if 0.1 <= distance_m <= self.max_display_range:
                    # Convert lidar coordinates to display coordinates
                    # LiDAR: 0° = forward (up), angles increase clockwise
                    # Display: 0° = right, angles increase counter-clockwise
                    # Transform: lidar_angle → display_angle = 90° - lidar_angle
                    
                    display_angle_rad = math.radians(90 - angle)
                    
                    x = self.center_x + int(distance_m * self.display_scale * math.cos(display_angle_rad))
                    y = self.center_y - int(distance_m * self.display_scale * math.sin(display_angle_rad))
                    
                    # Ensure point is within screen bounds
                    if (50 <= x < self.screen_width - 50 and 50 <= y < self.screen_height - 50):
                        # Draw obstacle point (larger and more visible)
                        pygame.draw.circle(screen, self.colors['obstacles'], (x, y), 6)
                        drawn_obstacles += 1
                        
                        # Add small debug label for first few points
                        if len(debug_points) <= 3:
                            font = pygame.font.Font(None, 20)
                            label = f"{angle:.0f}°"
                            text = font.render(label, True, self.colors['obstacles'])
                            screen.blit(text, (x + 8, y - 8))
                    
            except Exception as e:
                print(f"📊 Error drawing obstacle: {e}")
                continue
        
        # Debug output
        if drawn_obstacles > 0:
            print(f"📊 Successfully drew {drawn_obstacles} obstacles")
            if debug_points:
                print(f"📊 Sample obstacles:")
                for point in debug_points:
                    print(point)
        else:
            print(f"📊 No obstacles drawn - check coordinate transformation")
            if len(obstacles) > 0:
                print(f"📊 Raw data sample: angle={obstacles[0].get('angle', 'N/A')}, distance={obstacles[0].get('distance', 'N/A')}")
    
    def draw_coordinate_debug(self, screen):
        """Draw coordinate system debugging info"""
        font = pygame.font.Font(None, 24)
        
        # Draw coordinate system reference
        reference_points = [
            (0, "0° (N/Forward)", (0, 255, 0)),      # Green for forward
            (90, "90° (E/Right)", (255, 255, 0)),    # Yellow for right  
            (180, "180° (S/Back)", (255, 0, 255)),   # Magenta for back
            (270, "270° (W/Left)", (0, 255, 255))    # Cyan for left
        ]
        
        radius = min(self.center_x, self.center_y) - 120
        
        for angle, label, color in reference_points:
            # Convert to display coordinates
            display_angle_rad = math.radians(90 - angle)
            
            x = self.center_x + int(radius * math.cos(display_angle_rad))
            y = self.center_y - int(radius * math.sin(display_angle_rad))
            
            # Draw reference point
            pygame.draw.circle(screen, color, (x, y), 8)
            
            # Draw label
            text = font.render(label, True, color)
            text_rect = text.get_rect(center=(x, y - 25))
            screen.blit(text, text_rect)
    
    def draw_robot(self, screen):
        """Draw robot at center with orientation indicator"""
        # Robot body
        pygame.draw.circle(screen, self.colors['robot'], 
                         (self.center_x, self.center_y), 15, 4)
        
        # Robot direction arrow (pointing up = 0° forward)
        arrow_length = 25
        end_x = self.center_x
        end_y = self.center_y - arrow_length
        pygame.draw.line(screen, self.colors['robot'], 
                       (self.center_x, self.center_y), (end_x, end_y), 6)
        
        # Add 0° direction label
        font = pygame.font.Font(None, 24)
        text = font.render("0°", True, self.colors['robot'])
        screen.blit(text, (self.center_x - 10, self.center_y - arrow_length - 25))
    
    def draw_target_direction(self, screen, target_angle, distance=None):
        """Draw target person direction"""
        if target_angle is None:
            return
        
        # Convert camera angle to display coordinates
        lidar_angle = -target_angle  # Convert camera angle to lidar angle
        display_angle = math.radians(90 - lidar_angle)
        max_radius = min(self.center_x, self.center_y) - 150
        
        end_x = self.center_x + int(max_radius * 0.8 * math.cos(display_angle))
        end_y = self.center_y - int(max_radius * 0.8 * math.sin(display_angle))
        
        pygame.draw.line(screen, self.colors['target'], 
                       (self.center_x, self.center_y), (end_x, end_y), 6)
        
        # Target label
        font = pygame.font.Font(None, 48)
        text = font.render("TARGET", True, self.colors['target'])
        text_x = end_x + 20
        text_y = end_y - 20
        screen.blit(text, (text_x, text_y))
    
    def draw_safe_path(self, screen, safe_angle, is_clear):
        """Draw calculated safe path"""
        if safe_angle is None:
            return
        
        color = self.colors['safe_path'] if is_clear else self.colors['status_warning']
        display_angle = math.radians(90 - safe_angle)
        max_radius = min(self.center_x, self.center_y) - 150
        
        end_x = self.center_x + int(max_radius * 0.6 * math.cos(display_angle))
        end_y = self.center_y - int(max_radius * 0.6 * math.sin(display_angle))
        
        pygame.draw.line(screen, color, 
                       (self.center_x, self.center_y), (end_x, end_y), 8)
    
    def draw_status_info(self, screen, status_info):
        """Draw navigation status information"""
        font = pygame.font.Font(None, 48)
        small_font = pygame.font.Font(None, 36)
        
        # Create status panel on the left side
        panel_width = 600
        panel_height = self.screen_height - 40
        panel_x = 20
        panel_y = 20
        
        # Semi-transparent background
        status_surface = pygame.Surface((panel_width, panel_height))
        status_surface.set_alpha(200)
        status_surface.fill((0, 0, 0))
        screen.blit(status_surface, (panel_x, panel_y))
        
        y_offset = 40
        
        # Title
        title = font.render("PROVEN STABLE LIDAR NAV", True, self.colors['text'])
        screen.blit(title, (panel_x + 20, y_offset))
        y_offset += 60
        
        # Status lines
        status_lines = [
            f"Mode: {status_info.get('mode', 'UNKNOWN')}",
            f"Person Detected: {status_info.get('person_detected', False)}",
            f"Person Angle: {status_info.get('person_angle', 0):.1f}°",
            f"Smoothed Angle: {status_info.get('smoothed_angle', 0):.1f}°",
            f"Angle Variance: {status_info.get('angle_variance', 0):.1f}°",
            f"Stability: {status_info.get('stable_count', 0)}/{status_info.get('stability_threshold', 5)}",
            f"Head Locked: {status_info.get('head_locked', False)}",
            "",
            f"Proven LiDAR Active: {status_info.get('lidar_active', False)}",
            f"Confident Obstacles: {status_info.get('lidar_points', 0)}",
            f"Safe Path: {status_info.get('safe_angle', 0):.1f}°",
            f"Path Clear: {status_info.get('path_clear', False)}",
            f"Target Distance: {status_info.get('target_distance', 0):.1f}m",
            f"Obstacle Warning: {status_info.get('obstacle_warning', False)}",
            f"Navigation Active: {status_info.get('navigation_active', False)}"
        ]
        
        for line in status_lines:
            if line == "":  # Skip empty lines
                y_offset += 25
                continue
                
            if "Person Detected: True" in line or "Path Clear: True" in line:
                color = self.colors['status_good']
            elif "Proven LiDAR Active: True" in line:
                color = self.colors['status_good']
            elif "Proven LiDAR Active: False" in line:
                color = self.colors['status_warning']
            elif "Head Locked: True" in line:
                color = self.colors['status_warning']
            elif "Navigation Active: True" in line:
                color = self.colors['status_good']
            elif "Mode: ARRIVED" in line:
                color = self.colors['status_good']
            elif "Mode: NAVIGATING" in line or "Mode: APPROACHING" in line:
                color = self.colors['status_warning']
            else:
                color = self.colors['text']
            
            text = small_font.render(line, True, color)
            screen.blit(text, (panel_x + 20, y_offset))
            y_offset += 45
        
        # Controls info at bottom
        y_offset = self.screen_height - 200
        controls = [
            "CONTROLS:",
            "ESC - Emergency Stop & Exit",
            "Q - Quit System", 
            "D - Debug Test Pattern",
            "S - Simulate Room Layout",
            "",
            "PROVEN STABLE LIDAR:",
            "Direct obstacle rendering",
            "Confidence-based filtering",
            "Multi-threaded acquisition"
        ]
        
        for line in controls:
            if line.startswith("CONTROLS"):
                color = self.colors['status_warning']
                text = font.render(line, True, color)
            else:
                color = self.colors['text']
                text = small_font.render(line, True, color)
            
            screen.blit(text, (panel_x + 20, y_offset))
            y_offset += 35
    
    def render_frame(self, screen, obstacle_detector, navigation_status):
        """Render complete fullscreen lidar display frame with debugging"""
        # Clear screen
        screen.fill(self.colors['background'])
        
        # Draw static elements
        self.draw_distance_rings(screen)
        self.draw_angle_markings(screen)
        
        # Draw coordinate system debugging
        self.draw_coordinate_debug(screen)
        
        # Draw dynamic elements - obstacles from proven stable system
        self.draw_obstacles(screen, obstacle_detector)
        self.draw_robot(screen)
        
        # Draw navigation elements
        if navigation_status.get('person_angle') is not None:
            self.draw_target_direction(screen, navigation_status['person_angle'], 
                                     navigation_status.get('target_distance'))
        
        if navigation_status.get('safe_angle') is not None:
            self.draw_safe_path(screen, navigation_status['safe_angle'], 
                              navigation_status.get('path_clear', False))
        
        # Draw status information
        self.draw_status_info(screen, navigation_status)
        
        # Update display
        pygame.display.flip()


class NavigationController:
    """
    Anti-oscillation navigation controller using proven stable lidar
    """
    def __init__(self, robot):
        self.robot = robot
        self.obstacle_detector = LidarObstacleDetector(robot)
        
        # Navigation parameters
        self.target_distance = 1500  # Stop 1.5m from person (in mm)
        self.approach_speed = 0.6    # Approach speed
        self.turn_speed = 0.4        # Turn speed for navigation
        self.dead_zone_angle = 15.0  # LARGER dead zone when person is centered (degrees)
        self.head_dead_zone = 3.0    # Dead zone for head movement to prevent oscillation
        self.max_detection_range = 8000  # 8m detection range
        
        # Navigation state
        self.navigation_active = False
        self.current_mode = "SEARCHING"
        self.target_person_angle = None
        self.target_person_distance = None
        self.safe_path_angle = None
        self.path_clear = False
        self.obstacle_warning = False
        
        # Anti-oscillation smoothing
        self.angle_history = []
        self.angle_history_length = 8
        self.smoothed_person_angle = 0.0
        self.last_head_angle = 0.0
        self.stable_count = 0
        self.stability_threshold = 5
        
        # Timing
        self.last_update_time = 0
        self.update_interval = 0.1  # 10Hz navigation updates
        
        # Emergency stop
        self.emergency_stop = False

    def smooth_person_angle(self, raw_angle):
        """Smooth person angle to prevent oscillation"""
        if raw_angle is None:
            return None
        
        # Add to history
        self.angle_history.append(raw_angle)
        if len(self.angle_history) > self.angle_history_length:
            self.angle_history.pop(0)
        
        # Need at least 3 readings for smoothing
        if len(self.angle_history) < 3:
            return raw_angle
        
        # Calculate weighted average
        total_weight = 0
        weighted_sum = 0
        
        for i, angle in enumerate(self.angle_history):
            weight = i + 1
            weighted_sum += angle * weight
            total_weight += weight
        
        smoothed_angle = weighted_sum / total_weight
        
        # Check for stability
        angle_variance = max(self.angle_history) - min(self.angle_history)
        if angle_variance < 5.0:
            self.stable_count += 1
        else:
            self.stable_count = 0
        
        self.smoothed_person_angle = smoothed_angle
        return smoothed_angle
    
    def should_move_head(self, target_head_angle):
        """Determine if head should move to prevent oscillation"""
        # Don't move if angle change is too small
        angle_diff = abs(target_head_angle - self.last_head_angle)
        if angle_diff < self.head_dead_zone:
            return False
        
        # Require stable person detection before moving
        if self.stable_count < self.stability_threshold:
            return False
        
        return True

    def get_person_direction_and_distance(self):
        """Get person direction and distance with smoothing"""
        try:
            if not self.robot.camera_sensor:
                return None, None
            
            reading = self.robot.camera_sensor.get_reading()
            if not reading:
                return None, None
            
            people = reading.get_people_locations()
            if not people or len(people) == 0:
                self.angle_history.clear()
                self.stable_count = 0
                return None, None
            
            # Get closest person
            closest_person = None
            min_distance = float('inf')
            
            for person in people:
                if hasattr(person, 'spatialCoordinates') and person.spatialCoordinates:
                    distance = person.spatialCoordinates.z
                    if 0 < distance < min_distance and distance < self.max_detection_range:
                        min_distance = distance
                        closest_person = person
            
            if not closest_person:
                self.angle_history.clear()
                self.stable_count = 0
                return None, None
            
            # Calculate raw angle
            person_center_x = (closest_person.xmin + closest_person.xmax) / 2.0
            camera_fov = 127.0
            angle_from_center = (person_center_x - 0.5) * camera_fov
            raw_navigation_angle = -angle_from_center
            
            # Apply smoothing
            smoothed_angle = self.smooth_person_angle(raw_navigation_angle)
            distance = closest_person.spatialCoordinates.z
            
            return smoothed_angle, distance
            
        except Exception as e:
            return None, None
    
    def calculate_navigation_command(self):
        """Calculate navigation commands with anti-oscillation"""
        current_time = time.time()
        if current_time - self.last_update_time < self.update_interval:
            return None
        
        self.last_update_time = current_time
        
        # Process lidar data
        self.obstacle_detector.process_lidar_scan()
        
        # Get person location
        person_angle, person_distance = self.get_person_direction_and_distance()
        
        self.target_person_angle = person_angle
        self.target_person_distance = person_distance
        
        # Check if person is detected
        if person_angle is None or person_distance is None:
            self.current_mode = "SEARCHING"
            self.navigation_active = False
            return {
                'movement_type': 'search',
                'direction': MovementDirection.NONE,
                'speed': 0.0,
                'head_angle': 0.0
            }
        
        # Check if already at target
        if person_distance < self.target_distance:
            self.current_mode = "ARRIVED"
            self.navigation_active = False
            return {
                'movement_type': 'stop',
                'direction': MovementDirection.NONE,
                'speed': 0.0,
                'head_angle': person_angle
            }
        
        # Check if person is in dead zone
        if abs(person_angle) < self.dead_zone_angle:
            forward_clear = self.obstacle_detector.is_path_clear(0, robot_width=600)
            
            if forward_clear:
                self.current_mode = "APPROACHING"
                self.navigation_active = True
                self.safe_path_angle = 0
                self.path_clear = True
                
                return {
                    'movement_type': 'approach',
                    'direction': MovementDirection.LEFT,
                    'speed': 0.2,
                    'head_angle': person_angle
                }
            else:
                safe_angle, is_clear = self.obstacle_detector.get_safe_direction(person_angle)
                self.safe_path_angle = safe_angle
                self.path_clear = is_clear
                self.obstacle_warning = True
                
                if is_clear:
                    self.current_mode = "AVOIDING"
                    return self._calculate_turn_command(safe_angle, person_angle)
                else:
                    self.current_mode = "BLOCKED"
                    return {
                        'movement_type': 'stop',
                        'direction': MovementDirection.NONE,
                        'speed': 0.0,
                        'head_angle': person_angle
                    }
        else:
            safe_angle, is_clear = self.obstacle_detector.get_safe_direction(person_angle)
            self.safe_path_angle = safe_angle
            self.path_clear = is_clear
            
            if is_clear:
                self.current_mode = "NAVIGATING"
                self.navigation_active = True
                return self._calculate_turn_command(safe_angle, person_angle)
            else:
                self.current_mode = "AVOIDING"
                self.obstacle_warning = True
                return self._calculate_avoidance_command(person_angle)
    
    def _calculate_turn_command(self, safe_angle, person_angle):
        """Calculate turn command toward safe angle"""
        if safe_angle > 0:
            direction = MovementDirection.RIGHT
        else:
            direction = MovementDirection.LEFT
        
        angle_magnitude = abs(safe_angle)
        speed = min(self.turn_speed * (angle_magnitude / 45.0), 1.0)
        speed = max(speed, 0.2)
        
        return {
            'movement_type': 'turn',
            'direction': direction,
            'speed': speed,
            'head_angle': person_angle
        }
    
    def _calculate_avoidance_command(self, person_angle):
        """Calculate obstacle avoidance command"""
        best_angle = 0
        best_clearance = 0
        
        for test_angle in range(-90, 91, 15):
            clearance = self.obstacle_detector.get_obstacles_in_sector(test_angle, 30)
            if clearance > best_clearance:
                best_clearance = clearance
                best_angle = test_angle
        
        if best_clearance > self.obstacle_detector.obstacle_buffer_distance:
            return self._calculate_turn_command(best_angle, person_angle)
        else:
            return {
                'movement_type': 'stop',
                'direction': MovementDirection.NONE,
                'speed': 0.0,
                'head_angle': person_angle
            }
    
    def get_navigation_status(self):
        """Get current navigation status with lidar info"""
        # Count valid obstacle points from raw obstacles
        lidar_points = len(self.obstacle_detector.latest_raw_obstacles) if hasattr(self.obstacle_detector, 'latest_raw_obstacles') else 0
        
        return {
            'mode': self.current_mode,
            'person_detected': self.target_person_angle is not None,
            'person_angle': self.target_person_angle or 0,
            'smoothed_angle': self.smoothed_person_angle,
            'target_distance': (self.target_person_distance / 1000.0) if self.target_person_distance else 0,
            'safe_angle': self.safe_path_angle or 0,
            'path_clear': self.path_clear,
            'obstacle_warning': self.obstacle_warning,
            'navigation_active': self.navigation_active,
            'stable_count': self.stable_count,
            'stability_threshold': self.stability_threshold,
            'angle_variance': max(self.angle_history) - min(self.angle_history) if len(self.angle_history) > 1 else 0,
            'head_locked': not self.should_move_head(self.target_person_angle or 0),
            'lidar_active': lidar_points > 0,
            'lidar_points': lidar_points
        }


class LidarTestBehavior(MaxineBehavior):
    """
    Proven Stable Lidar Navigation System with Anti-Oscillation
    
    Features:
    - Uses proven stable PyRPlidar A3 acquisition system
    - Confidence-based obstacle filtering
    - Anti-oscillation head tracking
    - Fullscreen visualization
    - Safe navigation with obstacle avoidance
    """
    
    def __init__(self):
        super().__init__("Proven Stable Lidar Navigation with Anti-Oscillation")
        self.navigation_controller = None
        self.lidar_display = None
        self.running = False
        self.original_camera_mode = None
        self.screen = None
        self.original_display_mode = None

    def setup_systems(self, robot):
        """Setup camera system - proven stable lidar handles itself"""
        try:
            print("🔧 Setting up camera system...")
            
            # Setup camera for object detection
            if not robot.camera_sensor:
                print("❌ Camera sensor not available")
                return False
            
            print(f"📷 Current camera mode: {robot.camera_sensor.current_mode}")
            self.original_camera_mode = robot.camera_sensor.current_mode
            
            print("📷 Switching to object detection mode...")
            robot.camera_sensor.switch_mode(CameraMode.OBJECT_DETECTION)
            time.sleep(0.5)
            
            print(f"📷 New camera mode: {robot.camera_sensor.current_mode}")
            
            if robot.camera_sensor.current_mode != CameraMode.OBJECT_DETECTION:
                print("❌ Failed to switch camera to object detection mode")
                return False
            
            print("✅ Camera system setup complete")
            print("📡 Proven stable LiDAR will initialize automatically")
            return True
            
        except Exception as e:
            print(f"❌ System setup error: {e}")
            import traceback
            traceback.print_exc()
            return False

    def cleanup_systems(self, robot):
        """Cleanup with proper proven stable lidar shutdown"""
        try:
            print("🛑 Stopping all robot movement...")
            if hasattr(robot, 'velocity_manager') and robot.velocity_manager:
                for _ in range(10):
                    stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                    robot.velocity_manager.perform_action(stop_config)
                    time.sleep(0.05)
            
            print("🎯 Centering head...")
            if hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager:
                for i in range(10):
                    robot.head_velocity_manager.set_head_position(0.0)
                    time.sleep(0.1)
                    print(f"🎯 Setting head position to 0.000 (attempt {i+1}/10)")
            
            # Stop proven stable lidar system
            if self.navigation_controller and self.navigation_controller.obstacle_detector:
                print("📡 Shutting down proven stable LiDAR...")
                self.navigation_controller.obstacle_detector.stop_lidar()
            
            # Restore camera mode
            if robot.camera_sensor and self.original_camera_mode:
                robot.camera_sensor.switch_mode(self.original_camera_mode)
                print("📷 Camera mode restored")
            
            print("✅ All systems cleanup complete")
            
        except Exception as e:
            print(f"❌ Cleanup error: {e}")

    def execute_navigation_command(self, robot, nav_command):
        """Execute navigation with anti-oscillation"""
        if not nav_command:
            return
        
        try:
            movement_type = nav_command.get('movement_type', 'stop')
            direction = nav_command.get('direction', MovementDirection.NONE)
            speed = nav_command.get('speed', 0.0)
            head_angle = nav_command.get('head_angle', 0.0)
            
            # Execute body movement
            if movement_type == 'stop' or direction == MovementDirection.NONE:
                velocity_config = VelocityConfig(MovementDirection.NONE, 0.0)
            else:
                velocity_config = VelocityConfig(direction, speed)
            
            if hasattr(robot, 'velocity_manager') and robot.velocity_manager:
                robot.velocity_manager.perform_action(velocity_config)
            
            # Execute head movement with anti-oscillation
            if hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager:
                if self.navigation_controller.should_move_head(head_angle):
                    servo_position = max(-0.98, min(0.98, head_angle / 90.0 * 0.98))
                    robot.head_velocity_manager.set_head_position(servo_position)
                    self.navigation_controller.last_head_angle = head_angle
                    print(f"✅ Head moved to {head_angle:.1f}° - stable for {self.navigation_controller.stable_count} frames")
                else:
                    angle_diff = abs(head_angle - self.navigation_controller.last_head_angle)
                    stability = self.navigation_controller.stable_count
                    print(f"🔒 ANTI-OSCILLATION: Head locked - diff={angle_diff:.2f}°, stability={stability}/{self.navigation_controller.stability_threshold}")
            
        except Exception as e:
            print(f"❌ Navigation execution error: {e}")
    
    def update(self):
        """Main behavior update loop with proven stable lidar system"""
        robot = self.get_robot()
        
        try:
            # Store original display mode
            current_surface = pygame.display.get_surface()
            if current_surface:
                self.original_display_mode = current_surface.get_size()
                current_flags = current_surface.get_flags()
                if current_flags & pygame.FULLSCREEN:
                    self.original_display_mode = "FULLSCREEN"
                print(f"📺 Stored original display mode: {self.original_display_mode}")
            
            # Setup fullscreen
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
            pygame.display.set_caption("Proven Stable Lidar Navigation - Anti-Oscillation")
            
            # Setup systems
            if not self.setup_systems(robot):
                return Status.FAILURE
            
            # Initialize controllers
            self.navigation_controller = NavigationController(robot)
            self.lidar_display = LidarDisplay()
            
            self.running = True
            
            print("🚀 PROVEN STABLE LIDAR NAVIGATION ACTIVE")
            print("━" * 70)
            print("  🖥️  Fullscreen Display Mode")
            print("  🎯 Person Tracking (127° FOV)")
            print("  🛡️  Proven Stable Obstacle Avoidance")
            print("  🔄 Anti-Oscillation Head Tracking")
            print("  📊 Real-time Visualization")
            print("  🔒 Advanced Stability Systems:")
            print("     • Confidence-based obstacle filtering")
            print("     • Multi-threaded lidar acquisition")
            print("     • 15° navigation dead zone")
            print("     • 3° head movement dead zone")
            print("     • 8-frame angle smoothing")
            print("     • 5-frame stability requirement")
            print("     • 10Hz update rate")
            print("━" * 70)
            
            clock = pygame.time.Clock()
            frame_count = 0
            
            while self.running:
                frame_count += 1
                
                # Handle events
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            print("🛑 Emergency stop activated!")
                            self.running = False
                            break
                        elif event.key == pygame.K_q:
                            print("🛑 Quit requested")
                            self.running = False
                            break
                        elif event.key == pygame.K_d:
                            # Debug mode - force coordinate test pattern
                            print("🔧 Debug mode: Creating coordinate test pattern")
                            test_obstacles = []
                            for angle in range(0, 360, 15):  # Every 15 degrees
                                distance = 2000 + (angle % 90) * 20  # Varying distances
                                test_obstacles.append({
                                    'angle': angle,
                                    'distance': distance,
                                    'quality': 90
                                })
                            self.navigation_controller.obstacle_detector.latest_raw_obstacles = test_obstacles
                            print(f"🔧 Created test pattern with {len(test_obstacles)} points")
                        elif event.key == pygame.K_s:
                            # Simulate room mode
                            print("🔧 Simulation mode: Creating room layout")
                            self.navigation_controller.obstacle_detector.latest_raw_obstacles = \
                                self.navigation_controller.obstacle_detector.get_simulated_obstacles()
                    elif event.type == pygame.QUIT:
                        self.running = False
                        break
                
                if not self.running:
                    break
                
                # Debug info every 100 frames (10 seconds at 10Hz)
                if frame_count % 100 == 0:
                    print(f"🔄 Frame {frame_count}: Proven stable system running normally")
                    
                    # Debug lidar data
                    if hasattr(self.navigation_controller.obstacle_detector, 'latest_raw_obstacles'):
                        obstacles = self.navigation_controller.obstacle_detector.latest_raw_obstacles
                        if obstacles and len(obstacles) > 0:
                            print(f"📡 Current obstacles: {len(obstacles)} points")
                            # Show first few for debugging
                            sample = obstacles[:3]
                            for i, obs in enumerate(sample):
                                print(f"   Point {i+1}: {obs.get('angle', 'N/A')}° {obs.get('distance', 'N/A')}mm")
                        else:
                            print(f"📡 No obstacle data available")
                    
                    # Show navigation status
                    nav_status = self.navigation_controller.get_navigation_status()
                    print(f"📊 Navigation: mode={nav_status.get('mode')}, "
                          f"person={nav_status.get('person_detected')}, "
                          f"lidar_points={nav_status.get('lidar_points', 0)}")
                
                # Process navigation
                nav_command = self.navigation_controller.calculate_navigation_command()
                
                if nav_command:
                    self.execute_navigation_command(robot, nav_command)
                
                # Get status for display
                nav_status = self.navigation_controller.get_navigation_status()
                
                # Render fullscreen display
                self.lidar_display.render_frame(
                    self.screen, 
                    self.navigation_controller.obstacle_detector,
                    nav_status
                )
                
                # Maintain 10Hz for stability
                clock.tick(10)
                
        except Exception as e:
            print(f"❌ Proven stable navigation system error: {e}")
            import traceback
            traceback.print_exc()
            return Status.FAILURE
        
        finally:
            # Guaranteed cleanup
            print("🧹 Starting proven stable system cleanup...")
            
            try:
                # Emergency stop
                if self.navigation_controller:
                    self.navigation_controller.emergency_stop = True
                
                self.cleanup_systems(robot)
                
                # Restore display mode
                if self.original_display_mode == "FULLSCREEN":
                    display_info = pygame.display.Info()
                    pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
                    print("📺 Restored to fullscreen mode")
                elif self.original_display_mode and isinstance(self.original_display_mode, tuple):
                    pygame.display.set_mode(self.original_display_mode)
                    print(f"📺 Restored to windowed mode: {self.original_display_mode}")
                else:
                    display_info = pygame.display.Info()
                    pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
                    print("📺 Fallback: restored to fullscreen mode")
                
                # Set robot to idle
                robot.set_mode(RobotMode.IDLE)
                
            except Exception as e:
                print(f"❌ Final cleanup error: {e}")
        
        return Status.SUCCESS