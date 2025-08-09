from ..sensors.RobotSensor import RobotSensor
import time
import math
import threading
import queue
from pyrplidar import PyRPlidar
from ..path_finding.Position import Position


class WorkingPyRPLidarA3:
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000, timeout=1.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.lidar = None
        self.is_connected = False
        self.scan_generator = None
        self.scan_iterator = None
        
        # Working parameters from proven system
        self.motor_pwm = 650
        self.scan_mode = 2

    def connect(self):
        """Connect to the LiDAR device"""
        try:
            self.lidar = PyRPlidar()
            self.lidar.connect(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            self.is_connected = True
            return True
        except Exception as e:
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
        except Exception as e:
            pass

    def start_scanning(self, mode='high_performance'):
        """Start scanning using working settings"""
        try:
            if not self.is_connected:
                return False
                
            self.lidar.stop()
            self.lidar.set_motor_pwm(self.motor_pwm)
            time.sleep(1)
            
            if mode == 'high_performance':
                self.scan_generator = self.lidar.start_scan_express(self.scan_mode)
                self.scan_iterator = self.scan_generator()
            elif mode == 'stability':
                self.scan_generator = self.lidar.start_scan_express(1)
                self.scan_iterator = self.scan_generator()
            elif mode == 'force':
                self.scan_generator = self.lidar.force_scan()
                self.scan_iterator = self.scan_generator()
            
            return True
            
        except Exception as e:
            return False

    def get_scan_data_generator(self, shutdown_flag):
        """Working generator from proven system"""
        if not self.is_connected or not self.lidar or not self.scan_iterator:
            return
            
        consecutive_failures = 0
        max_failures = 50
        scan_buffer = []
        last_angle = None
        min_scan_points = 200
        restart_count = 0
        max_restarts = 3
        
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
                        
                        if quality > 0 and distance > 0 and distance < 8000:
                            scan_buffer.append((quality, angle, distance))
                            
                            if last_angle is not None and angle < last_angle and len(scan_buffer) > min_scan_points:
                                yield scan_buffer.copy()
                                scan_buffer.clear()
                            
                            last_angle = angle
                        
                    except Exception as measurement_error:
                        continue
                else:
                    consecutive_failures += 1
                    time.sleep(0.01)
                    
            except StopIteration:
                restart_count += 1
                if restart_count >= max_restarts:
                    break
                
                time.sleep(1)
                
                if self.start_scanning('high_performance'):
                    consecutive_failures = 0
                    scan_buffer.clear()
                    last_angle = None
                    continue
                elif self.start_scanning('stability'):
                    consecutive_failures = 0
                    scan_buffer.clear()
                    last_angle = None
                    continue
                else:
                    break
            except Exception as e:
                consecutive_failures += 1
                time.sleep(0.1)


class WorkingLidarSystem:
    """
    Working LiDAR system that interfaces with the existing LidarSensor
    """
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        
        # Working data structures from proven system
        self.scan_queue = queue.Queue(maxsize=1)
        self.latest_obstacles = []
        self.data_lock = threading.Lock()
        
        # FIXED obstacle mapping parameters for stable detection
        self.obstacle_confidence = {}
        self.scan_cycle_count = 0
        self.angle_resolution = 1.0  # Increased from 0.5 to reduce noise
        self.distance_resolution = 50  # Increased from 20 to reduce noise
        
        # FIXED decay parameters for stable obstacles
        self.confidence_threshold = 0.6  # Increased from 0.2 for more stable obstacles
        self.confidence_increment = 0.4  # Slightly increased
        self.confidence_decay = 0.02     # Reduced from 0.08 for more persistence
        self.max_confidence = 1.0
        
        # Control flags
        self.running = False
        self.shutdown_flag = [False]
        self.threads = []
        self.start_time = None
        
    def data_acquisition_thread(self):
        """Working data acquisition thread"""
        try:
            self.lidar = WorkingPyRPLidarA3(self.port, self.baudrate, timeout=1.5)
            
            if not self.lidar.connect():
                return
            
            if not self.lidar.start_scanning('high_performance'):
                if not self.lidar.start_scanning('stability'):
                    if not self.lidar.start_scanning('force'):
                        return
            
            try:
                for scan_data in self.lidar.get_scan_data_generator(self.shutdown_flag):
                    if not self.running or self.shutdown_flag[0]:
                        break
                    
                    try:
                        if scan_data and len(scan_data) > 0:
                            # Working obstacle confidence update
                            self.update_obstacle_confidence_stable(scan_data)
                            
                            # Get confident obstacles
                            stable_obstacles = self.get_stable_obstacles()
                            
                            if len(stable_obstacles) > 0:
                                with self.data_lock:
                                    self.latest_obstacles = stable_obstacles
                                
                                try:
                                    if not self.scan_queue.empty():
                                        self.scan_queue.get_nowait()
                                    self.scan_queue.put_nowait(stable_obstacles)
                                except (queue.Full, queue.Empty):
                                    pass
                        
                    except Exception as e:
                        if not self.shutdown_flag[0]:
                            pass
                        continue
                        
            except Exception as loop_error:
                pass
                
        except Exception as e:
            pass
        finally:
            if self.lidar:
                self.lidar.disconnect()
    
    def update_obstacle_confidence_stable(self, scan_data):
        """FIXED confidence update for stable obstacle detection"""
        self.scan_cycle_count += 1
        
        seen_obstacles = set()
        
        # Process scan data with better filtering
        for quality, angle, distance in scan_data:
            # Better quality and distance filtering
            if (quality > 8 and  # Higher quality threshold
                distance > 150 and  # Minimum distance (15cm)
                distance < 4000 and  # Maximum room distance (4m)
                distance != 0):  # Avoid zero readings
                
                # FIXED angle normalization for consistent coordinates
                normalized_angle = angle % 360  # Keep angles 0-360
                if normalized_angle > 180:
                    normalized_angle -= 360  # Convert to -180 to +180
                
                # Bin coordinates with larger bins for stability
                angle_bin = round(normalized_angle / self.angle_resolution) * self.angle_resolution
                distance_bin = round(distance / self.distance_resolution) * self.distance_resolution
                
                key = (angle_bin, distance_bin)
                seen_obstacles.add(key)
                
                # Confidence update
                current_confidence = self.obstacle_confidence.get(key, 0)
                self.obstacle_confidence[key] = min(
                    self.max_confidence, 
                    current_confidence + self.confidence_increment
                )
        
        # Slower decay for more persistent obstacles
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
    
    def get_stable_obstacles(self):
        """Get stable obstacles that exceed confidence threshold"""
        stable_obstacles = []
        for (angle, distance), confidence in self.obstacle_confidence.items():
            if confidence >= self.confidence_threshold:
                stable_obstacles.append((angle, distance))
        return stable_obstacles
    
    def get_latest_obstacles(self):
        """Get latest obstacles for external use"""
        try:
            return self.scan_queue.get_nowait()
        except queue.Empty:
            return []
    
    def start(self):
        """Start the working LiDAR system"""
        self.running = True
        self.shutdown_flag[0] = False
        self.start_time = time.time()
        
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


class WorkingLidarSensor(RobotSensor):
    """
    Working LiDAR sensor using proven system with stable obstacle detection
    """
    
    def __init__(self) -> None:
        super().__init__("Working Lidar Sensor")
        self.obstacles = []
        self.lidar_system = None
        self.is_initialized = False
        
    def initialize(self):
        """Initialize working LiDAR system"""
        if self.is_initialized:
            return True
        
        try:
            self.lidar_system = WorkingLidarSystem()
            
            if self.lidar_system.start():
                time.sleep(2.0)  # Stabilization time
                self.is_initialized = True
                return True
            else:
                return False
                
        except Exception as e:
            return False
    
    def shutdown(self):
        """Shutdown working LiDAR system"""
        try:
            if self.lidar_system:
                self.lidar_system.stop()
                
            self.is_initialized = False
            self.lidar_system = None
            self.obstacles.clear()
            
        except Exception as e:
            self.is_initialized = False
            self.lidar_system = None
    
    def get_reading(self):
        """Get current obstacle reading as Position objects"""
        if not self.is_initialized or not self.lidar_system:
            return []
        
        try:
            # Get stable obstacles
            obstacles_data = self.lidar_system.get_latest_obstacles()
            
            if obstacles_data:
                # Convert to Position objects with correct coordinate system
                obstacles = []
                for angle, distance in obstacles_data:
                    # Convert angle to radians for Position object
                    angle_rad = math.radians(angle)
                    position = Position(angle=angle_rad, distance=distance)
                    obstacles.append(position)
                
                self.obstacles = obstacles
                return obstacles
            else:
                return []
                
        except Exception as e:
            return []
    
    def get_status(self):
        """Get sensor status"""
        return {
            "is_initialized": self.is_initialized,
            "obstacle_count": len(self.obstacles),
            "system_type": "Working LiDAR System - Stable Detection"
        }


# Aliases for backward compatibility
LidarSensor = WorkingLidarSensor
EnhancedLidarSensor = WorkingLidarSensor