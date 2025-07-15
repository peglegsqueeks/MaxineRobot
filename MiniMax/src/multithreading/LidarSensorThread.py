from typing import List
import time
import math
from .LoopingThread import LoopingThread
from pyrplidar import PyRPlidar
from ..path_finding.Position import Position


class CoherentPyRPLidarA3:
    """LiDAR wrapper that accumulates coherent 360° scans"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000, timeout=1.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.lidar = None
        self.is_connected = False
        self.scan_generator = None
        self.scan_iterator = None
        self.motor_pwm = 600  # Slightly slower for more stable scans

    def connect(self):
        try:
            self.lidar = PyRPlidar()
            self.lidar.connect(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            self.is_connected = True
            return True
        except Exception as e:
            print(f"LiDAR connection error: {e}")
            return False

    def disconnect(self):
        try:
            if self.lidar and self.is_connected:
                self.lidar.stop()
                self.lidar.set_motor_pwm(0)
                self.lidar.disconnect()
                self.is_connected = False
        except Exception as e:
            print(f"LiDAR disconnect error: {e}")

    def start_scanning(self):
        try:
            if not self.is_connected:
                return False
            self.lidar.stop()
            self.lidar.set_motor_pwm(self.motor_pwm)
            time.sleep(3)  # Longer stabilization for coherent scans
            self.scan_generator = self.lidar.start_scan_express(2)
            self.scan_iterator = self.scan_generator()
            return True
        except Exception as e:
            print(f"Start scanning error: {e}")
            return False

    def get_complete_360_scan(self):
        """Accumulate a complete 360° scan for coherent wall detection"""
        if not self.is_connected or not self.lidar or not self.scan_iterator:
            return []
        
        scan_buffer = []
        last_angle = None
        start_time = time.time()
        timeout = 3.0  # 3 second timeout
        
        while time.time() - start_time < timeout:
            try:
                measurement = next(self.scan_iterator)
                if measurement:
                    quality = getattr(measurement, 'quality', 0)
                    angle = getattr(measurement, 'angle', 0)
                    distance = getattr(measurement, 'distance', 0)
                    
                    if quality > 0 and distance > 0 and distance < 8000:
                        scan_buffer.append((quality, angle, distance))
                        
                        # Detect complete rotation (angle wraps from ~359 to ~0)
                        if (last_angle is not None and 
                            angle < last_angle and 
                            len(scan_buffer) > 100):
                            print(f"Complete 360° scan: {len(scan_buffer)} points")
                            return scan_buffer
                        
                        last_angle = angle
            except:
                break
        
        # Return partial scan if timeout
        print(f"Partial scan: {len(scan_buffer)} points")
        return scan_buffer


class LidarSensorThread(LoopingThread):
    """Coherent LiDAR - Accumulates complete scans for wall patterns"""
    
    def __init__(self, obstacles: List[Position], ms_delay: int = 100):  # Slower updates for complete scans
        super().__init__(ms_delay)
        self.obstacles = obstacles
        self.lidar = None
        self.scan_count = 0
        self.last_performance_report = time.time()

    def tick(self):
        """Get complete 360° scans for coherent wall detection"""
        if not self.lidar:
            return
        
        # Get complete 360° scan
        complete_scan = self.lidar.get_complete_360_scan()
        if not complete_scan:
            return
        
        # Filter the complete scan for coherent obstacles
        coherent_obstacles = []
        for quality, angle, distance in complete_scan:
            # Moderate filtering for walls and furniture
            if (quality > 5 and           # Basic quality filter
                distance > 100 and        # Minimum distance
                distance < 4000):         # Maximum room distance
                
                angle_rad = math.radians(angle)
                position = Position(angle=angle_rad, distance=distance)
                coherent_obstacles.append(position)
        
        # Update obstacles list with complete scan
        self.obstacles.clear()
        self.obstacles.extend(coherent_obstacles)
        
        # Performance monitoring
        self.scan_count += 1
        current_time = time.time()
        if current_time - self.last_performance_report >= 10.0:
            rate = self.scan_count / (current_time - self.last_performance_report)
            print(f"COHERENT LiDAR: {rate:.1f} complete scans/sec, {len(coherent_obstacles)} obstacles")
            self.scan_count = 0
            self.last_performance_report = current_time

    def on_start(self):
        print("Starting COHERENT LiDAR - accumulates complete 360° scans...")
        
        self.lidar = CoherentPyRPLidarA3(port="/dev/ttyUSB0", baudrate=256000, timeout=1.5)
        
        if not self.lidar.connect():
            print("Failed to connect to LiDAR")
            return
        
        if not self.lidar.start_scanning():
            print("Failed to start LiDAR scanning")
            return
        
        print("COHERENT LiDAR active - should show wall patterns now!")
        
    def on_finish(self):
        print("Shutting down COHERENT LiDAR...")
        if self.lidar:
            self.lidar.disconnect()
        print("COHERENT LiDAR shutdown complete")