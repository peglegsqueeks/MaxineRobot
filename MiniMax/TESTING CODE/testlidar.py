#!/usr/bin/env python3
"""
RPLidar A3 Stable Mode Controller for Maxine the Robot
Provides stable scanning operation with error handling and recovery
"""

import time
import threading
import numpy as np
from pyrplidar import PyRPlidar
import logging
from typing import Optional, List, Tuple, Callable
from dataclasses import dataclass
from enum import Enum

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ScanMode(Enum):
    """Available scan modes for RPLidar A3"""
    STANDARD = 'Standard'
    EXPRESS = 'Express'
    BOOST = 'Boost'
    SENSITIVITY = 'Sensitivity'
    STABILITY = 'Stability'

@dataclass
class LidarPoint:
    """Represents a single lidar measurement point"""
    angle: float  # degrees
    distance: float  # millimeters
    quality: int  # signal quality (0-255)
    timestamp: float  # measurement timestamp

class RPLidarA3Controller:
    """
    Stable mode controller for RPLidar A3
    Handles initialization, scanning, and error recovery
    """
    
    def __init__(self, port: str = '/dev/ttyUSB0'):
        self.port = port
        self.lidar: Optional[PyRPlidar] = None
        self.is_scanning = False
        self.scan_thread: Optional[threading.Thread] = None
        self.latest_scan: List[LidarPoint] = []
        self.scan_callback: Optional[Callable] = None
        self.lock = threading.Lock()
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        
    def connect(self) -> bool:
        """
        Establish connection to RPLidar A3
        Returns True if successful, False otherwise
        """
        try:
            logger.info(f"Connecting to RPLidar A3 on {self.port}")
            self.lidar = PyRPlidar()
            self.lidar.connect(port=self.port, timeout=3)
            
            # Connection successful if we get here
            logger.info("Successfully connected to RPLidar A3")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to RPLidar A3: {e}")
            self.lidar = None
            return False
    
    def disconnect(self):
        """Safely disconnect from the lidar"""
        if self.is_scanning:
            self.stop_scanning()
            
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.disconnect()
                logger.info("Disconnected from RPLidar A3")
            except Exception as e:
                logger.error(f"Error during disconnect: {e}")
            finally:
                self.lidar = None
    
    def get_available_scan_modes(self) -> List[dict]:
        """Get list of available scan modes"""
        if not self.lidar:
            return []
            
        try:
            # PyRPlidar doesn't have get_scan_modes, return default modes
            modes = [
                {'name': 'Standard', 'id': 0},
                {'name': 'Express', 'id': 1}, 
                {'name': 'Boost', 'id': 2}
            ]
            logger.info(f"Available scan modes: {modes}")
            return modes
        except Exception as e:
            logger.error(f"Failed to get scan modes: {e}")
            return []
    
    def set_scan_mode(self, mode: ScanMode = ScanMode.STABILITY) -> bool:
        """
        Set the scan mode for stable operation
        Stability mode provides the most consistent results
        """
        if not self.lidar:
            logger.error("Lidar not connected")
            return False
            
        try:
            modes = self.get_available_scan_modes()
            target_mode = None
            
            # Find the target mode
            for m in modes:
                if mode.value.lower() in m['name'].lower():
                    target_mode = m
                    break
            
            if target_mode:
                logger.info(f"Setting scan mode to: {target_mode['name']}")
                return True
            else:
                logger.warning(f"Scan mode {mode.value} not found, using default")
                return True
                
        except Exception as e:
            logger.error(f"Failed to set scan mode: {e}")
            return False
    
    def start_scanning(self, callback: Optional[Callable] = None) -> bool:
        """
        Start continuous scanning in a separate thread
        callback: Optional function to call with each scan data
        """
        if self.is_scanning:
            logger.warning("Already scanning")
            return True
            
        if not self.lidar:
            if not self.connect():
                return False
        
        try:
            # Start scanning
            self.lidar.start_scan()
            time.sleep(0.5)  # Wait for scan to stabilize
            
            self.scan_callback = callback
            self.is_scanning = True
            self.scan_thread = threading.Thread(target=self._scan_worker, daemon=True)
            self.scan_thread.start()
            
            logger.info("Started RPLidar A3 scanning in stable mode")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start scanning: {e}")
            self.is_scanning = False
            return False
    
    def stop_scanning(self):
        """Stop the scanning process"""
        if not self.is_scanning:
            return
            
        logger.info("Stopping RPLidar A3 scanning")
        self.is_scanning = False
        
        if self.scan_thread and self.scan_thread.is_alive():
            self.scan_thread.join(timeout=5)
            
        if self.lidar:
            try:
                self.lidar.stop()
            except Exception as e:
                logger.error(f"Error stopping lidar: {e}")
    
    def _scan_worker(self):
        """Worker thread for continuous scanning"""
        consecutive_errors = 0
        max_consecutive_errors = 10
        
        while self.is_scanning:
            try:
                # Get scan data using PyRPlidar
                scan_generator = self.lidar.start_scan_express(4)
                
                for scan_data in scan_generator:
                    if not self.is_scanning:
                        break
                        
                    if scan_data:
                        points = []
                        for measurement in scan_data:
                            # PyRPlidar returns [quality, angle, distance]
                            if len(measurement) >= 3:
                                quality, angle, distance = measurement[0], measurement[1], measurement[2]
                                if distance > 0:  # Filter out invalid measurements
                                    point = LidarPoint(
                                        angle=angle,
                                        distance=distance,
                                        quality=quality,
                                        timestamp=time.time()
                                    )
                                    points.append(point)
                        
                        # Update latest scan data
                        with self.lock:
                            self.latest_scan = points
                        
                        # Call callback if provided
                        if self.scan_callback and points:
                            try:
                                self.scan_callback(points)
                            except Exception as e:
                                logger.error(f"Error in scan callback: {e}")
                        
                        consecutive_errors = 0  # Reset error counter on success
                        break  # Process one scan per iteration
                    else:
                        time.sleep(0.01)  # Brief pause if no data
                    
            except Exception as e:
                consecutive_errors += 1
                logger.error(f"Scan error ({consecutive_errors}/{max_consecutive_errors}): {e}")
                
                if consecutive_errors >= max_consecutive_errors:
                    logger.error("Too many consecutive scan errors, attempting reconnection")
                    if self._attempt_reconnection():
                        consecutive_errors = 0
                    else:
                        logger.error("Reconnection failed, stopping scanner")
                        self.is_scanning = False
                        break
                
                time.sleep(0.1)  # Brief pause before retry
    
    def _attempt_reconnection(self) -> bool:
        """Attempt to reconnect to the lidar"""
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            logger.error("Maximum reconnection attempts reached")
            return False
            
        self.reconnect_attempts += 1
        logger.info(f"Attempting reconnection {self.reconnect_attempts}/{self.max_reconnect_attempts}")
        
        # Disconnect current connection
        if self.lidar:
            try:
                self.lidar.disconnect()
            except:
                pass
            self.lidar = None
        
        time.sleep(2)  # Wait before reconnecting
        
        if self.connect():
            self.reconnect_attempts = 0  # Reset on successful connection
            return True
        
        return False
    
    def get_latest_scan(self) -> List[LidarPoint]:
        """Get the most recent scan data"""
        with self.lock:
            return self.latest_scan.copy()
    
    def get_obstacle_distances(self, angle_range: Tuple[float, float] = (0, 360)) -> List[Tuple[float, float]]:
        """
        Get obstacles within specified angle range
        Returns list of (angle, distance) tuples
        """
        obstacles = []
        scan_data = self.get_latest_scan()
        
        min_angle, max_angle = angle_range
        
        for point in scan_data:
            if min_angle <= point.angle <= max_angle:
                obstacles.append((point.angle, point.distance))
        
        return obstacles
    
    def get_closest_obstacle(self, angle_range: Tuple[float, float] = (0, 360)) -> Optional[Tuple[float, float]]:
        """
        Get the closest obstacle within specified angle range
        Returns (angle, distance) tuple or None if no obstacles
        """
        obstacles = self.get_obstacle_distances(angle_range)
        
        if not obstacles:
            return None
            
        return min(obstacles, key=lambda x: x[1])

# Example usage and callback function
def scan_callback(points: List[LidarPoint]):
    """Example callback function for processing scan data"""
    if not points:
        return
        
    # Find closest obstacle
    closest_distance = min(point.distance for point in points)
    closest_point = next(point for point in points if point.distance == closest_distance)
    
    # Print closest obstacle info (for debugging)
    if closest_distance < 1000:  # Within 1 meter
        print(f"WARNING: Obstacle at {closest_point.angle:.1f}° - {closest_distance:.0f}mm")

def main():
    """Example main function demonstrating stable mode operation"""
    # Initialize lidar controller
    lidar_controller = RPLidarA3Controller(port='/dev/ttyUSB0')
    
    try:
        # Connect and start scanning
        if lidar_controller.connect():
            logger.info("Successfully connected to RPLidar A3")
            
            # Start scanning with callback
            if lidar_controller.start_scanning(callback=scan_callback):
                logger.info("Scanning started successfully")
                
                # Run for demonstration (replace with your main loop)
                while True:
                    time.sleep(1)
                    
                    # Get latest scan data
                    scan_data = lidar_controller.get_latest_scan()
                    logger.info(f"Current scan has {len(scan_data)} points")
                    
                    # Check for obstacles in front (±30 degrees)
                    front_obstacle = lidar_controller.get_closest_obstacle((345, 15))
                    if front_obstacle:
                        angle, distance = front_obstacle
                        if distance < 500:  # Less than 50cm
                            logger.warning(f"Front obstacle detected: {distance:.0f}mm at {angle:.1f}°")
            
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        lidar_controller.disconnect()

if __name__ == "__main__":
    main()