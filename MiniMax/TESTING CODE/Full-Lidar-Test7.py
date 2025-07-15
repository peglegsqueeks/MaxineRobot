#!/usr/bin/env python3
"""
RPlidar A3 Multi-threaded Real-time Scanner for Maxine the Robot
Corrected for PyRPlidar v0.1.2 API - STABLE OPERATION ONLY
Focus: Real-time obstacle data plotting without robot control
"""
import time
import math
import threading
import queue
from typing import List, Tuple, Optional
import numpy as np
import matplotlib.pyplot as plt
from pyrplidar import PyRPlidar
import sys

class CorrectPyRPLidarA3:
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000, timeout=3):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.lidar = None
        self.is_connected = False
        self.scan_generator = None
        self.scan_iterator = None

    def connect(self):
        """Connect to the LiDAR device"""
        try:
            self.lidar = PyRPlidar()
            self.lidar.connect(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            self.is_connected = True
            print("PyRPlidar connected successfully")
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            self.is_connected = False
            return False

    def disconnect(self):
        """Disconnect from the LiDAR device"""
        try:
            if self.lidar and self.is_connected:
                self.lidar.stop()
                self.lidar.set_motor_pwm(0)  # Stop motor
                self.lidar.disconnect()
                self.is_connected = False
                print("PyRPlidar disconnected successfully")
        except Exception as e:
            print(f"Disconnect error: {e}")

    def get_info_safe(self):
        """Safely get device information - corrected for pyrplidar API"""
        try:
            if self.lidar and self.is_connected:
                info = self.lidar.get_info()
                # PyRPlidar returns PyRPlidarDeviceInfo object
                return {
                    'model': getattr(info, 'model', 'Unknown'),
                    'firmware': getattr(info, 'firmware_version', 'Unknown'),
                    'hardware': getattr(info, 'hardware_version', 'Unknown'),
                    'serialnumber': getattr(info, 'serialnumber', 'Unknown')
                }
            else:
                return {'model': 'A3_Manual', 'status': 'Not Connected'}
        except Exception as e:
            return {'model': 'A3_Fallback', 'status': 'Connected', 'error': str(e)}

    def get_health_safe(self):
        """Safely get device health - corrected for pyrplidar API"""
        try:
            if self.lidar and self.is_connected:
                health = self.lidar.get_health()
                # PyRPlidar returns PyRPlidarHealth object
                return (getattr(health, 'status', 'Unknown'), getattr(health, 'error_code', 0))
            else:
                return ('Disconnected', 0)
        except Exception as e:
            return ('Unknown', 0)

    def start_scanning(self, mode='force'):
        """Start scanning - using exact pattern from working example"""
        try:
            if not self.is_connected:
                return False
                
            # Stop any existing scan
            self.lidar.stop()
            
            # Start motor with PWM (from your working example)
            self.lidar.set_motor_pwm(500)
            time.sleep(2)  # Wait for motor to stabilize
            
            # Start scanning using the exact API from your example
            if mode == 'force':
                self.scan_generator = self.lidar.force_scan()
                self.scan_iterator = self.scan_generator()  # Call to get iterator
                print("Force scanning started successfully")
            elif mode == 'express':
                # Try express mode for A3
                self.scan_generator = self.lidar.start_scan_express(4)
                self.scan_iterator = self.scan_generator()  # Call to get iterator
                print("Express scanning started successfully")
            
            return True
            
        except Exception as e:
            print(f"Start scanning error: {e}")
            return False

    def get_scan_data_generator(self):
        """Generator for continuous scan data using exact pattern from working example"""
        if not self.is_connected or not self.lidar or not self.scan_iterator:
            print("Scanner not properly initialized")
            return
            
        consecutive_failures = 0
        max_failures = 50
        
        print("Starting scan data collection...")
        
        while consecutive_failures < max_failures:
            try:
                # Get measurement from the scan iterator (following your working example)
                measurement = next(self.scan_iterator)
                
                if measurement:
                    consecutive_failures = 0
                    
                    # Extract data from PyRPlidarMeasurement object
                    try:
                        quality = getattr(measurement, 'quality', 0)
                        angle = getattr(measurement, 'angle', 0)
                        distance = getattr(measurement, 'distance', 0)
                        
                        # Basic filtering - yield individual measurements as scan data
                        if quality > 0 and distance > 0 and distance < 8000:
                            yield [(quality, angle, distance)]
                        
                    except Exception as measurement_error:
                        print(f"Measurement parsing error: {measurement_error}")
                        continue
                else:
                    consecutive_failures += 1
                    time.sleep(0.01)
                    
            except StopIteration:
                print("Scan iterator stopped - restarting...")
                # Restart scanning
                if self.start_scanning():
                    consecutive_failures = 0
                    continue
                else:
                    break
            except Exception as e:
                consecutive_failures += 1
                if consecutive_failures % 10 == 0:
                    print(f"Scan data error (attempt {consecutive_failures}): {e}")
                time.sleep(0.1)
                
                # Try to restart scanning
                if consecutive_failures % 20 == 0:
                    try:
                        print("Attempting to restart scanning...")
                        if self.start_scanning('force'):
                            print("Successfully restarted force scanning")
                        elif self.start_scanning('express'):
                            print("Successfully restarted express scanning")
                        else:
                            print("Failed to restart scanning")
                    except Exception as restart_error:
                        print(f"Restart error: {restart_error}")

class LidarDataProcessor:
    """High-speed data processor for obstacle detection"""
    
    def __init__(self, max_range=4000):
        self.max_range = max_range
        
    def process_scan_fast(self, scan_data):
        """Process scan data for obstacle visualization"""
        obstacles = []
        zone_obstacles = {'front': [], 'left': [], 'right': [], 'rear': []}
        
        for quality, angle, distance in scan_data:
            if distance > 0 and quality > 0 and distance < self.max_range:
                # Normalize angle to -180 to 180
                normalized_angle = ((angle + 180) % 360) - 180
                obstacles.append((normalized_angle, distance))
                
                # Zone classification for visualization
                if -30 <= normalized_angle <= 30:
                    zone_obstacles['front'].append(distance)
                elif 30 < normalized_angle <= 150:
                    zone_obstacles['left'].append(distance)
                elif -150 <= normalized_angle < -30:
                    zone_obstacles['right'].append(distance)
                elif normalized_angle > 150 or normalized_angle < -150:
                    zone_obstacles['rear'].append(distance)
        
        # Calculate minimum distances for each zone
        zone_min_distances = {}
        for zone, distances in zone_obstacles.items():
            zone_min_distances[zone] = min(distances) if distances else float('inf')
        
        return obstacles, zone_min_distances

class StableLidarSystem:
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        self.data_processor = LidarDataProcessor()
        
        # Thread-safe data structures
        self.scan_queue = queue.Queue(maxsize=3)
        self.latest_obstacles = []
        self.latest_zone_distances = {}
        self.data_lock = threading.Lock()
        
        # Control flags
        self.running = False
        self.threads = []
        
        # Performance monitoring
        self.scan_count = 0
        self.last_fps_time = time.time()
        self.fps = 0
        
    def data_acquisition_thread(self):
        """Data acquisition thread"""
        print("Starting LiDAR data acquisition thread...")
        
        try:
            self.lidar = CorrectPyRPLidarA3(self.port, self.baudrate, timeout=3)
            
            # Connect to LiDAR
            if not self.lidar.connect():
                print("Failed to connect to LiDAR")
                return
            
            # Get device info
            info = self.lidar.get_info_safe()
            health = self.lidar.get_health_safe()
            print(f"LiDAR Info: {info}")
            print(f"LiDAR Health: {health}")
            
            # Start scanning
            if not self.lidar.start_scanning('force'):  # Start with force mode like your example
                print("Force scan failed, trying express scan...")
                if not self.lidar.start_scanning('express'):
                    print("All scanning modes failed")
                    return
                else:
                    print("Express scan mode started successfully")
            
            print("LiDAR data acquisition active")
            
            # Main data acquisition loop
            for scan_data in self.lidar.get_scan_data_generator():
                if not self.running:
                    break
                
                try:
                    # Process scan data
                    if scan_data and len(scan_data) > 0:
                        obstacles, zone_distances = self.data_processor.process_scan_fast(scan_data)
                        
                        if len(obstacles) > 0:
                            # Update shared data
                            with self.data_lock:
                                self.latest_obstacles = obstacles
                                self.latest_zone_distances = zone_distances
                            
                            # Queue data for visualization (non-blocking)
                            try:
                                # Clear old data
                                while not self.scan_queue.empty():
                                    try:
                                        self.scan_queue.get_nowait()
                                    except queue.Empty:
                                        break
                                self.scan_queue.put_nowait((obstacles, zone_distances))
                            except queue.Full:
                                pass
                            
                            # Performance monitoring
                            self.scan_count += 1
                            current_time = time.time()
                            if current_time - self.last_fps_time >= 1.0:
                                self.fps = self.scan_count / (current_time - self.last_fps_time)
                                self.scan_count = 0
                                self.last_fps_time = current_time
                    
                except Exception as e:
                    print(f"Scan processing error: {e}")
                    continue
                    
        except Exception as e:
            print(f"LiDAR acquisition error: {e}")
        finally:
            if self.lidar:
                self.lidar.disconnect()
    
    def update_visualization(self, fig, ax, scan_plot):
        """Update visualization - simplified to only polar plot"""
        try:
            obstacles, zone_distances = self.scan_queue.get_nowait()
            
            if obstacles:
                # Update polar plot with blue dots
                angles = [math.radians(angle) for angle, _ in obstacles]
                distances = [distance for _, distance in obstacles]
                
                scan_plot.set_data(angles, distances)
                ax.set_title(f"LiDAR - {len(obstacles)} points", fontsize=10)
                
                return True
                
        except queue.Empty:
            return False
    
    def get_obstacle_data(self):
        """Get latest obstacle data"""
        with self.data_lock:
            return self.latest_obstacles.copy(), self.latest_zone_distances.copy()
    
    def start(self):
        """Start the LiDAR system"""
        print("Starting Stable PyRPlidar System (Corrected API)...")
        self.running = True
        
        # Start data acquisition thread
        data_thread = threading.Thread(target=self.data_acquisition_thread, daemon=True)
        data_thread.start()
        self.threads.append(data_thread)
        
        print("LiDAR system started successfully!")
        return True
    
    def stop(self):
        """Stop the system"""
        print("Stopping LiDAR System...")
        self.running = False
        
        # Wait for threads to finish
        for thread in self.threads:
            thread.join(timeout=2.0)
        
        print("LiDAR system stopped.")

def stable_lidar_test():
    """Stable LiDAR test with real-time plotting"""
    lidar_system = StableLidarSystem()
    
    try:
        if lidar_system.start():
            print("\nStable LiDAR test running with CORRECTED PyRPlidar API...")
            print("Real-time obstacle plotting active")
            print("Press Ctrl+C to stop")
            
            # Setup matplotlib in main thread
            plt.ion()
            fig = plt.figure(figsize=(12, 8))
            
            # Main polar plot
            ax1 = fig.add_subplot(221, polar=True)
            scan_plot, = ax1.plot([], [], 'ro', markersize=0.5, alpha=0.7)
            ax1.set_theta_zero_location('N')
            ax1.set_theta_direction(-1)
            ax1.set_ylim(0, 4000)
            ax1.set_title("Maxine's LiDAR View")
            
            # Zone distance bar chart
            ax2 = fig.add_subplot(222)
            zones = ['Front', 'Left', 'Right', 'Rear']
            zone_bars = ax2.bar(zones, [0, 0, 0, 0], color=['red', 'blue', 'green', 'orange'])
            ax2.set_ylim(0, 2000)
            ax2.set_ylabel('Min Distance (mm)')
            ax2.set_title('Zone Distances')
            
            # Status display
            ax3 = fig.add_subplot(223)
            ax3.axis('off')
            status_text = ax3.text(0.05, 0.95, '', fontsize=10, transform=ax3.transAxes, 
                                 verticalalignment='top', fontfamily='monospace')
            
            # Obstacle count per zone
            ax4 = fig.add_subplot(224)
            zone_count_bars = ax4.bar(zones, [0, 0, 0, 0], color=['red', 'blue', 'green', 'orange'])
            ax4.set_ylabel('Point Count')
            ax4.set_title('Points per Zone')
            
            plt.tight_layout()
            
            # Wait for system to initialize
            time.sleep(5)
            
            last_update = time.time()
            update_interval = 0.05  # 20 FPS
            
            while True:
                current_time = time.time()
                
                if current_time - last_update >= update_interval:
                    if lidar_system.update_visualization(fig, ax, scan_plot):
                        fig.canvas.draw_idle()
                        fig.canvas.flush_events()
                        last_update = current_time
                
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        lidar_system.stop()
        plt.ioff()
        plt.close('all')

def test_connection_only():
    """Simple connection test"""
    print("Testing PyRPlidar connection...")
    
    lidar = CorrectPyRPLidarA3()
    
    if lidar.connect():
        print("✓ Connection successful")
        
        info = lidar.get_info_safe()
        health = lidar.get_health_safe()
        
        print(f"Device Info: {info}")
        print(f"Device Health: {health}")
        
        lidar.disconnect()
        return True
    else:
        print("✗ Connection failed")
        return False

if __name__ == "__main__":
    print("=== Maxine LiDAR Stable Test (Corrected PyRPlidar API) ===")
    
    # Test connection first
    if test_connection_only():
        print("\nStarting stable LiDAR test...")
        stable_lidar_test()
    else:
        print("Please check your LiDAR connection and try again.")
        sys.exit(1)