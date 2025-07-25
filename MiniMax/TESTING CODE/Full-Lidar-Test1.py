#!/usr/bin/env python3
"""
RPlidar A3 Multi-threaded Real-time Scanner for Maxine the Robot
Optimized for pyrplidar library v0.1.2 with real-time obstacle avoidance
"""
import time
import math
import threading
import queue
import multiprocessing
from collections import deque
from typing import List, Tuple, Optional
import numpy as np
import matplotlib.pyplot as plt
from pyrplidar import PyRPlidar
import serial

class OptimizedPyRPLidarA3:
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000, timeout=3000):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.lidar = None
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5
        self.is_connected = False

    def connect(self):
        """Connect to the LiDAR device"""
        try:
            self.lidar = PyRPlidar()
            self.lidar.connect(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            self.is_connected = True
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
                self.lidar.stop_motor()
                self.lidar.disconnect()
                self.is_connected = False
        except Exception as e:
            print(f"Disconnect error: {e}")

    def get_info_safe(self):
        """Safely get device information"""
        try:
            if self.lidar and self.is_connected:
                info = self.lidar.get_info()
                return {
                    'model': info.get('model', 'A3'),
                    'firmware': info.get('firmware_version', 'Unknown'),
                    'hardware': info.get('hardware_version', 'Unknown'),
                    'serialnumber': info.get('serialnumber', 'A3_Device')
                }
            else:
                return {'model': 'A3_Manual', 'status': 'Not Connected'}
        except Exception as e:
            return {'model': 'A3_Fallback', 'status': 'Connected', 'error': str(e)}

    def get_health_safe(self):
        """Safely get device health"""
        try:
            if self.lidar and self.is_connected:
                health = self.lidar.get_health()
                return (health.get('status', 'Unknown'), health.get('error_code', 0))
            else:
                return ('Disconnected', 0)
        except Exception as e:
            return ('Unknown', 0)

    def reset_device(self):
        """Reset the device and clear buffers"""
        try:
            if self.lidar and self.is_connected:
                self.lidar.stop()
                time.sleep(0.1)
                self.lidar.reset()
                time.sleep(0.2)
        except Exception as e:
            print(f"Reset error: {e}")

    def start_scanning(self, scan_type='express'):
        """Start scanning with specified mode"""
        try:
            if not self.is_connected:
                return False
                
            self.lidar.stop()
            time.sleep(0.1)
            
            # Start motor
            self.lidar.start_motor()
            time.sleep(2)  # Wait for motor to stabilize
            
            # Start scanning - pyrplidar supports different scan modes
            if scan_type == 'express':
                self.lidar.start_scan_express(4)  # Express scan mode 4 for A3
            else:
                self.lidar.start_scan()
            
            time.sleep(0.5)
            return True
            
        except Exception as e:
            print(f"Start scanning error: {e}")
            return False

    def get_scan_data_generator(self):
        """Generator for continuous scan data"""
        if not self.is_connected or not self.lidar:
            return
            
        consecutive_failures = 0
        max_failures = 10
        
        while consecutive_failures < max_failures:
            try:
                # Get scan data from pyrplidar
                scan_data = self.lidar.grab_scan()
                
                if scan_data and len(scan_data) > 0:
                    consecutive_failures = 0
                    
                    # Convert pyrplidar format to consistent format
                    # pyrplidar returns list of [quality, angle, distance]
                    processed_scan = []
                    for point in scan_data:
                        if len(point) >= 3:
                            quality = point[0]
                            angle = point[1]
                            distance = point[2]
                            processed_scan.append((quality, angle, distance))
                    
                    if len(processed_scan) > 10:  # Only yield if we have sufficient data
                        yield processed_scan
                else:
                    consecutive_failures += 1
                    time.sleep(0.01)
                    
            except Exception as e:
                consecutive_failures += 1
                print(f"Scan data error: {e}")
                time.sleep(0.1)
                
                # Try to recover
                if consecutive_failures % 5 == 0:
                    try:
                        self.reset_device()
                        self.start_scanning()
                    except:
                        pass

class LidarDataProcessor:
    """High-speed data processor for obstacle detection"""
    
    def __init__(self, max_range=4000):
        self.max_range = max_range
        self.obstacle_zones = {
            'front': {'min_angle': -30, 'max_angle': 30, 'obstacles': []},
            'left': {'min_angle': 30, 'max_angle': 150, 'obstacles': []},
            'right': {'min_angle': -150, 'max_angle': -30, 'obstacles': []},
            'rear': {'min_angle': 150, 'max_angle': 180, 'obstacles': []}
        }
        
    def process_scan_fast(self, scan_data):
        """Ultra-fast scan processing optimized for obstacle detection"""
        obstacles = []
        zone_obstacles = {zone: [] for zone in self.obstacle_zones.keys()}
        
        for quality, angle, distance in scan_data:
            if distance > 0 and quality > 0 and distance < self.max_range:
                # Normalize angle to -180 to 180
                normalized_angle = ((angle + 180) % 360) - 180
                
                obstacles.append((normalized_angle, distance))
                
                # Quick zone classification for obstacle avoidance
                if -30 <= normalized_angle <= 30:
                    zone_obstacles['front'].append(distance)
                elif 30 < normalized_angle <= 150:
                    zone_obstacles['left'].append(distance)
                elif -150 <= normalized_angle < -30:
                    zone_obstacles['right'].append(distance)
                elif normalized_angle > 150 or normalized_angle < -150:
                    zone_obstacles['rear'].append(distance)
        
        # Calculate minimum distances for each zone (critical for obstacle avoidance)
        zone_min_distances = {}
        for zone, distances in zone_obstacles.items():
            zone_min_distances[zone] = min(distances) if distances else float('inf')
        
        return obstacles, zone_min_distances

class MultiThreadedLidarSystem:
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000, enable_visualization=True):
        self.port = port
        self.baudrate = baudrate
        self.enable_visualization = enable_visualization
        self.lidar = None
        self.data_processor = LidarDataProcessor()
        
        # Thread-safe data structures
        self.scan_queue = queue.Queue(maxsize=5)  # Small buffer for real-time performance
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
        """High-priority thread for LiDAR data acquisition"""
        print("Starting LiDAR data acquisition thread...")
        
        try:
            self.lidar = OptimizedPyRPLidarA3(self.port, self.baudrate, timeout=3000)
            
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
            if not self.lidar.start_scanning('express'):
                print("Failed to start scanning")
                return
            
            print("LiDAR scanning started successfully")
            reset_counter = 0
            
            # Main data acquisition loop
            for scan_data in self.lidar.get_scan_data_generator():
                if not self.running:
                    break
                
                try:
                    # Process scan data at maximum speed
                    obstacles, zone_distances = self.data_processor.process_scan_fast(scan_data)
                    
                    # Update shared data with minimal locking
                    with self.data_lock:
                        self.latest_obstacles = obstacles
                        self.latest_zone_distances = zone_distances
                    
                    # Queue data for visualization (non-blocking)
                    if self.enable_visualization:
                        try:
                            # Only keep latest scan for visualization
                            while not self.scan_queue.empty():
                                try:
                                    self.scan_queue.get_nowait()
                                except queue.Empty:
                                    break
                            self.scan_queue.put_nowait((obstacles, zone_distances))
                        except queue.Full:
                            pass  # Skip if queue is full to maintain real-time performance
                    
                    # Performance monitoring
                    self.scan_count += 1
                    current_time = time.time()
                    if current_time - self.last_fps_time >= 1.0:
                        self.fps = self.scan_count / (current_time - self.last_fps_time)
                        self.scan_count = 0
                        self.last_fps_time = current_time
                    
                    # Periodic device reset for stability
                    reset_counter += 1
                    if reset_counter >= 500:  # Every 500 scans
                        self.lidar.reset_device()
                        reset_counter = 0
                        time.sleep(0.01)
                    
                except Exception as e:
                    print(f"Scan processing error: {e}")
                    continue
                    
        except Exception as e:
            print(f"LiDAR acquisition error: {e}")
        finally:
            if self.lidar:
                self.lidar.disconnect()
    
    def visualization_thread(self):
        """Lower priority thread for real-time visualization"""
        if not self.enable_visualization:
            return
            
        print("Starting visualization thread...")
        
        # Setup matplotlib for real-time plotting
        plt.ion()
        fig = plt.figure(figsize=(12, 10))
        
        # Main polar plot
        ax1 = fig.add_subplot(221, polar=True)
        scan_plot, = ax1.plot([], [], 'ro', markersize=0.8, alpha=0.7)
        ax1.set_theta_zero_location('N')
        ax1.set_theta_direction(-1)
        ax1.set_ylim(0, 4000)
        ax1.set_title("Maxine's LiDAR View (PyRPlidar)")
        
        # Zone distance bar chart
        ax2 = fig.add_subplot(222)
        zones = ['Front', 'Left', 'Right', 'Rear']
        zone_bars = ax2.bar(zones, [0, 0, 0, 0], color=['red', 'blue', 'green', 'orange'])
        ax2.set_ylim(0, 2000)
        ax2.set_ylabel('Min Distance (mm)')
        ax2.set_title('Zone Obstacle Distances')
        
        # FPS and status display
        ax3 = fig.add_subplot(223)
        ax3.axis('off')
        status_text = ax3.text(0.1, 0.5, '', fontsize=12, transform=ax3.transAxes)
        
        # Obstacle count per zone
        ax4 = fig.add_subplot(224)
        zone_count_bars = ax4.bar(zones, [0, 0, 0, 0], color=['red', 'blue', 'green', 'orange'])
        ax4.set_ylabel('Obstacle Count')
        ax4.set_title('Obstacles per Zone')
        
        plt.tight_layout()
        
        last_update = time.time()
        update_interval = 0.05  # 20 FPS for visualization
        
        while self.running:
            try:
                current_time = time.time()
                if current_time - last_update < update_interval:
                    time.sleep(0.01)
                    continue
                
                # Get latest data
                try:
                    obstacles, zone_distances = self.scan_queue.get_nowait()
                    
                    if obstacles:
                        # Update main polar plot
                        angles = [math.radians(angle) for angle, _ in obstacles]
                        distances = [distance for _, distance in obstacles]
                        
                        scan_plot.set_data(angles, distances)
                        ax1.set_title(f"Maxine's LiDAR - {len(obstacles)} points (PyRPlidar)")
                        
                        # Update zone distance bars
                        zone_values = [
                            zone_distances.get('front', 0),
                            zone_distances.get('left', 0),
                            zone_distances.get('right', 0),
                            zone_distances.get('rear', 0)
                        ]
                        
                        # Handle infinite values for display
                        display_values = [min(val, 2000) if val != float('inf') else 0 for val in zone_values]
                        
                        for bar, val in zip(zone_bars, display_values):
                            bar.set_height(val)
                            # Color coding: red for close obstacles
                            if val > 0 and val < 500:
                                bar.set_color('red')
                            elif val > 0 and val < 1000:
                                bar.set_color('orange')
                            else:
                                bar.set_color('green')
                        
                        # Update obstacle count bars
                        zone_counts = [0, 0, 0, 0]  # front, left, right, rear
                        for angle, distance in obstacles:
                            if -30 <= angle <= 30:
                                zone_counts[0] += 1
                            elif 30 < angle <= 150:
                                zone_counts[1] += 1
                            elif -150 <= angle < -30:
                                zone_counts[2] += 1
                            else:
                                zone_counts[3] += 1
                        
                        for bar, count in zip(zone_count_bars, zone_counts):
                            bar.set_height(count)
                        
                        # Update status text
                        status_info = f"""
PyRPlidar v0.1.2
Scan Rate: {self.fps:.1f} Hz
Total Points: {len(obstacles)}
Zones (mm):
  Front: {zone_distances.get('front', 'Clear') if zone_distances.get('front') != float('inf') else 'Clear'}
  Left: {zone_distances.get('left', 'Clear') if zone_distances.get('left') != float('inf') else 'Clear'}
  Right: {zone_distances.get('right', 'Clear') if zone_distances.get('right') != float('inf') else 'Clear'}
  Rear: {zone_distances.get('rear', 'Clear') if zone_distances.get('rear') != float('inf') else 'Clear'}
"""
                        status_text.set_text(status_info)
                        
                        # Efficient drawing
                        fig.canvas.draw_idle()
                        fig.canvas.flush_events()
                        
                        last_update = current_time
                        
                except queue.Empty:
                    time.sleep(0.01)
                    continue
                    
            except Exception as e:
                print(f"Visualization error: {e}")
                time.sleep(0.1)
        
        plt.ioff()
        plt.close('all')
    
    def get_obstacle_data(self):
        """Thread-safe method to get latest obstacle data for navigation"""
        with self.data_lock:
            return self.latest_obstacles.copy(), self.latest_zone_distances.copy()
    
    def is_path_clear(self, direction='front', min_distance=800):
        """Quick obstacle avoidance check"""
        with self.data_lock:
            zone_distance = self.latest_zone_distances.get(direction, float('inf'))
            return zone_distance > min_distance
    
    def get_scan_rate(self):
        """Get current scan rate in Hz"""
        return self.fps
    
    def start(self):
        """Start the multi-threaded LiDAR system"""
        print("Starting Multi-threaded PyRPlidar System...")
        self.running = True
        
        # Start data acquisition thread with high priority
        data_thread = threading.Thread(target=self.data_acquisition_thread, daemon=True)
        data_thread.start()
        self.threads.append(data_thread)
        
        # Start visualization thread with lower priority
        if self.enable_visualization:
            viz_thread = threading.Thread(target=self.visualization_thread, daemon=True)
            viz_thread.start()
            self.threads.append(viz_thread)
        
        print("All PyRPlidar threads started successfully!")
        return True
    
    def stop(self):
        """Stop all threads and clean up"""
        print("Stopping Multi-threaded PyRPlidar System...")
        self.running = False
        
        # Wait for threads to finish
        for thread in self.threads:
            thread.join(timeout=2.0)
        
        print("PyRPlidar system stopped.")

def demo_obstacle_avoidance():
    """Demo function showing how to use the system for obstacle avoidance"""
    lidar_system = MultiThreadedLidarSystem(enable_visualization=True)
    
    try:
        if lidar_system.start():
            print("\nReal-time obstacle avoidance demo running with PyRPlidar...")
            print("Press Ctrl+C to stop")
            
            # Wait a moment for system to initialize
            time.sleep(3)
            
            while True:
                # Get real-time obstacle data
                obstacles, zone_distances = lidar_system.get_obstacle_data()
                
                # Example obstacle avoidance logic
                front_clear = lidar_system.is_path_clear('front', 800)
                left_clear = lidar_system.is_path_clear('left', 600)
                right_clear = lidar_system.is_path_clear('right', 600)
                
                # Simple navigation decisions (this would integrate with your behavior tree)
                if not front_clear:
                    if left_clear:
                        action = "TURN LEFT"
                    elif right_clear:
                        action = "TURN RIGHT"
                    else:
                        action = "STOP/REVERSE"
                else:
                    action = "FORWARD OK"
                
                # Print status every 2 seconds
                if int(time.time()) % 2 == 0 and time.time() % 1 < 0.1:
                    scan_rate = lidar_system.get_scan_rate()
                    print(f"Action: {action} | Scan Rate: {scan_rate:.1f} Hz | Points: {len(obstacles)}")
                
                time.sleep(0.1)  # Main loop can run at lower frequency
                
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        lidar_system.stop()

def test_pyrplidar_connection():
    """Simple connection test for PyRPlidar"""
    print("Testing PyRPlidar connection...")
    
    lidar = OptimizedPyRPLidarA3()
    
    if lidar.connect():
        print("✓ Connection successful")
        
        info = lidar.get_info_safe()
        health = lidar.get_health_safe()
        
        print(f"Device Info: {info}")
        print(f"Device Health: {health}")
        
        lidar.disconnect()
        print("✓ Disconnection successful")
        return True
    else:
        print("✗ Connection failed")
        return False

if __name__ == "__main__":
    # Set process to use multiple CPU cores efficiently
    multiprocessing.set_start_method('spawn', force=True)
    
    # Test connection first
    if test_pyrplidar_connection():
        print("\nStarting full demo...")
        demo_obstacle_avoidance()
    else:
        print("Please check your LiDAR connection and try again.")