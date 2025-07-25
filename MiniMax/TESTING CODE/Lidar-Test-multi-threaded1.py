#!/usr/bin/env python3
"""
RPlidar A3 Multi-threaded Real-time Scanner for Maxine the Robot
Optimized for real-time obstacle avoidance with separate data acquisition and visualization threads
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
from rplidar import RPLidar, RPLidarException
import serial

class OptimizedRPLidarA3(RPLidar):
    def __init__(self, port, baudrate=256000, timeout=1):
        super().__init__(port, baudrate, timeout)
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5

    def get_info_safe(self):
        try:
            return self.get_info()
        except Exception as e:
            try:
                self._send_cmd(0x50)
                time.sleep(0.1)
                descriptor = self._serial_port.read(7)
                if len(descriptor) >= 7:
                    return {
                        'model': 'A3',
                        'firmware': 'Unknown',
                        'hardware': 'Unknown',
                        'serialnumber': 'A3_Device'
                    }
                else:
                    return {'model': 'A3_Manual', 'status': 'Connected'}
            except:
                return {'model': 'A3_Fallback', 'status': 'Connected'}

    def get_health_safe(self):
        try:
            return self.get_health()
        except Exception as e:
            return ('Unknown', 0)

    def aggressive_buffer_clear(self):
        """Aggressively clear all buffers and reset connection"""
        try:
            self.stop()
            time.sleep(0.05)  # Reduced delay
            
            if hasattr(self._serial_port, 'reset_input_buffer'):
                self._serial_port.reset_input_buffer()
                self._serial_port.reset_output_buffer()
            
            if hasattr(self._serial_port, 'flush'):
                self._serial_port.flush()
                
            time.sleep(0.1)  # Reduced delay
            
        except Exception as e:
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
    def __init__(self, port='/dev/ttyUSB0', enable_visualization=True):
        self.port = port
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
            self.lidar = OptimizedRPLidarA3(self.port, baudrate=256000, timeout=1)
            
            # Initialize LiDAR
            info = self.lidar.get_info_safe()
            health = self.lidar.get_health_safe()
            print(f"LiDAR Info: {info}")
            print(f"LiDAR Health: {health}")
            
            self.lidar.stop()
            time.sleep(0.5)
            self.lidar.stop_motor()
            time.sleep(1)
            self.lidar.aggressive_buffer_clear()
            self.lidar.start_motor()
            time.sleep(2)
            
            buffer_clear_counter = 0
            
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                
                try:
                    # Process scan data at maximum speed
                    obstacles, zone_distances = self.data_processor.process_scan_fast(scan)
                    
                    # Update shared data with minimal locking
                    with self.data_lock:
                        self.latest_obstacles = obstacles
                        self.latest_zone_distances = zone_distances
                    
                    # Queue data for visualization (non-blocking)
                    if self.enable_visualization:
                        try:
                            # Only keep latest scan for visualization
                            if not self.scan_queue.empty():
                                try:
                                    self.scan_queue.get_nowait()
                                except queue.Empty:
                                    pass
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
                    
                    # Periodic buffer maintenance
                    buffer_clear_counter += 1
                    if buffer_clear_counter >= 100:  # Every 100 scans
                        self.lidar.aggressive_buffer_clear()
                        buffer_clear_counter = 0
                        time.sleep(0.01)
                    
                except Exception as e:
                    print(f"Scan processing error: {e}")
                    continue
                    
        except Exception as e:
            print(f"LiDAR acquisition error: {e}")
        finally:
            if self.lidar:
                try:
                    self.lidar.stop()
                    self.lidar.stop_motor()
                    self.lidar.disconnect()
                except:
                    pass
    
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
        ax1.set_title("Maxine's LiDAR View")
        
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
                        ax1.set_title(f"Maxine's LiDAR - {len(obstacles)} points")
                        
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
    
    def start(self):
        """Start the multi-threaded LiDAR system"""
        print("Starting Multi-threaded LiDAR System...")
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
        
        print("All threads started successfully!")
        return True
    
    def stop(self):
        """Stop all threads and clean up"""
        print("Stopping Multi-threaded LiDAR System...")
        self.running = False
        
        # Wait for threads to finish
        for thread in self.threads:
            thread.join(timeout=2.0)
        
        print("System stopped.")

def demo_obstacle_avoidance():
    """Demo function showing how to use the system for obstacle avoidance"""
    lidar_system = MultiThreadedLidarSystem(enable_visualization=True)
    
    try:
        if lidar_system.start():
            print("\nReal-time obstacle avoidance demo running...")
            print("Press Ctrl+C to stop")
            
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
                if time.time() % 2 < 0.1:
                    print(f"Action: {action} | Scan Rate: {lidar_system.fps:.1f} Hz")
                
                time.sleep(0.1)  # Main loop can run at lower frequency
                
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        lidar_system.stop()

if __name__ == "__main__":
    # Set process to use multiple CPU cores efficiently
    multiprocessing.set_start_method('spawn', force=True)
    
    demo_obstacle_avoidance()