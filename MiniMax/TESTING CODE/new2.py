#!/usr/bin/env python3
"""
RPlidar A3 Simple Real-time Scanner for Maxine the Robot
Simplified for PyRPlidar v0.1.2 API - POLAR PLOT ONLY
"""
import time
import math
import threading
import queue
import matplotlib.pyplot as plt
from pyrplidar import PyRPlidar
import sys

class SimplePyRPLidarA3:
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
                self.lidar.set_motor_pwm(0)
                self.lidar.disconnect()
                self.is_connected = False
                print("PyRPlidar disconnected successfully")
        except Exception as e:
            print(f"Disconnect error: {e}")

    def start_scanning(self):
        """Start scanning using force scan mode"""
        try:
            if not self.is_connected:
                return False
                
            self.lidar.stop()
            self.lidar.set_motor_pwm(500)
            time.sleep(2)
            
            self.scan_generator = self.lidar.force_scan()
            self.scan_iterator = self.scan_generator()
            print("Force scanning started successfully")
            return True
            
        except Exception as e:
            print(f"Start scanning error: {e}")
            return False

    def get_scan_data_generator(self):
        """Generator for continuous scan data"""
        if not self.is_connected or not self.lidar or not self.scan_iterator:
            print("Scanner not properly initialized")
            return
            
        consecutive_failures = 0
        max_failures = 50
        
        print("Starting scan data collection...")
        
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
                            yield [(quality, angle, distance)]
                        
                    except Exception as measurement_error:
                        continue
                else:
                    consecutive_failures += 1
                    time.sleep(0.01)
                    
            except StopIteration:
                if self.start_scanning():
                    consecutive_failures = 0
                    continue
                else:
                    break
            except Exception as e:
                consecutive_failures += 1
                time.sleep(0.1)
        
        print("Scan data generator exiting...")

class SimpleLidarSystem:
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        
        # Thread-safe data structures
        self.scan_queue = queue.Queue(maxsize=3)
        self.latest_obstacles = []
        self.data_lock = threading.Lock()
        
        # Control flags
        self.running = False
        self.threads = []
        
    def data_acquisition_thread(self):
        """Data acquisition thread"""
        print("Starting LiDAR data acquisition thread...")
        
        try:
            self.lidar = SimplePyRPLidarA3(self.port, self.baudrate, timeout=3)
            
            if not self.lidar.connect():
                print("Failed to connect to LiDAR")
                return
            
            if not self.lidar.start_scanning():
                print("Failed to start scanning")
                return
            
            print("LiDAR data acquisition active")
            
            try:
                for scan_data in self.lidar.get_scan_data_generator():
                    if not self.running:
                        print("Stopping due to running flag")
                        break
                    
                    try:
                        if scan_data and len(scan_data) > 0:
                            # Simple obstacle processing
                            obstacles = []
                            for quality, angle, distance in scan_data:
                                if distance > 0 and quality > 0 and distance < 4000:
                                    normalized_angle = ((angle + 180) % 360) - 180
                                    obstacles.append((normalized_angle, distance))
                            
                            if len(obstacles) > 0:
                                with self.data_lock:
                                    self.latest_obstacles = obstacles
                                
                                try:
                                    while not self.scan_queue.empty():
                                        try:
                                            self.scan_queue.get_nowait()
                                        except queue.Empty:
                                            break
                                    self.scan_queue.put_nowait(obstacles)
                                except queue.Full:
                                    pass
                        
                    except Exception as e:
                        print(f"Scan processing error: {e}")
                        continue
            except Exception as loop_error:
                print(f"Data acquisition loop error: {loop_error}")
                
        except Exception as e:
            print(f"LiDAR acquisition error: {e}")
        finally:
            if self.lidar:
                self.lidar.disconnect()
    
    def update_visualization(self, figure, polar_axis, scan_plot_line):
        """Update visualization - ONLY polar plot"""
        try:
            obstacles = self.scan_queue.get_nowait()
            
            if obstacles:
                angles = [math.radians(angle) for angle, _ in obstacles]
                distances = [distance for _, distance in obstacles]
                
                scan_plot_line.set_data(angles, distances)
                polar_axis.set_title(f"LiDAR - {len(obstacles)} points", fontsize=10)
                
                return True
                
        except queue.Empty:
            return False
    
    def start(self):
        """Start the LiDAR system"""
        print("Starting Simple LiDAR System...")
        self.running = True
        
        data_thread = threading.Thread(target=self.data_acquisition_thread, daemon=True)
        data_thread.start()
        self.threads.append(data_thread)
        
        print("LiDAR system started successfully!")
        return True
    
    def stop(self):
        """Stop the system"""
        print("Stopping LiDAR System...")
        self.running = False
        
        for thread in self.threads:
            thread.join(timeout=2.0)
        
        print("LiDAR system stopped.")

def simple_lidar_test():
    """Simple LiDAR test with ONLY polar plot"""
    
    # Initialize ALL variables at top level
    lidar_system = SimpleLidarSystem()
    matplotlib_figure = None
    matplotlib_polar_axis = None
    matplotlib_scan_plot = None
    
    # Start the LiDAR system
    if not lidar_system.start():
        print("Failed to start LiDAR system")
        return
    
    print("\nLiDAR polar plot active")
    print("Press Ctrl+C to stop")
    
    # Setup matplotlib
    plt.ion()
    matplotlib_figure = plt.figure(figsize=(6, 6))
    matplotlib_polar_axis = matplotlib_figure.add_subplot(111, polar=True)
    matplotlib_scan_plot, = matplotlib_polar_axis.plot([], [], 'bo', markersize=1, alpha=0.8)
    matplotlib_polar_axis.set_theta_zero_location('N')
    matplotlib_polar_axis.set_theta_direction(-1)
    matplotlib_polar_axis.set_ylim(0, 4000)
    matplotlib_polar_axis.set_title("LiDAR View", fontsize=12)
    matplotlib_polar_axis.grid(True)
    plt.tight_layout()
    
    # Wait for system to initialize
    time.sleep(5)
    
    last_update_time = time.time()
    update_rate = 0.05
    
    print("Starting visualization loop...")
    
    try:
        while True:
            current_time = time.time()
            
            if current_time - last_update_time >= update_rate:
                if lidar_system.update_visualization(matplotlib_figure, matplotlib_polar_axis, matplotlib_scan_plot):
                    matplotlib_figure.canvas.draw_idle()
                    matplotlib_figure.canvas.flush_events()
                    last_update_time = current_time
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error in main loop: {e}")
        import traceback
        traceback.print_exc()
    finally:
        lidar_system.stop()
        if matplotlib_figure is not None:
            plt.ioff()
            plt.close('all')

def test_connection_only():
    """Simple connection test"""
    print("Testing PyRPlidar connection...")
    
    lidar = SimplePyRPLidarA3()
    
    if lidar.connect():
        print("✓ Connection successful")
        lidar.disconnect()
        return True
    else:
        print("✗ Connection failed")
        return False

if __name__ == "__main__":
    print("=== Simple LiDAR Test ===")
    
    if test_connection_only():
        print("\nStarting simple LiDAR test...")
        simple_lidar_test()
    else:
        print("Please check your LiDAR connection and try again.")
        sys.exit(1)