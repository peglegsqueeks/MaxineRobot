#!/usr/bin/env python3
"""
RPlidar A3 High-Performance Real-time Scanner for Maxine the Robot
Optimized for minimal lag and accurate real-world obstacle representation
"""
import time
import math
import threading
import queue
import signal
import sys
import matplotlib.pyplot as plt
from pyrplidar import PyRPlidar

class BalancedPerformancePyRPLidarA3:
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000, timeout=1.5):  # More reasonable timeout
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.lidar = None
        self.is_connected = False
        self.scan_generator = None
        self.scan_iterator = None
        
        # A3 Balanced Performance Parameters
        self.motor_pwm = 650      # Slightly reduced for stability
        self.scan_mode = 2        # Mode 2 for optimal A3 performance

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

    def start_scanning(self, mode='high_performance'):
        """Start scanning using high-performance settings"""
        try:
            if not self.is_connected:
                return False
                
            self.lidar.stop()
            
            # Set balanced motor speed
            print(f"Setting motor PWM to {self.motor_pwm} for balanced performance...")
            self.lidar.set_motor_pwm(self.motor_pwm)
            time.sleep(2)  # Proper stabilization time
            
            if mode == 'high_performance':
                # High-performance express mode
                print(f"Starting A3 in balanced express mode {self.scan_mode}...")
                self.scan_generator = self.lidar.start_scan_express(self.scan_mode)
                self.scan_iterator = self.scan_generator()
                print("Balanced express mode scanning started successfully")
            elif mode == 'stability':
                # Stability mode
                print("Starting A3 in stability mode...")
                self.scan_generator = self.lidar.start_scan_express(1)
                self.scan_iterator = self.scan_generator()
                print("Stability mode scanning started successfully")
            elif mode == 'force':
                # Force scan
                self.scan_generator = self.lidar.force_scan()
                self.scan_iterator = self.scan_generator()
                print("Force scanning started successfully")
            
            return True
            
        except Exception as e:
            print(f"Start scanning error: {e}")
            return False

    def get_scan_data_generator(self):
        """Balanced generator for performance and stability"""
        if not self.is_connected or not self.lidar or not self.scan_iterator:
            print("Scanner not properly initialized")
            return
            
        consecutive_failures = 0
        max_failures = 50  # Increased for stability
        scan_buffer = []
        last_angle = None
        min_scan_points = 200  # Restored reasonable minimum
        restart_count = 0
        max_restarts = 3  # Limit restarts to prevent loops
        
        print("Starting balanced scan data collection...")
        
        while consecutive_failures < max_failures:
            if shutdown_requested:
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
                            
                            # Stable scan completion detection
                            if last_angle is not None and angle < last_angle and len(scan_buffer) > min_scan_points:
                                yield scan_buffer.copy()
                                scan_buffer.clear()
                            
                            last_angle = angle
                        
                    except Exception as measurement_error:
                        continue
                else:
                    consecutive_failures += 1
                    time.sleep(0.01)  # Reasonable sleep
                    
            except StopIteration:
                # Prevent restart loops
                restart_count += 1
                if restart_count >= max_restarts:
                    print(f"Maximum restarts ({max_restarts}) reached, stopping to prevent loop")
                    break
                
                print(f"Scanner stopped, attempting restart {restart_count}/{max_restarts}...")
                time.sleep(1)  # Delay before restart
                
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
        
        print("Scan data generator exiting...")

# Global shutdown flag
shutdown_requested = False
global_lidar_system = None

def signal_handler(signum, frame):
    """Handle Ctrl+C signal for immediate graceful shutdown"""
    global shutdown_requested, global_lidar_system
    print("\n🛑 Ctrl+C detected - initiating immediate graceful shutdown...")
    shutdown_requested = True
    
    if global_lidar_system:
        print("Stopping LiDAR motor safely...")
        global_lidar_system.stop()
    
    print("Graceful shutdown complete!")
    sys.exit(0)

class BalancedPerformanceLidarSystem:
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        
        # High-performance data structures
        self.scan_queue = queue.Queue(maxsize=1)  # Minimal queue for lowest latency
        self.latest_obstacles = []
        self.data_lock = threading.Lock()
        
        # Optimized obstacle mapping for real-world accuracy
        self.obstacle_confidence = {}
        self.scan_cycle_count = 0
        self.angle_resolution = 0.5    # Fine resolution
        self.distance_resolution = 20  # Very fine distance resolution
        
        # Balanced decay parameters for real-world accuracy
        self.confidence_threshold = 0.2    # Balanced threshold
        self.confidence_increment = 0.3    # Faster confidence building
        self.confidence_decay = 0.08       # Moderate decay for moving obstacles
        self.max_confidence = 1.0
        
        # Performance monitoring
        self.scan_rate = 0
        self.last_scan_time = time.time()
        self.scan_count = 0
        self.processing_time = 0
        
        # Control flags
        self.running = False
        self.threads = []
        
    def data_acquisition_thread(self):
        """High-performance data acquisition thread"""
        print("Starting balanced LiDAR data acquisition thread...")
        
        try:
            self.lidar = BalancedPerformancePyRPLidarA3(self.port, self.baudrate, timeout=1.5)
            
            if not self.lidar.connect():
                print("Failed to connect to LiDAR")
                return
            
            if not self.lidar.start_scanning('high_performance'):
                print("Balanced mode failed, trying stability mode...")
                if not self.lidar.start_scanning('stability'):
                    print("Stability mode failed, trying force scan...")
                    if not self.lidar.start_scanning('force'):
                        print("All scanning modes failed")
                        return
            
            print("LiDAR balanced data acquisition active")
            
            try:
                for scan_data in self.lidar.get_scan_data_generator():
                    if not self.running or shutdown_requested:
                        print("Stopping due to shutdown request")
                        break
                    
                    start_time = time.time()
                    
                    try:
                        if scan_data and len(scan_data) > 0:
                            # Performance monitoring
                            self.scan_count += 1
                            current_time = time.time()
                            if current_time - self.last_scan_time >= 2.0:  # Report every 2 seconds
                                self.scan_rate = self.scan_count / (current_time - self.last_scan_time)
                                avg_processing = self.processing_time / self.scan_count if self.scan_count > 0 else 0
                                print(f"Performance: {self.scan_rate:.1f} scans/sec, {avg_processing*1000:.1f}ms processing")
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
                        if not shutdown_requested:
                            print(f"Scan processing error: {e}")
                        continue
                        
            except Exception as loop_error:
                if not shutdown_requested:
                    print(f"Data acquisition loop error: {loop_error}")
                
        except Exception as e:
            print(f"LiDAR acquisition error: {e}")
        finally:
            if self.lidar:
                self.lidar.disconnect()
    
    def update_obstacle_confidence_fast(self, scan_data):
        """Optimized confidence update with aggressive decay"""
        self.scan_cycle_count += 1
        
        # Pre-allocate set for performance
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
        """Fast obstacle retrieval"""
        return [(angle, distance) for (angle, distance), confidence 
                in self.obstacle_confidence.items() 
                if confidence >= self.confidence_threshold]
    
    def update_visualization(self, figure, polar_axis, scan_plot_line):
        """Optimized visualization update"""
        try:
            obstacles = self.scan_queue.get_nowait()
            
            if obstacles:
                # Fast conversion to radians
                angles = [math.radians(angle) for angle, _ in obstacles]
                distances = [distance for _, distance in obstacles]
                
                scan_plot_line.set_data(angles, distances)
                polar_axis.set_title(f"LiDAR Balanced - {len(obstacles)} points", fontsize=10)
                
                return True
                
        except queue.Empty:
            return False
    
    def start(self):
        """Start the high-performance LiDAR system"""
        print("Starting Balanced LiDAR System...")
        self.running = True
        
        # High-priority thread
        data_thread = threading.Thread(target=self.data_acquisition_thread, daemon=True)
        data_thread.start()
        self.threads.append(data_thread)
        
        print("Balanced LiDAR system started successfully!")
        return True
    
    def stop(self):
        """Stop the system"""
        print("Stopping Balanced LiDAR System...")
        self.running = False
        
        for thread in self.threads:
            thread.join(timeout=1.0)  # Reduced timeout
        
        print("Balanced LiDAR system stopped.")

def balanced_performance_lidar_test():
    """Balanced performance LiDAR test with optimized stability"""
    global global_lidar_system, shutdown_requested
    
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize system
    lidar_system = BalancedPerformanceLidarSystem()
    global_lidar_system = lidar_system
    
    if not lidar_system.start():
        print("Failed to start LiDAR system")
        return
    
    print("\nBalanced Performance LiDAR Active")
    print("Press Ctrl+C ONCE to stop gracefully")
    
    # Optimized matplotlib setup
    plt.ion()
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, polar=True)
    scan_plot, = ax.plot([], [], 'bo', markersize=0.8, alpha=0.9)  # Slightly larger dots
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.set_ylim(0, 4000)
    ax.set_title("LiDAR Balanced Performance - Real-time Tracking", fontsize=11)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # Reasonable initialization delay
    time.sleep(4)
    
    last_update = time.time()
    update_interval = 0.04  # 25 FPS - balanced performance
    
    print("Starting balanced visualization loop...")
    
    try:
        while not shutdown_requested:
            current_time = time.time()
            
            if current_time - last_update >= update_interval:
                if lidar_system.update_visualization(fig, ax, scan_plot):
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()
                    last_update = current_time
            
            time.sleep(0.01)  # Balanced sleep for responsiveness
            
    except Exception as e:
        print(f"Error in main loop: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if fig is not None:
            plt.ioff()
            plt.close('all')

if __name__ == "__main__":
    print("=== LiDAR A3 - BALANCED PERFORMANCE MODE ===")
    print("Optimized for stability with improved real-world tracking")
    
    balanced_performance_lidar_test()