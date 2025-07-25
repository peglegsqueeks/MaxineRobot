def start_scanning(self, mode='optimized'):
        """Start scanning using optimized settings for maximum data capture performance"""
        try:
            if not self.is_connected:
                return False
                
            self.lidar.stop()
            
            # Set optimized motor speed for A3 maximum data capture
            print(f"Setting motor PWM to {self.motor_pwm} for optimal data capture...")
            self.lidar.set_motor_pwm(self.motor_pwm)
            time.sleep(2)  # Allow motor to stabilize at optimal speed
            
            if mode == 'optimized':
                # Use express mode 2 for optimal A3 data rate and quality
                print(f"Starting A3 in optimized express mode {self.scan_mode}...")
                self.scan_generator = self.lidar.start_scan_express(self.scan_mode)
                self.scan_iterator = self.scan_generator()
                print("Optimized express mode scanning started successfully")
            elif mode == 'stability':
                # Stability mode (mode 1)
                print("Starting A3 in stability mode...")
                self.scan_generator = self.lidar.start_scan_express(1)
                self.scan_iterator = self.scan_generator()
                print("Stability mode scanning started successfully")
            elif mode == 'force':
                # Fallback to force scan
                self.scan_generator = self.lidar.force_scan()
                self.scan_iterator = self.scan_generator()
                print("Force scanning started successfully")
            else:
                # High-performance express mode
                self.scan_generator = self.lidar.start_scan_express(4)
                self.scan_iterator = self.scan_generator()
                print("High-performance express scanning started successfully")
            
            return True
            
        except Exception as e:
            print(f"Start scanning error: {e}")
            return False#!/usr/bin/env python3
"""
RPlidar A3 Simple Real-time Scanner for Maxine the Robot
Simplified for PyRPlidar v0.1.2 API - POLAR PLOT ONLY
"""
import time
import math
import threading
import queue
import signal
import sys
import matplotlib.pyplot as plt
from pyrplidar import PyRPlidar

class SimplePyRPLidarA3:
    def __init__(self, port='/dev/ttyUSB0', baudrate=460800, timeout=1):  # Higher baudrate for faster data
        self.port = port
        self.baudrate = baudrate  # Increased from 256000 to 460800
        self.timeout = timeout    # Reduced timeout for responsiveness
        self.lidar = None
        self.is_connected = False
        self.scan_generator = None
        self.scan_iterator = None
        
        # A3 Performance Parameters
        self.motor_pwm = 660      # Optimized motor speed for A3 (max 1023)
        self.scan_mode = 2        # Mode 2 is often optimal for A3 real-time scanning

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

    def start_scanning(self, mode='stability'):
        """Start scanning using stability mode for A3"""
        try:
            if not self.is_connected:
                return False
                
            self.lidar.stop()
            self.lidar.set_motor_pwm(500)
            time.sleep(2)
            
            if mode == 'stability':
                # Stability mode for A3 - typically mode 1 or 2
                print("Starting A3 in stability mode...")
                self.scan_generator = self.lidar.start_scan_express(1)  # Mode 1 for stability
                self.scan_iterator = self.scan_generator()
                print("Stability mode scanning started successfully")
            elif mode == 'force':
                # Fallback to force scan
                self.scan_generator = self.lidar.force_scan()
                self.scan_iterator = self.scan_generator()
                print("Force scanning started successfully")
            else:
                # Express mode
                self.scan_generator = self.lidar.start_scan_express(4)
                self.scan_iterator = self.scan_generator()
                print("Express scanning started successfully")
            
            return True
            
        except Exception as e:
            print(f"Start scanning error: {e}")
            return False

    def get_scan_data_generator(self):
        """Generator for continuous scan data - yield complete 360° scans only"""
        if not self.is_connected or not self.lidar or not self.scan_iterator:
            print("Scanner not properly initialized")
            return
            
        consecutive_failures = 0
        max_failures = 50
        scan_buffer = []
        last_angle = None
        
        print("Starting scan data collection...")
        
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
                            # Add measurement to current scan
                            scan_buffer.append((quality, angle, distance))
                            
                            # Check if we've completed a full rotation (angle wrapped around)
                            if last_angle is not None and angle < last_angle and len(scan_buffer) > 200:
                                # Yield the complete 360° scan
                                #print(f"Complete scan: {len(scan_buffer)} points (Cycle: {getattr(self, '_cycle_debug', 0)})")
                                if not hasattr(self, '_cycle_debug'):
                                    self._cycle_debug = 0
                                self._cycle_debug += 1
                                yield scan_buffer.copy()
                                scan_buffer.clear()
                            
                            last_angle = angle
                        
                    except Exception as measurement_error:
                        continue
                else:
                    consecutive_failures += 1
                    time.sleep(0.01)
                    
            except StopIteration:
                if self.start_scanning('optimized'):
                    consecutive_failures = 0
                    scan_buffer.clear()
                    last_angle = None
                    continue
                elif self.start_scanning('stability'):
                    consecutive_failures = 0
                    scan_buffer.clear()
                    last_angle = None
                    continue
                elif self.start_scanning('force'):
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

class SimpleLidarSystem:
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        
        # Thread-safe data structures
        self.scan_queue = queue.Queue(maxsize=2)  # Reduced queue size for lower latency
        self.latest_obstacles = []
        self.data_lock = threading.Lock()
        
        # Optimized obstacle mapping for A* pathfinding readiness
        self.obstacle_confidence = {}  # Dict: {(angle_bin, distance_bin): confidence_score}
        self.scan_cycle_count = 0
        self.angle_resolution = 0.5    # Higher resolution: 0.5 degree bins
        self.distance_resolution = 25  # Higher resolution: 25mm bins
        
        # Optimized stability parameters for pathfinding
        self.confidence_threshold = 0.25   # Lower threshold for faster obstacle detection
        self.confidence_increment = 0.25   # Faster confidence building
        self.confidence_decay = 0.03       # Slower decay for more stable map
        self.max_confidence = 1.0
        
        # Performance monitoring
        self.scan_rate = 0
        self.last_scan_time = time.time()
        self.scan_count = 0
        
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
            
            if not self.lidar.start_scanning('optimized'):
                print("Optimized mode failed, trying stability mode...")
                if not self.lidar.start_scanning('stability'):
                    print("Stability mode failed, trying force scan...")
                    if not self.lidar.start_scanning('force'):
                        print("All scanning modes failed")
                        return
                    else:
                        print("Force scan mode started as fallback")
                else:
                    print("Stability mode started as fallback")
            
            print("LiDAR data acquisition active")
            
            try:
                for scan_data in self.lidar.get_scan_data_generator():
                    if not self.running or shutdown_requested:
                        print("Stopping due to shutdown request")
                        break
                    
                    try:
                        if scan_data and len(scan_data) > 0:
                            # Update performance monitoring
                            self.scan_count += 1
                            current_time = time.time()
                            if current_time - self.last_scan_time >= 1.0:
                                self.scan_rate = self.scan_count / (current_time - self.last_scan_time)
                                # print(f"Scan rate: {self.scan_rate:.1f} scans/sec")  # Commented as requested
                                self.scan_count = 0
                                self.last_scan_time = current_time
                            
                            # Process complete scan for enhanced persistent mapping
                            self.update_obstacle_confidence(scan_data)
                            
                            # Get current stable obstacles for visualization
                            stable_obstacles = self.get_confident_obstacles()
                            
                            if len(stable_obstacles) > 0:
                                with self.data_lock:
                                    self.latest_obstacles = stable_obstacles
                                
                                try:
                                    while not self.scan_queue.empty():
                                        try:
                                            self.scan_queue.get_nowait()
                                        except queue.Empty:
                                            break
                                    self.scan_queue.put_nowait(stable_obstacles)
                                except queue.Full:
                                    pass
                        
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
    
    def update_obstacle_confidence(self, scan_data):
        """Update obstacle confidence scores with exponential decay"""
        self.scan_cycle_count += 1
        
        # Track which obstacles were seen in this scan
        seen_obstacles = set()
        
        # Process new scan data
        for quality, angle, distance in scan_data:
            if distance > 0 and quality > 0 and distance < 4000:
                # Normalize angle to -180 to 180
                normalized_angle = ((angle + 180) % 360) - 180
                
                # Bin the angle and distance for consistent mapping
                angle_bin = round(normalized_angle / self.angle_resolution) * self.angle_resolution
                distance_bin = round(distance / self.distance_resolution) * self.distance_resolution
                
                key = (angle_bin, distance_bin)
                seen_obstacles.add(key)
                
                # Increase confidence for seen obstacles
                if key in self.obstacle_confidence:
                    self.obstacle_confidence[key] = min(
                        self.max_confidence, 
                        self.obstacle_confidence[key] + self.confidence_increment
                    )
                else:
                    self.obstacle_confidence[key] = self.confidence_increment
        
        # Decay confidence for obstacles not seen in this scan
        obstacles_to_remove = []
        for key in self.obstacle_confidence:
            if key not in seen_obstacles:
                self.obstacle_confidence[key] -= self.confidence_decay
                
                # Remove obstacles with very low confidence
                if self.obstacle_confidence[key] <= 0:
                    obstacles_to_remove.append(key)
        
        # Clean up obstacles with zero confidence
        for key in obstacles_to_remove:
            del self.obstacle_confidence[key]
    
    def get_confident_obstacles(self):
        """Get obstacles that meet the confidence threshold"""
        confident_obstacles = []
        
        for (angle, distance), confidence in self.obstacle_confidence.items():
            if confidence >= self.confidence_threshold:
                confident_obstacles.append((angle, distance))
        
        return confident_obstacles
    
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
        """Stop the system with graceful motor shutdown"""
        print("Stopping LiDAR System...")
        self.running = False
        
        # Wait for threads to finish
        for thread in self.threads:
            thread.join(timeout=2.0)
        
        # Note: Motor shutdown is handled in the data thread disconnect
        print("LiDAR system stopped.")

def simple_lidar_test():
    """Simple LiDAR test with ONLY polar plot"""
    global global_lidar_system, shutdown_requested
    
    # Setup signal handler for immediate Ctrl+C response
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize ALL variables at top level
    lidar_system = SimpleLidarSystem()
    global_lidar_system = lidar_system  # Store globally for signal handler
    matplotlib_figure = None
    matplotlib_polar_axis = None
    matplotlib_scan_plot = None
    
    # Start the LiDAR system
    if not lidar_system.start():
        print("Failed to start LiDAR system")
        return
    
    print("\nLiDAR polar plot active - STABILITY MODE")
    print("Press Ctrl+C ONCE to stop gracefully")
    
    # Setup matplotlib
    plt.ion()
    matplotlib_figure = plt.figure(figsize=(6, 6))
    matplotlib_polar_axis = matplotlib_figure.add_subplot(111, polar=True)
    matplotlib_scan_plot, = matplotlib_polar_axis.plot([], [], 'bo', markersize=1, alpha=0.8)
    matplotlib_polar_axis.set_theta_zero_location('N')
    matplotlib_polar_axis.set_theta_direction(-1)
    matplotlib_polar_axis.set_ylim(0, 4000)
    matplotlib_polar_axis.set_title("LiDAR - Optimized for A* Pathfinding", fontsize=11)
    matplotlib_polar_axis.grid(True)
    plt.tight_layout()
    
    # Wait for system to initialize
    time.sleep(5)
    
    last_update_time = time.time()
    update_rate = 0.05
    
    print("Starting visualization loop...")
    
    try:
        while not shutdown_requested:
            current_time = time.time()
            
            if current_time - last_update_time >= update_rate:
                if lidar_system.update_visualization(matplotlib_figure, matplotlib_polar_axis, matplotlib_scan_plot):
                    matplotlib_figure.canvas.draw_idle()
                    matplotlib_figure.canvas.flush_events()
                    last_update_time = current_time
            
            time.sleep(0.01)
            
    except Exception as e:
        print(f"Error in main loop: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup matplotlib
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
    print("=== LiDAR A3 - OPTIMIZED FOR A* PATHFINDING ===")
    
    print("Starting LiDAR with optimized data capture settings...")
    simple_lidar_test()