#!/usr/bin/env python3
"""
RPlidar A3 Ultra-Stable Scanner for Maxine the Robot
Optimized for stability during robot movement with busy processor
Uses Mode 4 (Stability) and Force scan as fallback for maximum reliability
PYGAME PLOTTING ONLY - Clean version without debugging
"""
import time
import math
import threading
import queue
import signal
import sys
import pygame
from pyrplidar import PyRPlidar

class UltraStablePyRPLidarA3:
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000, timeout=2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.lidar = None
        self.is_connected = False
        self.scan_generator = None
        self.scan_iterator = None
        
        # Ultra-Stable Parameters for Moving Robot
        self.motor_pwm = 600
        self.stability_mode = 4
        self.current_mode = None

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
        """Graceful disconnect from the LiDAR device"""
        try:
            if self.lidar and self.is_connected:
                self.lidar.stop()
                time.sleep(0.5)
                self.lidar.set_motor_pwm(0)
                time.sleep(0.5)
                self.lidar.disconnect()
                self.is_connected = False
        except Exception as e:
            pass

    def start_scanning(self, mode='ultra_stable'):
        """Start scanning using ultra-stable settings"""
        try:
            if not self.is_connected:
                return False
                
            self.lidar.stop()
            time.sleep(1.0)
            
            self.lidar.set_motor_pwm(self.motor_pwm)
            time.sleep(3.0)
            
            if mode == 'ultra_stable':
                try:
                    self.scan_generator = self.lidar.start_scan_express(self.stability_mode)
                    self.scan_iterator = self.scan_generator()
                    self.current_mode = 'stability'
                    return True
                except Exception as e:
                    pass
                
                try:
                    self.scan_generator = self.lidar.force_scan()
                    self.scan_iterator = self.scan_generator()
                    self.current_mode = 'force'
                    return True
                except Exception as e:
                    pass
            
            elif mode == 'force_only':
                self.scan_generator = self.lidar.force_scan()
                self.scan_iterator = self.scan_generator()
                self.current_mode = 'force'
                return True
            
            return False
            
        except Exception as e:
            return False

    def get_scan_data_generator(self):
        """Ultra-stable generator optimized for moving robot"""
        if not self.is_connected or not self.lidar or not self.scan_iterator:
            return
            
        consecutive_failures = 0
        max_failures = 100
        scan_buffer = []
        last_angle = None
        min_scan_points = 150
        restart_count = 0
        max_restarts = 2
        
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
                        
                        if quality > 5 and distance > 100 and distance < 6000:
                            scan_buffer.append((quality, angle, distance))
                            
                            if (last_angle is not None and 
                                angle < last_angle and 
                                len(scan_buffer) > min_scan_points):
                                yield scan_buffer.copy()
                                scan_buffer.clear()
                            
                            last_angle = angle
                        
                    except Exception as measurement_error:
                        continue
                else:
                    consecutive_failures += 1
                    time.sleep(0.02)
                    
            except StopIteration:
                restart_count += 1
                if restart_count >= max_restarts:
                    break
                
                time.sleep(2)
                
                if self.start_scanning('ultra_stable'):
                    consecutive_failures = 0
                    scan_buffer.clear()
                    last_angle = None
                    continue
                elif self.start_scanning('force_only'):
                    consecutive_failures = 0
                    scan_buffer.clear()
                    last_angle = None
                    continue
                else:
                    break
                    
            except Exception as e:
                consecutive_failures += 1
                time.sleep(0.15)

# Global shutdown flag
shutdown_requested = False
global_lidar_system = None

def signal_handler(signum, frame):
    """Handle Ctrl+C signal for graceful shutdown"""
    global shutdown_requested, global_lidar_system
    shutdown_requested = True
    
    if global_lidar_system:
        global_lidar_system.stop()
    
    sys.exit(0)

class PygameLidarDisplay:
    def __init__(self, width=800, height=600):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Maxine Robot - LiDAR Display")
        
        # Colors
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.GREEN = (0, 255, 0)
        self.RED = (255, 0, 0)
        self.BLUE = (0, 0, 255)
        self.GRAY = (128, 128, 128)
        self.YELLOW = (255, 255, 0)
        
        # Maximize display - no margins
        self.center_x = width // 2
        self.center_y = height // 2
        self.max_radius = min(width, height) // 2 - 10  # Minimal margin
        self.max_distance = 5000  # mm
        
        # Larger fonts for degree labels
        self.degree_font = pygame.font.Font(None, 36)  # Larger degree font
        self.info_font = pygame.font.Font(None, 24)
        
    def draw_radar_grid(self):
        """Draw maximized radar-style grid"""
        # Draw range circles
        for i in range(1, 5):
            radius = (self.max_radius * i) // 4
            pygame.draw.circle(self.screen, self.GRAY, (self.center_x, self.center_y), radius, 1)
            
            # Distance labels (smaller, less prominent)
            distance_text = f"{(self.max_distance * i) // 4 // 1000}m"
            text_surface = pygame.font.Font(None, 16).render(distance_text, True, self.WHITE)
            self.screen.blit(text_surface, (self.center_x + radius - 15, self.center_y - 8))
        
        # Draw angle lines and large degree labels (every 45 degrees)
        for angle in range(0, 360, 45):
            end_x = self.center_x + self.max_radius * math.cos(math.radians(angle - 90))
            end_y = self.center_y + self.max_radius * math.sin(math.radians(angle - 90))
            pygame.draw.line(self.screen, self.GRAY, (self.center_x, self.center_y), (end_x, end_y), 1)
            
            # Large degree labels positioned outside the circle
            label_distance = self.max_radius + 25
            label_x = self.center_x + label_distance * math.cos(math.radians(angle - 90))
            label_y = self.center_y + label_distance * math.sin(math.radians(angle - 90))
            angle_text = f"{angle}°"
            text_surface = self.degree_font.render(angle_text, True, self.YELLOW)
            text_rect = text_surface.get_rect(center=(label_x, label_y))
            self.screen.blit(text_surface, text_rect)
    
    def draw_obstacles(self, obstacles):
        """Draw LiDAR obstacles on maximized radar display"""
        for angle, distance in obstacles:
            if distance <= self.max_distance:
                # Convert to display coordinates
                display_angle = math.radians(angle - 90)
                radius = (distance / self.max_distance) * self.max_radius
                
                x = self.center_x + radius * math.cos(display_angle)
                y = self.center_y + radius * math.sin(display_angle)
                
                # Draw obstacle point
                pygame.draw.circle(self.screen, self.GREEN, (int(x), int(y)), 3)
    
    def draw_info(self, scan_count, mode, scan_rate):
        """Draw minimal info in corner"""
        info_lines = [
            f"Obstacles: {scan_count}",
            f"Rate: {scan_rate:.1f}/s"
        ]
        
        y_offset = 10
        for line in info_lines:
            text_surface = self.info_font.render(line, True, self.WHITE)
            self.screen.blit(text_surface, (10, y_offset))
            y_offset += 25
    
    def update_display(self, obstacles, mode="unknown", scan_rate=0):
        """Update the pygame display with new LiDAR data"""
        # Handle pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    return False
        
        # Clear screen
        self.screen.fill(self.BLACK)
        
        # Draw maximized radar grid
        self.draw_radar_grid()
        
        # Draw obstacles
        if obstacles:
            self.draw_obstacles(obstacles)
        
        # Draw minimal info
        self.draw_info(len(obstacles) if obstacles else 0, mode, scan_rate)
        
        # Update display
        pygame.display.flip()
        return True
    
    def close(self):
        """Close pygame display"""
        pygame.quit()

class UltraStableLidarSystem:
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        
        # Ultra-stable data structures
        self.scan_queue = queue.Queue(maxsize=2)
        self.latest_obstacles = []
        self.data_lock = threading.Lock()
        
        # Conservative obstacle mapping
        self.obstacle_confidence = {}
        self.scan_cycle_count = 0
        self.angle_resolution = 1.0
        self.distance_resolution = 50
        
        # MODIFIED: Faster decay parameters for moving robot obstacle avoidance
        self.confidence_threshold = 0.3        # Increased from 0.15 - require higher confidence
        self.confidence_increment = 0.4        # Increased from 0.25 - faster buildup
        self.confidence_decay = 0.25           # Increased from 0.05 - MUCH faster decay
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
        """Ultra-stable data acquisition thread"""
        try:
            self.lidar = UltraStablePyRPLidarA3(self.port, self.baudrate, timeout=2.0)
            
            if not self.lidar.connect():
                return
            
            if not self.lidar.start_scanning('ultra_stable'):
                return
            
            try:
                for scan_data in self.lidar.get_scan_data_generator():
                    if not self.running or shutdown_requested:
                        break
                    
                    start_time = time.time()
                    
                    try:
                        if scan_data and len(scan_data) > 0:
                            self.scan_count += 1
                            current_time = time.time()
                            if current_time - self.last_scan_time >= 5.0:
                                self.scan_rate = self.scan_count / (current_time - self.last_scan_time)
                                self.scan_count = 0
                                self.last_scan_time = current_time
                                self.processing_time = 0
                            
                            self.update_obstacle_confidence_stable(scan_data)
                            stable_obstacles = self.get_confident_obstacles()
                            
                            if len(stable_obstacles) > 0:
                                with self.data_lock:
                                    self.latest_obstacles = stable_obstacles
                                
                                try:
                                    while not self.scan_queue.empty():
                                        self.scan_queue.get_nowait()
                                    self.scan_queue.put_nowait((stable_obstacles, self.lidar.current_mode, self.scan_rate))
                                except (queue.Full, queue.Empty):
                                    pass
                            
                            self.processing_time += time.time() - start_time
                        
                    except Exception as e:
                        if not shutdown_requested:
                            continue
                        
            except Exception as loop_error:
                pass
                
        except Exception as e:
            pass
        finally:
            if self.lidar:
                self.lidar.disconnect()
    
    def update_obstacle_confidence_stable(self, scan_data):
        """MODIFIED: Faster confidence decay optimized for moving robot obstacle avoidance"""
        self.scan_cycle_count += 1
        seen_obstacles = set()
        
        for quality, angle, distance in scan_data:
            if distance > 200 and quality > 5 and distance < 5000:
                normalized_angle = ((angle + 180) % 360) - 180
                angle_bin = round(normalized_angle / self.angle_resolution) * self.angle_resolution
                distance_bin = round(distance / self.distance_resolution) * self.distance_resolution
                
                key = (angle_bin, distance_bin)
                seen_obstacles.add(key)
                
                current_confidence = self.obstacle_confidence.get(key, 0)
                self.obstacle_confidence[key] = min(
                    self.max_confidence, 
                    current_confidence + self.confidence_increment
                )
        
        # MODIFIED: Much faster decay for moving robot - obstacles disappear quickly
        obstacles_to_remove = []
        for key, confidence in list(self.obstacle_confidence.items()):
            if key not in seen_obstacles:
                new_confidence = confidence - self.confidence_decay  # Now 0.25 instead of 0.05
                if new_confidence <= 0:
                    obstacles_to_remove.append(key)
                else:
                    self.obstacle_confidence[key] = new_confidence
        
        for key in obstacles_to_remove:
            del self.obstacle_confidence[key]
    
    def get_confident_obstacles(self):
        """Stable obstacle retrieval for moving robot with higher confidence threshold"""
        return [(angle, distance) for (angle, distance), confidence 
                in self.obstacle_confidence.items() 
                if confidence >= self.confidence_threshold]
    
    def start(self):
        """Start the ultra-stable LiDAR system"""
        self.running = True
        
        data_thread = threading.Thread(target=self.data_acquisition_thread, daemon=True)
        data_thread.start()
        self.threads.append(data_thread)
        
        return True
    
    def stop(self):
        """Stop the system gracefully"""
        self.running = False
        
        for thread in self.threads:
            thread.join(timeout=2.0)

def ultra_stable_lidar_test():
    """Ultra-stable LiDAR test - Clean version"""
    global global_lidar_system, shutdown_requested
    
    signal.signal(signal.SIGINT, signal_handler)
    
    lidar_system = UltraStableLidarSystem()
    global_lidar_system = lidar_system
    
    if not lidar_system.start():
        return
    
    # Initialize maximized pygame display
    display = PygameLidarDisplay(800, 600)
    
    time.sleep(6)
    
    clock = pygame.time.Clock()
    
    try:
        while not shutdown_requested:
            try:
                obstacles, mode, scan_rate = lidar_system.scan_queue.get_nowait()
                if not display.update_display(obstacles, mode, scan_rate):
                    break
            except queue.Empty:
                if not display.update_display([], "waiting", 0):
                    break
            
            clock.tick(10)
            
    except Exception as e:
        pass
    finally:
        display.close()
        global_lidar_system.stop()

if __name__ == "__main__":
    ultra_stable_lidar_test()