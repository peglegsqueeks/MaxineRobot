#!/usr/bin/env python3
"""
Proper py_trees LiDAR Test - Following Project Conventions
Respects py_trees lifecycle: __init__, setup(), initialise(), update(), terminate()
Uses existing working modes as template for correct py_tree structure
"""

import pygame
import math
import time
import threading
import queue
import py_trees
import csv
import statistics
from collections import deque
from datetime import datetime
from py_trees.common import Status
from pyrplidar import PyRPlidar

from src.behaviors.MaxineBehavior import MaxineBehavior
from src.types.RobotModes import RobotMode
from src.types.CameraMode import CameraMode
from src.types.MovementDirection import MovementDirection
from src.action_managers.VelocityManager import VelocityConfig

# Import existing working components
from .LidarTestBehavior import UltraStableLidarSystem, EnhancedFacialAnimationRestorer


class ProperPyTreesLidarTest(MaxineBehavior):
    """
    PROPER py_trees LiDAR Test following project conventions
    """
    
    def __init__(self):
        super().__init__("Proper py_trees LiDAR Test")
        
        # py_trees Lifecycle: __init__ - Just setup variables, NO hardware initialization
        
        # Blackboard setup
        self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.READ)
        self.blackboard.register_key("HEAD_CENTER_POSITION", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.WRITE)
        
        # Component references - NO INITIALIZATION HERE
        self.lidar_system = None
        self.screen = None
        self.head_tracker = None
        self.facial_restorer = None
        self.robot = None
        
        # Display parameters - Just variables
        self.center_x = 0
        self.center_y = 0
        self.scale = 0
        self.update_counter = 0
        self.display_update_rate = 3
        
        # CSV LOGGING - Variables only
        self.csv_log_filename = "LIDARTEST.csv"
        self.csv_initialized = False
        self.mode_start_time = 0
        
        # X-midpoint consistency tracking - Variables only
        self.x_midpoints_pixels = deque(maxlen=1000)
        self.x_midpoints_normalized = deque(maxlen=1000)
        self.variance_window = deque(maxlen=100)
        
        # Variance calculation variables
        self.current_variance_pixels = 0.0
        self.current_std_dev_pixels = 0.0
        self.current_mean_pixels = 0.0
        
        # Consistency metrics - Variables only
        self.detection_count = 0
        self.consistent_detection_count = 0
        self.large_jumps_count = 0
        self.jump_threshold_pixels = 50
        self.last_x_midpoint = None
        
        # Multi-term variance tracking - Variables only
        self.short_term_variance = deque(maxlen=75)
        self.medium_term_variance = deque(maxlen=250)
        self.long_term_variance = deque(maxlen=1500)
        
        # Stability classification - Variables only
        self.stability_zones = {
            'stable': 0, 'moderate': 0, 'unstable': 0, 'very_unstable': 0
        }
        
        # Head tracking - DISABLED for testing consistency
        self.head_tracking_enabled = False  # Disable to test detection consistency
        self.angle_history = []
        self.last_sent_angle = None
        self.last_person_detected = 0
        self.person_lost_timeout = 2.0
        
        # Grid overlay for potential field
        self.grid_size = 40
        self.show_grid = True
        self.last_potential_grid = {}
        self.frame_counter = 0
        
        # Z-depth smoothing variables
        self.z_depth_smoother = deque(maxlen=5)
        self.confidence_weights = deque(maxlen=5)
        self.smoothed_z_depth = 0
        self.depth_trust_threshold = 0.6
        
        # Initialize pygame if not already done
        if not pygame.get_init():
            pygame.init()
        pygame.font.init()
    
    def setup(self, **kwargs):
        """
        py_trees Lifecycle: setup() - Prepare for potential execution, NO hardware initialization
        """
        # Just return success - no hardware initialization allowed here
        return True
    
    def initialise(self):
        """
        py_trees Lifecycle: initialise() - Called when behavior STARTS running (when mode changes)
        THIS is where hardware initialization happens
        """
        print("🎯 LiDAR Test initializing...")
        
        # Get robot reference
        self.robot = self.get_robot()
        
        # Stop robot movement first
        self.stop_robot()
        
        # Initialize display
        self.initialize_display()
        
        # Initialize facial restorer
        self.facial_restorer = EnhancedFacialAnimationRestorer()
        
        # Initialize LiDAR system
        self.initialize_lidar()
        
        # Initialize CSV logging
        self.mode_start_time = time.time()
        self.initialize_csv_log()
        
        # Reset tracking data
        self.reset_tracking_data()
        
        print("✅ LiDAR Test initialized successfully")
    
    def initialize_display(self):
        """Initialize pygame display"""
        try:
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
            pygame.display.set_caption("MAXINE PROPER PY_TREES LIDAR TEST")
            
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            self.scale = min(display_info.current_w, display_info.current_h) // 7
            
            # Draw initial interface
            self.draw_clean_interface()
            pygame.display.flip()
            
        except Exception as e:
            print(f"❌ Display initialization failed: {e}")
    
    def initialize_lidar(self):
        """Initialize LiDAR system"""
        try:
            print("🚀 Starting LiDAR system...")
            self.lidar_system = UltraStableLidarSystem()
            success = self.lidar_system.start()
            if success:
                self.blackboard.set("LIDAR_SYSTEM", self.lidar_system)
                print("✅ LiDAR system started successfully")
                time.sleep(2)  # Allow stabilization
            else:
                print("❌ Failed to start LiDAR system")
                self.lidar_system = None
        except Exception as e:
            print(f"❌ LiDAR initialization error: {e}")
            self.lidar_system = None
    
    def initialize_csv_log(self):
        """Initialize CSV log"""
        if self.csv_initialized:
            return
        
        try:
            with open(self.csv_log_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'mode_time_elapsed', 'timestamp', 'frame_number',
                    'x_midpoint_pixels', 'x_midpoint_normalized', 'x_camera_mm',
                    'z_depth_mm', 'confidence',
                    'bbox_xmin', 'bbox_ymin', 'bbox_xmax', 'bbox_ymax', 'bbox_width', 'bbox_height',
                    'x_jump_from_previous', 'is_large_jump', 
                    'rolling_variance_pixels', 'rolling_std_dev_pixels', 'rolling_mean_pixels',
                    'short_term_variance', 'medium_term_variance', 'long_term_variance',
                    'detection_count', 'consistent_detections', 'large_jumps_count',
                    'stability_classification', 'head_angle_deg', 'head_tracking_active',
                    'detection_method'
                ])
            self.csv_initialized = True
            print(f"✅ CSV logging to: {self.csv_log_filename}")
        except Exception as e:
            print(f"⚠️ CSV initialization failed: {e}")
    
    def reset_tracking_data(self):
        """Reset all tracking data for new session"""
        self.detection_count = 0
        self.consistent_detection_count = 0
        self.large_jumps_count = 0
        self.last_x_midpoint = None
        
        # Clear tracking data
        self.x_midpoints_pixels.clear()
        self.x_midpoints_normalized.clear()
        self.variance_window.clear()
        self.short_term_variance.clear()
        self.medium_term_variance.clear()
        self.long_term_variance.clear()
        
        # Reset stability zones
        for zone in self.stability_zones:
            self.stability_zones[zone] = 0
    
    def get_person_detection_simple(self):
        """
        SIMPLE person detection using robot's camera sensor with minimal processing
        """
        try:
            if not hasattr(self.robot, 'camera_sensor') or not self.robot.camera_sensor:
                return None
            
            reading = self.robot.camera_sensor.get_reading()
            if not reading or not hasattr(reading, 'get_people_locations'):
                return None
            
            people = reading.get_people_locations()
            if not people:
                return None
            
            # Use first person with very low confidence threshold
            closest_person = people[0]
            
            # VERY LOW threshold for maximum consistency
            if closest_person.confidence < 0.3:
                return None
            
            # SIMPLE bounding box calculation - no complex filtering
            x_center_normalized = (closest_person.xmax + closest_person.xmin) / 2.0
            y_center_normalized = (closest_person.ymax + closest_person.ymin) / 2.0
            
            x_center_pixels = int(x_center_normalized * self.screen.get_width())
            y_center_pixels = int(y_center_normalized * self.screen.get_height())
            
            return {
                'x_camera': closest_person.spatialCoordinates.x,
                'z_camera': closest_person.spatialCoordinates.z,
                'confidence': closest_person.confidence,
                'bbox_center': {
                    'x_normalized': x_center_normalized,
                    'y_normalized': y_center_normalized,
                    'x_pixels': x_center_pixels,
                    'y_pixels': y_center_pixels
                },
                'bounding_box': {
                    'xmin': closest_person.xmin,
                    'xmax': closest_person.xmax,
                    'ymin': closest_person.ymin,
                    'ymax': closest_person.ymax
                },
                'detection_method': 'SIMPLE_SENSOR'
            }
            
        except Exception:
            return None
    
    def update(self) -> Status:
        """
        py_trees Lifecycle: update() - Called during behavior execution
        PROPER py_trees implementation with correct exit handling
        """
        try:
            self.update_counter += 1
            
            # Handle pygame events - PROPER exit handling
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        print("🔄 ESC pressed - returning to IDLE mode")
                        # PROPER py_trees exit: Return SUCCESS to trigger terminate()
                        return Status.SUCCESS
                elif event.type == pygame.QUIT:
                    print("🔄 Window closed - returning to IDLE mode")
                    return Status.SUCCESS
            
            # Handle person detection (NO head tracking for consistency testing)
            person_data = self.get_person_detection_simple()
            if person_data:
                self.log_detection_consistency_to_csv(person_data)
                self.last_person_detected = time.time()
            
            # Display update
            if self.update_counter % self.display_update_rate == 0:
                try:
                    self.update_display(person_data)
                except Exception as e:
                    print(f"Display error: {e}")
            
            # Grid overlay update (every 8 frames)
            if self.show_grid and self.update_counter % 8 == 0:
                self.frame_counter += 1
                self.update_potential_field_grid(person_data)
            
            # PROPER py_trees: Return RUNNING to continue
            return Status.RUNNING
            
        except Exception as e:
            print(f"Update error: {e}")
            # PROPER py_trees: Return SUCCESS to exit on error
            return Status.SUCCESS
    
    def update_display(self, person_data):
        """Update pygame display"""
        if not self.screen:
            return
        
        # Clear and draw base interface
        self.screen.fill((0, 0, 0))
        self.draw_radar_grid()
        self.draw_robot()
        
        # Draw LiDAR obstacles
        obstacle_count = 0
        if self.lidar_system:
            obstacles = self.lidar_system.get_display_obstacles()
            if obstacles:
                obstacle_count = self.draw_lidar_data(obstacles)
        
        # Draw person detection
        if person_data:
            self.draw_person_detection(person_data)
        
        # Draw grid overlay
        if self.show_grid:
            self.draw_grid_lines()
            self.draw_cached_potential_numbers()
        
        # Draw info
        self.draw_info(obstacle_count)
        
        pygame.display.flip()
    
    def update_potential_field_grid(self, person_data):
        """Update potential field grid for overlay"""
        try:
            if person_data:
                # Simple grid update around person position
                x_camera = person_data['x_camera']
                z_camera = person_data['z_camera']
                
                if z_camera > 0:
                    # Calculate person position on radar
                    person_angle_rad = math.atan2(x_camera, z_camera)
                    person_angle_deg = math.degrees(person_angle_rad)
                    
                    display_angle_deg = 360 - person_angle_deg
                    while display_angle_deg < 0:
                        display_angle_deg += 360
                    while display_angle_deg >= 360:
                        display_angle_deg -= 360
                    
                    distance_m = z_camera / 1000.0
                    display_angle_rad = math.radians(90 - display_angle_deg)
                    
                    person_x = self.center_x + int(distance_m * self.scale * math.cos(display_angle_rad))
                    person_y = self.center_y - int(distance_m * self.scale * math.sin(display_angle_rad))
                    
                    # Create simple grid around person
                    grid_values = {}
                    
                    # Add attraction around person
                    for dx in [-40, 0, 40]:
                        for dy in [-40, 0, 40]:
                            px = person_x + dx
                            py = person_y + dy
                            
                            if 0 <= px < self.screen.get_width() and 0 <= py < self.screen.get_height():
                                grid_col = px // self.grid_size
                                grid_row = py // self.grid_size
                                
                                if dx == 0 and dy == 0:
                                    grid_values[(grid_col, grid_row)] = "+9"
                                else:
                                    grid_values[(grid_col, grid_row)] = "+5"
                    
                    self.last_potential_grid = grid_values
        except Exception:
            pass
    
    def draw_clean_interface(self):
        """Draw clean radar interface"""
        self.screen.fill((0, 0, 0))
        self.draw_radar_grid()
        self.draw_robot()
    
    def draw_radar_grid(self):
        """Draw radar-style grid"""
        for distance in [1000, 2000, 3000, 4000, 5000, 6000]:
            radius = distance * self.scale // 1000
            if radius < min(self.center_x, self.center_y) - 50:
                line_width = 3 if distance == 6000 else 2
                color = (0, 150, 0) if distance < 6000 else (255, 255, 0)
                pygame.draw.circle(self.screen, color, (self.center_x, self.center_y), radius, line_width)
        
        for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
            display_angle_rad = math.radians(90 - angle)
            line_length = min(self.center_x, self.center_y) - 80
            end_x = self.center_x + int(line_length * math.cos(display_angle_rad))
            end_y = self.center_y - int(line_length * math.sin(display_angle_rad))
            line_width = 3 if angle % 90 == 0 else 1
            pygame.draw.line(self.screen, (0, 150, 0), (self.center_x, self.center_y), (end_x, end_y), line_width)
    
    def draw_robot(self):
        """Draw robot representation"""
        pygame.draw.circle(self.screen, (0, 255, 0), (self.center_x, self.center_y), 15, 3)
        arrow_end_x = self.center_x
        arrow_end_y = self.center_y - 30
        pygame.draw.line(self.screen, (0, 255, 0), (self.center_x, self.center_y), (arrow_end_x, arrow_end_y), 5)
    
    def draw_lidar_data(self, obstacles):
        """Draw LiDAR obstacles"""
        if not obstacles:
            return 0
        
        obstacle_count = 0
        for angle, distance in obstacles:
            corrected_angle_rad = math.radians(90 - angle)
            
            x = self.center_x + math.cos(corrected_angle_rad) * (distance * self.scale // 1000)
            y = self.center_y - math.sin(corrected_angle_rad) * (distance * self.scale // 1000)
            
            if (0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height()):
                pygame.draw.circle(self.screen, (255, 255, 255), (int(x), int(y)), 2)
                obstacle_count += 1
        
        return obstacle_count
    
    def draw_person_detection(self, person_data):
        """Draw person detection"""
        try:
            x_camera = person_data['x_camera']
            z_camera = person_data['z_camera']
            confidence = person_data['confidence']
            bbox_center = person_data['bbox_center']
            
            if z_camera <= 0:
                return
            
            # Calculate angle and position for radar display
            person_angle_rad = math.atan2(x_camera, z_camera)
            person_angle_deg = math.degrees(person_angle_rad)
            
            display_angle_deg = 360 - person_angle_deg
            while display_angle_deg < 0:
                display_angle_deg += 360
            while display_angle_deg >= 360:
                display_angle_deg -= 360
            
            distance_m = z_camera / 1000.0
            display_angle_rad = math.radians(90 - display_angle_deg)
            
            x = self.center_x + int(distance_m * self.scale * math.cos(display_angle_rad))
            y = self.center_y - int(distance_m * self.scale * math.sin(display_angle_rad))
            
            # Color-code by stability
            stability_class = self.classify_stability(self.current_std_dev_pixels)
            if stability_class == 'stable':
                color = (0, 255, 0)      # Green
            elif stability_class == 'moderate':
                color = (255, 255, 0)    # Yellow
            elif stability_class == 'unstable':
                color = (255, 165, 0)    # Orange
            else:
                color = (255, 0, 0)      # Red
            
            # Draw detection
            if 0 <= x < self.screen.get_width() and 0 <= y < self.screen.get_height():
                pygame.draw.circle(self.screen, color, (x, y), 8)
                pygame.draw.circle(self.screen, (255, 255, 255), (x, y), 9, 2)
            
            # Draw large x-coordinate information
            self.draw_person_x_coordinate_info(bbox_center)
            
        except Exception:
            pass
    
    def draw_grid_lines(self):
        """Draw grid lines overlay"""
        try:
            grid_color = (100, 100, 100)  # Gray overlay
            screen_width = self.screen.get_width()
            screen_height = self.screen.get_height()
            
            # Vertical grid lines
            for x in range(0, screen_width, self.grid_size):
                pygame.draw.line(self.screen, grid_color, (x, 0), (x, screen_height), 1)
            
            # Horizontal grid lines
            for y in range(0, screen_height, self.grid_size):
                pygame.draw.line(self.screen, grid_color, (0, y), (screen_width, y), 1)
                
        except Exception:
            pass
    
    def draw_cached_potential_numbers(self):
        """Draw potential field numbers"""
        try:
            font = pygame.font.Font(None, 24)
            
            for (grid_col, grid_row), text in self.last_potential_grid.items():
                grid_center_x = grid_col * self.grid_size + self.grid_size // 2
                grid_center_y = grid_row * self.grid_size + self.grid_size // 2
                
                # Choose color based on text
                if text.startswith('+'):
                    color = (0, 255, 0)  # Green for attraction
                else:
                    color = (255, 0, 0)  # Red for repulsion
                
                # Draw text with background
                text_surface = font.render(text, True, color)
                text_rect = text_surface.get_rect(center=(grid_center_x, grid_center_y))
                
                bg_rect = text_rect.inflate(6, 4)
                pygame.draw.rect(self.screen, (0, 0, 0), bg_rect)
                self.screen.blit(text_surface, text_rect)
                
        except Exception:
            pass
    
    def draw_person_x_coordinate_info(self, bbox_center):
        """Draw person's x-coordinate info"""
        try:
            screen_width = self.screen.get_width()
            
            x_norm = bbox_center['x_normalized']
            x_pixels = bbox_center['x_pixels']
            
            large_font = pygame.font.Font(None, 72)
            medium_font = pygame.font.Font(None, 48)
            
            main_text = f"Person X-Center: {x_pixels}px (HEAD TRACKING DISABLED)"
            detail_text = f"Normalized: {x_norm:.3f} | Variance: ±{self.current_std_dev_pixels:.1f}px"
            consistency_text = f"Detections: {self.detection_count} | Consistent: {self.consistent_detection_count} | Jumps: {self.large_jumps_count}"
            
            # Color based on consistency
            if self.current_std_dev_pixels <= 10:
                main_color = (0, 255, 0)      # Green
            elif self.current_std_dev_pixels <= 25:
                main_color = (255, 255, 0)    # Yellow
            elif self.current_std_dev_pixels <= 50:
                main_color = (255, 165, 0)    # Orange
            else:
                main_color = (255, 0, 0)      # Red
            
            # Render and position text
            main_surface = large_font.render(main_text, True, main_color)
            detail_surface = medium_font.render(detail_text, True, (255, 255, 255))
            consistency_surface = medium_font.render(consistency_text, True, (0, 255, 255))
            
            main_x = (screen_width - main_surface.get_width()) // 2
            main_y = 50
            
            detail_x = (screen_width - detail_surface.get_width()) // 2
            detail_y = main_y + main_surface.get_height() + 10
            
            consistency_x = (screen_width - consistency_surface.get_width()) // 2
            consistency_y = detail_y + detail_surface.get_height() + 10
            
            # Draw background
            total_height = main_surface.get_height() + detail_surface.get_height() + consistency_surface.get_height() + 30
            max_width = max(main_surface.get_width(), detail_surface.get_width(), consistency_surface.get_width())
            status_bg = pygame.Rect(main_x - 10, main_y - 5, max_width + 20, total_height)
            
            pygame.draw.rect(self.screen, (0, 0, 0), status_bg)
            pygame.draw.rect(self.screen, main_color, status_bg, 3)
            
            # Draw text
            self.screen.blit(main_surface, (main_x, main_y))
            self.screen.blit(detail_surface, (detail_x, detail_y))
            self.screen.blit(consistency_surface, (consistency_x, consistency_y))
            
        except Exception:
            pass
    
    def draw_info(self, obstacle_count):
        """Draw test information"""
        try:
            lidar_status = "ACTIVE" if self.lidar_system else "INACTIVE"
            consistency_rate = (self.consistent_detection_count / max(1, self.detection_count)) * 100
            
            info_lines = [
                f"PROPER PY_TREES LIDAR TEST - LiDAR: {lidar_status} | Head Tracking: DISABLED",
                f"LiDAR Obstacles: {obstacle_count} | Detection Method: SIMPLE_SENSOR",
                f"Detection Consistency: {consistency_rate:.1f}% | Variance: ±{self.current_std_dev_pixels:.1f}px",
                f"Detections: {self.detection_count} | Large Jumps: {self.large_jumps_count}",
                f"CSV Logging: {self.csv_log_filename} | ESC: Exit to Idle Mode"
            ]
            
            y_offset = self.screen.get_height() - 150
            font = pygame.font.Font(None, 36)
            
            for i, line in enumerate(info_lines):
                if i == 0:
                    color = (0, 255, 255)  # Cyan for header
                elif "Consistency" in line:
                    if consistency_rate > 90:
                        color = (0, 255, 0)
                    elif consistency_rate > 80:
                        color = (255, 255, 0)
                    else:
                        color = (255, 0, 0)
                else:
                    color = (255, 255, 255)
                
                text_surface = font.render(line, True, color)
                self.screen.blit(text_surface, (20, y_offset + i * 30))
            
        except Exception:
            pass
    
    # Variance calculation methods (same as before)
    def calculate_x_midpoint_variance(self):
        """Calculate real-time X midpoint variance"""
        try:
            if len(self.variance_window) < 2:
                return 0.0, 0.0, 0.0
            
            window_data = list(self.variance_window)
            mean_val = statistics.mean(window_data)
            variance_val = statistics.variance(window_data) if len(window_data) > 1 else 0.0
            std_dev_val = math.sqrt(variance_val)
            
            return variance_val, std_dev_val, mean_val
        except Exception:
            return 0.0, 0.0, 0.0
    
    def calculate_multi_term_variance(self, x_pixel):
        """Calculate multi-term variance"""
        self.short_term_variance.append(x_pixel)
        self.medium_term_variance.append(x_pixel)
        self.long_term_variance.append(x_pixel)
        
        try:
            short_var = statistics.variance(list(self.short_term_variance)) if len(self.short_term_variance) > 1 else 0.0
            medium_var = statistics.variance(list(self.medium_term_variance)) if len(self.medium_term_variance) > 1 else 0.0
            long_var = statistics.variance(list(self.long_term_variance)) if len(self.long_term_variance) > 1 else 0.0
            
            return short_var, medium_var, long_var
        except Exception:
            return 0.0, 0.0, 0.0
    
    def classify_stability(self, std_dev_pixels):
        """Classify detection stability"""
        if std_dev_pixels < 10:
            return 'stable'
        elif std_dev_pixels < 25:
            return 'moderate'
        elif std_dev_pixels < 50:
            return 'unstable'
        else:
            return 'very_unstable'
    
    def log_detection_consistency_to_csv(self, person_data):
        """Log detection consistency data"""
        try:
            if not self.csv_initialized:
                self.initialize_csv_log()
            
            bbox_center = person_data['bbox_center']
            x_midpoint_pixels = bbox_center['x_pixels']
            x_midpoint_normalized = bbox_center['x_normalized']
            
            # Track midpoints for variance analysis
            self.x_midpoints_pixels.append(x_midpoint_pixels)
            self.x_midpoints_normalized.append(x_midpoint_normalized)
            self.variance_window.append(x_midpoint_pixels)
            
            # Calculate jump from previous detection
            x_jump = 0
            is_large_jump = False
            if self.last_x_midpoint is not None:
                x_jump = abs(x_midpoint_pixels - self.last_x_midpoint)
                is_large_jump = x_jump > self.jump_threshold_pixels
                if is_large_jump:
                    self.large_jumps_count += 1
            
            self.last_x_midpoint = x_midpoint_pixels
            
            # Update detection counts
            self.detection_count += 1
            if not is_large_jump:
                self.consistent_detection_count += 1
            
            # Calculate variance metrics
            self.current_variance_pixels, self.current_std_dev_pixels, self.current_mean_pixels = self.calculate_x_midpoint_variance()
            
            # Calculate multi-term variance
            short_var, medium_var, long_var = self.calculate_multi_term_variance(x_midpoint_pixels)
            
            # Classify stability
            stability_class = self.classify_stability(self.current_std_dev_pixels)
            self.stability_zones[stability_class] += 1
            
            # Calculate mode elapsed time
            mode_elapsed = time.time() - self.mode_start_time
            
            # Write to CSV
            with open(self.csv_log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    mode_elapsed, time.time(), self.update_counter,
                    x_midpoint_pixels, x_midpoint_normalized,
                    person_data['x_camera'], person_data['z_camera'], person_data['confidence'],
                    person_data['bounding_box']['xmin'], person_data['bounding_box']['ymin'],
                    person_data['bounding_box']['xmax'], person_data['bounding_box']['ymax'],
                    person_data['bounding_box']['xmax'] - person_data['bounding_box']['xmin'],
                    person_data['bounding_box']['ymax'] - person_data['bounding_box']['ymin'],
                    x_jump, is_large_jump,
                    self.current_variance_pixels, self.current_std_dev_pixels, self.current_mean_pixels,
                    short_var, medium_var, long_var,
                    self.detection_count, self.consistent_detection_count, self.large_jumps_count,
                    stability_class, 0.0, False,  # Head tracking disabled
                    person_data.get('detection_method', 'SIMPLE_SENSOR')
                ])
        except Exception:
            pass
    
    def stop_robot(self):
        """Stop robot movement"""
        try:
            if not self.robot:
                return
                
            velocity_manager = None
            if hasattr(self.robot, 'direct_velocity_manager') and self.robot.direct_velocity_manager:
                velocity_manager = self.robot.direct_velocity_manager
            elif hasattr(self.robot, 'velocity_manager') and self.robot.velocity_manager:
                velocity_manager = self.robot.velocity_manager
            
            if velocity_manager:
                stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                velocity_manager.perform_action(stop_config)
                
        except Exception:
            pass
    
    def center_head_for_idle_mode(self):
        """Center head for idle mode"""
        try:
            if not self.robot:
                return
                
            if hasattr(self.robot, 'servo_controller') and self.robot.servo_controller:
                self.robot.servo_controller.center()
                time.sleep(0.5)
            elif hasattr(self.robot, 'head_velocity_manager') and self.robot.head_velocity_manager:
                self.robot.head_velocity_manager.center_head()
                time.sleep(0.5)
                    
        except Exception:
            pass
    
    def terminate(self, new_status: Status):
        """
        py_trees Lifecycle: terminate() - Called when behavior STOPS running
        PROPER cleanup and return to IDLE mode
        """
        print("🔄 LiDAR Test terminating...")
        
        try:
            # Generate final CSV summary
            if self.detection_count > 0:
                overall_variance = statistics.variance(list(self.x_midpoints_pixels)) if len(self.x_midpoints_pixels) > 1 else 0
                overall_std_dev = math.sqrt(overall_variance)
                consistency_rate = (self.consistent_detection_count / self.detection_count) * 100
                
                print(f"\n📊 PROPER PY_TREES CONSISTENCY SUMMARY:")
                print(f"   Detection Method: SIMPLE_SENSOR (Head tracking disabled)")
                print(f"   Total Detections: {self.detection_count}")
                print(f"   Consistency Rate: {consistency_rate:.1f}%")
                print(f"   Overall Std Dev: ±{overall_std_dev:.2f} pixels")
                print(f"   Large Jumps: {self.large_jumps_count}")
                print(f"   CSV Data: {self.csv_log_filename}")
            
            # Stop robot movement
            self.stop_robot()
            
            # Center head
            self.center_head_for_idle_mode()
            
            # Stop LiDAR system
            if self.lidar_system:
                print("🛑 Stopping LiDAR system...")
                self.lidar_system.stop()
                self.lidar_system = None
                print("✅ LiDAR system stopped")
            
            # Clear pygame events
            if pygame.get_init():
                pygame.event.clear()
            
            # Restore facial animation
            if self.facial_restorer and self.robot:
                self.facial_restorer.restore_resting_face_immediately(self.robot)
            
            # Clean up blackboard
            try:
                if self.blackboard.exists("LIDAR_SYSTEM"):
                    self.blackboard.unset("LIDAR_SYSTEM")
            except Exception:
                pass
            
            # PROPER py_trees: Set robot mode to IDLE
            if self.robot:
                print("🎯 Setting robot mode to IDLE")
                self.robot.set_mode(RobotMode.IDLE)
            
            print("✅ LiDAR Test terminated successfully")
            
        except Exception as e:
            print(f"⚠️ Termination error: {e}")
            # Ensure robot goes to IDLE even if cleanup fails
            try:
                if self.robot:
                    self.robot.set_mode(RobotMode.IDLE)
            except Exception:
                pass
        
        # Call parent terminate
        super().terminate(new_status)


# Use the proper implementation as the main class
PotentialFieldLidarTest = ProperPyTreesLidarTest