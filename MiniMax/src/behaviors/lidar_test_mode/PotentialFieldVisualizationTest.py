#!/usr/bin/env python3
"""
Optimized Potential Field Visualization Test - EXTENDS CoordinateVerificationLidarTest
High-performance version with minimal overhead and preserved head tracking

File: src/behaviors/lidar_test_mode/PotentialFieldVisualizationTest.py
"""
from __future__ import annotations

import pygame
import math
import time
import threading
import queue
import py_trees
import csv
import os
import statistics
import numpy as np
import cv2
from collections import deque
from datetime import datetime
from py_trees.common import Status
from pyrplidar import PyRPlidar

from src.behaviors.MaxineBehavior import MaxineBehavior
from src.types.RobotModes import RobotMode
from src.types.MovementDirection import MovementDirection
from src.action_managers.VelocityManager import VelocityConfig

# Import existing working coordinate verification system
from .CoordinateVerificationLidarTest import CoordinateVerificationLidarTest

# Import potential field navigation
from .PotentialFieldNavigation import PotentialFieldNavigator, PotentialFieldConfig, Vector2D

try:
    import depthai as dai
except ImportError:
    dai = None

class PotentialFieldVisualizationTest(CoordinateVerificationLidarTest):
    """
    OPTIMIZED potential field visualization - preserves ALL existing functionality
    """
    
    def __init__(self):
        # Initialize the working base system first
        super().__init__()
        
        # Override behavior name and CSV filename
        self.name = "Potential Field Visualization Test"
        self.csv_log_filename = "POTENTIAL_FIELD_NAVIGATION.csv"
        
        # PERFORMANCE OPTIMIZATIONS
        self.display_update_rate = 6  # Update display less frequently (was 3)
        
        # Simplified potential field system (NEW)
        self.potential_field_config = PotentialFieldConfig(
            attractive_strength=2.0,
            repulsive_strength=1.5,
            target_distance=1500.0,
            repulsive_max_distance=800.0,
            max_linear_speed=0.6,
            max_angular_speed=0.4,
            movement_threshold=0.1,
            stop_at_target=True
        )
        self.navigator = PotentialFieldNavigator(self.potential_field_config)
        
        # OPTIMIZED Grid visualization (3x3 = 300mm spacing, not 100mm)
        self.grid_size = 300  # 300mm grid cells (much less dense)
        self.grid_extent = 2400  # Smaller extent for performance
        self.potential_field_cache = {}
        self.last_field_calculation = 0.0
        self.field_calculation_interval = 0.3  # Update field every 300ms for responsiveness
        
        # Minimal navigation state tracking
        self.navigation_enabled = True
        self.last_navigation_metrics = {}
    
    def initialize_csv_log(self):
        """Enhanced CSV with navigation metrics - SAME columns as parent plus nav data"""
        try:
            if os.path.exists(self.csv_log_filename):
                os.remove(self.csv_log_filename)
                
            with open(self.csv_log_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    # EXISTING columns from parent
                    'timestamp', 'mode_time_elapsed',
                    'person_detected', 'person_x_camera', 'person_z_camera', 'person_confidence',
                    'person_robot_angle_deg', 'person_robot_distance', 'person_transform_confidence',
                    'head_angle_deg', 'head_tracking_active',
                    'lidar_obstacles_count', 'closest_lidar_angle_deg', 'closest_lidar_distance',
                    'person_lidar_angle_diff_deg', 'person_lidar_distance_diff', 'coordinates_aligned',
                    'alignment_score', 'detection_quality',
                    # NEW navigation columns (minimal)
                    'nav_total_force', 'nav_attractive_force', 'nav_repulsive_force',
                    'nav_movement_command', 'nav_decision', 'person_in_target_zone', 'obstacles_filtered'
                ])
            self.csv_initialized = True
            print(f"✅ Optimized potential field CSV: {self.csv_log_filename}")
        except Exception as e:
            print(f"⚠️ CSV initialization failed: {e}")
    
    def calculate_potential_field_grid(self):
        """ENHANCED potential field with person filtering, force spreading, and proper force summation"""
        try:
            current_time = time.time()
            if current_time - self.last_field_calculation < self.field_calculation_interval:
                return
            
            self.last_field_calculation = current_time
            self.potential_field_cache.clear()
            
            # Get person position first for filtering
            person_screen_x = None
            person_screen_y = None
            
            if self._cached_person_data:
                person_data = self._cached_person_data
                x_camera = person_data['x_camera']
                z_camera = person_data['z_camera']
                
                if z_camera > 0:
                    # Calculate person screen position
                    person_angle_rad = math.atan2(x_camera, z_camera)
                    person_angle_deg = math.degrees(person_angle_rad)
                    display_angle_deg = 360 - person_angle_deg
                    while display_angle_deg < 0:
                        display_angle_deg += 360
                    while display_angle_deg >= 360:
                        display_angle_deg -= 360
                    
                    distance_m = z_camera / 1000.0
                    display_angle_rad = math.radians(90 - display_angle_deg)
                    person_screen_x = self.center_x + int(distance_m * self.scale * math.cos(display_angle_rad))
                    person_screen_y = self.center_y - int(distance_m * self.scale * math.sin(display_angle_rad))
            
            # 1. FILTER OBSTACLES - Remove any within 300mm of person
            filtered_obstacles = []
            if self.lidar_system and person_screen_x is not None:
                raw_obstacles = self.lidar_system.get_display_obstacles()
                for angle, distance in raw_obstacles:
                    # Calculate obstacle screen position
                    corrected_angle_rad = math.radians(90 - angle)
                    obs_screen_x = self.center_x + math.cos(corrected_angle_rad) * (distance * self.scale // 1000)
                    obs_screen_y = self.center_y - math.sin(corrected_angle_rad) * (distance * self.scale // 1000)
                    
                    # Check distance from person (in screen pixels, convert to mm)
                    screen_distance = math.sqrt((obs_screen_x - person_screen_x)**2 + (obs_screen_y - person_screen_y)**2)
                    world_distance = screen_distance * 1000 / self.scale  # Convert back to mm
                    
                    # Only keep obstacles more than 300mm from person
                    if world_distance > 300:
                        filtered_obstacles.append((angle, distance, obs_screen_x, obs_screen_y))
            
            # 2. CREATE REPULSIVE FORCES AROUND FILTERED OBSTACLES (with spreading)
            for angle, distance, obs_screen_x, obs_screen_y in filtered_obstacles:
                # Convert obstacle screen position to grid coordinates
                obs_grid_x = int(round((obs_screen_x - self.center_x) / (self.grid_size * self.scale // 1000)))
                obs_grid_y = int(round((self.center_y - obs_screen_y) / (self.grid_size * self.scale // 1000)))
                
                # Create spreading repulsive force (2 cells radius)
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        gx = obs_grid_x + dx
                        gy = obs_grid_y + dy
                        
                        distance_from_obstacle = math.sqrt(dx*dx + dy*dy)
                        if distance_from_obstacle == 0:
                            repulsive_value = 10  # Strongest at obstacle
                        elif distance_from_obstacle <= 1:
                            repulsive_value = 8   # Strong adjacent
                        elif distance_from_obstacle <= 1.5:
                            repulsive_value = 6   # Medium diagonal
                        else:
                            repulsive_value = 4   # Weak at edges
                        
                        # Add to grid (sum with existing values)
                        if (gx, gy) in self.potential_field_cache:
                            self.potential_field_cache[(gx, gy)] += repulsive_value
                        else:
                            self.potential_field_cache[(gx, gy)] = repulsive_value
            
            # 3. CREATE ATTRACTIVE FORCES AROUND PERSON (with spreading)
            if person_screen_x is not None:
                # Convert person screen position to grid coordinates
                person_grid_x = int(round((person_screen_x - self.center_x) / (self.grid_size * self.scale // 1000)))
                person_grid_y = int(round((self.center_y - person_screen_y) / (self.grid_size * self.scale // 1000)))
                
                # Create spreading attractive force (2 cells radius)
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        gx = person_grid_x + dx
                        gy = person_grid_y + dy
                        
                        distance_from_person = math.sqrt(dx*dx + dy*dy)
                        if distance_from_person == 0:
                            attractive_value = -12  # Strongest at person
                        elif distance_from_person <= 1:
                            attractive_value = -10  # Strong adjacent
                        elif distance_from_person <= 1.5:
                            attractive_value = -8   # Medium diagonal
                        else:
                            attractive_value = -6   # Weak at edges
                        
                        # Add to grid (sum with existing values)
                        if (gx, gy) in self.potential_field_cache:
                            self.potential_field_cache[(gx, gy)] += attractive_value
                        else:
                            self.potential_field_cache[(gx, gy)] = attractive_value
            
            # 4. FILTER OUT INSIGNIFICANT VALUES
            # Remove entries with very small absolute values
            self.potential_field_cache = {k: v for k, v in self.potential_field_cache.items() if abs(v) >= 2}
                    
        except Exception as e:
            print(f"Field calculation error: {e}")
    
    def draw_potential_field_grid(self):
        """Display summed potential field values - red for positive, green for negative"""
        try:
            if not self.potential_field_cache:
                return
            
            font = pygame.font.Font(None, 32)  # Clear, readable font
            
            for (grid_x, grid_y), potential in self.potential_field_cache.items():
                # Convert grid coordinates to screen coordinates (center of grid cell)
                world_x = grid_x * self.grid_size
                world_y = grid_y * self.grid_size
                
                screen_x = self.center_x + int(world_x * self.scale // 1000)
                screen_y = self.center_y - int(world_y * self.scale // 1000)
                
                # Check if within screen bounds
                if not (50 <= screen_x < self.screen.get_width() - 50 and 
                       50 <= screen_y < self.screen.get_height() - 50):
                    continue
                
                # Convert to integer for display
                display_val = int(abs(potential))  # Always show as positive number
                
                # Color based on SUMMED result: GREEN for negative (attractive), RED for positive (repulsive)
                if potential < 0:
                    color = (0, 255, 0)  # Bright green for net attractive forces
                else:
                    color = (255, 0, 0)  # Bright red for net repulsive forces
                
                text = str(display_val)
                
                # Only display significant values
                if display_val >= 2:
                    # Draw text with black outline for better visibility
                    outline_color = (0, 0, 0)
                    
                    # Draw text outline
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            if dx != 0 or dy != 0:
                                outline_surface = font.render(text, True, outline_color)
                                outline_rect = outline_surface.get_rect(center=(screen_x + dx, screen_y + dy))
                                self.screen.blit(outline_surface, outline_rect)
                    
                    # Draw main text
                    text_surface = font.render(text, True, color)
                    text_rect = text_surface.get_rect(center=(screen_x, screen_y))
                    self.screen.blit(text_surface, text_rect)
                        
        except Exception:
            pass  # Fail silently for performance
    
    def update_navigation_system(self, person_data):
        """ENHANCED navigation update with same obstacle filtering as potential field"""
        try:
            if not person_data:
                self.navigator.reset_navigation_state()
                return
            
            # Transform person to robot coordinates
            x_camera = person_data['x_camera']
            z_camera = person_data['z_camera']
            head_angle_rad = math.radians(self.get_head_angle_degrees())
            
            camera_angle = math.atan2(x_camera, z_camera) if z_camera > 0 else 0.0
            robot_angle = camera_angle + head_angle_rad
            
            # Normalize angle
            while robot_angle > math.pi:
                robot_angle -= 2 * math.pi
            while robot_angle < -math.pi:
                robot_angle += 2 * math.pi
            
            robot_distance = max(100, z_camera - 130)
            self.navigator.set_person_position(robot_angle, robot_distance)
            
            # Get obstacles and FILTER using same 300mm criteria as potential field
            filtered_obstacles = []
            if self.lidar_system:
                raw_obstacles = self.lidar_system.get_display_obstacles()
                
                # Calculate person screen position for filtering
                person_angle_rad = math.atan2(x_camera, z_camera)
                person_angle_deg = math.degrees(person_angle_rad)
                display_angle_deg = 360 - person_angle_deg
                while display_angle_deg < 0:
                    display_angle_deg += 360
                while display_angle_deg >= 360:
                    display_angle_deg -= 360
                
                distance_m = z_camera / 1000.0
                display_angle_rad = math.radians(90 - display_angle_deg)
                person_screen_x = self.center_x + int(distance_m * self.scale * math.cos(display_angle_rad))
                person_screen_y = self.center_y - int(distance_m * self.scale * math.sin(display_angle_rad))
                
                for lidar_angle_deg, distance in raw_obstacles:
                    # Calculate obstacle screen position
                    corrected_angle_rad = math.radians(90 - lidar_angle_deg)
                    obs_screen_x = self.center_x + math.cos(corrected_angle_rad) * (distance * self.scale // 1000)
                    obs_screen_y = self.center_y - math.sin(corrected_angle_rad) * (distance * self.scale // 1000)
                    
                    # Check distance from person (same calculation as potential field)
                    screen_distance = math.sqrt((obs_screen_x - person_screen_x)**2 + (obs_screen_y - person_screen_y)**2)
                    world_distance = screen_distance * 1000 / self.scale
                    
                    # Only keep obstacles more than 300mm from person
                    if world_distance > 300:
                        # Convert to robot coordinates for navigation
                        obs_angle_rad = math.radians(lidar_angle_deg)
                        while obs_angle_rad > math.pi:
                            obs_angle_rad -= 2 * math.pi
                        while obs_angle_rad < -math.pi:
                            obs_angle_rad += 2 * math.pi
                        
                        filtered_obstacles.append((obs_angle_rad, distance))
            
            self.navigator.set_obstacles(filtered_obstacles)
            
            # Calculate navigation (lightweight)
            if self.navigation_enabled:
                movement_direction, speed_factor = self.navigator.calculate_navigation_command()
                nav_metrics = self.navigator.get_navigation_metrics()
                self.last_navigation_metrics = {
                    'total_force': nav_metrics.get('total_force_magnitude', 0.0),
                    'attractive_force': nav_metrics.get('attractive_force_magnitude', 0.0),
                    'repulsive_force': nav_metrics.get('repulsive_force_magnitude', 0.0),
                    'movement_command': movement_direction.name,
                    'decision': nav_metrics.get('navigation_decision', 'none'),
                    'in_target_zone': abs(robot_distance - 1500) < 200,
                    'obstacles_filtered': len(filtered_obstacles)  # Track filtered count
                }
            
            # Update field visualization
            self.calculate_potential_field_grid()
            
        except Exception:
            pass  # Fail silently for performance
    
    def log_detection_consistency_to_csv(self, person_data):
        """OPTIMIZED CSV logging - calls parent method then adds minimal nav data"""
        try:
            # Do ALL the parent logging first (this preserves existing functionality)
            # We'll recreate the parent's CSV structure manually to avoid calling the parent method
            # since we need different columns
            
            if not self.csv_initialized:
                self.initialize_csv_log()
            
            # Standard detection tracking (from parent)
            bbox_center = person_data['bbox_center']
            x_midpoint_pixels = bbox_center['x_pixels']
            self.x_midpoints_pixels.append(x_midpoint_pixels)
            self.variance_window.append(x_midpoint_pixels)
            
            is_large_jump = False
            if self.last_x_midpoint is not None:
                x_jump = abs(x_midpoint_pixels - self.last_x_midpoint)
                is_large_jump = x_jump > self.jump_threshold_pixels
                if is_large_jump:
                    self.large_jumps_count += 1
            self.last_x_midpoint = x_midpoint_pixels
            self.detection_count += 1
            if not is_large_jump:
                self.consistent_detection_count += 1
            
            self.current_variance_pixels, self.current_std_dev_pixels, self.current_mean_pixels = self.calculate_x_midpoint_variance()
            
            # Coordinate verification (from parent)
            current_time = time.time()
            mode_elapsed = current_time - self.mode_start_time
            head_angle_deg = self.get_head_angle_degrees()
            head_angle_rad = math.radians(head_angle_deg)
            
            x_camera = person_data['x_camera']
            z_camera = person_data['z_camera']
            camera_angle = math.atan2(x_camera, z_camera) if z_camera > 0 else 0.0
            robot_angle = camera_angle + head_angle_rad
            while robot_angle > math.pi:
                robot_angle -= 2 * math.pi
            while robot_angle < -math.pi:
                robot_angle += 2 * math.pi
            robot_angle_deg = math.degrees(robot_angle)
            robot_distance = max(100, z_camera - 130)
            
            # LiDAR analysis (simplified from parent)
            raw_lidar_obstacles = []
            if self.lidar_system:
                raw_lidar_obstacles = self.lidar_system.get_display_obstacles()
            
            closest_lidar_angle = 0
            closest_lidar_distance = 0
            angle_diff = 999
            distance_diff = 9999
            coordinates_aligned = False
            alignment_score = 0.0
            detection_quality = 0
            
            if raw_lidar_obstacles:
                # Find best match (simplified)
                best_match = None
                best_score = 0
                for lidar_angle, lidar_distance in raw_lidar_obstacles[:5]:  # Check only first 5
                    lidar_norm = lidar_angle if lidar_angle <= 180 else lidar_angle - 360
                    ang_diff = abs(robot_angle_deg - lidar_norm)
                    if ang_diff > 180:
                        ang_diff = 360 - ang_diff
                    dist_diff = abs(robot_distance - lidar_distance)
                    
                    if ang_diff <= 20:
                        match_score = 1.0 / (1.0 + ang_diff + dist_diff/1000.0)
                        if match_score > best_score:
                            best_score = match_score
                            best_match = (lidar_angle, lidar_distance, ang_diff, dist_diff)
                
                if best_match:
                    closest_lidar_angle, closest_lidar_distance, angle_diff, distance_diff = best_match
                    coordinates_aligned = (angle_diff <= 5.0 and distance_diff <= 500)
                    alignment_score = max(0, 1.0 - (angle_diff + distance_diff/1000.0) / 20.0)
                    detection_quality = min(5, len(raw_lidar_obstacles))  # Cap at 5
            
            # Write to CSV with navigation data
            with open(self.csv_log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    current_time, mode_elapsed,
                    True, x_camera, z_camera, person_data.get('confidence', 0.0),
                    robot_angle_deg, robot_distance, 1.0,
                    head_angle_deg, self.head_tracking_enabled,
                    len(raw_lidar_obstacles), closest_lidar_angle, closest_lidar_distance,
                    angle_diff, distance_diff, coordinates_aligned, alignment_score, detection_quality,
                    # Navigation data (minimal)
                    self.last_navigation_metrics.get('total_force', 0.0),
                    self.last_navigation_metrics.get('attractive_force', 0.0),
                    self.last_navigation_metrics.get('repulsive_force', 0.0),
                    self.last_navigation_metrics.get('movement_command', 'NONE'),
                    self.last_navigation_metrics.get('decision', 'none'),
                    self.last_navigation_metrics.get('in_target_zone', False),
                    self.last_navigation_metrics.get('obstacles_filtered', 0)
                ])
                
        except Exception:
            pass  # Fail silently for performance
    
    def draw_info(self, obstacle_count):
        """DEBUG info display with obstacle filtering details"""
        try:
            lidar_status = "ACTIVE" if self.lidar_system else "INACTIVE"
            detection_status = "OPTIMIZED" if (self.detection_system and self.detection_system.camera_initialized) else "FALLBACK"
            consistency_rate = (self.consistent_detection_count / max(1, self.detection_count)) * 100
            
            if self.head_tracker and self.head_tracking_enabled:
                head_angle = self.get_head_angle_degrees()
                head_status = f"HEAD: {head_angle:.1f}° (TRACKING)"
            else:
                head_status = "HEAD: Disabled"
            
            # Enhanced navigation status
            nav_force = self.last_navigation_metrics.get('total_force', 0.0)
            nav_cmd = self.last_navigation_metrics.get('movement_command', 'NONE')
            obstacles_filtered = self.last_navigation_metrics.get('obstacles_filtered', 0)
            nav_status = f"NAV: {nav_cmd} | Force:{nav_force:.2f} | Obs:{obstacles_filtered}"
            
            # Calculate obstacle filtering stats
            total_obstacles = 0
            filtered_obstacles = 0
            if self.lidar_system:
                raw_obstacles = self.lidar_system.get_display_obstacles()
                total_obstacles = len(raw_obstacles)
                
                # Count how many would be filtered (same logic as in calculate_potential_field_grid)
                if self._cached_person_data:
                    person_data = self._cached_person_data
                    x_camera = person_data['x_camera']
                    z_camera = person_data['z_camera']
                    
                    if z_camera > 0:
                        person_angle_rad = math.atan2(x_camera, z_camera)
                        person_angle_deg = math.degrees(person_angle_rad)
                        display_angle_deg = 360 - person_angle_deg
                        while display_angle_deg < 0:
                            display_angle_deg += 360
                        while display_angle_deg >= 360:
                            display_angle_deg -= 360
                        
                        distance_m = z_camera / 1000.0
                        display_angle_rad = math.radians(90 - display_angle_deg)
                        person_screen_x = self.center_x + int(distance_m * self.scale * math.cos(display_angle_rad))
                        person_screen_y = self.center_y - int(distance_m * self.scale * math.sin(display_angle_rad))
                        
                        for angle, distance in raw_obstacles:
                            corrected_angle_rad = math.radians(90 - angle)
                            obs_screen_x = self.center_x + math.cos(corrected_angle_rad) * (distance * self.scale // 1000)
                            obs_screen_y = self.center_y - math.sin(corrected_angle_rad) * (distance * self.scale // 1000)
                            
                            screen_distance = math.sqrt((obs_screen_x - person_screen_x)**2 + (obs_screen_y - person_screen_y)**2)
                            world_distance = screen_distance * 1000 / self.scale
                            
                            if world_distance > 300:
                                filtered_obstacles += 1
            
            # Field status with filtering info
            field_cells = len(self.potential_field_cache)
            red_cells = sum(1 for v in self.potential_field_cache.values() if v > 0)
            green_cells = sum(1 for v in self.potential_field_cache.values() if v < 0)
            
            filtering_status = f"Obstacles: {total_obstacles} total, {filtered_obstacles} kept, {total_obstacles - filtered_obstacles} filtered (within 300mm)"
            field_status = f"Field: {field_cells} cells ({red_cells} red, {green_cells} green) | Forces summed"
            
            info_lines = [
                f"POTENTIAL FIELD NAV - Detection: {detection_status} | LiDAR: {lidar_status}",
                f"Consistency: {consistency_rate:.1f}% | {head_status}",
                f"Variance: ±{self.current_std_dev_pixels:.1f}px | Detections: {self.detection_count}",
                nav_status,
                filtering_status,
                field_status,
                f"RED=Net Repulsive | GREEN=Net Attractive | Forces spread 2 cells | ESC=Exit"
            ]
            
            y_offset = self.screen.get_height() - 210  # Space for all lines
            font = pygame.font.Font(None, 26)  # Smaller font for more lines
            
            for i, line in enumerate(info_lines):
                if i == 0:
                    color = (0, 255, 255)
                elif i == 3:  # Navigation line
                    color = (255, 255, 0) if nav_force > 0.1 else (128, 128, 128)
                elif i == 4:  # Filtering status line
                    color = (255, 128, 0)  # Orange for filtering info
                elif i == 5:  # Field status line
                    color = (255, 128, 255)  # Magenta for field info
                elif i == 6:  # Legend line
                    color = (200, 200, 200)
                elif "Variance" in line:
                    color = (0, 255, 0) if self.current_std_dev_pixels <= 37 else (255, 255, 0)
                else:
                    color = (255, 255, 255)
                
                text_surface = font.render(line, True, color)
                self.screen.blit(text_surface, (20, y_offset + i * 25))
                
        except Exception:
            pass
    
    def update(self) -> Status:
        """OPTIMIZED update method - minimal overhead"""
        try:
            if not self.initialized:
                if not self.initialize_components():
                    return Status.FAILURE
            
            self.update_counter += 1
            pygame.event.pump()

            # Detection and head tracking (UNCHANGED - preserve existing functionality)
            person_data = self.get_person_detection()
            if person_data:
                self._cached_person_data = person_data
                self._cache_t = time.time()
                
                # HEAD TRACKING (ensure this works exactly as parent)
                if self.head_tracking_enabled:
                    self.update_head_tracking(person_data)
                
                # Navigation update (lightweight)
                self.update_navigation_system(person_data)
            else:
                # Head tracking fallback (preserve parent behavior)
                if self._cached_person_data and (time.time() - self._cache_t) <= 0.25 and self.head_tracking_enabled:
                    self.update_head_tracking(self._cached_person_data)
                elif self.head_tracker and time.time() - self.last_person_detected > self.person_lost_timeout:
                    try:
                        self.angle_history.clear()
                        self.last_sent_angle = None
                        self.head_tracker.set_manual_position(0.0)
                    except Exception:
                        pass
                # Reset navigation
                self.navigator.reset_navigation_state()

            # OPTIMIZED display update (less frequent)
            if self.update_counter % self.display_update_rate == 0:
                try:
                    if self.screen:
                        self.screen.fill((0, 0, 0))
                        
                        # Draw radar grid with potential field overlay
                        self.draw_radar_grid()  # This now includes the grid overlay
                        self.draw_robot()  # From parent
                        
                        # LiDAR obstacles (from parent)
                        obstacle_count = 0
                        if self.lidar_system:
                            obstacles = self.lidar_system.get_display_obstacles()
                            if obstacles:
                                obstacle_count = self.draw_lidar_data(obstacles)
                        
                        # Person detection (from parent)
                        self.draw_person_detection(self._cached_person_data)
                        
                        # POTENTIAL FIELD VALUES (drawn after obstacles/person for visibility)
                        self.draw_potential_field_grid()
                        
                        # Info display
                        self.draw_info(obstacle_count)
                        
                        pygame.display.flip()
                except Exception:
                    pass  # Fail silently
            
            return Status.RUNNING
        except Exception:
            return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """Clean termination"""
        print("🔥 Potential Field Navigation terminating...")
        
        # Reset navigation system
        if hasattr(self, 'navigator'):
            self.navigator.reset_navigation_state()
        
        # Call parent termination (handles everything else)
        super().terminate(new_status)

# Export for use
__all__ = ['PotentialFieldVisualizationTest']