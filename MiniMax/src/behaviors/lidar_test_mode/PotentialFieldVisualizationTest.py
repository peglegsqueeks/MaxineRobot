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
    OPTIMIZED potential field visualization with Camera-LiDAR Calibration System
    
    CALIBRATION MODE:
    - Orange circle: Original camera-detected person position  
    - Blue circle: Offset-corrected position (should align with lidar)
    - Green circle: Manual adjustment marker (use WASD to move)
    - White dots: LiDAR obstacle detections
    
    CONTROLS:
    - WASD: Move green marker to align with lidar detection
    - X: Save calibration offset to offsetN.csv  
    - C: Calculate and apply offset from saved calibration data
    - ESC: Exit to IDLE mode
    
    CALIBRATION PROCEDURE:
    1. Person stands at fixed distance (2-3 meters)
    2. Test at multiple head angles: 0°, +30°, -30°, +45°, -45°
    3. Use WASD to align green circle with lidar detection (white dots)
    4. Press X to save offset for each head angle
    5. Press C to calculate average offset and apply correction
    6. Blue circle should now align with lidar detections
    
    The system will display the current applied offset at the top of the screen.
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
        
        # Manual calibration system for camera-lidar alignment
        self.calibration_mode = True
        self.adjustment_x = 0  # Pixel offset from original camera position
        self.adjustment_y = 0  # Pixel offset from original camera position
        self.adjustment_step = 10  # Pixels per keypress
        self.offset_file_counter = 1  # For offset1.csv, offset2.csv, etc.
        self.original_person_x = 0  # Store original camera detection
        self.original_person_y = 0
        self.last_save_message = ""  # Display save confirmation
        self.last_save_time = 0  # For timed display of save message
        
        # Applied offset for camera-lidar alignment (calculated from calibration data)
        self.applied_offset_x = 0  # Current offset being applied to camera detections
        self.applied_offset_y = 0  # Current offset being applied to camera detections
        self.offset_calculated = False  # Whether we have calculated an offset from data

    def save_calibration_offset(self):
        """Save REAL calibration offset data to CSV file"""
        print(f"💾 FUNCTION CALLED: save_calibration_offset() - counter: {self.offset_file_counter}")
        
        try:
            # Get current working directory and create filename
            import os
            current_dir = os.getcwd()
            filename = f"offset{self.offset_file_counter}.csv"
            full_path = os.path.join(current_dir, filename)
            
            print(f"💾 Creating REAL calibration file: {full_path}")
            
            # Get current person detection data
            person_data = self._cached_person_data
            if not person_data:
                print("❌ No person data available for calibration")
                self.last_save_message = "ERROR: No person data"
                self.last_save_time = time.time()
                return
            
            # Calculate offset between original camera position and adjusted position
            offset_x = self.adjustment_x
            offset_y = self.adjustment_y
            
            # Get head angle
            try:
                head_angle_deg = self.get_head_angle_degrees()
                print(f"📐 Head angle: {head_angle_deg}")
            except Exception as e:
                print(f"⚠️ Head angle error: {e}")
                head_angle_deg = 0.0
            
            # Get person distance and position
            x_camera = person_data['x_camera']
            z_camera = person_data['z_camera']
            confidence = person_data.get('confidence', 0.0)
            
            # Get closest lidar obstacle for comparison
            closest_lidar_distance = 0
            closest_lidar_angle = 0
            if self.lidar_system:
                obstacles = self.lidar_system.get_display_obstacles()
                if obstacles:
                    # Find closest obstacle
                    closest = min(obstacles, key=lambda obs: obs[1])
                    closest_lidar_angle, closest_lidar_distance = closest
                    print(f"📐 Closest lidar: angle={closest_lidar_angle}, dist={closest_lidar_distance}")
            
            # Write REAL calibration data to CSV
            with open(full_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'timestamp', 'head_angle_deg', 'offset_x_pixels', 'offset_y_pixels',
                    'original_camera_x_pixels', 'original_camera_y_pixels',
                    'adjusted_camera_x_pixels', 'adjusted_camera_y_pixels',
                    'camera_x_mm', 'camera_z_mm', 'camera_confidence',
                    'closest_lidar_angle_deg', 'closest_lidar_distance_mm',
                    'person_distance_mm', 'screen_center_x', 'screen_center_y'
                ])
                writer.writerow([
                    time.time(), head_angle_deg, offset_x, offset_y,
                    self.original_person_x, self.original_person_y,
                    self.original_person_x + offset_x, self.original_person_y + offset_y,
                    x_camera, z_camera, confidence,
                    closest_lidar_angle, closest_lidar_distance,
                    z_camera, self.center_x, self.center_y
                ])
            
            print(f"💾 REAL calibration data written to file")
            
            # Check if file exists and verify content
            if os.path.exists(full_path):
                file_size = os.path.getsize(full_path)
                print(f"✅ SUCCESS: Real calibration file created - {filename} ({file_size} bytes)")
                
                # Display offset information
                offset_magnitude = math.sqrt(offset_x**2 + offset_y**2)
                print(f"✅ Calibration data saved to {filename}")
                print(f"   Offset: ({offset_x:+d}, {offset_y:+d}) pixels, magnitude: {offset_magnitude:.1f}px")
                print(f"   Head angle: {head_angle_deg:.1f}°, Person distance: {z_camera:.0f}mm")
                print(f"   Camera pos: ({self.original_person_x}, {self.original_person_y})")
                print(f"   Lidar: {closest_lidar_angle:.1f}° at {closest_lidar_distance:.0f}mm")
                
                self.last_save_message = f"SAVED: {filename}"
                self.offset_file_counter += 1
            else:
                print(f"❌ FAILED: File not found - {full_path}")
                self.last_save_message = "ERROR: File not created"
            
            self.last_save_time = time.time()
            
        except Exception as e:
            print(f"❌ EXCEPTION in save_calibration_offset(): {e}")
            import traceback
            traceback.print_exc()
            self.last_save_message = f"ERROR: {str(e)}"
            self.last_save_time = time.time()
    
    def calculate_offset_from_data(self):
        """Calculate camera-lidar offset from saved calibration data"""
        try:
            print("🔍 Analyzing calibration data to calculate offset...")
            
            # Look for offset CSV files
            import os
            import glob
            
            csv_files = glob.glob("offset*.csv")
            if not csv_files:
                print("❌ No offset CSV files found")
                self.last_save_message = "ERROR: No offset files found"
                self.last_save_time = time.time()
                return False
            
            total_offset_x = 0
            total_offset_y = 0
            valid_samples = 0
            
            for csv_file in csv_files:
                try:
                    print(f"📄 Reading {csv_file}")
                    with open(csv_file, 'r') as f:
                        reader = csv.reader(f)
                        header = next(reader)  # Skip header
                        row = next(reader)     # Read data row
                        
                        # Check if this is real calibration data (has offset columns)
                        if 'offset_x_pixels' in header and 'offset_y_pixels' in header:
                            offset_x_idx = header.index('offset_x_pixels')
                            offset_y_idx = header.index('offset_y_pixels')
                            
                            offset_x = float(row[offset_x_idx])
                            offset_y = float(row[offset_y_idx])
                            
                            print(f"   Offset: ({offset_x:+.0f}, {offset_y:+.0f}) pixels")
                            
                            total_offset_x += offset_x
                            total_offset_y += offset_y
                            valid_samples += 1
                        else:
                            print(f"   Skipping {csv_file} - test data format")
                            
                except Exception as e:
                    print(f"⚠️ Error reading {csv_file}: {e}")
            
            if valid_samples > 0:
                # Calculate average offset
                avg_offset_x = total_offset_x / valid_samples
                avg_offset_y = total_offset_y / valid_samples
                
                # Apply the calculated offset
                self.applied_offset_x = int(round(avg_offset_x))
                self.applied_offset_y = int(round(avg_offset_y))
                self.offset_calculated = True
                
                print(f"✅ Calculated offset from {valid_samples} samples:")
                print(f"   Average offset: ({avg_offset_x:+.1f}, {avg_offset_y:+.1f}) pixels")
                print(f"   Applied offset: ({self.applied_offset_x:+d}, {self.applied_offset_y:+d}) pixels")
                
                self.last_save_message = f"OFFSET CALCULATED: ({self.applied_offset_x:+d}, {self.applied_offset_y:+d})"
                self.last_save_time = time.time()
                
                return True
            else:
                print("❌ No valid calibration data found")
                self.last_save_message = "ERROR: No valid calibration data"
                self.last_save_time = time.time()
                return False
                
        except Exception as e:
            print(f"❌ Error calculating offset: {e}")
            self.last_save_message = f"ERROR: {str(e)}"
            self.last_save_time = time.time()
            return False

    def set_manual_offset(self, offset_x, offset_y):
        """Manually set the camera-lidar offset (for testing purposes)"""
        self.applied_offset_x = offset_x
        self.applied_offset_y = offset_y
        self.offset_calculated = True
        print(f"🔧 Manual offset set: ({offset_x:+d}, {offset_y:+d}) pixels")
        self.last_save_message = f"MANUAL OFFSET: ({offset_x:+d}, {offset_y:+d})"
        self.last_save_time = time.time()
    
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
            
            # Show user where calibration files will be saved
            import os
            current_dir = os.getcwd()
            print(f"📁 Calibration files will be saved to: {current_dir}")
            print(f"   Files: offset1.csv, offset2.csv, etc.")
            print(f"")
            print(f"🎯 CALIBRATION INSTRUCTIONS:")
            print(f"   1. Position person at 2-3 meters distance")
            print(f"   2. Start with head at 0° (centered)")
            print(f"   3. Use WASD to align GREEN circle with white LiDAR dots")
            print(f"   4. Press X to save calibration data")
            print(f"   5. Repeat at different head angles: ±30°, ±45°")
            print(f"   6. Press C to calculate and apply offset")
            print(f"   7. BLUE circle should then align with LiDAR detections")
            print(f"")
            print(f"🔧 TEST OFFSETS: Press 1 (no offset), 2 (+50px), 3 (-50px)")
            
        except Exception as e:
            pass
    
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
            
            # 2. CREATE REPULSIVE FORCES AROUND FILTERED OBSTACLES (very localized)
            for angle, distance, obs_screen_x, obs_screen_y in filtered_obstacles:
                # Convert obstacle screen position to grid coordinates
                obs_grid_x = int(round((obs_screen_x - self.center_x) / (self.grid_size * self.scale // 1000)))
                obs_grid_y = int(round((self.center_y - obs_screen_y) / (self.grid_size * self.scale // 1000)))
                
                # Create very localized repulsive force - only immediate obstacle location
                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        gx = obs_grid_x + dx
                        gy = obs_grid_y + dy
                        
                        distance_from_obstacle = math.sqrt(dx*dx + dy*dy)
                        if distance_from_obstacle == 0:
                            repulsive_value = 2   # Only at exact obstacle location
                        elif distance_from_obstacle <= 1.0:
                            repulsive_value = 1   # Minimal for adjacent cells
                        else:
                            continue  # No repulsive force for diagonal cells
                        
                        # Add to grid (sum with existing values)
                        if (gx, gy) in self.potential_field_cache:
                            self.potential_field_cache[(gx, gy)] += repulsive_value
                        else:
                            self.potential_field_cache[(gx, gy)] = repulsive_value
            
            # 3. CREATE ATTRACTIVE FORCES AROUND PERSON (minimal values)
            if person_screen_x is not None:
                # Convert person screen position to grid coordinates
                person_grid_x = int(round((person_screen_x - self.center_x) / (self.grid_size * self.scale // 1000)))
                person_grid_y = int(round((self.center_y - person_screen_y) / (self.grid_size * self.scale // 1000)))
                
                # Create spreading attractive force (1 cell radius only) - MINIMAL VALUES
                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        gx = person_grid_x + dx
                        gy = person_grid_y + dy
                        
                        distance_from_person = math.sqrt(dx*dx + dy*dy)
                        if distance_from_person == 0:
                            attractive_value = -5   # Increased to max cap
                        elif distance_from_person <= 1:
                            attractive_value = -4   # Increased 
                        else:
                            attractive_value = -3   # Increased
                        
                        # Add to grid (sum with existing values)
                        if (gx, gy) in self.potential_field_cache:
                            self.potential_field_cache[(gx, gy)] += attractive_value
                        else:
                            self.potential_field_cache[(gx, gy)] = attractive_value
            
            # 4. CAP VALUES TO PREVENT EXTREME ACCUMULATION
            # Clamp all values to reasonable range to prevent 50+ numbers
            for key in self.potential_field_cache:
                value = self.potential_field_cache[key]
                self.potential_field_cache[key] = max(-5, min(5, value))  # Cap between -5 and +5
            
            # 5. FILTER OUT INSIGNIFICANT VALUES
            # Remove entries with very small absolute values (adjusted for capped values)
            self.potential_field_cache = {k: v for k, v in self.potential_field_cache.items() if abs(v) >= 1}
                    
        except Exception as e:
            pass
    
    def draw_potential_field_grid(self):
        """Display summed potential field values - STRICTLY WITHIN concentric rings only"""
        try:
            if not self.potential_field_cache:
                return
            
            # Calculate outer ring radius very conservatively with large buffer
            # Use the same calculation as the actual radar grid drawing
            outer_ring_radius = (6000 * self.scale // 1000) - 60  # 60px buffer well inside ring
            
            font = pygame.font.Font(None, 28)  # Readable font
            
            for (grid_x, grid_y), potential in self.potential_field_cache.items():
                # Convert grid coordinates to screen coordinates (center of grid cell)
                world_x = grid_x * self.grid_size
                world_y = grid_y * self.grid_size
                
                screen_x = self.center_x + int(world_x * self.scale // 1000)
                screen_y = self.center_y - int(world_y * self.scale // 1000)
                
                # Calculate distance from center (robot position)
                distance_from_center = math.sqrt((screen_x - self.center_x)**2 + (screen_y - self.center_y)**2)
                
                # STRICT boundary check - must be well within the outermost ring
                # Also check minimum distance to avoid numbers too close to center
                if distance_from_center > outer_ring_radius or distance_from_center < 50:
                    continue
                
                # Additional safety check - ensure we're within reasonable screen bounds
                screen_margin = 80
                if not (screen_margin <= screen_x < self.screen.get_width() - screen_margin and 
                       screen_margin <= screen_y < self.screen.get_height() - screen_margin):
                    continue
                
                # Convert to integer for display
                display_val = int(abs(potential))  # Always show as positive number
                
                # Color based on SUMMED result: GREEN for negative (attractive), RED for positive (repulsive)
                if potential < 0:
                    color = (0, 255, 0)  # Bright green for net attractive forces
                else:
                    color = (255, 0, 0)  # Bright red for net repulsive forces
                
                text = str(display_val)
                
                # Only display significant values (adjusted for reduced force values)
                if display_val >= 1:
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
    
    def draw_grid_overlay(self):
        """Draw potential field grid overlay lines"""
        try:
            if not self.screen:
                return
                
            # Draw grid lines at regular intervals
            grid_spacing = self.grid_size * self.scale // 1000  # Convert to screen pixels
            
            # Calculate grid bounds
            max_screen_distance = min(self.center_x, self.center_y) - 50
            
            # Draw vertical grid lines
            for x_offset in range(-max_screen_distance, max_screen_distance + 1, grid_spacing):
                grid_x = self.center_x + x_offset
                if 0 <= grid_x < self.screen.get_width():
                    pygame.draw.line(self.screen, (0, 60, 0), 
                                   (grid_x, self.center_y - max_screen_distance), 
                                   (grid_x, self.center_y + max_screen_distance), 1)
            
            # Draw horizontal grid lines
            for y_offset in range(-max_screen_distance, max_screen_distance + 1, grid_spacing):
                grid_y = self.center_y + y_offset
                if 0 <= grid_y < self.screen.get_height():
                    pygame.draw.line(self.screen, (0, 60, 0), 
                                   (self.center_x - max_screen_distance, grid_y), 
                                   (self.center_x + max_screen_distance, grid_y), 1)
                        
        except Exception:
            pass  # Fail silently for performance
    
    def draw_radar_grid(self):
        """Draw radar-style grid - NO TEXT VERSION"""
        try:
            if not self.screen:
                return
                
            # Draw range circles (NO LABELS)
            for distance in [1000, 2000, 3000, 4000, 5000, 6000]:
                radius = distance * self.scale // 1000
                if radius < min(self.center_x, self.center_y) - 50:
                    line_width = 3 if distance == 6000 else 2
                    color = (0, 150, 0) if distance < 6000 else (255, 255, 0)
                    pygame.draw.circle(self.screen, color, (self.center_x, self.center_y), radius, line_width)
            
            # Draw angle lines (NO LABELS)
            for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
                display_angle_rad = math.radians(90 - angle)
                line_length = min(self.center_x, self.center_y) - 80
                end_x = self.center_x + int(line_length * math.cos(display_angle_rad))
                end_y = self.center_y - int(line_length * math.sin(display_angle_rad))
                line_width = 3 if angle % 90 == 0 else 1
                pygame.draw.line(self.screen, (0, 150, 0), (self.center_x, self.center_y), (end_x, end_y), line_width)
                
        except Exception:
            pass
    
    def draw_person_detection_no_text(self, person_data):
        """Draw person detection with calibration markers and pixel position display"""
        try:
            if not self.screen or not person_data:
                return
            
            # DISPLAY CURRENT APPLIED OFFSET AT TOP OF SCREEN IN LARGE FONT
            huge_font = pygame.font.Font(None, 96)  # Very large font for offset display
            if self.offset_calculated:
                offset_text = huge_font.render(f"APPLIED OFFSET: ({self.applied_offset_x:+d}, {self.applied_offset_y:+d})", True, (0, 255, 255))
            else:
                offset_text = huge_font.render(f"NO OFFSET APPLIED - Use Calibration", True, (255, 255, 0))
            
            # Center the offset display at the top
            offset_rect = offset_text.get_rect(center=(self.screen.get_width() // 2, 50))
            self.screen.blit(offset_text, offset_rect)
                
            x_camera = person_data['x_camera']
            z_camera = person_data['z_camera']
            
            if z_camera <= 0:
                return
            
            # Calculate person position on radar (same calculation as parent)
            person_angle_rad = math.atan2(x_camera, z_camera)
            person_angle_deg = math.degrees(person_angle_rad)
            
            # Convert to display coordinates
            display_angle_deg = 360 - person_angle_deg
            while display_angle_deg < 0:
                display_angle_deg += 360
            while display_angle_deg >= 360:
                display_angle_deg -= 360
            
            distance_m = z_camera / 1000.0
            display_angle_rad = math.radians(90 - display_angle_deg)
            
            person_x = self.center_x + int(distance_m * self.scale * math.cos(display_angle_rad))
            person_y = self.center_y - int(distance_m * self.scale * math.sin(display_angle_rad))
            
            # Store original position for calibration
            self.original_person_x = person_x
            self.original_person_y = person_y
            
            # Calculate offset-corrected position (this is what should align with lidar)
            corrected_x = person_x + self.applied_offset_x
            corrected_y = person_y + self.applied_offset_y
            
            # Draw original camera detection (orange circle - smaller)
            if (0 <= person_x < self.screen.get_width() and 0 <= person_y < self.screen.get_height()):
                pygame.draw.circle(self.screen, (255, 165, 0), (person_x, person_y), 6, 2)  # Orange outline
                pygame.draw.circle(self.screen, (255, 165, 0), (person_x, person_y), 2)      # Orange center
            
            # Draw offset-corrected position (BLUE circle - this should align with lidar)
            if (0 <= corrected_x < self.screen.get_width() and 0 <= corrected_y < self.screen.get_height()):
                pygame.draw.circle(self.screen, (0, 0, 255), (corrected_x, corrected_y), 8, 3)  # Blue outline (corrected)
                pygame.draw.circle(self.screen, (0, 0, 255), (corrected_x, corrected_y), 3)      # Blue center
            
            # Draw adjustable calibration marker (GREEN filled circle - for manual adjustment)
            adjusted_x = person_x + self.adjustment_x
            adjusted_y = person_y + self.adjustment_y
            
            if (0 <= adjusted_x < self.screen.get_width() and 0 <= adjusted_y < self.screen.get_height()):
                pygame.draw.circle(self.screen, (0, 255, 0), (adjusted_x, adjusted_y), 12)     # Large green filled circle
                pygame.draw.circle(self.screen, (255, 255, 255), (adjusted_x, adjusted_y), 12, 2)  # White outline
            
            # Display pixel position in large font (top-left area, below the offset display)
            large_font = pygame.font.Font(None, 64)  # Slightly smaller to fit more info
            
            # Original camera position
            original_text = large_font.render(f"Raw Camera: ({person_x}, {person_y})", True, (255, 165, 0))
            self.screen.blit(original_text, (20, 120))
            
            # Offset-corrected position (this should match lidar)
            corrected_text = large_font.render(f"Corrected: ({corrected_x}, {corrected_y})", True, (0, 0, 255))
            self.screen.blit(corrected_text, (20, 180))
            
            # Manual adjustment position  
            adjusted_text = large_font.render(f"Manual Adj: ({adjusted_x}, {adjusted_y})", True, (0, 255, 0))
            self.screen.blit(adjusted_text, (20, 240))
            
            # Current manual offset
            offset_text = large_font.render(f"Manual Offset: ({self.adjustment_x:+d}, {self.adjustment_y:+d})", True, (255, 255, 255))
            self.screen.blit(offset_text, (20, 300))
            
            # Instructions (smaller font)
            instruction_font = pygame.font.Font(None, 36)
            instructions = [
                "WASD: Move green marker to align with lidar",
                f"X: Save to offset{self.offset_file_counter}.csv",
                "C: Calculate & apply offset from saved data", 
                "1: No offset | 2: +50px right | 3: -50px left",
                f"Orange: Raw | Blue: Corrected | Green: Manual",
                f"Center: 960px"
            ]
            
            for i, instruction in enumerate(instructions):
                inst_text = instruction_font.render(instruction, True, (200, 200, 200))
                self.screen.blit(inst_text, (20, 380 + i * 45))
            
            # Show save confirmation message (if recent)
            if self.last_save_message and (time.time() - self.last_save_time) < 3.0:
                save_font = pygame.font.Font(None, 64)
                if "ERROR" in self.last_save_message:
                    save_color = (255, 100, 100)  # Light red for errors
                else:
                    save_color = (100, 255, 100)  # Light green for success
                
                save_text = save_font.render(self.last_save_message, True, save_color)
                # Position it prominently in the center-left area
                self.screen.blit(save_text, (20, 560))
                
        except Exception:
            pass  # Fail silently
    
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
    
    def update(self) -> Status:
        """OPTIMIZED update method - minimal overhead"""
        try:
            if not self.initialized:
                if not self.initialize_components():
                    return Status.FAILURE
            
            self.update_counter += 1
            
            # Handle calibration keys (WASD for movement, X for saving offset, C for calculating offset)
            keys = pygame.key.get_pressed()
            if keys[pygame.K_w]:
                self.adjustment_y -= self.adjustment_step  # Move up (decrease Y)
            if keys[pygame.K_s]:
                self.adjustment_y += self.adjustment_step  # Move down (increase Y)
            if keys[pygame.K_a]:
                self.adjustment_x -= self.adjustment_step  # Move left (decrease X)
            if keys[pygame.K_d]:
                self.adjustment_x += self.adjustment_step  # Move right (increase X)
            
            # Check for X key press to save offset (doesn't interfere with ESC handling)
            if keys[pygame.K_x]:
                # Use a simple debounce mechanism to avoid multiple saves
                current_time = time.time()
                if not hasattr(self, '_last_x_press') or current_time - self._last_x_press > 1.0:
                    print("🔑 X key pressed - attempting to save calibration offset...")
                    if self._cached_person_data:
                        print("✅ Person data available, saving...")
                        print(f"📁 About to call save_calibration_offset() function...")
                        try:
                            self.save_calibration_offset()
                            print(f"📁 save_calibration_offset() completed")
                        except Exception as e:
                            print(f"❌ Exception in save_calibration_offset(): {e}")
                            import traceback
                            traceback.print_exc()
                    else:
                        print("❌ No person detected - cannot save offset")
                        self.last_save_message = "ERROR: No person detected"
                        self.last_save_time = current_time
                    self._last_x_press = current_time
            
            # Check for C key press to calculate and apply offset from saved data
            if keys[pygame.K_c]:
                current_time = time.time()
                if not hasattr(self, '_last_c_press') or current_time - self._last_c_press > 1.0:
                    print("🔑 C key pressed - calculating offset from calibration data...")
                    self.calculate_offset_from_data()
                    self._last_c_press = current_time
            
            # Test offset keys (for demonstration)
            if keys[pygame.K_1]:
                current_time = time.time()
                if not hasattr(self, '_last_1_press') or current_time - self._last_1_press > 1.0:
                    self.set_manual_offset(0, 0)  # No offset
                    self._last_1_press = current_time
                    
            if keys[pygame.K_2]:
                current_time = time.time()
                if not hasattr(self, '_last_2_press') or current_time - self._last_2_press > 1.0:
                    self.set_manual_offset(50, 0)  # 50 pixels right
                    self._last_2_press = current_time
                    
            if keys[pygame.K_3]:
                current_time = time.time()
                if not hasattr(self, '_last_3_press') or current_time - self._last_3_press > 1.0:
                    self.set_manual_offset(-50, 0)  # 50 pixels left
                    self._last_3_press = current_time
            
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
                        
                        # Draw radar grid 
                        self.draw_radar_grid()  # Base radar circles and lines
                        self.draw_grid_overlay()  # Potential field grid overlay
                        self.draw_robot()  # From parent
                        
                        # LiDAR obstacles (from parent)
                        obstacle_count = 0
                        if self.lidar_system:
                            obstacles = self.lidar_system.get_display_obstacles()
                            if obstacles:
                                obstacle_count = self.draw_lidar_data(obstacles)
                        
                        # Person detection with calibration markers
                        if self._cached_person_data:
                            self.draw_person_detection_no_text(self._cached_person_data)
                        
                        # POTENTIAL FIELD VALUES (drawn after obstacles/person for visibility)
                        # FORCE NUMBERS ONLY (no other text)
                        self.draw_potential_field_grid()
                        
                        pygame.display.flip()
                except Exception:
                    pass  # Fail silently
            
            return Status.RUNNING
        except Exception:
            return Status.RUNNING
    
    def terminate(self, new_status: Status):
        """Clean termination"""
        # Reset navigation system
        if hasattr(self, 'navigator'):
            self.navigator.reset_navigation_state()
        
        # Call parent termination (handles everything else)
        super().terminate(new_status)

# Export for use
__all__ = ['PotentialFieldVisualizationTest']