#!/usr/bin/env python3
"""
Camera Depth Test Behavior for Maxine the Robot
Integrates with existing robot framework to test Oak-d pro W camera
z depth vs y coordinate distance method for SSDMobileNet detection
"""
import pygame
import math
import time
import csv
import os
from collections import deque
from datetime import datetime
import py_trees
from py_trees.common import Status

from ...behaviors.MaxineBehavior import MaxineBehavior
from ...types.RobotModes import RobotMode
from ...types.CameraMode import CameraMode


class CameraDepthTestBehavior(MaxineBehavior):
    """Camera Depth vs Y-Coordinate Test Behavior integrated with robot framework"""
    
    def __init__(self):
        super().__init__("Camera Depth Analysis Test")
        
        # Core components
        self.screen = None
        self.initialized = False
        
        # Speech tracking
        self.idle_mode_announced = False
        self.test_mode_announced = False
        
        # Display parameters
        self.center_x = 0
        self.center_y = 0
        
        # Data tracking
        self.detection_history = deque(maxlen=100)  # Keep last 100 detections
        self.z_distance_history = deque(maxlen=50)   # For flicker detection
        self.flicker_events = []
        self.current_detection = None
        
        # Analysis parameters
        self.flicker_threshold = 300  # mm - consider it flickering if z changes by this much
        self.edge_threshold = 0.7     # Consider edges if |x| > this * max_x_coordinate
        
        # Camera specs for Oak-d pro W with SSDMobileNet
        self.camera_resolution_width = 800   # Oak-d pro W width
        self.camera_resolution_height = 800  # Assuming square for SSDMobileNet
        self.camera_hfov_degrees = 120       # Oak-d pro W horizontal FOV
        
        # File logging
        self.log_filename = f"camera_depth_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.initialize_csv_log()
        
        # Control flags
        self.paused = False
        
        if not pygame.get_init():
            pygame.init()
        pygame.font.init()
    
    def initialize_csv_log(self):
        """Initialize CSV log file for data recording"""
        try:
            with open(self.log_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'timestamp', 'x_camera', 'y_camera', 'z_depth', 'confidence',
                    'bbox_ymin', 'bbox_ymax', 'y_bottom_distance', 'estimated_distance_from_y',
                    'x_position_ratio', 'is_at_edge', 'z_flicker_detected', 'z_change_amount'
                ])
        except Exception as e:
            pass
    
    def initialize_components(self):
        """Initialize test mode components"""
        if self.initialized:
            return True
        
        try:
            # Ensure camera is in object detection mode
            robot = self.get_robot()
            if hasattr(robot, 'camera_sensor') and robot.camera_sensor:
                robot.camera_sensor.switch_mode(CameraMode.OBJECT_DETECTION)
                time.sleep(2)  # Allow camera to initialize
            
            display_info = pygame.display.Info()
            self.screen = pygame.display.set_mode((display_info.current_w, display_info.current_h), pygame.FULLSCREEN)
            pygame.display.set_caption("MAXINE CAMERA DEPTH TEST")
            
            self.center_x = display_info.current_w // 2
            self.center_y = display_info.current_h // 2
            
            self.draw_initial_interface()
            pygame.display.flip()
            
            self.initialized = True
            return True
            
        except Exception:
            self.initialized = False
            return False
    
    def calculate_y_distance_method(self, detection):
        """Calculate estimated distance using y-coordinate method"""
        try:
            # Get bounding box coordinates (normalized 0-1)
            ymin = detection.ymin
            ymax = detection.ymax
            
            # Convert to pixel coordinates
            bbox_bottom_pixel = ymax * self.camera_resolution_height
            screen_bottom_pixel = self.camera_resolution_height
            
            # Calculate distance from bottom of person to bottom of screen
            y_bottom_distance = screen_bottom_pixel - bbox_bottom_pixel
            
            # Simple linear relationship for testing (will be calibrated based on results)
            # Assumption: person at bottom of screen is very close, person higher up is further
            max_distance = 8000  # mm
            min_distance = 500   # mm
            
            # Normalize y_bottom_distance (0 to screen_height)
            distance_ratio = y_bottom_distance / self.camera_resolution_height
            estimated_distance = min_distance + (distance_ratio * (max_distance - min_distance))
            
            return {
                'bbox_bottom_pixel': bbox_bottom_pixel,
                'y_bottom_distance': y_bottom_distance,
                'estimated_distance': estimated_distance
            }
            
        except Exception:
            return None
    
    def analyze_fov_position(self, x_camera):
        """Analyze how close the detection is to FOV edges"""
        try:
            # Normalize x position (-1 to +1, where 0 is center)
            # Assuming x_camera is in mm from center
            max_x_at_1m = 1000 * math.tan(math.radians(self.camera_hfov_degrees / 2))
            x_position_ratio = x_camera / max_x_at_1m if max_x_at_1m > 0 else 0
            
            # Clamp to reasonable range
            x_position_ratio = max(-1.0, min(1.0, x_position_ratio))
            
            is_at_edge = abs(x_position_ratio) > self.edge_threshold
            
            return {
                'x_position_ratio': x_position_ratio,
                'is_at_edge': is_at_edge,
                'distance_from_center': abs(x_position_ratio)
            }
            
        except Exception:
            return {'x_position_ratio': 0, 'is_at_edge': False, 'distance_from_center': 0}
    
    def detect_z_flickering(self, current_z):
        """Detect if z distance is flickering"""
        self.z_distance_history.append(current_z)
        
        if len(self.z_distance_history) < 5:
            return False, 0
        
        # Check for sudden changes in recent history
        recent_distances = list(self.z_distance_history)[-5:]
        max_z = max(recent_distances)
        min_z = min(recent_distances)
        z_range = max_z - min_z
        
        # Check if current distance differs significantly from recent average
        recent_avg = sum(recent_distances[:-1]) / len(recent_distances[:-1])
        z_change = abs(current_z - recent_avg)
        
        is_flickering = z_change > self.flicker_threshold
        
        return is_flickering, z_change
    
    def process_detection(self):
        """Process current camera detection using robot's camera sensor"""
        try:
            robot = self.get_robot()
            
            if not hasattr(robot, 'camera_sensor') or not robot.camera_sensor:
                return None
            
            # Get camera reading
            reading = robot.camera_sensor.get_reading()
            
            if not reading:
                return None
            
            # Get people detections
            people = reading.get_people_locations()
            
            if not people:
                return None
            
            # Get closest person
            closest_person = people[0]
            
            # Extract data
            x_camera = closest_person.spatialCoordinates.x
            y_camera = closest_person.spatialCoordinates.y
            z_depth = closest_person.spatialCoordinates.z
            confidence = closest_person.confidence
            
            # Calculate y-coordinate method
            y_method_data = self.calculate_y_distance_method(closest_person)
            
            # Analyze FOV position
            fov_analysis = self.analyze_fov_position(x_camera)
            
            # Detect flickering
            is_flickering, z_change = self.detect_z_flickering(z_depth)
            
            # Compile analysis data
            analysis_data = {
                'timestamp': time.time(),
                'x_camera': x_camera,
                'y_camera': y_camera,
                'z_depth': z_depth,
                'confidence': confidence,
                'bbox_ymin': closest_person.ymin,
                'bbox_ymax': closest_person.ymax,
                'y_method_data': y_method_data,
                'fov_analysis': fov_analysis,
                'is_flickering': is_flickering,
                'z_change': z_change,
                'detection_object': closest_person
            }
            
            # Log flicker events
            if is_flickering:
                flicker_event = {
                    'timestamp': analysis_data['timestamp'],
                    'x_camera': x_camera,
                    'z_change': z_change,
                    'is_at_edge': fov_analysis['is_at_edge']
                }
                self.flicker_events.append(flicker_event)
            
            # Log to CSV
            self.log_to_csv(analysis_data)
            
            # Store current detection
            self.current_detection = analysis_data
            self.detection_history.append(analysis_data)
            
            return analysis_data
            
        except Exception:
            return None
    
    def log_to_csv(self, data):
        """Log detection data to CSV file"""
        try:
            y_method = data.get('y_method_data', {})
            fov_analysis = data.get('fov_analysis', {})
            
            with open(self.log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    data['timestamp'],
                    data['x_camera'],
                    data['y_camera'],
                    data['z_depth'],
                    data['confidence'],
                    data['bbox_ymin'],
                    data['bbox_ymax'],
                    y_method.get('y_bottom_distance', 0),
                    y_method.get('estimated_distance', 0),
                    fov_analysis.get('x_position_ratio', 0),
                    fov_analysis.get('is_at_edge', False),
                    data['is_flickering'],
                    data['z_change']
                ])
        except Exception:
            pass
    
    def draw_initial_interface(self):
        """Draw initial interface"""
        self.screen.fill((0, 0, 0))
        
        # Title
        title_font = pygame.font.Font(None, 64)
        title = title_font.render("CAMERA DEPTH ANALYSIS TEST", True, (255, 255, 0))
        title_rect = title.get_rect(center=(self.center_x, 100))
        self.screen.blit(title, title_rect)
    
    def draw_analysis_display(self):
        """Draw real-time analysis display"""
        self.screen.fill((0, 0, 0))
        
        if not self.current_detection:
            # No detection
            title_font = pygame.font.Font(None, 64)
            medium_font = pygame.font.Font(None, 48)
            
            title = title_font.render("NO PERSON DETECTED", True, (255, 0, 0))
            title_rect = title.get_rect(center=(self.center_x, 150))
            self.screen.blit(title, title_rect)
            
            # Instructions
            instructions = [
                "Stand in front of camera and move around",
                "Especially test the far left and right edges",
                "",
                "SPACE - Pause/Resume",
                "ESC - Exit and save data"
            ]
            
            y_offset = 250
            for instruction in instructions:
                if instruction == "":
                    y_offset += 25
                    continue
                elif "ESC" in instruction or "SPACE" in instruction:
                    color = (255, 100, 100)
                else:
                    color = (255, 255, 255)
                
                text = medium_font.render(instruction, True, color)
                text_rect = text.get_rect(center=(self.center_x, y_offset))
                self.screen.blit(text, text_rect)
                y_offset += 60
            
            pygame.display.flip()
            return
        
        # Draw detection analysis
        data = self.current_detection
        y_method = data.get('y_method_data', {})
        fov_analysis = data.get('fov_analysis', {})
        
        large_font = pygame.font.Font(None, 64)
        medium_font = pygame.font.Font(None, 48)
        small_font = pygame.font.Font(None, 32)
        
        y_offset = 50
        
        # Title
        title = large_font.render("CAMERA DEPTH ANALYSIS", True, (255, 255, 0))
        self.screen.blit(title, (50, y_offset))
        y_offset += 80
        
        # Camera coordinates
        coord_text = medium_font.render(f"Camera X: {data['x_camera']:.0f}mm", True, (255, 255, 255))
        self.screen.blit(coord_text, (50, y_offset))
        y_offset += 50
        
        coord_text = medium_font.render(f"Camera Y: {data['y_camera']:.0f}mm", True, (255, 255, 255))
        self.screen.blit(coord_text, (50, y_offset))
        y_offset += 50
        
        # Z depth (with flicker indication)
        z_color = (255, 0, 0) if data['is_flickering'] else (0, 255, 0)
        z_text = medium_font.render(f"Z Depth: {data['z_depth']:.0f}mm", True, z_color)
        self.screen.blit(z_text, (50, y_offset))
        
        if data['is_flickering']:
            flicker_text = medium_font.render(f"FLICKER! Change: {data['z_change']:.0f}mm", True, (255, 0, 0))
            self.screen.blit(flicker_text, (400, y_offset))
        y_offset += 50
        
        # Confidence
        conf_text = medium_font.render(f"Confidence: {data['confidence']:.2f}", True, (255, 255, 255))
        self.screen.blit(conf_text, (50, y_offset))
        y_offset += 70
        
        # Y-coordinate method
        if y_method:
            y_title = medium_font.render("Y-COORDINATE METHOD:", True, (0, 255, 255))
            self.screen.blit(y_title, (50, y_offset))
            y_offset += 50
            
            y_dist_text = small_font.render(f"Y Bottom Distance: {y_method.get('y_bottom_distance', 0):.0f}px", True, (255, 255, 255))
            self.screen.blit(y_dist_text, (50, y_offset))
            y_offset += 40
            
            est_dist_text = small_font.render(f"Estimated Distance: {y_method.get('estimated_distance', 0):.0f}mm", True, (255, 255, 255))
            self.screen.blit(est_dist_text, (50, y_offset))
            y_offset += 60
        
        # FOV analysis
        fov_title = medium_font.render("FOV POSITION ANALYSIS:", True, (0, 255, 255))
        self.screen.blit(fov_title, (50, y_offset))
        y_offset += 50
        
        x_ratio = fov_analysis.get('x_position_ratio', 0)
        x_ratio_text = small_font.render(f"X Position Ratio: {x_ratio:.3f} (-1=left edge, +1=right edge)", True, (255, 255, 255))
        self.screen.blit(x_ratio_text, (50, y_offset))
        y_offset += 40
        
        edge_color = (255, 0, 0) if fov_analysis.get('is_at_edge', False) else (0, 255, 0)
        edge_text = small_font.render(f"At Edge: {fov_analysis.get('is_at_edge', False)}", True, edge_color)
        self.screen.blit(edge_text, (50, y_offset))
        y_offset += 60
        
        # Statistics
        stats_title = medium_font.render("STATISTICS:", True, (255, 255, 0))
        self.screen.blit(stats_title, (50, y_offset))
        y_offset += 50
        
        flicker_count = len(self.flicker_events)
        flicker_text = small_font.render(f"Total Flicker Events: {flicker_count}", True, (255, 255, 255))
        self.screen.blit(flicker_text, (50, y_offset))
        y_offset += 40
        
        detection_count = len(self.detection_history)
        detection_text = small_font.render(f"Total Detections: {detection_count}", True, (255, 255, 255))
        self.screen.blit(detection_text, (50, y_offset))
        y_offset += 40
        
        # Log file info
        log_text = small_font.render(f"Data logged to: {self.log_filename}", True, (200, 200, 200))
        self.screen.blit(log_text, (50, y_offset))
        y_offset += 50
        
        # Controls
        controls = [
            "SPACE - Pause/Resume",
            "ESC - Exit and save data"
        ]
        
        for control in controls:
            control_text = small_font.render(control, True, (255, 100, 100))
            self.screen.blit(control_text, (50, y_offset))
            y_offset += 35
        
        # Visual FOV indicator
        self.draw_fov_indicator(fov_analysis)
        
        pygame.display.flip()
    
    def draw_fov_indicator(self, fov_analysis):
        """Draw visual FOV position indicator"""
        try:
            # Draw FOV indicator in top right
            indicator_x = self.screen.get_width() - 300
            indicator_y = 100
            indicator_width = 200
            indicator_height = 50
            
            # Draw FOV bar
            pygame.draw.rect(self.screen, (100, 100, 100), 
                           (indicator_x, indicator_y, indicator_width, indicator_height), 2)
            
            # Draw center line
            center_x = indicator_x + indicator_width // 2
            pygame.draw.line(self.screen, (255, 255, 255), 
                           (center_x, indicator_y), (center_x, indicator_y + indicator_height), 2)
            
            # Draw position indicator
            x_ratio = fov_analysis.get('x_position_ratio', 0)
            pos_x = center_x + int(x_ratio * (indicator_width // 2))
            pos_color = (255, 0, 0) if fov_analysis.get('is_at_edge', False) else (0, 255, 0)
            
            pygame.draw.circle(self.screen, pos_color, (pos_x, indicator_y + indicator_height // 2), 8)
            
            # Labels
            small_font = pygame.font.Font(None, 32)
            left_text = small_font.render("L", True, (255, 255, 255))
            self.screen.blit(left_text, (indicator_x - 20, indicator_y + 15))
            
            right_text = small_font.render("R", True, (255, 255, 255))
            self.screen.blit(right_text, (indicator_x + indicator_width + 5, indicator_y + 15))
            
        except Exception:
            pass
    
    def generate_summary_report(self):
        """Generate summary report and announce results"""
        try:
            flicker_count = len(self.flicker_events)
            detection_count = len(self.detection_history)
            
            edge_flickers = sum(1 for event in self.flicker_events if event.get('is_at_edge', False))
            center_flickers = flicker_count - edge_flickers
            
            # Announce summary via speech
            robot = self.get_robot()
            if hasattr(robot, 'speech_manager') and robot.speech_manager:
                summary_text = f"Test complete. {flicker_count} flicker events detected. {edge_flickers} at edges, {center_flickers} at center."
                robot.speech_manager.perform_action(summary_text)
            
        except Exception:
            pass
    
    def perform_enhanced_idle_mode_transition(self):
        """Perform transition back to IDLE mode"""
        try:
            robot = self.get_robot()
            
            # Step 1: Generate and announce summary
            self.generate_summary_report()
            
            # Step 2: Cleanup pygame display
            try:
                if self.screen:
                    self.screen.fill((0, 0, 0))
                    pygame.display.flip()
            except Exception:
                pass
            
            # Step 3: Announce mode transition ONLY ONCE
            if not self.idle_mode_announced:
                try:
                    if hasattr(robot, 'speech_manager') and robot.speech_manager:
                        robot.speech_manager.perform_action("IDLE MODE")
                        self.idle_mode_announced = True
                except Exception:
                    pass
            
            return True
            
        except Exception:
            return False

    def update(self) -> Status:
        """Main update loop"""
        try:
            if not self.initialized:
                if not self.initialize_components():
                    return Status.RUNNING
                
                # Announce test mode once
                if not self.test_mode_announced:
                    robot = self.get_robot()
                    if hasattr(robot, 'speech_manager') and robot.speech_manager:
                        robot.speech_manager.perform_action("Camera Depth Test Mode")
                    self.test_mode_announced = True
                
                return Status.RUNNING
            
            # Handle pygame events
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        # Enhanced IDLE mode transition
                        self.perform_enhanced_idle_mode_transition()
                        robot = self.get_robot()
                        robot.set_mode(RobotMode.IDLE)
                        return Status.SUCCESS
                    elif event.key == pygame.K_SPACE:
                        self.paused = not self.paused
                        robot = self.get_robot()
                        if hasattr(robot, 'speech_manager') and robot.speech_manager:
                            status_text = "Test Paused" if self.paused else "Test Resumed"
                            robot.speech_manager.perform_action(status_text)
                elif event.type == pygame.QUIT:
                    self.perform_enhanced_idle_mode_transition()
                    robot = self.get_robot()
                    robot.set_mode(RobotMode.IDLE)
                    return Status.SUCCESS
            
            # Process detection if not paused
            if not self.paused:
                self.process_detection()
            
            # Update display
            self.draw_analysis_display()
            
            return Status.RUNNING
            
        except Exception:
            return Status.FAILURE

    def terminate(self, new_status: Status):
        """Termination with cleanup"""
        try:
            # Generate final summary
            self.generate_summary_report()
            
            # Perform enhanced IDLE mode transition
            self.perform_enhanced_idle_mode_transition()
            
            self.initialized = False
            
        except Exception:
            pass
        
        super().terminate(new_status)


def make_camera_depth_test_sub_tree():
    """
    Returns the camera depth test behavior
    """
    return CameraDepthTestBehavior()