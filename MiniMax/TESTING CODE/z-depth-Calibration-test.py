#!/usr/bin/env python3
"""
Camera Depth vs Y-Coordinate Test Program
Tests Oak-d pro W camera z depth vs y coordinate distance method
Tracks flickering behavior and FOV edge effects for SSDMobileNet detection
"""
import pygame
import math
import time
import csv
import os
from collections import deque
from datetime import datetime
import threading
import queue

# Assuming these imports work from your existing project structure
try:
    from MiniMax.src.sensors.CameraSensor import CameraSensor
    from MiniMax.src.depth_ai.ObjectDetectionReading import ObjectDetectionReading
    from MiniMax.src.types.CameraMode import CameraMode
except ImportError:
    print("Note: This test assumes the MiniMax project structure")
    print("You may need to adjust imports based on your project layout")


class CameraDepthAnalyzer:
    """Analyzes camera depth vs y-coordinate method for distance estimation"""
    
    def __init__(self):
        # Initialize pygame for display
        pygame.init()
        pygame.font.init()
        
        # Get fullscreen resolution
        display_info = pygame.display.Info()
        self.screen_width = display_info.current_w
        self.screen_height = display_info.current_h
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.FULLSCREEN)
        pygame.display.set_caption("Camera Depth Analysis Test")
        
        # Fonts for different text sizes
        self.large_font = pygame.font.Font(None, 64)
        self.medium_font = pygame.font.Font(None, 48)
        self.small_font = pygame.font.Font(None, 32)
        
        # Camera sensor (you may need to initialize this properly for your setup)
        self.camera_sensor = None
        self.camera_initialized = False
        
        # Data tracking
        self.detection_history = deque(maxlen=100)  # Keep last 100 detections
        self.z_distance_history = deque(maxlen=50)   # For flicker detection
        self.flicker_events = []
        self.current_detection = None
        
        # Analysis parameters
        self.flicker_threshold = 300  # mm - consider it flickering if z changes by this much
        self.edge_threshold = 0.7     # Consider edges if |x| > this * max_x_coordinate
        
        # Camera specs (you may need to adjust these)
        self.camera_resolution_width = 800   # Oak-d pro W width
        self.camera_resolution_height = 800  # Assuming square for SSDMobileNet
        self.camera_hfov_degrees = 120       # Oak-d pro W horizontal FOV
        
        # File logging
        self.log_filename = f"camera_depth_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.initialize_csv_log()
        
        # Control flags
        self.running = True
        self.paused = False
        
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
            print(f"Failed to initialize CSV log: {e}")
    
    def initialize_camera(self):
        """Initialize camera sensor"""
        try:
            # You may need to adjust this initialization based on your robot setup
            self.camera_sensor = CameraSensor()
            self.camera_sensor.switch_mode(CameraMode.OBJECT_DETECTION)
            time.sleep(2)  # Allow camera to initialize
            self.camera_initialized = True
            return True
        except Exception as e:
            print(f"Failed to initialize camera: {e}")
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
            
            # Simple linear relationship for now (this is what we're testing to calibrate)
            # Assumption: person at bottom of screen is very close, person higher up is further
            # You can adjust this formula based on test results
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
            
        except Exception as e:
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
        """Process current camera detection"""
        if not self.camera_initialized or not self.camera_sensor:
            return None
        
        try:
            # Get camera reading
            reading = self.camera_sensor.get_reading()
            
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
                print(f"FLICKER DETECTED: x={x_camera:.0f}mm, z_change={z_change:.0f}mm, at_edge={fov_analysis['is_at_edge']}")
            
            # Log to CSV
            self.log_to_csv(analysis_data)
            
            # Store current detection
            self.current_detection = analysis_data
            self.detection_history.append(analysis_data)
            
            return analysis_data
            
        except Exception as e:
            print(f"Error processing detection: {e}")
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
        except Exception as e:
            print(f"Failed to log to CSV: {e}")
    
    def draw_analysis_display(self):
        """Draw real-time analysis display"""
        self.screen.fill((0, 0, 0))
        
        if not self.current_detection:
            # No detection
            text = self.large_font.render("NO PERSON DETECTED", True, (255, 0, 0))
            self.screen.blit(text, (50, 50))
            
            # Instructions
            instructions = [
                "CAMERA DEPTH vs Y-COORDINATE TEST",
                "",
                "Stand in front of camera and move around",
                "Especially test the far left and right edges",
                "",
                "SPACE - Pause/Resume",
                "ESC - Exit and save data"
            ]
            
            y_offset = 150
            for instruction in instructions:
                if instruction == "CAMERA DEPTH vs Y-COORDINATE TEST":
                    color = (255, 255, 0)
                    text = self.large_font.render(instruction, True, color)
                elif instruction == "":
                    y_offset += 25
                    continue
                elif "ESC" in instruction or "SPACE" in instruction:
                    color = (255, 100, 100)
                    text = self.medium_font.render(instruction, True, color)
                else:
                    color = (255, 255, 255)
                    text = self.medium_font.render(instruction, True, color)
                
                self.screen.blit(text, (50, y_offset))
                y_offset += 60
            
            pygame.display.flip()
            return
        
        # Draw detection analysis
        data = self.current_detection
        y_method = data.get('y_method_data', {})
        fov_analysis = data.get('fov_analysis', {})
        
        y_offset = 50
        
        # Title
        title = self.large_font.render("CAMERA DEPTH ANALYSIS TEST", True, (255, 255, 0))
        self.screen.blit(title, (50, y_offset))
        y_offset += 80
        
        # Camera coordinates
        coord_text = self.medium_font.render(f"Camera X: {data['x_camera']:.0f}mm", True, (255, 255, 255))
        self.screen.blit(coord_text, (50, y_offset))
        y_offset += 50
        
        coord_text = self.medium_font.render(f"Camera Y: {data['y_camera']:.0f}mm", True, (255, 255, 255))
        self.screen.blit(coord_text, (50, y_offset))
        y_offset += 50
        
        # Z depth (with flicker indication)
        z_color = (255, 0, 0) if data['is_flickering'] else (0, 255, 0)
        z_text = self.medium_font.render(f"Z Depth: {data['z_depth']:.0f}mm", True, z_color)
        self.screen.blit(z_text, (50, y_offset))
        y_offset += 50
        
        if data['is_flickering']:
            flicker_text = self.medium_font.render(f"FLICKER! Change: {data['z_change']:.0f}mm", True, (255, 0, 0))
            self.screen.blit(flicker_text, (400, y_offset - 50))
        
        # Y-coordinate method
        if y_method:
            y_offset += 20
            y_title = self.medium_font.render("Y-COORDINATE METHOD:", True, (0, 255, 255))
            self.screen.blit(y_title, (50, y_offset))
            y_offset += 50
            
            y_dist_text = self.small_font.render(f"Y Bottom Distance: {y_method.get('y_bottom_distance', 0):.0f}px", True, (255, 255, 255))
            self.screen.blit(y_dist_text, (50, y_offset))
            y_offset += 40
            
            est_dist_text = self.small_font.render(f"Estimated Distance: {y_method.get('estimated_distance', 0):.0f}mm", True, (255, 255, 255))
            self.screen.blit(est_dist_text, (50, y_offset))
            y_offset += 50
        
        # FOV analysis
        y_offset += 20
        fov_title = self.medium_font.render("FOV POSITION ANALYSIS:", True, (0, 255, 255))
        self.screen.blit(fov_title, (50, y_offset))
        y_offset += 50
        
        x_ratio = fov_analysis.get('x_position_ratio', 0)
        x_ratio_text = self.small_font.render(f"X Position Ratio: {x_ratio:.3f} (-1=left edge, +1=right edge)", True, (255, 255, 255))
        self.screen.blit(x_ratio_text, (50, y_offset))
        y_offset += 40
        
        edge_color = (255, 0, 0) if fov_analysis.get('is_at_edge', False) else (0, 255, 0)
        edge_text = self.small_font.render(f"At Edge: {fov_analysis.get('is_at_edge', False)}", True, edge_color)
        self.screen.blit(edge_text, (50, y_offset))
        y_offset += 50
        
        # Statistics
        y_offset += 20
        stats_title = self.medium_font.render("STATISTICS:", True, (255, 255, 0))
        self.screen.blit(stats_title, (50, y_offset))
        y_offset += 50
        
        flicker_count = len(self.flicker_events)
        flicker_text = self.small_font.render(f"Total Flicker Events: {flicker_count}", True, (255, 255, 255))
        self.screen.blit(flicker_text, (50, y_offset))
        y_offset += 40
        
        detection_count = len(self.detection_history)
        detection_text = self.small_font.render(f"Total Detections: {detection_count}", True, (255, 255, 255))
        self.screen.blit(detection_text, (50, y_offset))
        y_offset += 40
        
        # Controls
        y_offset += 40
        controls = [
            "SPACE - Pause/Resume",
            "ESC - Exit and save data",
            f"Data logged to: {self.log_filename}"
        ]
        
        for control in controls:
            if "ESC" in control or "SPACE" in control:
                color = (255, 100, 100)
            else:
                color = (200, 200, 200)
            
            control_text = self.small_font.render(control, True, color)
            self.screen.blit(control_text, (50, y_offset))
            y_offset += 35
        
        # Visual FOV indicator
        self.draw_fov_indicator(fov_analysis)
        
        pygame.display.flip()
    
    def draw_fov_indicator(self, fov_analysis):
        """Draw visual FOV position indicator"""
        try:
            # Draw FOV indicator in top right
            indicator_x = self.screen_width - 300
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
            left_text = self.small_font.render("L", True, (255, 255, 255))
            self.screen.blit(left_text, (indicator_x - 20, indicator_y + 15))
            
            right_text = self.small_font.render("R", True, (255, 255, 255))
            self.screen.blit(right_text, (indicator_x + indicator_width + 5, indicator_y + 15))
            
        except Exception:
            pass
    
    def run(self):
        """Run the test program"""
        print(f"Starting Camera Depth Analysis Test")
        print(f"Data will be logged to: {self.log_filename}")
        
        if not self.initialize_camera():
            print("Failed to initialize camera. Test cannot proceed.")
            return
        
        clock = pygame.time.Clock()
        
        try:
            while self.running:
                # Handle events
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.running = False
                        elif event.key == pygame.K_SPACE:
                            self.paused = not self.paused
                            print(f"Test {'PAUSED' if self.paused else 'RESUMED'}")
                    elif event.type == pygame.QUIT:
                        self.running = False
                
                # Process detection if not paused
                if not self.paused:
                    self.process_detection()
                
                # Update display
                self.draw_analysis_display()
                
                # Control frame rate
                clock.tick(10)  # 10 FPS for analysis
                
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources and generate summary report"""
        print(f"\nGenerating summary report...")
        
        # Generate summary
        flicker_count = len(self.flicker_events)
        detection_count = len(self.detection_history)
        
        edge_flickers = sum(1 for event in self.flicker_events if event.get('is_at_edge', False))
        center_flickers = flicker_count - edge_flickers
        
        print(f"\n=== CAMERA DEPTH TEST SUMMARY ===")
        print(f"Total Detections: {detection_count}")
        print(f"Total Flicker Events: {flicker_count}")
        print(f"Flickers at Edge: {edge_flickers}")
        print(f"Flickers at Center: {center_flickers}")
        print(f"Data saved to: {self.log_filename}")
        
        if flicker_count > 0:
            avg_z_change = sum(event.get('z_change', 0) for event in self.flicker_events) / flicker_count
            print(f"Average Z Change During Flicker: {avg_z_change:.0f}mm")
        
        # Cleanup pygame
        pygame.quit()
        
        # Cleanup camera
        if self.camera_sensor:
            try:
                self.camera_sensor.switch_mode(CameraMode.DISABLED)
            except Exception:
                pass


def main():
    """Main function to run the camera depth test"""
    print("Camera Depth vs Y-Coordinate Analysis Test")
    print("==========================================")
    print("This test analyzes Oak-d pro W camera depth perception")
    print("vs y-coordinate distance method for person detection.")
    print()
    print("Instructions:")
    print("1. Stand in front of the camera")
    print("2. Move slowly from left to right")
    print("3. Pay special attention to the far edges")
    print("4. Watch for distance flickering")
    print("5. Press SPACE to pause, ESC to exit")
    print()
    
    analyzer = CameraDepthAnalyzer()
    analyzer.run()


if __name__ == "__main__":
    main()