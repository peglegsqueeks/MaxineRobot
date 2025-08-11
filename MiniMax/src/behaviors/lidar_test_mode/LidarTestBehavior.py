#!/usr/bin/env python3
"""
LIDAR TEST MODE - Fixed with proper detection processing from detection_consistency_test.py
Implements exact same detection pipeline for consistent results
"""

import py_trees
import pygame
import time
import math
import csv
import statistics
from collections import deque
from datetime import datetime
from enum import Enum
import numpy as np

# Robot imports
from robot_config import RobotConfig
from robot_state_manager import RobotStateManager, RobotMode
from sensors.lidar_sensor import LidarSensor
from sensors.camera_sensor import CameraSensor
from sensors.sound_sensor import TTSEngine
from displays.lidar_display import LidarDisplay
from displays.shared_display import SharedDisplay
from controllers.head_controller import HeadController


class LidarTestBehavior(py_trees.behaviour.Behaviour):
    """Fixed Lidar Test Mode with proper detection processing"""
    
    def __init__(self, state_manager, config, lidar_sensor, camera_sensor, 
                 tts_engine, lidar_display, shared_display, head_controller):
        super().__init__(name="LidarTestMode")
        
        # Store references
        self.state_manager = state_manager
        self.config = config
        self.lidar_sensor = lidar_sensor
        self.camera_sensor = camera_sensor
        self.tts_engine = tts_engine
        self.lidar_display = lidar_display
        self.shared_display = shared_display
        self.head_controller = head_controller
        
        # Mode tracking
        self.mode_start_time = None
        self.test_duration = 30.0
        self.mode_active = False
        
        # CRITICAL: Frame timing control
        self.last_update_time = 0
        self.target_fps = 25
        self.frame_interval = 1.0 / self.target_fps
        self.frame_counter = 0
        self.detection_skip_frames = 1  # Process every frame like standalone
        
        # FPS monitoring
        self.fps_counter = deque(maxlen=30)
        self.last_frame_time = time.time()
        self.current_fps = 0.0
        
        # Detection tracking - EXACT from standalone
        self.current_detection = None
        self.detection_history = deque(maxlen=2000)
        self.has_detection = False
        
        # X midpoint variance tracking - EXACT from standalone
        self.x_midpoints = deque(maxlen=1000)
        self.x_midpoints_pixels = deque(maxlen=1000)
        self.x_midpoints_normalized = deque(maxlen=1000)
        self.variance_window = deque(maxlen=100)
        self.current_variance_pixels = 0.0
        self.current_std_dev_pixels = 0.0
        self.current_mean_pixels = 0.0
        
        # Consistency metrics
        self.detection_count = 0
        self.consistent_detection_count = 0
        self.large_jumps_count = 0
        self.jump_threshold_pixels = 50
        self.last_x_midpoint = None
        
        # Multi-term variance
        self.short_term_variance = deque(maxlen=25)
        self.medium_term_variance = deque(maxlen=125)
        self.long_term_variance = deque(maxlen=750)
        
        # Stability zones
        self.stability_zones = {
            'stable': 0,
            'moderate': 0,
            'unstable': 0,
            'very_unstable': 0
        }
        
        # Z-depth smoothing - EXACT from standalone
        self.z_depth_smoother = deque(maxlen=5)
        self.confidence_weights = deque(maxlen=5)
        self.smoothed_z_depth = 0
        self.depth_trust_threshold = 0.6
        
        # Head tracking
        self.head_angle = 0.0
        self.head_tracking_active = False
        self.person_settled_threshold = 10
        self.person_stable_count = 0
        
        # CSV logging
        self.log_filename = "LIDARTEST.csv"
        self.csv_initialized = False
        
        # Display settings
        self.variance_summary_interval = 5.0
        self.last_variance_summary_time = 0
        
        # CRITICAL: Queue drain flag
        self.queue_drained = False
    
    def setup(self, **kwargs):
        """Prepare for execution"""
        self.mode_active = False
        self.mode_start_time = None
        self.queue_drained = False
        return py_trees.common.Status.SUCCESS
    
    def initialise(self):
        """Called when behavior starts - initialize test mode"""
        self.mode_active = True
        self.mode_start_time = time.time()
        self.last_update_time = time.time()
        self.last_frame_time = time.time()
        self.frame_counter = 0
        self.queue_drained = False
        
        # Clear all tracking data
        self.detection_count = 0
        self.consistent_detection_count = 0
        self.large_jumps_count = 0
        self.last_x_midpoint = None
        
        self.x_midpoints.clear()
        self.x_midpoints_pixels.clear()
        self.x_midpoints_normalized.clear()
        self.variance_window.clear()
        self.short_term_variance.clear()
        self.medium_term_variance.clear()
        self.long_term_variance.clear()
        self.detection_history.clear()
        
        self.z_depth_smoother.clear()
        self.confidence_weights.clear()
        
        self.stability_zones = {
            'stable': 0,
            'moderate': 0,
            'unstable': 0,
            'very_unstable': 0
        }
        
        # Initialize CSV
        self.initialize_csv_log()
        
        # CRITICAL: Drain the detection queue to remove stale data
        self.drain_detection_queue()
        
        # Set head to center
        if self.head_controller:
            self.head_controller.set_angle(0)
            self.head_angle = 0.0
        
        # Announce mode
        if self.tts_engine:
            self.tts_engine.speak("Test Lidar")
        
        print("\n" + "="*60)
        print("🎯 LIDAR TEST MODE - FIXED DETECTION PIPELINE")
        print("="*60)
        print("✅ LiDAR Test Mode initialized with fixed camera pipeline")
        print(f"📊 Logging variance data to: {self.log_filename}")
        print("📍 Grid overlay: ON | LiDAR obstacles: ON | Person circle: ON")
        print("-"*60)
    
    def drain_detection_queue(self):
        """CRITICAL: Drain stale detections from queue"""
        if not self.camera_sensor or not self.camera_sensor.camera_initialized:
            return
        
        try:
            # Get the detection queue directly
            detection_queue = self.camera_sensor.detection_queue
            if detection_queue:
                # Drain all old detections
                drained_count = 0
                while True:
                    old_detection = detection_queue.tryGet()
                    if old_detection is None:
                        break
                    drained_count += 1
                    if drained_count > 100:  # Safety limit
                        break
                
                if drained_count > 0:
                    print(f"🔄 Drained {drained_count} stale detections from queue")
                
                # Mark as drained
                self.queue_drained = True
                
                # Wait for fresh data
                time.sleep(0.1)
        except Exception as e:
            print(f"⚠️ Error draining queue: {e}")
    
    def smooth_z_depth(self, raw_z_depth, confidence):
        """Smooth Z depth - EXACT from standalone"""
        try:
            self.z_depth_smoother.append(raw_z_depth)
            self.confidence_weights.append(confidence)
            
            if len(self.z_depth_smoother) < 2:
                self.smoothed_z_depth = raw_z_depth
                return raw_z_depth
            
            total_weight = 0
            weighted_sum = 0
            
            for i, (z_val, conf) in enumerate(zip(self.z_depth_smoother, self.confidence_weights)):
                recency_weight = (i + 1) / len(self.z_depth_smoother)
                confidence_weight = max(0.1, conf)
                combined_weight = recency_weight * confidence_weight
                
                weighted_sum += z_val * combined_weight
                total_weight += combined_weight
            
            smoothed = weighted_sum / total_weight if total_weight > 0 else raw_z_depth
            
            if confidence < self.depth_trust_threshold and len(self.z_depth_smoother) > 1:
                prev_z = self.z_depth_smoother[-2]
                max_jump = 500
                if abs(smoothed - prev_z) > max_jump:
                    blend_factor = confidence / self.depth_trust_threshold
                    smoothed = prev_z + (smoothed - prev_z) * blend_factor
            
            self.smoothed_z_depth = smoothed
            return smoothed
        except Exception:
            return raw_z_depth
    
    def process_camera_detection(self):
        """Process detection - EXACT logic from standalone test"""
        if not self.camera_sensor or not self.camera_sensor.camera_initialized:
            return None
        
        if not self.queue_drained:
            self.drain_detection_queue()
            return None
        
        try:
            # CRITICAL: Use tryGet() to avoid blocking
            detection_queue = self.camera_sensor.detection_queue
            if not detection_queue:
                return None
            
            detections = detection_queue.tryGet()
            if not detections:
                return None
            
            # Filter for person detections (label 15)
            person_detections = [det for det in detections.detections if det.label == 15]
            
            if not person_detections:
                return None
            
            # CRITICAL: Always take the CLOSEST person by Z depth
            # This prevents label swapping issues
            closest_person = min(person_detections, key=lambda p: p.spatialCoordinates.z)
            
            # Get coordinates
            x_camera = closest_person.spatialCoordinates.x
            y_camera = closest_person.spatialCoordinates.y
            raw_z_depth = closest_person.spatialCoordinates.z
            confidence = closest_person.confidence
            
            # Validate Z depth
            if raw_z_depth <= 0 or raw_z_depth > 15000:
                return None
            
            # Smooth Z depth
            smoothed_z_depth = self.smooth_z_depth(raw_z_depth, confidence)
            
            # Calculate X midpoint - EXACT from standalone
            bbox_xmin = closest_person.xmin
            bbox_xmax = closest_person.xmax
            bbox_ymin = closest_person.ymin
            bbox_ymax = closest_person.ymax
            
            # X midpoint calculations
            x_midpoint_normalized = (bbox_xmin + bbox_xmax) / 2.0
            
            # Get screen dimensions
            if self.shared_display and self.shared_display.screen:
                screen_width = self.shared_display.screen.get_width()
            else:
                screen_width = 1920  # Default
            
            x_midpoint_pixels = int(x_midpoint_normalized * screen_width)
            
            # Track midpoints
            self.x_midpoints.append(x_camera)
            self.x_midpoints_pixels.append(x_midpoint_pixels)
            self.x_midpoints_normalized.append(x_midpoint_normalized)
            self.variance_window.append(x_midpoint_pixels)
            
            # Calculate jump from previous
            x_jump = 0
            is_large_jump = False
            if self.last_x_midpoint is not None:
                x_jump = abs(x_midpoint_pixels - self.last_x_midpoint)
                is_large_jump = x_jump > self.jump_threshold_pixels
                if is_large_jump:
                    self.large_jumps_count += 1
            
            self.last_x_midpoint = x_midpoint_pixels
            
            # Update counts
            self.detection_count += 1
            if not is_large_jump:
                self.consistent_detection_count += 1
            
            # Calculate variance metrics
            self.calculate_variance_metrics()
            
            # Calculate multi-term variance
            self.short_term_variance.append(x_midpoint_pixels)
            self.medium_term_variance.append(x_midpoint_pixels)
            self.long_term_variance.append(x_midpoint_pixels)
            
            short_var = self.calculate_deque_variance(self.short_term_variance)
            medium_var = self.calculate_deque_variance(self.medium_term_variance)
            long_var = self.calculate_deque_variance(self.long_term_variance)
            
            # Classify stability
            stability_class = self.classify_stability(self.current_std_dev_pixels)
            self.stability_zones[stability_class] += 1
            
            # Check if person is stable for head tracking
            if self.current_std_dev_pixels < 25:  # Stable enough
                self.person_stable_count += 1
            else:
                self.person_stable_count = 0
            
            person_is_settled = self.person_stable_count >= self.person_settled_threshold
            
            # Head tracking (only if stable)
            if person_is_settled and self.head_controller:
                target_angle = self.calculate_head_angle(x_midpoint_normalized)
                if abs(target_angle - self.head_angle) > 2:  # Only move if significant change
                    self.head_controller.set_angle(target_angle)
                    self.head_angle = target_angle
                    self.head_tracking_active = True
            
            # Create detection data
            detection_data = {
                'mode_time_elapsed': time.time() - self.mode_start_time,
                'timestamp': time.time(),
                'frame_number': self.frame_counter,
                'x_midpoint_pixels': x_midpoint_pixels,
                'x_midpoint_normalized': x_midpoint_normalized,
                'x_camera_mm': x_camera,
                'z_depth_mm': smoothed_z_depth,
                'raw_z_depth_mm': raw_z_depth,
                'confidence': confidence,
                'bbox_xmin': bbox_xmin,
                'bbox_ymin': bbox_ymin,
                'bbox_xmax': bbox_xmax,
                'bbox_ymax': bbox_ymax,
                'bbox_width': bbox_xmax - bbox_xmin,
                'bbox_height': bbox_ymax - bbox_ymin,
                'x_jump_from_previous': x_jump,
                'is_large_jump': int(is_large_jump),
                'rolling_variance_pixels': self.current_variance_pixels,
                'rolling_std_dev_pixels': self.current_std_dev_pixels,
                'rolling_mean_pixels': self.current_mean_pixels,
                'short_term_variance': short_var,
                'medium_term_variance': medium_var,
                'long_term_variance': long_var,
                'detection_count': self.detection_count,
                'consistent_detections': self.consistent_detection_count,
                'large_jumps_count': self.large_jumps_count,
                'stability_classification': stability_class,
                'head_angle_deg': self.head_angle,
                'head_tracking_active': int(self.head_tracking_active),
                'person_is_settled': int(person_is_settled),
                'detection_method': 'camera'
            }
            
            # Log to CSV
            self.log_to_csv(detection_data)
            
            # Store as current
            self.current_detection = detection_data
            self.detection_history.append(detection_data)
            
            # Print status with stability indicator
            if self.current_std_dev_pixels < 10:
                stability_icon = "✅ STABLE"
                stability_color = ""
            elif self.current_std_dev_pixels < 25:
                stability_icon = "🔶 MODERATE"
                stability_color = ""
            elif self.current_std_dev_pixels < 50:
                stability_icon = "⚠️ UNSTABLE"
                stability_color = ""
            else:
                stability_icon = "❌ VERY UNSTABLE"
                stability_color = ""
            
            print(f"📍 X:{x_midpoint_pixels:4d}px | Variance: ±{self.current_std_dev_pixels:5.1f}px | {stability_icon} | Z:{int(smoothed_z_depth)}mm")
            
            # Check for multiple people (debugging)
            if len(person_detections) > 1:
                print(f"⚠️ Multiple people detected: {len(person_detections)}")
                for i, person in enumerate(person_detections[:3]):  # Show first 3
                    px = int(((person.xmin + person.xmax) / 2.0) * screen_width)
                    pz = int(person.spatialCoordinates.z)
                    pc = person.confidence
                    print(f"  Person {i}: X={px}px, Z={pz}mm, Conf={pc:.2f}")
            
            return detection_data
            
        except Exception as e:
            print(f"⚠️ Detection processing error: {e}")
            return None
    
    def calculate_variance_metrics(self):
        """Calculate variance metrics - EXACT from standalone"""
        try:
            if len(self.variance_window) < 2:
                self.current_variance_pixels = 0.0
                self.current_std_dev_pixels = 0.0
                self.current_mean_pixels = 0.0
                return
            
            window_data = list(self.variance_window)
            self.current_mean_pixels = statistics.mean(window_data)
            self.current_variance_pixels = statistics.variance(window_data)
            self.current_std_dev_pixels = math.sqrt(self.current_variance_pixels)
        except Exception:
            self.current_variance_pixels = 0.0
            self.current_std_dev_pixels = 0.0
            self.current_mean_pixels = 0.0
    
    def calculate_deque_variance(self, data_deque):
        """Calculate variance for a deque"""
        try:
            if len(data_deque) < 2:
                return 0.0
            return statistics.variance(list(data_deque))
        except Exception:
            return 0.0
    
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
    
    def calculate_head_angle(self, x_normalized):
        """Calculate head angle for tracking"""
        # Convert normalized x (0-1) to angle (-45 to +45)
        angle = (x_normalized - 0.5) * 90
        return max(-45, min(45, angle))
    
    def calculate_fps(self):
        """Calculate current FPS"""
        current_time = time.time()
        frame_time = current_time - self.last_frame_time
        self.last_frame_time = current_time
        
        if frame_time > 0:
            fps = 1.0 / frame_time
            self.fps_counter.append(fps)
            if len(self.fps_counter) > 0:
                self.current_fps = sum(self.fps_counter) / len(self.fps_counter)
    
    def initialize_csv_log(self):
        """Initialize CSV logging"""
        if self.csv_initialized:
            return
        
        try:
            with open(self.log_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'mode_time_elapsed', 'timestamp', 'frame_number',
                    'x_midpoint_pixels', 'x_midpoint_normalized', 'x_camera_mm',
                    'z_depth_mm', 'raw_z_depth_mm', 'confidence',
                    'bbox_xmin', 'bbox_ymin', 'bbox_xmax', 'bbox_ymax',
                    'bbox_width', 'bbox_height',
                    'x_jump_from_previous', 'is_large_jump',
                    'rolling_variance_pixels', 'rolling_std_dev_pixels', 'rolling_mean_pixels',
                    'short_term_variance', 'medium_term_variance', 'long_term_variance',
                    'detection_count', 'consistent_detections', 'large_jumps_count',
                    'stability_classification', 'head_angle_deg', 
                    'head_tracking_active', 'person_is_settled', 'detection_method'
                ])
            self.csv_initialized = True
            print(f"✅ CSV log created: {self.log_filename}")
        except Exception as e:
            print(f"⚠️ CSV initialization failed: {e}")
    
    def log_to_csv(self, data):
        """Log data to CSV"""
        try:
            with open(self.log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    data['mode_time_elapsed'],
                    data['timestamp'],
                    data['frame_number'],
                    data['x_midpoint_pixels'],
                    data['x_midpoint_normalized'],
                    data['x_camera_mm'],
                    data['z_depth_mm'],
                    data['raw_z_depth_mm'],
                    data['confidence'],
                    data['bbox_xmin'],
                    data['bbox_ymin'],
                    data['bbox_xmax'],
                    data['bbox_ymax'],
                    data['bbox_width'],
                    data['bbox_height'],
                    data['x_jump_from_previous'],
                    data['is_large_jump'],
                    data['rolling_variance_pixels'],
                    data['rolling_std_dev_pixels'],
                    data['rolling_mean_pixels'],
                    data['short_term_variance'],
                    data['medium_term_variance'],
                    data['long_term_variance'],
                    data['detection_count'],
                    data['consistent_detections'],
                    data['large_jumps_count'],
                    data['stability_classification'],
                    data['head_angle_deg'],
                    data['head_tracking_active'],
                    data['person_is_settled'],
                    data['detection_method']
                ])
        except Exception:
            pass
    
    def print_variance_summary(self):
        """Print variance summary"""
        print("\n" + "-"*60)
        print("📊 VARIANCE SUMMARY")
        print("-"*60)
        print(f"Total Detections: {self.detection_count}")
        
        if self.detection_count > 0:
            consistency_rate = (self.consistent_detection_count / self.detection_count) * 100
            print(f"Consistency Rate: {consistency_rate:.1f}%")
        
        print(f"Large Jumps (>{self.jump_threshold_pixels}px): {self.large_jumps_count}")
        print(f"Current Std Dev: ±{self.current_std_dev_pixels:.1f}px")
        print(f"Current Mean X: {self.current_mean_pixels:.0f}px")
        
        # Stability distribution
        total_zones = sum(self.stability_zones.values())
        if total_zones > 0:
            print("\nStability Distribution:")
            for zone, count in self.stability_zones.items():
                percentage = (count / total_zones) * 100
                if zone == 'stable':
                    print(f"  Stable (<10px):      {percentage:6.1f}%")
                elif zone == 'moderate':
                    print(f"  Moderate (10-25px):  {percentage:6.1f}%")
                elif zone == 'unstable':
                    print(f"  Unstable (25-50px):  {percentage:6.1f}%")
                else:
                    print(f"  Very Unstable (>50px): {percentage:6.1f}%")
        print("-"*60)
    
    def update_display(self):
        """Update the display with lidar and detection info"""
        if not self.lidar_display or not self.shared_display:
            return
        
        try:
            # Get lidar data
            lidar_data = self.lidar_sensor.get_scan_data() if self.lidar_sensor else None
            
            # Prepare detection info for display
            detection_info = None
            if self.current_detection:
                detection_info = {
                    'x_pixels': self.current_detection['x_midpoint_pixels'],
                    'z_mm': self.current_detection['z_depth_mm'],
                    'variance': self.current_std_dev_pixels,
                    'stability': self.current_detection['stability_classification'],
                    'head_angle': self.head_angle
                }
            
            # Update lidar display
            self.lidar_display.draw_lidar_view(
                lidar_data=lidar_data,
                person_detection=detection_info,
                show_grid=True,
                show_obstacles=True,
                show_person=True
            )
            
            # Update shared display
            self.shared_display.refresh()
            
        except Exception as e:
            print(f"⚠️ Display update error: {e}")
    
    def update(self):
        """Main update - called by py_trees"""
        if not self.mode_active:
            return py_trees.common.Status.FAILURE
        
        current_time = time.time()
        
        # CRITICAL: Enforce frame timing
        time_since_last_update = current_time - self.last_update_time
        if time_since_last_update < self.frame_interval:
            # Skip this update to maintain consistent FPS
            return py_trees.common.Status.RUNNING
        
        self.last_update_time = current_time
        
        # Update frame counter and FPS
        self.frame_counter += 1
        self.calculate_fps()
        
        # Process detection if it's time
        if self.frame_counter % self.detection_skip_frames == 0:
            self.process_camera_detection()
        
        # Update display
        self.update_display()
        
        # Print variance summary periodically
        if current_time - self.last_variance_summary_time > self.variance_summary_interval:
            self.print_variance_summary()
            self.last_variance_summary_time = current_time
        
        # Check for ESC key
        keys = pygame.key.get_pressed()
        if keys[pygame.K_ESCAPE]:
            return py_trees.common.Status.SUCCESS
        
        # Check test duration
        elapsed = current_time - self.mode_start_time
        if elapsed >= self.test_duration:
            self.print_final_summary()
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING
    
    def print_final_summary(self):
        """Print final test summary"""
        print("\n" + "="*60)
        print("📊 LIDAR TEST MODE - FINAL VARIANCE SUMMARY")
        print("="*60)
        
        test_duration = time.time() - self.mode_start_time
        print(f"Total Test Duration: {test_duration:.1f} seconds")
        print(f"Total Detections: {self.detection_count}")
        
        if self.detection_count > 0:
            consistency_rate = (self.consistent_detection_count / self.detection_count) * 100
            print(f"Consistency Rate: {consistency_rate:.1f}%")
            
            if len(self.x_midpoints_pixels) > 1:
                overall_variance = statistics.variance(list(self.x_midpoints_pixels))
                overall_std_dev = math.sqrt(overall_variance)
                print(f"Overall Std Dev: ±{overall_std_dev:.1f}px")
        
        print(f"Large Jumps (>{self.jump_threshold_pixels}px): {self.large_jumps_count}")
        print(f"Data saved to: {self.log_filename}")
        print("="*60)
    
    def terminate(self, new_status):
        """Called when behavior stops"""
        self.mode_active = False
        
        # Center head
        if self.head_controller:
            self.head_controller.set_angle(0)
        
        # Final summary if we ran
        if self.mode_start_time:
            elapsed = time.time() - self.mode_start_time
            if elapsed > 2:  # Only if we ran for more than 2 seconds
                self.print_final_summary()


def make_lidar_test_behavior(state_manager, config, lidar_sensor=None, 
                             camera_sensor=None, tts_engine=None, 
                             lidar_display=None, shared_display=None,
                             head_controller=None):
    """Factory function to create Lidar Test behavior"""
    return LidarTestBehavior(
        state_manager=state_manager,
        config=config,
        lidar_sensor=lidar_sensor,
        camera_sensor=camera_sensor,
        tts_engine=tts_engine,
        lidar_display=lidar_display,
        shared_display=shared_display,
        head_controller=head_controller
    )