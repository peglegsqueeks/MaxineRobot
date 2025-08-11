#!/usr/bin/env python3
"""
SIMPLE FIX - Just modify the existing FixedSettlementLidarTest parameters
Don't break any existing functionality - just make head movement less sensitive
"""

import time
import math
import csv
import statistics
from collections import deque
from .StandaloneLidarTest import ExactStandaloneLidarTest


class StableHeadLidarTest(ExactStandaloneLidarTest):
    """
    SIMPLE modification of existing working code - just make head movement more stable
    Keeps ALL existing functionality: LiDAR display, person detection, obstacles, etc.
    """
    
    def __init__(self):
        super().__init__()
        
        # SIMPLE CHANGES: Just make head movement much less sensitive
        self.person_movement_threshold = 100   # INCREASED: 100 pixels (was 50)
        self.person_settled_time = 3.0         # INCREASED: 3 seconds (was 1.5)
        
        # SIMPLE CHANGES: More stable head tracking
        self.angle_change_threshold = math.radians(12)  # INCREASED: 12° (was 5°)
        self.tracking_update_interval = 25     # INCREASED: every 25 frames (was 12)
        self.max_angle_history = 25            # INCREASED: 25 samples (was 15)
        
        # Position smoothing - just increase slightly
        self.position_smoothing_window = deque(maxlen=8)  # INCREASED: 8 samples (was 5)
        self.last_smoothed_x = None
        
        print("🔧 SIMPLE HEAD MOVEMENT FIX:")
        print(f"   Movement threshold: {self.person_movement_threshold} pixels (was 50)")
        print(f"   Settlement time: {self.person_settled_time} seconds (was 1.5)")
        print(f"   Angle change threshold: {math.degrees(self.angle_change_threshold):.1f}° (was 5°)")
        print(f"   Update interval: every {self.tracking_update_interval} frames (was 12)")
    
    def update_conservative_head_tracking(self, person_data):
        """SIMPLE FIX - just use more stable parameters in existing logic"""
        if not self.head_tracker or not person_data or not self.head_tracking_enabled:
            return
        
        try:
            bbox_center = person_data['bbox_center']
            x_pixels = bbox_center['x_pixels']
            z_camera = person_data['z_camera']
            
            if z_camera <= 0:
                return
            
            # Person movement detection with MORE STABLE smoothing
            current_time = time.time()
            
            # Add position to smoothing window
            self.position_smoothing_window.append(x_pixels)
            
            # Calculate smoothed position - require more samples
            if len(self.position_smoothing_window) >= 5:  # Need 5 samples minimum
                smoothed_x = sum(self.position_smoothing_window) / len(self.position_smoothing_window)
                
                # Check for significant movement using smoothed position
                if self.last_smoothed_x is not None:
                    movement = abs(smoothed_x - self.last_smoothed_x)
                    if movement > self.person_movement_threshold:
                        self.last_significant_movement = current_time
                        if self.person_is_settled:  # Only print when changing state
                            print(f"🏃 Person moving {movement:.1f}px (threshold: {self.person_movement_threshold}px) - UNSETTLED")
                        self.person_is_settled = False
                    elif current_time - self.last_significant_movement > self.person_settled_time:
                        if not self.person_is_settled:
                            print(f"✅ Person SETTLED after {current_time - self.last_significant_movement:.1f}s")
                        self.person_is_settled = True
                
                self.last_smoothed_x = smoothed_x
            
            # MUCH MORE conservative frame skipping
            self.update_counter_tracking += 1
            if self.update_counter_tracking % self.tracking_update_interval != 0:
                return
            
            # Only track if person is settled for long enough
            if not self.person_is_settled:
                return
            
            # Calculate angle using smoothed position
            screen_center_x = self.screen.get_width() // 2
            pixel_offset = self.last_smoothed_x - screen_center_x
            pixel_offset_normalized = pixel_offset / screen_center_x
            
            camera_hfov_rad = math.radians(108)
            raw_angle_rad = -pixel_offset_normalized * (camera_hfov_rad / 2.0)
            
            # MORE STABLE angle smoothing
            self.angle_history.append(raw_angle_rad)
            if len(self.angle_history) > self.max_angle_history:
                self.angle_history.pop(0)
            
            # Need MORE samples for stability
            if len(self.angle_history) < 15:
                return
            
            # Weighted smoothing favoring recent angles
            weights = [i+1 for i in range(len(self.angle_history))]
            weighted_sum = sum(angle * weight for angle, weight in zip(self.angle_history, weights))
            total_weight = sum(weights)
            smoothed_angle_rad = weighted_sum / total_weight
            
            # LARGER dead zone for more stability (8 degrees instead of 5)
            dead_zone_rad = math.radians(8)
            is_in_dead_zone = abs(smoothed_angle_rad) <= dead_zone_rad
            
            # Significant change check (12 degree minimum instead of 5)
            significant_change = (self.last_sent_angle is None or 
                                abs(smoothed_angle_rad - self.last_sent_angle) > self.angle_change_threshold)
            
            # Execute head tracking ONLY when all conditions met
            will_send_command = (not is_in_dead_zone) and significant_change and self.person_is_settled
            
            print(f"🔍 STABLE Debug - settled:{self.person_is_settled}, dead_zone:{is_in_dead_zone}, significant:{significant_change}, angle:{math.degrees(smoothed_angle_rad):.1f}°, dead_zone:8°")
            
            if will_send_command:
                settle_duration = current_time - self.last_significant_movement
                print(f"🎯 STABLE Head tracking to {math.degrees(smoothed_angle_rad):.1f}° (Person settled {settle_duration:.1f}s)")
                self.head_tracker.set_person_tracking(smoothed_angle_rad)
                self.last_sent_angle = smoothed_angle_rad
            
        except Exception as e:
            print(f"⚠️ STABLE Head tracking error: {e}")
    
    def log_detection_to_csv(self, person_data):
        """SIMPLE FIX - just log ACTUAL values instead of forcing false ones"""
        try:
            if not self.csv_initialized:
                self.initialize_csv_log()
            
            bbox_center = person_data['bbox_center']
            x_midpoint_pixels = bbox_center['x_pixels']
            x_midpoint_normalized = bbox_center['x_normalized']
            
            # Track midpoints for variance analysis (keep existing logic)
            self.x_midpoints_pixels.append(x_midpoint_pixels)
            self.x_midpoints_normalized.append(x_midpoint_normalized)
            self.variance_window.append(x_midpoint_pixels)
            
            # Calculate jump from previous detection (keep existing logic)
            x_jump = 0
            is_large_jump = False
            if self.last_x_midpoint is not None:
                x_jump = abs(x_midpoint_pixels - self.last_x_midpoint)
                is_large_jump = x_jump > self.jump_threshold_pixels
                if is_large_jump:
                    self.large_jumps_count += 1
            
            self.last_x_midpoint = x_midpoint_pixels
            
            # Update detection counts (keep existing logic)
            self.detection_count += 1
            if not is_large_jump:
                self.consistent_detection_count += 1
            
            # Calculate variance metrics (keep existing logic)
            self.current_variance_pixels, self.current_std_dev_pixels, self.current_mean_pixels = self.calculate_x_midpoint_variance()
            
            # Calculate multi-term variance (keep existing logic)
            short_var, medium_var, long_var = self.calculate_multi_term_variance(x_midpoint_pixels)
            
            # Classify stability (keep existing logic)
            stability_class = self.classify_stability(self.current_std_dev_pixels)
            self.stability_zones[stability_class] += 1
            
            # SIMPLE FIX: Log ACTUAL head tracking info instead of forcing 0.0
            head_angle_deg = math.degrees(self.last_sent_angle) if self.last_sent_angle else 0.0
            head_tracking_active = self.head_tracker is not None
            
            # Calculate mode elapsed time (keep existing logic)
            mode_elapsed = time.time() - self.mode_start_time
            
            # Write to CSV with ACTUAL values (not forced fake ones)
            with open(self.csv_log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    mode_elapsed, time.time(), self.update_counter,
                    x_midpoint_pixels, x_midpoint_normalized,
                    person_data['x_camera'], person_data['z_camera'], person_data['raw_z_depth'], person_data['confidence'],
                    person_data['bounding_box']['xmin'], person_data['bounding_box']['ymin'],
                    person_data['bounding_box']['xmax'], person_data['bounding_box']['ymax'],
                    person_data['bounding_box']['xmax'] - person_data['bounding_box']['xmin'],
                    person_data['bounding_box']['ymax'] - person_data['bounding_box']['ymin'],
                    x_jump, is_large_jump,
                    self.current_variance_pixels, self.current_std_dev_pixels, self.current_mean_pixels,
                    short_var, medium_var, long_var,
                    self.detection_count, self.consistent_detection_count, self.large_jumps_count,
                    stability_class, head_angle_deg, head_tracking_active,
                    self.person_is_settled,  # ACTUAL settlement status - NO MORE FORCING
                    person_data.get('detection_method', 'STABLE_HEAD_FIXED')
                ])
            
            # Log ACTUAL values being written (no more fake forced values)
            print(f"📝 ACTUAL CSV: person_is_settled={self.person_is_settled}, head_angle={head_angle_deg:.1f}° (REAL VALUES)")
            
        except Exception as e:
            print(f"⚠️ CSV logging error: {e}")
    
    def initialise(self):
        """SIMPLE initialization - just call parent and reset our variables"""
        super().initialise()
        
        # Reset settlement variables
        self.last_significant_movement = time.time()
        self.person_is_settled = False
        self.position_smoothing_window.clear()
        self.last_smoothed_x = None
        
        print("✅ STABLE HEAD: Settlement detection initialized with stable parameters")


# Export the fixed class
__all__ = ['StableHeadLidarTest']