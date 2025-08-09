#!/usr/bin/env python3
"""
COMPLETE OVERRIDE - FixedSettlementLidarTest.py 
Completely disables original settlement detection that was overwriting my fixed values
"""

import time
import math
import csv
import statistics
from collections import deque
from .StandaloneLidarTest import ExactStandaloneLidarTest


class FixedSettlementLidarTest(ExactStandaloneLidarTest):
    """
    COMPLETE OVERRIDE of settlement detection to prevent conflicts
    """
    
    def __init__(self):
        super().__init__()
        
        # OVERRIDE problematic parameters - BETTER VALUES
        self.person_movement_threshold = 50    # FIXED: was 25 (too sensitive to jitter)
        self.person_settled_time = 1.5         # FIXED: was 1.0 (need longer to avoid false triggers)
        
        # Position smoothing - MORE AGGRESSIVE
        self.position_smoothing_window = deque(maxlen=5)  # INCREASED from 3 to 5
        self.last_smoothed_x = None
        
        # CRITICAL: Disable original settlement detection completely
        self.original_settlement_disabled = True
        
        # CRITICAL: Override head tracking parameters for responsiveness  
        self.angle_change_threshold = math.radians(5)  # FIXED: was 15° (too high) now 5°
        
        print("🔧 COMPLETE OVERRIDE: Settlement detection parameters:")
        print(f"   Movement threshold: {self.person_movement_threshold} pixels (was 25)")
        print(f"   Settlement time: {self.person_settled_time} seconds (was 1.0)")
        print(f"   Angle change threshold: {math.degrees(self.angle_change_threshold):.1f}° (was 15°)")
        print(f"   Original settlement: DISABLED")
    
    def update_conservative_head_tracking(self, person_data):
        """COMPLETE OVERRIDE - Only settlement detection that runs"""
        if not self.head_tracker or not person_data or not self.head_tracking_enabled:
            return
        
        try:
            bbox_center = person_data['bbox_center']
            x_pixels = bbox_center['x_pixels']
            z_camera = person_data['z_camera']
            
            if z_camera <= 0:
                return
            
            # Position smoothing - MORE AGGRESSIVE
            self.position_smoothing_window.append(x_pixels)
            if len(self.position_smoothing_window) >= 3:
                # Use weighted average instead of median for smoother results
                weights = [1, 2, 3, 4, 5][:len(self.position_smoothing_window)]
                weighted_sum = sum(pos * weight for pos, weight in zip(self.position_smoothing_window, weights))
                total_weight = sum(weights)
                smoothed_x = weighted_sum / total_weight
            else:
                smoothed_x = x_pixels
            
            # FIXED settlement detection
            current_time = time.time()
            if self.last_smoothed_x is not None:
                movement = abs(smoothed_x - self.last_smoothed_x)
                
                if movement > self.person_movement_threshold:
                    self.last_significant_movement = current_time
                    self.person_is_settled = False
                    if movement > 50:
                        print(f"🏃 COMPLETE OVERRIDE: Person moving {movement:.1f}px")
                        
                elif current_time - self.last_significant_movement > self.person_settled_time:
                    if not self.person_is_settled:
                        self.person_is_settled = True
                        time_still = current_time - self.last_significant_movement
                        print(f"✅ COMPLETE OVERRIDE: Person SETTLED after {time_still:.1f}s")
                
                # CRITICAL: Update last_x_midpoint for CSV consistency
                self.last_x_midpoint = smoothed_x
                        
            else:
                self.last_significant_movement = current_time
                self.last_x_midpoint = smoothed_x
                print("🔄 COMPLETE OVERRIDE: Settlement started")
            
            self.last_smoothed_x = smoothed_x
            
            # Frame skipping
            self.update_counter_tracking += 1
            if self.update_counter_tracking % self.tracking_update_interval != 0:
                return
            
            # Head tracking only when settled
            if not self.person_is_settled:
                if self.update_counter_tracking % 50 == 0:
                    time_since = current_time - self.last_significant_movement
                    remaining = self.person_settled_time - time_since
                    if remaining > 0:
                        print(f"⏳ COMPLETE OVERRIDE: Waiting {remaining:.1f}s for settlement")
                return
            
            # Head tracking logic
            screen_center_x = self.screen.get_width() // 2
            pixel_offset = smoothed_x - screen_center_x
            pixel_offset_normalized = pixel_offset / screen_center_x
            
            camera_hfov_rad = math.radians(108)
            raw_angle_rad = -pixel_offset_normalized * (camera_hfov_rad / 2.0)
            
            # Angle smoothing
            self.angle_history.append(raw_angle_rad)
            if len(self.angle_history) > self.max_angle_history:
                self.angle_history.pop(0)
            
            # Need enough samples
            if len(self.angle_history) >= 10:
                weights = [i+1 for i in range(len(self.angle_history))]
                weighted_sum = sum(angle * weight for angle, weight in zip(self.angle_history, weights))
                total_weight = sum(weights)
                smoothed_angle_rad = weighted_sum / total_weight
            else:
                print(f"📊 COMPLETE OVERRIDE: Building angle history {len(self.angle_history)}/10")
                return
            
            # Dead zone check - FIXED: Much smaller dead zone
            dead_zone_rad = math.radians(5)  # FIXED: was 20° (too large!) now 5°
            is_in_dead_zone = abs(smoothed_angle_rad) <= dead_zone_rad
            
            # Significant change check
            significant_change = (self.last_sent_angle is None or 
                                abs(smoothed_angle_rad - self.last_sent_angle) > self.angle_change_threshold)
            
            # Execute head tracking
            will_send_command = (not is_in_dead_zone) and significant_change and self.person_is_settled
            
            print(f"🔍 COMPLETE OVERRIDE: Debug - settled:{self.person_is_settled}, dead_zone:{is_in_dead_zone}, significant:{significant_change}, angle:{math.degrees(smoothed_angle_rad):.1f}°, dead_zone_threshold:5°")
            
            if will_send_command:
                print(f"🎯 COMPLETE OVERRIDE: Head tracking to {math.degrees(smoothed_angle_rad):.1f}° (Person settled)")
                self.head_tracker.set_person_tracking(smoothed_angle_rad)
                self.last_sent_angle = smoothed_angle_rad
            
        except Exception as e:
            print(f"⚠️ COMPLETE OVERRIDE: Error: {e}")
    
    def log_detection_to_csv(self, person_data):
        """COMPLETE OVERRIDE of CSV logging to inject our settlement values"""
        try:
            if not self.csv_initialized:
                self.initialize_csv_log()
            
            bbox_center = person_data['bbox_center']
            x_midpoint_pixels = bbox_center['x_pixels']
            x_midpoint_normalized = bbox_center['x_normalized']
            
            # Calculate all the metrics ourselves
            self.x_midpoints_pixels.append(x_midpoint_pixels)
            self.x_midpoints_normalized.append(x_midpoint_normalized)
            self.variance_window.append(x_midpoint_pixels)
            
            # Calculate jump from previous
            x_jump = 0
            is_large_jump = False
            if self.last_x_midpoint is not None:
                x_jump = abs(x_midpoint_pixels - self.last_x_midpoint)
                is_large_jump = x_jump > 50  # Use our threshold
                if is_large_jump:
                    self.large_jumps_count += 1
            
            # Update detection counts  
            self.detection_count += 1
            if not is_large_jump:
                self.consistent_detection_count += 1
            
            # Calculate variance
            variance, std_dev, mean_val = self.calculate_x_midpoint_variance()
            
            # Multi-term variance
            short_var, medium_var, long_var = self.calculate_multi_term_variance(x_midpoint_pixels)
            
            # Stability classification
            stability_class = self.classify_stability(std_dev)
            
            # FORCE our settlement values into CSV
            head_angle_deg = math.degrees(self.last_sent_angle) if self.last_sent_angle else 0.0
            head_tracking_active = self.last_sent_angle is not None
            
            # Calculate elapsed time
            mode_elapsed = time.time() - self.mode_start_time
            
            # Write directly to CSV with OUR values
            with open(self.csv_log_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    mode_elapsed, time.time(), self.update_counter,
                    x_midpoint_pixels, x_midpoint_normalized,
                    person_data['x_camera'], person_data['z_camera'], person_data['raw_z_depth'],
                    person_data['confidence'],
                    person_data['bounding_box']['xmin'], person_data['bounding_box']['ymin'],
                    person_data['bounding_box']['xmax'], person_data['bounding_box']['ymax'],
                    person_data['bounding_box']['xmax'] - person_data['bounding_box']['xmin'],
                    person_data['bounding_box']['ymax'] - person_data['bounding_box']['ymin'],
                    x_jump, is_large_jump,
                    variance, std_dev, mean_val,
                    short_var, medium_var, long_var,
                    self.detection_count, self.consistent_detection_count, self.large_jumps_count,
                    stability_class, head_angle_deg, head_tracking_active,
                    self.person_is_settled,  # FORCE our settlement value
                    person_data.get('detection_method', 'COMPLETE_OVERRIDE')
                ])
            
            # Debug: Show what we actually wrote
            print(f"📝 CSV FORCED: person_is_settled={self.person_is_settled}, head_angle={head_angle_deg:.1f}°")
            
        except Exception as e:
            print(f"⚠️ CSV override error: {e}")
    
    def calculate_x_midpoint_variance(self):
        """Calculate variance for CSV logging"""
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
        """Calculate multi-term variance for CSV logging"""
        if not hasattr(self, 'short_term_variance'):
            self.short_term_variance = deque(maxlen=25)
            self.medium_term_variance = deque(maxlen=125) 
            self.long_term_variance = deque(maxlen=750)
        
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
        """Classify stability for CSV logging"""
        if std_dev_pixels < 10:
            return 'stable'
        elif std_dev_pixels < 25:
            return 'moderate'  
        elif std_dev_pixels < 50:
            return 'unstable'
        else:
            return 'very_unstable'
    
    def conservative_head_tracking(self, person_data):
        """DISABLED - prevent original method from running"""
        pass  # Do nothing - this prevents any original settlement detection
    
    def update(self):
        """Override update to prevent any other settlement detection"""
        # Call parent update but ensure our settlement values aren't overwritten
        settlement_backup = self.person_is_settled
        last_sent_backup = self.last_sent_angle
        
        result = super().update()
        
        # Restore our values if they were overwritten
        if hasattr(self, 'original_settlement_disabled'):
            self.person_is_settled = settlement_backup
            if last_sent_backup is not None:
                self.last_sent_angle = last_sent_backup
        
        return result
    
    def initialise(self):
        """COMPLETE OVERRIDE initialization"""
        super().initialise()
        
        # Reset all settlement variables
        self.last_significant_movement = time.time()
        self.person_is_settled = False
        self.position_smoothing_window.clear()
        self.last_smoothed_x = None
        
        print("✅ COMPLETE OVERRIDE: Settlement detection initialized")
        print(f"📊 Parameters: {self.person_settled_time}s settle, {self.person_movement_threshold}px threshold")


# Export the fixed class
__all__ = ['FixedSettlementLidarTest']