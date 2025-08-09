import math
import time
from enum import Enum
from typing import Optional, Tuple
import depthai as dai
from src.path_finding.Position import Position


class PersonZone(Enum):
    """5-zone person detection areas"""
    FAR_LEFT = "far_left"
    LEFT = "left" 
    MIDDLE = "middle"
    RIGHT = "right"
    FAR_RIGHT = "far_right"


class PersonTracker:
    """
    Tracks person position and provides 5-zone detection for navigation decisions
    Handles coordinate transformations and position smoothing
    """
    
    def __init__(self, camera_fov_degrees=110, smoothing_factor=0.7):
        self.camera_fov = camera_fov_degrees
        self.smoothing_factor = smoothing_factor
        
        # Zone boundaries (percentage of screen width)
        self.zone_boundaries = {
            PersonZone.FAR_LEFT: (-1.0, -0.6),   # Far left zone
            PersonZone.LEFT: (-0.6, -0.2),       # Left zone  
            PersonZone.MIDDLE: (-0.2, 0.2),      # Middle zone
            PersonZone.RIGHT: (0.2, 0.6),        # Right zone
            PersonZone.FAR_RIGHT: (0.6, 1.0)     # Far right zone
        }
        
        # Person tracking state
        self.last_position = None
        self.last_zone = PersonZone.MIDDLE
        self.position_history = []
        self.max_history = 5
        
        # Distance thresholds for movement decisions
        self.close_distance_threshold = 1500  # mm - close enough for forward movement
        self.very_close_threshold = 500       # mm - stop distance
        
    def normalize_camera_x(self, x_camera_mm: float, distance_mm: float) -> float:
        """
        Normalize camera X coordinate to -1.0 to 1.0 range
        Accounts for perspective - objects farther away appear smaller
        """
        try:
            # Calculate angular position from camera center
            if distance_mm <= 0:
                return 0.0
                
            # Convert to angular offset
            angle_rad = math.atan2(x_camera_mm, distance_mm)
            
            # Normalize to FOV range (-1.0 to 1.0)
            max_angle_rad = math.radians(self.camera_fov / 2)
            normalized_x = angle_rad / max_angle_rad
            
            # Clamp to valid range
            return max(-1.0, min(1.0, normalized_x))
            
        except Exception as e:
            return 0.0
    
    def get_person_zone(self, normalized_x: float) -> PersonZone:
        """Determine which zone the person is in based on normalized X position"""
        for zone, (min_x, max_x) in self.zone_boundaries.items():
            if min_x <= normalized_x < max_x:
                return zone
        
        # Default to middle if somehow outside all zones
        return PersonZone.MIDDLE
    
    def smooth_position(self, new_position: Position) -> Position:
        """Apply position smoothing to reduce jitter"""
        if self.last_position is None:
            self.last_position = new_position
            return new_position
        
        # Smooth angle and distance separately
        smooth_angle = (self.last_position.angle * self.smoothing_factor + 
                       new_position.angle * (1 - self.smoothing_factor))
        
        smooth_distance = (self.last_position.distance * self.smoothing_factor + 
                          new_position.distance * (1 - self.smoothing_factor))
        
        smoothed = Position(angle=smooth_angle, distance=smooth_distance)
        self.last_position = smoothed
        return smoothed
    
    def update_person_detection(self, target_person: dai.SpatialImgDetection, 
                               head_angle_degrees: float = 0.0) -> Tuple[Position, PersonZone]:
        """
        Update person tracking with latest detection
        
        Args:
            target_person: Detection from camera
            head_angle_degrees: Current head angle for coordinate transformation
            
        Returns:
            Tuple of (smoothed_position, person_zone)
        """
        try:
            # Extract camera coordinates
            x_camera = target_person.spatialCoordinates.x  # mm left/right from camera
            z_camera = target_person.spatialCoordinates.z  # mm forward from camera
            
            # Account for head rotation in coordinate transformation
            head_angle_rad = math.radians(head_angle_degrees)
            
            # Transform camera coordinates to robot-relative coordinates
            # When head is turned, person's apparent angle changes
            camera_angle = math.atan2(x_camera, z_camera)
            robot_angle = camera_angle + head_angle_rad
            
            # Calculate distance (account for robot geometry - camera is ~50cm above and 20cm back from lidar)
            # Use 3D distance calculation
            camera_height_offset = 500  # mm
            camera_back_offset = 200    # mm
            
            # Adjust for camera position relative to robot center
            adjusted_distance = max(100, z_camera - camera_back_offset)
            
            # Create position relative to robot center
            robot_position = Position(angle=robot_angle, distance=adjusted_distance)
            
            # Apply smoothing
            smoothed_position = self.smooth_position(robot_position)
            
            # Determine zone based on normalized camera X (before head angle adjustment)
            normalized_x = self.normalize_camera_x(x_camera, z_camera)
            person_zone = self.get_person_zone(normalized_x)
            
            # Update history
            self.position_history.append(smoothed_position)
            if len(self.position_history) > self.max_history:
                self.position_history.pop(0)
            
            self.last_zone = person_zone
            
            return smoothed_position, person_zone
            
        except Exception as e:
            # Return last known position if available
            if self.last_position:
                return self.last_position, self.last_zone
            else:
                return Position(angle=0.0, distance=2000.0), PersonZone.MIDDLE
    
    def get_movement_recommendation(self, person_zone: PersonZone, 
                                  distance_mm: float, 
                                  obstacles_in_path: bool = False) -> str:
        """
        Get movement recommendation based on person zone and obstacles
        
        Returns:
            Movement command string
        """
        # Check if person is within stopping distance
        if distance_mm < self.very_close_threshold:
            return "STOP"
        
        # If obstacles block path, defer to pathfinding
        if obstacles_in_path:
            return "PATHFIND"
        
        # Simple movement logic when path is clear
        if person_zone == PersonZone.MIDDLE:
            return "FORWARD"
        elif person_zone == PersonZone.LEFT:
            if distance_mm > self.close_distance_threshold:
                return "TURN_LEFT"  # Turn on spot if far
            else:
                return "FORWARD_LEFT"  # Move while turning if close
        elif person_zone == PersonZone.RIGHT:
            if distance_mm > self.close_distance_threshold:
                return "TURN_RIGHT"  # Turn on spot if far
            else:
                return "FORWARD_RIGHT"  # Move while turning if close
        elif person_zone == PersonZone.FAR_LEFT:
            return "TURN_LEFT"  # Always turn on spot for far zones
        elif person_zone == PersonZone.FAR_RIGHT:
            return "TURN_RIGHT"  # Always turn on spot for far zones
        
        return "FORWARD"  # Default
    
    def get_head_tracking_angle(self, person_zone: PersonZone, 
                               current_head_angle: float) -> Optional[float]:
        """
        Get recommended head angle to track person
        
        Args:
            person_zone: Current person zone
            current_head_angle: Current head position in degrees
            
        Returns:
            Target head angle in degrees, or None if no change needed
        """
        # Target angles for each zone (degrees)
        zone_head_angles = {
            PersonZone.FAR_LEFT: -60,
            PersonZone.LEFT: -30,
            PersonZone.MIDDLE: 0,
            PersonZone.RIGHT: 30,
            PersonZone.FAR_RIGHT: 60
        }
        
        target_angle = zone_head_angles.get(person_zone, 0)
        
        # Only return new angle if significantly different
        if abs(target_angle - current_head_angle) > 10:
            return target_angle
        
        return None
    
    def is_person_stable(self, min_detections: int = 3) -> bool:
        """Check if person detection is stable (multiple consistent detections)"""
        return len(self.position_history) >= min_detections
    
    def get_average_position(self) -> Optional[Position]:
        """Get average position from recent history for stability"""
        if not self.position_history:
            return None
        
        avg_angle = sum(pos.angle for pos in self.position_history) / len(self.position_history)
        avg_distance = sum(pos.distance for pos in self.position_history) / len(self.position_history)
        
        return Position(angle=avg_angle, distance=avg_distance)
    
    def reset_tracking(self):
        """Reset tracking state"""
        self.last_position = None
        self.last_zone = PersonZone.MIDDLE
        self.position_history.clear()
    
    def get_status(self) -> dict:
        """Get current tracking status for debugging"""
        return {
            'last_zone': self.last_zone.value if self.last_zone else None,
            'position_history_count': len(self.position_history),
            'last_position': str(self.last_position) if self.last_position else None,
            'is_stable': self.is_person_stable()
        }