#!/usr/bin/env python3
"""
Potential Field Navigation System for Maxine Robot
Uses attractive forces toward detected person and repulsive forces from LiDAR obstacles
Accounts for correct coordinate transformations and robot dimensions

File: src/behaviors/lidar_test_mode/PotentialFieldNavigation.py
"""

import math
import time
from typing import List, Tuple, Optional
from dataclasses import dataclass
from collections import deque

from src.types.MovementDirection import MovementDirection
from src.action_managers.VelocityManager import VelocityConfig


@dataclass
class PotentialFieldConfig:
    """Configuration for potential field navigation"""
    # Attractive force parameters (toward person)
    attractive_strength: float = 2.0          # Strength of attraction to person
    attractive_max_distance: float = 3000.0   # Max distance for attraction (mm)
    target_distance: float = 1500.0           # Desired distance from person (mm)
    
    # Repulsive force parameters (from obstacles)
    repulsive_strength: float = 1.5           # Strength of repulsion from obstacles
    repulsive_max_distance: float = 800.0     # Max distance for repulsion (mm)
    repulsive_min_distance: float = 200.0     # Min safe distance (mm)
    
    # Robot physical parameters
    robot_length: float = 660.0               # Robot length (mm)
    robot_width: float = 550.0                # Robot width (mm)
    safety_buffer: float = 100.0              # Additional safety margin (mm)
    
    # Movement parameters
    max_linear_speed: float = 0.8             # Max forward/backward speed
    max_angular_speed: float = 0.6            # Max rotation speed
    movement_threshold: float = 0.1           # Min force to trigger movement
    angular_threshold: float = 0.05           # Min angular force to trigger rotation
    
    # Obstacle filtering
    obstacle_min_distance: float = 100.0      # Ignore obstacles closer than this
    obstacle_max_distance: float = 6000.0     # Ignore obstacles farther than this
    
    # Navigation behavior
    stop_at_target: bool = True               # Stop when reaching target distance
    rotate_in_place_threshold: float = 45.0   # Degrees - rotate in place if person is to side


class Vector2D:
    """Simple 2D vector class for force calculations"""
    
    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = x
        self.y = y
    
    def __add__(self, other: 'Vector2D') -> 'Vector2D':
        return Vector2D(self.x + other.x, self.y + other.y)
    
    def __mul__(self, scalar: float) -> 'Vector2D':
        return Vector2D(self.x * scalar, self.y * scalar)
    
    def magnitude(self) -> float:
        return math.sqrt(self.x**2 + self.y**2)
    
    def normalize(self) -> 'Vector2D':
        mag = self.magnitude()
        if mag > 0:
            return Vector2D(self.x / mag, self.y / mag)
        return Vector2D(0, 0)
    
    def to_polar(self) -> Tuple[float, float]:
        """Convert to polar coordinates (angle, magnitude)"""
        angle = math.atan2(self.x, self.y)  # Note: y-forward coordinate system
        magnitude = self.magnitude()
        return angle, magnitude


class PotentialFieldNavigator:
    """
    Potential field navigation system for person-following with obstacle avoidance
    """
    
    def __init__(self, config: PotentialFieldConfig = None):
        self.config = config or PotentialFieldConfig()
        
        # Navigation state
        self.current_person_position: Optional[Tuple[float, float]] = None  # (angle, distance) in robot coords
        self.current_obstacles: List[Tuple[float, float]] = []              # [(angle, distance), ...] in robot coords
        self.last_navigation_time: float = 0.0
        self.navigation_active: bool = False
        
        # Force calculation history for smoothing
        self.force_history: deque = deque(maxlen=5)
        self.movement_history: deque = deque(maxlen=10)
        
        # Navigation metrics for CSV logging
        self.navigation_metrics = {
            'total_force_magnitude': 0.0,
            'attractive_force_magnitude': 0.0,
            'repulsive_force_magnitude': 0.0,
            'movement_command': MovementDirection.NONE,
            'movement_speed': 0.0,
            'distance_to_person': 0.0,
            'obstacles_in_range': 0,
            'navigation_decision': 'stopped'
        }
    
    def set_person_position(self, angle_rad: float, distance_mm: float):
        """
        Set the current person position in robot coordinates
        
        Args:
            angle_rad: Angle to person in radians (robot coordinate system)
            distance_mm: Distance to person in millimeters
        """
        self.current_person_position = (angle_rad, distance_mm)
        self.navigation_metrics['distance_to_person'] = distance_mm
    
    def set_obstacles(self, obstacles: List[Tuple[float, float]]):
        """
        Set current obstacles in robot coordinates
        
        Args:
            obstacles: List of (angle_rad, distance_mm) tuples
        """
        # Filter obstacles within relevant range
        filtered_obstacles = []
        for angle, distance in obstacles:
            if (self.config.obstacle_min_distance <= distance <= self.config.obstacle_max_distance):
                filtered_obstacles.append((angle, distance))
        
        self.current_obstacles = filtered_obstacles
        self.navigation_metrics['obstacles_in_range'] = len(filtered_obstacles)
    
    def calculate_attractive_force(self) -> Vector2D:
        """Calculate attractive force toward the person"""
        if not self.current_person_position:
            return Vector2D(0, 0)
        
        person_angle, person_distance = self.current_person_position
        
        # Check if we're at target distance
        distance_error = person_distance - self.config.target_distance
        
        if self.config.stop_at_target and abs(distance_error) < 200:  # 20cm tolerance
            self.navigation_metrics['navigation_decision'] = 'at_target'
            return Vector2D(0, 0)
        
        # Calculate attractive force magnitude based on distance
        if person_distance > self.config.attractive_max_distance:
            force_magnitude = 0.0
        else:
            # Linear decrease with distance, but always some attraction if not at target
            normalized_distance = person_distance / self.config.attractive_max_distance
            force_magnitude = self.config.attractive_strength * (1.0 - normalized_distance)
            
            # Increase force if far from target
            if abs(distance_error) > 500:  # 50cm
                force_magnitude *= 1.5
        
        # Convert to cartesian force vector
        force_x = force_magnitude * math.sin(person_angle)
        force_y = force_magnitude * math.cos(person_angle)
        
        attractive_force = Vector2D(force_x, force_y)
        self.navigation_metrics['attractive_force_magnitude'] = attractive_force.magnitude()
        
        return attractive_force
    
    def calculate_repulsive_force(self) -> Vector2D:
        """Calculate repulsive force from all obstacles"""
        total_repulsive_force = Vector2D(0, 0)
        
        if not self.current_obstacles:
            self.navigation_metrics['repulsive_force_magnitude'] = 0.0
            return total_repulsive_force
        
        for obstacle_angle, obstacle_distance in self.current_obstacles:
            # Skip obstacles that are too far
            if obstacle_distance > self.config.repulsive_max_distance:
                continue
            
            # Calculate repulsive force magnitude (stronger when closer)
            if obstacle_distance <= self.config.repulsive_min_distance:
                # Very strong repulsion for very close obstacles
                force_magnitude = self.config.repulsive_strength * 3.0
            else:
                # Inverse square law for repulsion
                normalized_distance = obstacle_distance / self.config.repulsive_max_distance
                force_magnitude = self.config.repulsive_strength / (normalized_distance**2 + 0.1)
            
            # Direction: away from obstacle
            repulsion_angle = obstacle_angle + math.pi  # Opposite direction
            
            # Convert to cartesian and add to total
            force_x = force_magnitude * math.sin(repulsion_angle)
            force_y = force_magnitude * math.cos(repulsion_angle)
            
            total_repulsive_force = total_repulsive_force + Vector2D(force_x, force_y)
        
        self.navigation_metrics['repulsive_force_magnitude'] = total_repulsive_force.magnitude()
        return total_repulsive_force
    
    def calculate_total_force(self) -> Vector2D:
        """Calculate total force from attractive and repulsive components"""
        attractive_force = self.calculate_attractive_force()
        repulsive_force = self.calculate_repulsive_force()
        
        total_force = attractive_force + repulsive_force
        self.navigation_metrics['total_force_magnitude'] = total_force.magnitude()
        
        # Smooth the force using history
        self.force_history.append(total_force)
        if len(self.force_history) > 1:
            # Simple moving average
            smooth_x = sum(f.x for f in self.force_history) / len(self.force_history)
            smooth_y = sum(f.y for f in self.force_history) / len(self.force_history)
            total_force = Vector2D(smooth_x, smooth_y)
        
        return total_force
    
    def force_to_movement_command(self, force: Vector2D) -> Tuple[MovementDirection, float]:
        """
        Convert force vector to robot movement command
        
        Args:
            force: Force vector in robot coordinates
            
        Returns:
            Tuple of (MovementDirection, speed_factor)
        """
        force_magnitude = force.magnitude()
        
        # Check if force is significant enough to move
        if force_magnitude < self.config.movement_threshold:
            self.navigation_metrics['navigation_decision'] = 'force_too_small'
            return MovementDirection.NONE, 0.0
        
        # Convert force to polar coordinates
        force_angle, _ = force.to_polar()
        force_angle_deg = math.degrees(force_angle)
        
        # Normalize angle to [-180, 180]
        while force_angle_deg > 180:
            force_angle_deg -= 360
        while force_angle_deg < -180:
            force_angle_deg += 360
        
        # Determine if we should rotate in place for large angles
        if abs(force_angle_deg) > self.config.rotate_in_place_threshold:
            self.navigation_metrics['navigation_decision'] = 'rotate_in_place'
            if force_angle_deg > 0:
                return MovementDirection.RIGHT, min(self.config.max_angular_speed, force_magnitude)
            else:
                return MovementDirection.LEFT, min(self.config.max_angular_speed, force_magnitude)
        
        # Calculate linear and angular components
        forward_component = math.cos(force_angle)  # How much forward/backward
        angular_component = math.sin(force_angle)  # How much left/right
        
        # Determine primary movement direction
        if abs(forward_component) > abs(angular_component):
            # Primarily forward/backward movement
            if forward_component > 0:
                if abs(angular_component) > self.config.angular_threshold:
                    # Diagonal movement
                    if angular_component > 0:
                        self.navigation_metrics['navigation_decision'] = 'forward_right'
                        return MovementDirection.FORWARDS_RIGHT, min(self.config.max_linear_speed, force_magnitude)
                    else:
                        self.navigation_metrics['navigation_decision'] = 'forward_left'
                        return MovementDirection.FORWARDS_LEFT, min(self.config.max_linear_speed, force_magnitude)
                else:
                    # Straight forward
                    self.navigation_metrics['navigation_decision'] = 'forward'
                    return MovementDirection.FORWARDS, min(self.config.max_linear_speed, force_magnitude)
            else:
                # Backward movement
                self.navigation_metrics['navigation_decision'] = 'backward'
                return MovementDirection.BACKWARDS, min(self.config.max_linear_speed, force_magnitude)
        else:
            # Primarily rotational movement
            if angular_component > 0:
                self.navigation_metrics['navigation_decision'] = 'turn_right'
                return MovementDirection.RIGHT, min(self.config.max_angular_speed, force_magnitude)
            else:
                self.navigation_metrics['navigation_decision'] = 'turn_left'
                return MovementDirection.LEFT, min(self.config.max_angular_speed, force_magnitude)
    
    def calculate_navigation_command(self) -> Tuple[MovementDirection, float]:
        """
        Calculate navigation command based on current person position and obstacles
        
        Returns:
            Tuple of (MovementDirection, speed_factor)
        """
        self.last_navigation_time = time.time()
        
        # Calculate total force
        total_force = self.calculate_total_force()
        
        # Convert to movement command
        movement_direction, speed_factor = self.force_to_movement_command(total_force)
        
        # Update metrics
        self.navigation_metrics['movement_command'] = movement_direction
        self.navigation_metrics['movement_speed'] = speed_factor
        
        # Add to movement history for analysis
        self.movement_history.append({
            'timestamp': self.last_navigation_time,
            'direction': movement_direction,
            'speed': speed_factor,
            'force_magnitude': total_force.magnitude()
        })
        
        return movement_direction, speed_factor
    
    def create_velocity_config(self, movement_direction: MovementDirection, speed_factor: float) -> VelocityConfig:
        """
        Create VelocityConfig for the movement system
        
        Args:
            movement_direction: Direction to move
            speed_factor: Speed multiplier (0.0 to 1.0)
            
        Returns:
            VelocityConfig object
        """
        return VelocityConfig(movement_direction, speed_factor)
    
    def get_navigation_metrics(self) -> dict:
        """
        Get current navigation metrics for CSV logging
        
        Returns:
            Dictionary of navigation metrics
        """
        return self.navigation_metrics.copy()
    
    def reset_navigation_state(self):
        """Reset navigation state"""
        self.current_person_position = None
        self.current_obstacles.clear()
        self.force_history.clear()
        self.movement_history.clear()
        self.navigation_active = False
        
        # Reset metrics
        for key in self.navigation_metrics:
            if isinstance(self.navigation_metrics[key], (int, float)):
                self.navigation_metrics[key] = 0.0
            elif key == 'movement_command':
                self.navigation_metrics[key] = MovementDirection.NONE
            else:
                self.navigation_metrics[key] = 'reset'
    
    def is_navigation_safe(self) -> bool:
        """
        Check if navigation is safe (no immediate collision risk)
        
        Returns:
            True if safe to navigate
        """
        # Check for very close obstacles
        for _, distance in self.current_obstacles:
            if distance < self.config.repulsive_min_distance:
                return False
        
        return True
    
    def get_debug_info(self) -> dict:
        """
        Get debug information for display/logging
        
        Returns:
            Dictionary of debug information
        """
        attractive_force = self.calculate_attractive_force()
        repulsive_force = self.calculate_repulsive_force()
        total_force = attractive_force + repulsive_force
        
        return {
            'person_position': self.current_person_position,
            'obstacles_count': len(self.current_obstacles),
            'attractive_force': (attractive_force.x, attractive_force.y),
            'repulsive_force': (repulsive_force.x, repulsive_force.y),
            'total_force': (total_force.x, total_force.y),
            'force_magnitude': total_force.magnitude(),
            'navigation_safe': self.is_navigation_safe(),
            'last_navigation_time': self.last_navigation_time
        }


# Export main classes
__all__ = ['PotentialFieldNavigator', 'PotentialFieldConfig', 'Vector2D']