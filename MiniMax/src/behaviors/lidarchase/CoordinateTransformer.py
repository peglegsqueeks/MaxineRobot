import math
import numpy as np
from typing import Tuple, List, Optional
from src.path_finding.Position import Position


class CoordinateTransformer:
    """
    Handles coordinate transformations between different reference frames:
    - Camera coordinates (Oak-D camera on rotating head)
    - LiDAR coordinates (RPLidar A3 fixed to robot body)
    - Robot coordinates (robot center/base)
    - World coordinates (fixed reference frame)
    
    Accounts for correct sensor positions and robot movement with proper dimensions
    """
    
    def __init__(self):
        # Robot physical dimensions (mm) - CORRECTED VALUES
        self.robot_length = 660  # Updated from 650mm
        self.robot_width = 550   # Updated from 500mm
        
        # Sensor positions relative to robot center (mm) - CORRECTED VALUES
        # LiDAR position: center front of robot body
        self.lidar_x_offset = self.robot_length / 2  # Front of robot (330mm from center)
        self.lidar_y_offset = 0                      # Center width
        self.lidar_z_offset = 0                      # Base level
        
        # Camera position: on top of head, 130mm back from lidar, 510mm above lidar
        self.camera_x_offset = (self.robot_length / 2) - 130  # 130mm back from front (200mm from center)
        self.camera_y_offset = 0                               # Center width  
        self.camera_z_offset = 510                             # 510mm above base
        
        # Robot state tracking
        self.robot_position = Position(angle=0.0, distance=0.0)  # Robot position in world
        self.robot_heading = 0.0  # Robot heading in radians
        self.head_angle = 0.0     # Head angle relative to robot body (±90 degrees max)
        
        # Movement tracking for coordinate updates
        self.last_update_time = 0
        self.movement_accumulator = Position(angle=0.0, distance=0.0)
        
        # Head rotation limits (radians)
        self.max_head_angle = math.radians(90)  # ±90 degrees maximum
    
    def set_robot_state(self, position: Position, heading: float, head_angle: float = 0.0):
        """
        Update robot state for coordinate transformations
        
        Args:
            position: Robot position in world coordinates
            heading: Robot heading in radians (0 = forward)
            head_angle: Head angle in radians relative to robot body (±π/2 max)
        """
        self.robot_position = position
        self.robot_heading = heading
        # Clamp head angle to physical limits
        self.head_angle = max(-self.max_head_angle, min(self.max_head_angle, head_angle))
    
    def camera_to_robot_coordinates(self, x_camera: float, z_camera: float, 
                                   head_angle_rad: float = None) -> Position:
        """
        Transform camera coordinates to robot-centered coordinates
        Accounts for head rotation and correct camera position
        
        Args:
            x_camera: X coordinate from camera (mm, left/right)
            z_camera: Z coordinate from camera (mm, forward)
            head_angle_rad: Head angle in radians (uses current if None)
            
        Returns:
            Position relative to robot center
        """
        if head_angle_rad is None:
            head_angle_rad = self.head_angle
        
        # Clamp head angle to physical limits
        head_angle_rad = max(-self.max_head_angle, min(self.max_head_angle, head_angle_rad))
        
        try:
            # Camera's view angle relative to camera
            camera_angle = math.atan2(x_camera, z_camera) if z_camera > 0 else 0.0
            camera_distance = math.sqrt(x_camera**2 + z_camera**2)
            
            # Transform to robot coordinates accounting for head angle and camera position
            # Head angle rotates the camera's view direction
            robot_angle = camera_angle + head_angle_rad
            
            # Account for camera position offset from robot center (130mm back from lidar)
            # Camera is mounted 130mm back from front of robot and 510mm above base
            adjusted_distance = max(100, camera_distance - self.camera_x_offset)
            
            return Position(angle=robot_angle, distance=adjusted_distance)
            
        except Exception:
            return Position(angle=0.0, distance=1000.0)
    
    def lidar_to_robot_coordinates(self, lidar_position: Position) -> Position:
        """
        Transform LiDAR coordinates to robot-centered coordinates
        LiDAR is at center front of robot body
        
        Args:
            lidar_position: Position from LiDAR sensor
            
        Returns:
            Position relative to robot center
        """
        try:
            # LiDAR is at front-center of robot (330mm forward from center)
            # Convert to cartesian, apply offset, convert back
            x_lidar, y_lidar = lidar_position.to_cartesian()
            
            # Account for LiDAR position offset (LiDAR is 330mm forward from robot center)
            x_robot = x_lidar 
            y_robot = y_lidar - self.lidar_x_offset  # Move back to robot center
            
            # Convert back to polar coordinates
            distance = math.sqrt(x_robot**2 + y_robot**2)
            angle = math.atan2(x_robot, y_robot) if y_robot != 0 else 0.0
            
            return Position(angle=angle, distance=distance)
            
        except Exception:
            return lidar_position  # Return original if transformation fails
    
    def robot_to_world_coordinates(self, robot_position: Position) -> Position:
        """
        Transform robot-relative coordinates to world coordinates
        
        Args:
            robot_position: Position relative to robot center
            
        Returns:
            Position in world coordinates
        """
        try:
            # Convert robot-relative position to cartesian
            x_rel, y_rel = robot_position.to_cartesian()
            
            # Rotate by robot heading
            cos_h = math.cos(self.robot_heading)
            sin_h = math.sin(self.robot_heading)
            
            x_world = x_rel * cos_h - y_rel * sin_h
            y_world = x_rel * sin_h + y_rel * cos_h
            
            # Translate by robot world position
            robot_x, robot_y = self.robot_position.to_cartesian()
            x_world += robot_x
            y_world += robot_y
            
            # Convert back to polar
            distance = math.sqrt(x_world**2 + y_world**2)
            angle = math.atan2(x_world, y_world) if y_world != 0 else 0.0
            
            return Position(angle=angle, distance=distance)
            
        except Exception:
            return robot_position
    
    def world_to_robot_coordinates(self, world_position: Position) -> Position:
        """
        Transform world coordinates to robot-relative coordinates
        
        Args:
            world_position: Position in world coordinates
            
        Returns:
            Position relative to robot center
        """
        try:
            # Convert to cartesian
            x_world, y_world = world_position.to_cartesian()
            
            # Translate by robot position
            robot_x, robot_y = self.robot_position.to_cartesian()
            x_rel = x_world - robot_x
            y_rel = y_world - robot_y
            
            # Rotate by negative robot heading
            cos_h = math.cos(-self.robot_heading)
            sin_h = math.sin(-self.robot_heading)
            
            x_robot = x_rel * cos_h - y_rel * sin_h
            y_robot = x_rel * sin_h + y_rel * cos_h
            
            # Convert back to polar
            distance = math.sqrt(x_robot**2 + y_robot**2)
            angle = math.atan2(x_robot, y_robot) if y_robot != 0 else 0.0
            
            return Position(angle=angle, distance=distance)
            
        except Exception:
            return world_position
    
    def fuse_camera_lidar_obstacles(self, camera_obstacles: List[Position], 
                                   lidar_obstacles: List[Position],
                                   head_angle_rad: float) -> List[Position]:
        """
        Fuse camera and LiDAR obstacle data accounting for different sensor positions
        
        Args:
            camera_obstacles: Obstacles detected by camera
            lidar_obstacles: Obstacles detected by LiDAR
            head_angle_rad: Current head angle in radians
            
        Returns:
            Fused obstacle list in robot coordinates
        """
        fused_obstacles = []
        
        # Transform camera obstacles to robot coordinates
        for cam_obs in camera_obstacles:
            # Convert camera detection to robot coordinates
            x_cam, z_cam = cam_obs.to_cartesian()
            robot_obs = self.camera_to_robot_coordinates(x_cam, z_cam, head_angle_rad)
            fused_obstacles.append(robot_obs)
        
        # Transform LiDAR obstacles to robot coordinates
        for lidar_obs in lidar_obstacles:
            robot_obs = self.lidar_to_robot_coordinates(lidar_obs)
            fused_obstacles.append(robot_obs)
        
        # Remove duplicates that are very close to each other
        return self._remove_duplicate_obstacles(fused_obstacles)
    
    def _remove_duplicate_obstacles(self, obstacles: List[Position], 
                                   distance_threshold: float = 200) -> List[Position]:
        """
        Remove duplicate obstacles that are very close to each other
        
        Args:
            obstacles: List of obstacle positions
            distance_threshold: Minimum distance between obstacles (mm)
            
        Returns:
            Filtered obstacle list
        """
        if not obstacles:
            return []
        
        filtered = [obstacles[0]]
        
        for obs in obstacles[1:]:
            is_duplicate = False
            obs_x, obs_y = obs.to_cartesian()
            
            for existing in filtered:
                ex_x, ex_y = existing.to_cartesian()
                distance = math.sqrt((obs_x - ex_x)**2 + (obs_y - ex_y)**2)
                
                if distance < distance_threshold:
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                filtered.append(obs)
        
        return filtered
    
    def transform_obstacle_list(self, obstacles: List[Position], 
                               from_frame: str, to_frame: str) -> List[Position]:
        """
        Transform a list of obstacle positions between coordinate frames
        
        Args:
            obstacles: List of Position objects
            from_frame: Source frame ('lidar', 'robot', 'world', 'camera')
            to_frame: Target frame ('lidar', 'robot', 'world', 'camera')
            
        Returns:
            List of transformed positions
        """
        transformed = []
        
        for obstacle in obstacles:
            try:
                if from_frame == 'lidar' and to_frame == 'robot':
                    transformed_pos = self.lidar_to_robot_coordinates(obstacle)
                elif from_frame == 'robot' and to_frame == 'world':
                    transformed_pos = self.robot_to_world_coordinates(obstacle)
                elif from_frame == 'world' and to_frame == 'robot':
                    transformed_pos = self.world_to_robot_coordinates(obstacle)
                elif from_frame == to_frame:
                    transformed_pos = obstacle  # No transformation needed
                else:
                    # Chain transformations for other combinations
                    if from_frame == 'lidar' and to_frame == 'world':
                        robot_pos = self.lidar_to_robot_coordinates(obstacle)
                        transformed_pos = self.robot_to_world_coordinates(robot_pos)
                    elif from_frame == 'world' and to_frame == 'lidar':
                        robot_pos = self.world_to_robot_coordinates(obstacle)
                        # Inverse of lidar_to_robot (simplified)
                        transformed_pos = robot_pos
                    else:
                        transformed_pos = obstacle
                
                transformed.append(transformed_pos)
                
            except Exception:
                # Keep original position if transformation fails
                transformed.append(obstacle)
        
        return transformed
    
    def update_robot_movement(self, movement_direction: str, distance_moved: float):
        """
        Update robot position based on movement
        
        Args:
            movement_direction: Movement direction string
            distance_moved: Distance moved in mm
        """
        try:
            # Convert movement to heading change and distance
            if movement_direction == "FORWARDS":
                self.robot_heading += 0
                # Update position
                x, y = self.robot_position.to_cartesian()
                x += distance_moved * math.sin(self.robot_heading)
                y += distance_moved * math.cos(self.robot_heading)
                
            elif movement_direction == "BACKWARDS":
                x, y = self.robot_position.to_cartesian()
                x -= distance_moved * math.sin(self.robot_heading)
                y -= distance_moved * math.cos(self.robot_heading)
                
            elif movement_direction == "LEFT":
                self.robot_heading -= math.radians(5)  # Estimate turn rate
                
            elif movement_direction == "RIGHT":
                self.robot_heading += math.radians(5)  # Estimate turn rate
                
            elif movement_direction == "FORWARDS_LEFT":
                self.robot_heading -= math.radians(2)
                x, y = self.robot_position.to_cartesian()
                x += distance_moved * 0.7 * math.sin(self.robot_heading)
                y += distance_moved * 0.7 * math.cos(self.robot_heading)
                
            elif movement_direction == "FORWARDS_RIGHT":
                self.robot_heading += math.radians(2)
                x, y = self.robot_position.to_cartesian()
                x += distance_moved * 0.7 * math.sin(self.robot_heading)
                y += distance_moved * 0.7 * math.cos(self.robot_heading)
            
            # Update robot position
            if 'x' in locals() and 'y' in locals():
                distance = math.sqrt(x**2 + y**2)
                angle = math.atan2(x, y) if y != 0 else 0.0
                self.robot_position = Position(angle=angle, distance=distance)
            
            # Normalize heading
            self.robot_heading = self.robot_heading % (2 * math.pi)
            
        except Exception:
            pass
    
    def get_robot_bounds_in_robot_coords(self) -> List[Position]:
        """
        Get robot boundary points in robot coordinates for collision detection
        Uses correct robot dimensions: 660mm x 550mm
        
        Returns:
            List of Position objects representing robot boundary
        """
        # Define robot corners relative to center (corrected dimensions)
        half_length = self.robot_length / 2  # 330mm
        half_width = self.robot_width / 2    # 275mm
        
        corners = [
            (half_length, half_width),    # Front-right
            (half_length, -half_width),   # Front-left
            (-half_length, -half_width),  # Rear-left
            (-half_length, half_width)    # Rear-right
        ]
        
        robot_bounds = []
        for x, y in corners:
            distance = math.sqrt(x**2 + y**2)
            angle = math.atan2(y, x)
            robot_bounds.append(Position(angle=angle, distance=distance))
        
        return robot_bounds
    
    def add_safety_buffer_to_obstacles(self, obstacles: List[Position], 
                                     buffer_mm: float = 300) -> List[Position]:
        """
        Add safety buffer around obstacles for robot dimensions
        
        Args:
            obstacles: List of obstacle positions
            buffer_mm: Safety buffer in millimeters
            
        Returns:
            List of expanded obstacle positions
        """
        buffered_obstacles = []
        
        for obstacle in obstacles:
            try:
                # Convert to cartesian
                x, y = obstacle.to_cartesian()
                
                # Add buffer points around the obstacle
                buffer_angles = [0, 45, 90, 135, 180, 225, 270, 315]  # 8 directions
                
                for angle_deg in buffer_angles:
                    angle_rad = math.radians(angle_deg)
                    buffer_x = x + buffer_mm * math.cos(angle_rad)
                    buffer_y = y + buffer_mm * math.sin(angle_rad)
                    
                    buffer_distance = math.sqrt(buffer_x**2 + buffer_y**2)
                    buffer_angle = math.atan2(buffer_x, buffer_y) if buffer_y != 0 else 0.0
                    
                    buffered_obstacles.append(Position(angle=buffer_angle, distance=buffer_distance))
                
                # Include original obstacle
                buffered_obstacles.append(obstacle)
                
            except Exception:
                # Keep original if buffering fails
                buffered_obstacles.append(obstacle)
        
        return buffered_obstacles
    
    def get_status(self) -> dict:
        """Get current transformer status for debugging"""
        return {
            'robot_position': str(self.robot_position),
            'robot_heading_deg': math.degrees(self.robot_heading),
            'head_angle_deg': math.degrees(self.head_angle),
            'robot_dimensions': f"{self.robot_length}x{self.robot_width}mm",
            'camera_offset': f"({self.camera_x_offset}, {self.camera_y_offset}, {self.camera_z_offset})",
            'lidar_offset': f"({self.lidar_x_offset}, {self.lidar_y_offset}, {self.lidar_z_offset})",
            'head_angle_limits': f"±{math.degrees(self.max_head_angle)}°"
        }