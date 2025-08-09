import math
import time
import threading
from typing import List, Dict, Tuple, Set
from src.path_finding.Position import Position


class ObstacleCell:
    """Represents a cell in the obstacle grid"""
    
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y
        self.obstacle_count = 0
        self.total_detections = 0
        self.last_seen = 0
        self.confidence = 0.0
        self.is_persistent = False
        
    def add_detection(self, is_obstacle: bool):
        """Add a detection to this cell"""
        self.total_detections += 1
        if is_obstacle:
            self.obstacle_count += 1
        self.last_seen = time.time()
        self.update_confidence()
    
    def update_confidence(self):
        """Update confidence based on detections"""
        if self.total_detections > 0:
            self.confidence = self.obstacle_count / self.total_detections
            # Mark as persistent if consistently detected
            if self.total_detections >= 5 and self.confidence >= 0.7:
                self.is_persistent = True
    
    def decay_over_time(self, decay_rate: float = 0.95):
        """Decay confidence over time for dynamic obstacles"""
        current_time = time.time()
        if current_time - self.last_seen > 5.0:  # Haven't seen in 5 seconds
            self.confidence *= decay_rate
            if self.confidence < 0.1:
                self.obstacle_count = 0
                self.total_detections = max(1, self.total_detections // 2)
                self.is_persistent = False


class ObstacleMapper:
    """
    Persistent obstacle mapping system that builds accuracy over time
    Creates a grid-based map of obstacles with confidence levels
    """
    
    def __init__(self, grid_size_mm: int = 100, map_radius_mm: int = 8000):
        self.grid_size_mm = grid_size_mm  # 10cm grid cells
        self.map_radius_mm = map_radius_mm  # 8m mapping radius
        
        # Grid dimensions
        self.grid_radius = map_radius_mm // grid_size_mm
        self.grid_diameter = self.grid_radius * 2 + 1
        
        # Obstacle grid - centered at robot position
        self.obstacle_grid: Dict[Tuple[int, int], ObstacleCell] = {}
        
        # Robot dimensions for safety buffer
        self.robot_width_mm = 500
        self.robot_length_mm = 650
        self.safety_buffer_mm = 200  # Additional safety margin
        self.total_buffer_mm = max(self.robot_width_mm, self.robot_length_mm) // 2 + self.safety_buffer_mm
        
        # Threading for background processing
        self.lock = threading.Lock()
        
        # Performance tracking
        self.total_lidar_points = 0
        self.mapped_obstacles = 0
        self.last_cleanup = time.time()
        self.cleanup_interval = 10.0  # Clean up every 10 seconds
    
    def world_to_grid(self, x_mm: float, y_mm: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        grid_x = int(round(x_mm / self.grid_size_mm))
        grid_y = int(round(y_mm / self.grid_size_mm))
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates"""
        x_mm = grid_x * self.grid_size_mm
        y_mm = grid_y * self.grid_size_mm
        return (x_mm, y_mm)
    
    def add_lidar_scan(self, lidar_obstacles: List[Tuple[float, float]]):
        """
        Add LiDAR scan data to the obstacle map
        
        Args:
            lidar_obstacles: List of (angle_degrees, distance_mm) tuples
        """
        with self.lock:
            try:
                current_time = time.time()
                
                # Process each LiDAR point
                for angle_deg, distance_mm in lidar_obstacles:
                    if 200 < distance_mm < self.map_radius_mm:
                        self.total_lidar_points += 1
                        
                        # Convert to Cartesian coordinates
                        angle_rad = math.radians(angle_deg)
                        x_mm = distance_mm * math.sin(angle_rad)
                        y_mm = distance_mm * math.cos(angle_rad)
                        
                        # Convert to grid coordinates
                        grid_x, grid_y = self.world_to_grid(x_mm, y_mm)
                        
                        # Add obstacle detection
                        if (grid_x, grid_y) not in self.obstacle_grid:
                            self.obstacle_grid[(grid_x, grid_y)] = ObstacleCell(grid_x, grid_y)
                        
                        self.obstacle_grid[(grid_x, grid_y)].add_detection(True)
                
                # Perform periodic cleanup
                if current_time - self.last_cleanup > self.cleanup_interval:
                    self._cleanup_old_data()
                    self.last_cleanup = current_time
                    
            except Exception as e:
                pass  # Continue even if there's an error
    
    def get_obstacle_positions(self, confidence_threshold: float = 0.3) -> List[Position]:
        """
        Get obstacle positions for pathfinding
        
        Args:
            confidence_threshold: Minimum confidence to consider as obstacle
            
        Returns:
            List of Position objects representing obstacles
        """
        obstacles = []
        
        with self.lock:
            try:
                for (grid_x, grid_y), cell in self.obstacle_grid.items():
                    if cell.confidence >= confidence_threshold:
                        # Convert back to world coordinates
                        x_mm, y_mm = self.grid_to_world(grid_x, grid_y)
                        
                        # Convert to polar coordinates for pathfinding
                        distance = math.sqrt(x_mm**2 + y_mm**2)
                        angle = math.atan2(x_mm, y_mm) if y_mm != 0 else 0.0
                        
                        if distance < self.map_radius_mm:
                            obstacles.append(Position(angle=angle, distance=distance))
                            
                            # Add safety buffer around each obstacle
                            buffer_obstacles = self._create_safety_buffer(x_mm, y_mm)
                            obstacles.extend(buffer_obstacles)
                
                self.mapped_obstacles = len(obstacles)
                
            except Exception as e:
                pass
        
        return obstacles
    
    def _create_safety_buffer(self, x_mm: float, y_mm: float) -> List[Position]:
        """Create safety buffer around an obstacle"""
        buffer_obstacles = []
        
        try:
            # Create buffer points around the obstacle
            buffer_angles = [0, 45, 90, 135, 180, 225, 270, 315]  # 8 directions
            
            for angle_deg in buffer_angles:
                angle_rad = math.radians(angle_deg)
                buffer_x = x_mm + self.total_buffer_mm * math.cos(angle_rad)
                buffer_y = y_mm + self.total_buffer_mm * math.sin(angle_rad)
                
                buffer_distance = math.sqrt(buffer_x**2 + buffer_y**2)
                buffer_angle = math.atan2(buffer_x, buffer_y) if buffer_y != 0 else 0.0
                
                if 100 < buffer_distance < self.map_radius_mm:
                    buffer_obstacles.append(Position(angle=buffer_angle, distance=buffer_distance))
        
        except Exception:
            pass
        
        return buffer_obstacles
    
    def is_path_clear(self, start_pos: Position, end_pos: Position, 
                     corridor_width_mm: float = 800) -> bool:
        """
        Check if path between two positions is clear of obstacles
        
        Args:
            start_pos: Starting position
            end_pos: Ending position
            corridor_width_mm: Width of safe corridor
            
        Returns:
            True if path is clear, False otherwise
        """
        with self.lock:
            try:
                # Convert positions to Cartesian
                start_x, start_y = start_pos.to_cartesian()
                end_x, end_y = end_pos.to_cartesian()
                
                # Calculate path vector
                path_length = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
                if path_length < 100:  # Very short path
                    return True
                
                # Check points along the path
                num_checks = max(10, int(path_length / 100))  # Check every 10cm
                
                for i in range(num_checks + 1):
                    t = i / num_checks
                    check_x = start_x + t * (end_x - start_x)
                    check_y = start_y + t * (end_y - start_y)
                    
                    # Check for obstacles near this point
                    if self._has_obstacle_near(check_x, check_y, corridor_width_mm / 2):
                        return False
                
                return True
                
            except Exception:
                return False  # Assume blocked if error
    
    def _has_obstacle_near(self, x_mm: float, y_mm: float, radius_mm: float) -> bool:
        """Check if there's an obstacle near the given point"""
        try:
            # Check grid cells within radius
            grid_radius = int(math.ceil(radius_mm / self.grid_size_mm))
            center_grid_x, center_grid_y = self.world_to_grid(x_mm, y_mm)
            
            for dx in range(-grid_radius, grid_radius + 1):
                for dy in range(-grid_radius, grid_radius + 1):
                    grid_x = center_grid_x + dx
                    grid_y = center_grid_y + dy
                    
                    if (grid_x, grid_y) in self.obstacle_grid:
                        cell = self.obstacle_grid[(grid_x, grid_y)]
                        if cell.confidence >= 0.3:  # Obstacle confidence threshold
                            # Check if within radius
                            cell_x, cell_y = self.grid_to_world(grid_x, grid_y)
                            distance = math.sqrt((cell_x - x_mm)**2 + (cell_y - y_mm)**2)
                            if distance <= radius_mm:
                                return True
            
            return False
            
        except Exception:
            return True  # Assume obstacle if error
    
    def _cleanup_old_data(self):
        """Clean up old obstacle data"""
        try:
            current_time = time.time()
            cells_to_remove = []
            
            for key, cell in self.obstacle_grid.items():
                # Decay confidence over time
                cell.decay_over_time()
                
                # Remove cells with very low confidence that aren't persistent
                if (cell.confidence < 0.05 and not cell.is_persistent and 
                    current_time - cell.last_seen > 30.0):
                    cells_to_remove.append(key)
            
            # Remove old cells
            for key in cells_to_remove:
                del self.obstacle_grid[key]
                
        except Exception:
            pass
    
    def get_map_statistics(self) -> Dict:
        """Get mapping statistics for debugging"""
        with self.lock:
            persistent_count = sum(1 for cell in self.obstacle_grid.values() if cell.is_persistent)
            high_confidence_count = sum(1 for cell in self.obstacle_grid.values() if cell.confidence > 0.7)
            
            return {
                'total_cells': len(self.obstacle_grid),
                'persistent_obstacles': persistent_count,
                'high_confidence_obstacles': high_confidence_count,
                'total_lidar_points': self.total_lidar_points,
                'mapped_obstacles': self.mapped_obstacles,
                'grid_size_mm': self.grid_size_mm,
                'safety_buffer_mm': self.total_buffer_mm
            }
    
    def clear_map(self):
        """Clear the entire obstacle map"""
        with self.lock:
            self.obstacle_grid.clear()
            self.total_lidar_points = 0
            self.mapped_obstacles = 0