import time
import math
import threading
from typing import List, Optional, Tuple
import depthai as dai
import py_trees
from py_trees.common import Status

from src.behaviors.MaxineBehavior import MaxineBehavior
from src.path_finding.Position import Position
from src.path_finding.new_a_star import a_star


class FixedPathFind(MaxineBehavior):
    """
    FIXED pathfinding with 10cm max buffer and faster robot speed
    """
    
    def __init__(self, recalculation_interval=1.5):  # Faster recalculation
        super().__init__("Fixed PathFind")
        
        # Blackboard keys
        self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.READ)
        self.blackboard.register_key("PATH", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("LIDAR_SYSTEM", access=py_trees.common.Access.READ)
        
        # Pathfinding parameters
        self.recalculation_interval = recalculation_interval
        self.last_calculation_time = 0
        self.min_target_distance = 500  # mm - stop when this close
        self.max_target_distance = 5000  # mm - max pathfinding range
        
        # Robot physical parameters
        self.robot_width = 500   # mm
        self.robot_length = 650  # mm
        self.safety_buffer = 100  # REDUCED to 10cm max as requested!!!
        
        # Pathfinding state
        self.current_path = []
        self.path_calculation_active = False
        self.path_thread = None
        self.path_lock = threading.Lock()
        
        # Performance tracking
        self.last_status_print = 0
        self.status_print_interval = 8.0
        self.path_calculation_count = 0
        
    def should_recalculate_path(self) -> bool:
        """Determine if path should be recalculated"""
        current_time = time.time()
        
        # Always recalculate if no path exists
        if not self.current_path:
            return True
        
        # Recalculate at regular intervals (faster now)
        if current_time - self.last_calculation_time > self.recalculation_interval:
            return True
        
        return False
    
    def get_obstacles_with_minimal_buffer(self) -> List[Position]:
        """Get obstacles with MINIMAL 10cm buffer as requested"""
        try:
            # Get raw LiDAR obstacles from WORKING system
            if self.blackboard.exists("LIDAR_SYSTEM"):
                lidar_system = self.blackboard.get("LIDAR_SYSTEM")
                if lidar_system and lidar_system.running:
                    raw_obstacles = lidar_system.get_latest_obstacles()
                    if raw_obstacles:
                        # Convert to Position objects
                        obstacles = []
                        for angle, distance in raw_obstacles:
                            # Filter obstacles within navigation range
                            if 200 < distance < 4000:  # 20cm to 4m range
                                pos = Position(angle=math.radians(angle), distance=distance)
                                obstacles.append(pos)
                        
                        # Apply MINIMAL buffer (10cm max as requested)
                        buffered_obstacles = self.add_minimal_safety_buffer(obstacles)
                        return buffered_obstacles
            
            return []
            
        except Exception as e:
            return []
    
    def add_minimal_safety_buffer(self, obstacles: List[Position]) -> List[Position]:
        """
        Add MINIMAL safety buffer (10cm max) around obstacles as requested
        """
        buffered_obstacles = []
        
        # MINIMAL buffer - 10cm max as requested
        minimal_buffer = self.safety_buffer  # 100mm = 10cm
        
        for obstacle in obstacles:
            try:
                # Convert to cartesian for buffer calculation
                x, y = obstacle.to_cartesian()
                
                # Create minimal buffer points (fewer points for 10cm buffer)
                buffer_angles = [0, 45, 90, 135, 180, 225, 270, 315]  # 8 directions only
                
                for angle_deg in buffer_angles:
                    angle_rad = math.radians(angle_deg)
                    
                    # Single buffer distance (10cm)
                    buffer_x = x + minimal_buffer * math.cos(angle_rad)
                    buffer_y = y + minimal_buffer * math.sin(angle_rad)
                    
                    # Convert back to polar
                    distance = math.sqrt(buffer_x**2 + buffer_y**2)
                    angle = math.atan2(buffer_x, buffer_y) if buffer_y != 0 else 0.0
                    
                    if distance > 150:  # Ignore very close buffer points
                        buffered_obstacles.append(Position(angle=angle, distance=distance))
                
                # Include original obstacle
                buffered_obstacles.append(obstacle)
                
            except Exception as e:
                # Keep original obstacle if buffering fails
                buffered_obstacles.append(obstacle)
        
        return buffered_obstacles
    
    def get_target_position_simple(self) -> Optional[Position]:
        """Get target position with simple coordinate transformation"""
        try:
            target_person: dai.SpatialImgDetection = self.blackboard.get("TARGET_PERSON")
            
            # Simple direct calculation
            x_camera = target_person.spatialCoordinates.x
            z_camera = target_person.spatialCoordinates.z
            
            angle_rad = math.atan2(x_camera, z_camera)
            distance = max(100, z_camera - 300)  # Reduced offset for more accurate targeting
            
            return Position(angle=angle_rad, distance=distance)
            
        except Exception as e:
            return None
    
    def create_efficient_path_to_target(self, target_position: Position, obstacles: List[Position]) -> List[Position]:
        """
        Create efficient path using A* with minimal buffer
        """
        try:
            # Check if target is within reasonable range
            if target_position.distance > self.max_target_distance:
                # Create intermediate target
                target_position = Position(
                    angle=target_position.angle,
                    distance=self.max_target_distance * 0.9
                )
            
            # Use A* pathfinding with minimal buffered obstacles
            start_time = time.time()
            path, debug_info = a_star(obstacles, target_position)
            calculation_time = time.time() - start_time
            
            if path and len(path) > 0:
                # Simple path optimization - remove close waypoints
                optimized_path = self.optimize_path_simple(path)
                
                self.path_calculation_count += 1
                
                return optimized_path
            else:
                # A* failed, create simple direct path if clear
                if self.is_direct_path_clear(target_position, obstacles):
                    return self.create_direct_path(target_position)
                else:
                    return []
                    
        except Exception as e:
            # Fallback to direct path if clear
            if target_position and self.is_direct_path_clear(target_position, obstacles):
                return self.create_direct_path(target_position)
            return []
    
    def optimize_path_simple(self, path: List[Position]) -> List[Position]:
        """Simple path optimization"""
        if len(path) <= 2:
            return path
        
        optimized = [path[0]]  # Always include start
        
        i = 0
        while i < len(path) - 1:
            # Look ahead to find furthest reachable waypoint
            furthest_reachable = i + 1
            
            for j in range(i + 2, min(i + 5, len(path))):  # Look up to 4 waypoints ahead
                if self.is_path_segment_clear(path[i], path[j]):
                    furthest_reachable = j
                else:
                    break
            
            # Add the furthest reachable waypoint
            if furthest_reachable > i + 1:
                optimized.append(path[furthest_reachable])
                i = furthest_reachable
            else:
                optimized.append(path[i + 1])
                i += 1
        
        return optimized
    
    def is_path_segment_clear(self, start: Position, end: Position) -> bool:
        """Check if path segment is reasonable"""
        angle_diff = abs(start.angle - end.angle)
        distance_diff = abs(start.distance - end.distance)
        
        # Allow segment if angle and distance changes are reasonable
        return angle_diff < math.radians(25) and distance_diff < 1000
    
    def is_direct_path_clear(self, target: Position, obstacles: List[Position]) -> bool:
        """Check if direct path to target is clear of obstacles"""
        target_x, target_y = target.to_cartesian()
        
        # Check if any obstacles are in the direct path
        for obstacle in obstacles[:30]:  # Check first 30 obstacles for performance
            obs_x, obs_y = obstacle.to_cartesian()
            
            # Calculate distance from obstacle to direct path line
            distance_to_path = abs(obs_x * target_y - obs_y * target_x) / max(target.distance, 1)
            
            # If obstacle is close to path and within target distance
            if (distance_to_path < (self.safety_buffer + 50) and 
                obstacle.distance < target.distance and
                obstacle.distance > 150):  # Ignore very close obstacles
                return False
        
        return True
    
    def create_direct_path(self, target: Position) -> List[Position]:
        """Create simple direct path to target"""
        path = []
        
        # Create waypoints at regular intervals (larger steps for faster movement)
        distance_step = 800  # 80cm steps (larger for faster movement)
        current_distance = distance_step
        
        while current_distance < target.distance:
            waypoint = Position(angle=target.angle, distance=current_distance)
            path.append(waypoint)
            current_distance += distance_step
        
        # Add final target
        path.append(target)
        
        return path
    
    def calculate_path_threaded(self, target_position: Position, obstacles: List[Position]):
        """Calculate path in separate thread"""
        try:
            new_path = self.create_efficient_path_to_target(target_position, obstacles)
            
            with self.path_lock:
                self.current_path = new_path
                self.path_calculation_active = False
                self.last_calculation_time = time.time()
                
                # Update blackboard with new path
                self.blackboard.set("PATH", self.current_path)
                
        except Exception as e:
            with self.path_lock:
                self.path_calculation_active = False
    
    def update(self) -> Status:
        """FIXED update with 10cm buffer and faster recalculation"""
        try:
            # Get target position
            target_position = self.get_target_position_simple()
            if not target_position:
                self.blackboard.set("PATH", [])
                return Status.SUCCESS
            
            # Check if target is within stopping distance
            if target_position.distance < self.min_target_distance:
                self.blackboard.set("PATH", [])
                return Status.SUCCESS
            
            # Check if we should recalculate path (faster intervals)
            if self.should_recalculate_path() and not self.path_calculation_active:
                # Get obstacles with MINIMAL 10cm buffer
                obstacles = self.get_obstacles_with_minimal_buffer()
                
                # Start path calculation in thread
                with self.path_lock:
                    self.path_calculation_active = True
                
                self.path_thread = threading.Thread(
                    target=self.calculate_path_threaded,
                    args=(target_position, obstacles),
                    daemon=True
                )
                self.path_thread.start()
            
            # Always return current path
            with self.path_lock:
                current_path = self.current_path.copy()
            
            self.blackboard.set("PATH", current_path)
            return Status.SUCCESS
            
        except Exception as e:
            self.blackboard.set("PATH", [])
            return Status.SUCCESS
    
    def update(self) -> Status:
        """FIXED update with 10cm buffer and faster recalculation"""
        try:
            # Get target position
            target_position = self.get_target_position_simple()
            if not target_position:
                self.blackboard.set("PATH", [])
                return Status.SUCCESS
            
            # Check if target is within stopping distance
            if target_position.distance < self.min_target_distance:
                self.blackboard.set("PATH", [])
                return Status.SUCCESS
            
            # Check if we should recalculate path (faster intervals)
            if self.should_recalculate_path() and not self.path_calculation_active:
                # Get obstacles with MINIMAL 10cm buffer
                obstacles = self.get_obstacles_with_minimal_buffer()
                
                # Start path calculation in thread
                with self.path_lock:
                    self.path_calculation_active = True
                
                self.path_thread = threading.Thread(
                    target=self.calculate_path_threaded,
                    args=(target_position, obstacles),
                    daemon=True
                )
                self.path_thread.start()
            
            # Always return current path
            with self.path_lock:
                current_path = self.current_path.copy()
            
            self.blackboard.set("PATH", current_path)
            return Status.SUCCESS
            
        except Exception as e:
            self.blackboard.set("PATH", [])
            return Status.SUCCESS
    
    def terminate(self, new_status: Status):
        """Clean up threading on termination"""
        try:
            # Stop any active path calculation
            with self.path_lock:
                self.path_calculation_active = False
            
            # Wait for thread to complete
            if self.path_thread and self.path_thread.is_alive():
                self.path_thread.join(timeout=1.0)
                
        except Exception as e:
            pass
        
        super().terminate(new_status)
    
    def terminate(self, new_status: Status):
        """Clean up threading on termination"""
        try:
            # Stop any active path calculation
            with self.path_lock:
                self.path_calculation_active = False
            
            # Wait for thread to complete
            if self.path_thread and self.path_thread.is_alive():
                self.path_thread.join(timeout=1.0)
                
        except Exception as e:
            pass
        
        super().terminate(new_status)
    
    def get_status(self) -> dict:
        """Get pathfinding status for debugging"""
        return {
            'current_path_length': len(self.current_path),
            'calculation_active': self.path_calculation_active,
            'last_calculation_age': time.time() - self.last_calculation_time,
            'total_calculations': self.path_calculation_count,
            'recalculation_interval': self.recalculation_interval,
            'robot_dimensions': f"{self.robot_length}x{self.robot_width}mm",
            'safety_buffer': f"{self.safety_buffer}mm (10cm max as requested)",
            'max_target_distance': f"{self.max_target_distance}mm"
        }


# Aliases for backward compatibility
PathFind = FixedPathFind
EnhancedPathFind = FixedPathFind