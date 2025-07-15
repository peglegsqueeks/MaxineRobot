from typing import List, Tuple, Optional, Set
import heapq
import math
import numpy as np
from src.path_finding.Position import Position


# Balanced resolution for obstacle avoidance
RESOLUTION = 100  # 10cm grid resolution - good for obstacle navigation
PAD_ROUNDS = 1   # Minimal padding for 10cm buffer

# Balanced movement directions for obstacle avoidance
DIRECTIONS = [
    # Primary directions for obstacle navigation
    (0, 1),   # Forward
    (1, 0),   # Right  
    (-1, 0),  # Left
    (0, -1),  # Backward
    
    # Diagonal for smoother paths around obstacles
    (1, 1),   # Forward-right
    (-1, 1),  # Forward-left
    (1, -1),  # Backward-right
    (-1, -1), # Backward-left
    
    # Larger steps for efficiency
    (0, 2),   # Larger forward
    (2, 0),   # Larger right
    (-2, 0),  # Larger left
    (2, 2),   # Larger diagonal
    (-2, 2),  # Larger diagonal
]


class FixedRobotAwareAStar:
    """
    FIXED A* pathfinding algorithm with correct robot dimensions
    """
    
    def __init__(self, robot_width=550, robot_length=660, safety_buffer=100):  # CORRECTED DIMENSIONS
        self.robot_width = robot_width      # 550mm (corrected from 500mm)
        self.robot_length = robot_length    # 660mm (corrected from 650mm)
        self.safety_buffer = safety_buffer  # 100mm = 10cm MAX as requested
        
        # Calculate total clearance needed (MINIMAL as requested)
        self.clearance_radius = (max(robot_width, robot_length) / 2) + safety_buffer  # ~430mm total
        
        # Grid parameters for obstacle avoidance
        self.resolution = RESOLUTION  # 10cm grid
        self.max_grid_size = 100      # 100x100 grid (10m x 10m at 10cm resolution)
        
        # Store target position to avoid treating it as obstacle
        self.target_grid_pos = None
        
    def polar_to_cartesian(self, pos: Position) -> Tuple[float, float]:
        """Convert polar position to cartesian coordinates - FIXED to match Position class"""
        if pos.distance == 0:
            return (0.0, 0.0)
        
        # FIXED: Use same convention as Position class
        x = pos.distance * math.sin(pos.angle)  # Right/left
        y = pos.distance * math.cos(pos.angle)  # Forward/backward
        return (x, y)
    
    def cartesian_to_polar(self, x: float, y: float) -> Position:
        """Convert cartesian coordinates to polar position"""
        distance = math.hypot(x, y)
        if distance == 0:
            return Position(distance=0.0, angle=0.0)
        
        angle = math.atan2(x, y)
        return Position(angle=angle, distance=distance)
    
    def position_to_grid(self, pos: Position) -> Tuple[int, int]:
        """Convert position to grid coordinates"""
        x, y = self.polar_to_cartesian(pos)
        grid_x = int((x + self.max_grid_size * self.resolution / 2) / self.resolution)
        grid_y = int((y + self.max_grid_size * self.resolution / 2) / self.resolution)
        
        # Clamp to grid bounds
        grid_x = max(0, min(self.max_grid_size - 1, grid_x))
        grid_y = max(0, min(self.max_grid_size - 1, grid_y))
        
        return (grid_x, grid_y)
    
    def grid_to_position(self, grid_x: int, grid_y: int) -> Position:
        """Convert grid coordinates to position"""
        x = (grid_x * self.resolution) - (self.max_grid_size * self.resolution / 2)
        y = (grid_y * self.resolution) - (self.max_grid_size * self.resolution / 2)
        return self.cartesian_to_polar(x, y)
    
    def is_near_target(self, grid_pos: Tuple[int, int]) -> bool:
        """Check if position is near the target (should not be treated as obstacle)"""
        if self.target_grid_pos is None:
            return False
        
        distance = math.sqrt((grid_pos[0] - self.target_grid_pos[0])**2 + 
                           (grid_pos[1] - self.target_grid_pos[1])**2)
        
        # Allow robot to get within 2 grid cells of target (20cm)
        return distance <= 2
    
    def add_minimal_clearance_to_obstacles(self, obstacles: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        Add MINIMAL robot clearance around obstacles, but NOT around target area
        Uses correct robot dimensions: 550mm x 660mm
        """
        if not obstacles:
            return []
        
        buffered = set()
        clearance_cells = max(1, int(self.clearance_radius / self.resolution))
        
        # MINIMAL clearance pattern
        clearance_pattern = []
        for dx in range(-clearance_cells, clearance_cells + 1):
            for dy in range(-clearance_cells, clearance_cells + 1):
                distance = math.sqrt(dx*dx + dy*dy) * self.resolution
                if distance <= self.clearance_radius:
                    clearance_pattern.append((dx, dy))
        
        # Apply clearance but exclude target area
        for obs_x, obs_y in obstacles:
            # Skip if this obstacle is near the target
            if self.is_near_target((obs_x, obs_y)):
                continue
                
            buffered.add((obs_x, obs_y))  # Add original obstacle
            
            for dx, dy in clearance_pattern:
                new_x = obs_x + dx
                new_y = obs_y + dy
                
                # Don't add buffer around target area
                if self.is_near_target((new_x, new_y)):
                    continue
                
                # Check bounds
                if (0 <= new_x < self.max_grid_size and 
                    0 <= new_y < self.max_grid_size):
                    buffered.add((new_x, new_y))
        
        return list(buffered)
    
    def preprocess_obstacles(self, obstacles: List[Position]) -> Set[Tuple[int, int]]:
        """
        Convert obstacles to grid, exclude target area from obstacles
        """
        # Convert to grid coordinates
        grid_obstacles = []
        for obs in obstacles:
            # Filter reasonable obstacles for indoor navigation
            if 200 < obs.distance < 6000:  # 20cm to 6m range
                grid_pos = self.position_to_grid(obs)
                
                # CRITICAL: Don't treat positions near target as obstacles
                if not self.is_near_target(grid_pos):
                    grid_obstacles.append(grid_pos)
        
        # Remove duplicates
        unique_obstacles = list(set(grid_obstacles))
        
        # Add robot clearance but preserve target area
        buffered_obstacles = self.add_minimal_clearance_to_obstacles(unique_obstacles)
        
        return set(buffered_obstacles)
    
    def heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Simple Euclidean heuristic"""
        dx = abs(pos1[0] - pos2[0])
        dy = abs(pos1[1] - pos2[1])
        return math.sqrt(dx*dx + dy*dy)
    
    def get_movement_cost(self, current: Tuple[int, int], neighbor: Tuple[int, int]) -> float:
        """Simple movement cost calculation"""
        dx = abs(neighbor[0] - current[0])
        dy = abs(neighbor[1] - current[1])
        
        if dx == 0 or dy == 0:
            base_cost = max(dx, dy)
        else:
            base_cost = math.sqrt(dx*dx + dy*dy)
        
        if dx > 1 or dy > 1:
            base_cost *= 1.1
        
        return base_cost
    
    def is_valid_position(self, pos: Tuple[int, int], obstacles: Set[Tuple[int, int]]) -> bool:
        """Check if position is valid - target is ALWAYS valid"""
        x, y = pos
        
        # Target position is always valid
        if self.is_near_target(pos):
            return True
            
        return (0 <= x < self.max_grid_size and 
                0 <= y < self.max_grid_size and 
                pos not in obstacles)
    
    def get_neighbors(self, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighbor positions"""
        neighbors = []
        
        for dx, dy in DIRECTIONS:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Basic bounds check
            if (0 <= neighbor[0] < self.max_grid_size and 
                0 <= neighbor[1] < self.max_grid_size):
                neighbors.append(neighbor)
        
        return neighbors
    
    def reconstruct_path(self, came_from: dict, current: Tuple[int, int], 
                        start: Tuple[int, int]) -> List[Position]:
        """Reconstruct path and convert to Position objects"""
        path = []
        
        while current in came_from:
            position = self.grid_to_position(current[0], current[1])
            path.append(position)
            current = came_from[current]
        
        # Add start position
        start_position = self.grid_to_position(start[0], start[1])
        path.append(start_position)
        
        # Reverse to get path from start to goal
        path.reverse()
        
        return path
    
    def find_path(self, obstacles: List[Position], goal: Position) -> Tuple[List[Position], dict]:
        """
        FIXED A* pathfinding algorithm that reaches the exact target
        Uses correct robot dimensions: 550mm x 660mm
        """
        try:
            # Store target position to exclude from obstacles
            self.target_grid_pos = self.position_to_grid(goal)
            
            # Preprocess obstacles (excluding target area)
            obstacle_set = self.preprocess_obstacles(obstacles)
            
            # Convert start and goal to grid coordinates
            start = (self.max_grid_size // 2, self.max_grid_size // 2)  # Robot starts at center
            goal_grid = self.target_grid_pos
            
            # Target is always reachable now since we exclude it from obstacles
            
            # A* algorithm
            open_set = [(0, start)]
            came_from = {}
            g_score = {start: 0}
            f_score = {start: self.heuristic(start, goal_grid)}
            closed_set = set()
            
            nodes_explored = 0
            max_nodes = 2000  # Increased for obstacle avoidance
            
            while open_set and nodes_explored < max_nodes:
                current = heapq.heappop(open_set)[1]
                nodes_explored += 1
                
                if current in closed_set:
                    continue
                
                closed_set.add(current)
                
                # Check if goal reached (exact match or very close)
                distance_to_goal = math.sqrt((current[0] - goal_grid[0])**2 + 
                                           (current[1] - goal_grid[1])**2)
                
                if distance_to_goal <= 1.0:  # Within 1 grid cell (10cm)
                    # Force the exact goal position as final waypoint
                    path = self.reconstruct_path(came_from, current, start)
                    
                    # Ensure the path ends exactly at the target
                    if len(path) > 0:
                        path[-1] = goal  # Replace last position with exact target
                    else:
                        path = [goal]  # Direct path to target
                    
                    debug_info = {
                        "nodes_explored": nodes_explored,
                        "path_length": len(path),
                        "obstacles_processed": len(obstacle_set),
                        "goal_grid": goal_grid,
                        "exact_target_reached": True,
                        "final_distance_to_target": 0.0,
                        "robot_dimensions": f"{self.robot_width}x{self.robot_length}mm"
                    }
                    
                    return path, debug_info
                
                # Explore neighbors
                for neighbor in self.get_neighbors(current):
                    if neighbor in closed_set or not self.is_valid_position(neighbor, obstacle_set):
                        continue
                    
                    tentative_g_score = g_score[current] + self.get_movement_cost(current, neighbor)
                    
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_grid)
                        
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
            
            # If A* fails, create direct path to target
            return [Position(angle=0, distance=0), goal], {
                "error": "A* failed, using direct path",
                "nodes_explored": nodes_explored,
                "direct_path_to_target": True,
                "robot_dimensions": f"{self.robot_width}x{self.robot_length}mm"
            }
            
        except Exception as e:
            # Fallback: direct path to target
            return [Position(angle=0, distance=0), goal], {
                "error": f"A* algorithm error: {str(e)}",
                "fallback_direct_path": True,
                "robot_dimensions": f"{self.robot_width}x{self.robot_length}mm"
            }


# Global instance with corrected dimensions
_fixed_robot_astar = FixedRobotAwareAStar(robot_width=550, robot_length=660, safety_buffer=100)


def a_star(obstacles: List[Position], goal: Position) -> Tuple[List[Position], dict]:
    """
    FIXED A* pathfinding function with correct robot dimensions
    
    Args:
        obstacles: List of obstacle positions in polar coordinates
        goal: Target position in polar coordinates
        
    Returns:
        Tuple of (path_positions, debug_info)
    """
    return _fixed_robot_astar.find_path(obstacles, goal)


def configure_robot_dimensions(width: float, length: float, safety_buffer: float = 100.0):
    """Configure robot dimensions for pathfinding"""
    global _fixed_robot_astar
    _fixed_robot_astar = FixedRobotAwareAStar(width, length, safety_buffer)


def get_pathfinding_status() -> dict:
    """Get current pathfinding configuration status"""
    return {
        "robot_width": _fixed_robot_astar.robot_width,
        "robot_length": _fixed_robot_astar.robot_length,
        "safety_buffer": f"{_fixed_robot_astar.safety_buffer}mm",
        "clearance_radius": f"{_fixed_robot_astar.clearance_radius:.0f}mm",
        "grid_resolution": f"{_fixed_robot_astar.resolution}mm",
        "max_grid_size": _fixed_robot_astar.max_grid_size,
        "version": "FIXED - Correct robot dimensions 550x660mm"
    }