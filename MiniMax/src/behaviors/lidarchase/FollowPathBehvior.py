from math import degrees, radians
import math
import time
from typing import List, Optional
from src.action_managers.VelocityManager import VelocityConfig
from src.path_finding.Position import Position
from src.types.MovementDirection import MovementDirection
from src.behaviors.MaxineBehavior import MaxineBehavior
from py_trees.common import Status
import py_trees


def to_radians(degrees_val) -> float:
    return radians(degrees_val) % (2 * math.pi)


class FasterFollowPathBehavior(MaxineBehavior):
    """
    FASTER path following behavior with INCREASED robot speeds
    """
    
    def __init__(self):
        super().__init__("Faster Follow Path")
        
        # Blackboard keys
        self.blackboard.register_key("PATH", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.READ)
        
        # Navigation parameters
        self.current_waypoint_index = 0
        self.waypoint_reached_threshold = 600  # 60cm threshold (larger for faster movement)
        self.close_waypoint_threshold = 1000   # 100cm to start looking ahead
        self.target_reached_threshold = 500    # 50cm to consider target reached
        
        # INCREASED movement parameters for faster robot
        self.base_speed = 1.2                  # INCREASED from 0.6 to 1.2
        self.turn_speed = 0.8                  # INCREASED from 0.4 to 0.8  
        self.precision_speed = 0.7             # INCREASED from 0.3 to 0.7
        self.max_speed = 1.8                   # INCREASED from 1.0 to 1.8
        self.forward_speed = 1.4               # INCREASED forward speed
        
        # State tracking
        self.last_debug_time = 0
        self.debug_interval = 4.0
        self.stuck_detection_time = 0
        self.last_waypoint_time = time.time()
        self.max_waypoint_time = 6.0  # Reduced time to reach waypoint (faster movement)
        
        # Performance tracking
        self.path_completion_count = 0
        self.last_position_update = time.time()
        
    def path_exists(self) -> bool:
        """Check if valid path exists"""
        try:
            if not self.blackboard.exists("PATH"):
                return False
                
            path = self.blackboard.get("PATH")
            return path is not None and isinstance(path, list) and len(path) > 0
        except Exception:
            return False
    
    def get_target_position(self) -> Optional[Position]:
        """Get current target person position"""
        try:
            import depthai as dai
            target_person: dai.SpatialImgDetection = self.blackboard.get("TARGET_PERSON")
            
            x_camera = target_person.spatialCoordinates.x
            z_camera = target_person.spatialCoordinates.z
            
            angle_rad = math.atan2(x_camera, z_camera)
            distance = max(100, z_camera - 300)  # Reduced offset for faster approach
            
            return Position(angle=angle_rad, distance=distance)
            
        except Exception:
            return None
    
    def calculate_movement_direction_fast(self, position: Position) -> MovementDirection:
        """
        FASTER movement direction calculation with larger angle tolerances
        """
        angle_deg = degrees(position.angle)
        
        # Normalize angle to -180 to +180 range
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360
        
        # WIDER angle tolerances for faster movement
        if -20 <= angle_deg <= 20:  # Wider straight ahead tolerance
            return MovementDirection.FORWARDS
        
        elif 20 < angle_deg <= 50:  # Forward-right
            return MovementDirection.FORWARDS_RIGHT
        
        elif -50 <= angle_deg < -20:  # Forward-left
            return MovementDirection.FORWARDS_LEFT
        
        elif 50 < angle_deg <= 130:  # Right turn
            return MovementDirection.RIGHT
        
        elif -130 <= angle_deg < -50:  # Left turn
            return MovementDirection.LEFT
        
        else:  # Behind robot (130 to 180 or -130 to -180)
            # Choose shorter turn direction
            if angle_deg > 0:
                return MovementDirection.RIGHT
            else:
                return MovementDirection.LEFT
    
    def calculate_movement_speed_fast(self, position: Position, movement_direction: MovementDirection) -> float:
        """
        Calculate FASTER movement speeds
        """
        distance = position.distance
        angle_deg = abs(degrees(position.angle))
        
        # FASTER base speed calculation
        if distance < 800:  # Close - but still fast
            speed = self.precision_speed  # 0.7 (was 0.3)
        elif distance < 1500:  # Medium distance - faster
            speed = self.base_speed       # 1.2 (was 0.6)
        elif distance < 2500:  # Far - much faster
            speed = self.forward_speed    # 1.4 (was 0.8)
        else:  # Very far - maximum speed
            speed = self.max_speed        # 1.8 (was 1.0)
        
        # Adjust for movement direction (less reduction for faster movement)
        if movement_direction in [MovementDirection.LEFT, MovementDirection.RIGHT]:
            speed *= 0.8  # Was 0.6, now 0.8 for faster turns
        elif movement_direction in [MovementDirection.FORWARDS_LEFT, MovementDirection.FORWARDS_RIGHT]:
            speed *= 0.9  # Was 0.8, now 0.9 for faster diagonal movement
        
        # Less speed reduction for angle corrections (faster movement)
        if angle_deg > 40:  # Was 30, now 40 for wider tolerance
            speed *= 0.9  # Was 0.7, now 0.9
        elif angle_deg > 70:  # Was 60, now 70
            speed *= 0.8  # Was 0.5, now 0.8
        
        # Clamp speed to valid range
        return max(0.4, min(self.max_speed, speed))  # Minimum speed increased to 0.4
    
    def find_optimal_waypoint_fast(self, path: List[Position]) -> Optional[Position]:
        """
        Find optimal waypoint with FASTER waypoint advancement
        """
        if not path:
            return None
        
        current_time = time.time()
        
        # Check for stuck condition (faster timeout)
        if current_time - self.last_waypoint_time > self.max_waypoint_time:
            # Skip ahead more conservatively to prevent spinning
            self.current_waypoint_index = min(self.current_waypoint_index + 1, len(path) - 1)
            self.last_waypoint_time = current_time
        
        # Ensure waypoint index is valid
        if self.current_waypoint_index >= len(path):
            self.current_waypoint_index = len(path) - 1
        
        current_waypoint = path[self.current_waypoint_index]
        
        # Check if current waypoint is reached (larger threshold for faster movement)
        if current_waypoint.distance < self.waypoint_reached_threshold:
            if self.current_waypoint_index < len(path) - 1:
                self.current_waypoint_index += 1
                self.last_waypoint_time = current_time
                return path[self.current_waypoint_index]
            else:
                # Reached final waypoint
                return current_waypoint
        
        # Conservative lookahead to prevent skipping waypoints
        if (current_waypoint.distance < self.close_waypoint_threshold and 
            self.current_waypoint_index < len(path) - 1):
            
            # Look ahead to next waypoint
            next_waypoint = path[self.current_waypoint_index + 1]
            
            # If next waypoint is in similar direction, target it instead
            angle_diff = abs(current_waypoint.angle - next_waypoint.angle)
            if angle_diff < math.radians(25):  # More conservative tolerance
                return next_waypoint
        
        return current_waypoint
    
    def execute_movement_fast(self, direction: MovementDirection, speed: float):
        """Execute movement command with robot"""
        try:
            robot = self.get_robot()
            if robot.velocity_manager:
                movement_config = VelocityConfig(direction, speed)
                robot.velocity_manager.perform_action(movement_config)
        except Exception as e:
            print(f"❌ Movement execution error: {e}")
    
    def stop_robot(self):
        """Stop robot movement"""
        try:
            robot = self.get_robot()
            if robot.velocity_manager:
                stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                robot.velocity_manager.perform_action(stop_config)
        except Exception as e:
            print(f"❌ Stop robot error: {e}")
    
    def update(self) -> Status:
        """FASTER update with increased speeds"""
        try:
            # Check path exists
            if not self.path_exists():
                self.stop_robot()
                return Status.FAILURE
            
            # Get path and target position
            path = self.blackboard.get("PATH")
            target_position = self.get_target_position()
            
            if len(path) == 0:
                self.stop_robot()
                return Status.SUCCESS
            
            # Check if target is reached
            if target_position and target_position.distance < self.target_reached_threshold:
                self.stop_robot()
                self.path_completion_count += 1
                return Status.SUCCESS
            
            # Find optimal waypoint with faster advancement
            next_waypoint = self.find_optimal_waypoint_fast(path)
            if not next_waypoint:
                self.stop_robot()
                return Status.FAILURE
            
            # Check if reached final waypoint
            if (self.current_waypoint_index >= len(path) - 1 and 
                next_waypoint.distance < self.waypoint_reached_threshold):
                self.stop_robot()
                return Status.SUCCESS
            
            # Calculate FASTER movement
            movement_direction = self.calculate_movement_direction_fast(next_waypoint)
            movement_speed = self.calculate_movement_speed_fast(next_waypoint, movement_direction)
            
            # Execute FASTER movement
            self.execute_movement_fast(movement_direction, movement_speed)
            
            return Status.RUNNING
            
        except Exception as e:
            self.stop_robot()
            return Status.FAILURE
    
    def initialise(self):
        """Reset navigation state when behavior starts"""
        self.current_waypoint_index = 0
        self.last_waypoint_time = time.time()
        self.stuck_detection_time = 0
    
    def terminate(self, new_status: Status):
        """Clean termination with robot stop"""
        try:
            self.stop_robot()
            
            # Reset state for next run
            self.current_waypoint_index = 0
                
        except Exception as e:
            pass
        
        super().terminate(new_status)
    
    def get_navigation_status(self) -> dict:
        """Get detailed navigation status for debugging"""
        try:
            path = self.blackboard.get("PATH") if self.path_exists() else []
            target_position = self.get_target_position()
            
            return {
                'waypoint_index': self.current_waypoint_index,
                'total_waypoints': len(path),
                'target_distance': target_position.distance if target_position else 0,
                'paths_completed': self.path_completion_count,
                'waypoint_threshold': self.waypoint_reached_threshold,
                'target_threshold': self.target_reached_threshold,
                'base_speed': self.base_speed,
                'max_speed': self.max_speed,
                'forward_speed': self.forward_speed,
                'speed_mode': 'FAST',
                'time_since_last_waypoint': time.time() - self.last_waypoint_time
            }
        except Exception:
            return {'error': 'Status unavailable'}


# Aliases for backward compatibility
FollowPathBehavior = FasterFollowPathBehavior
EnhancedFollowPathBehavior = FasterFollowPathBehavior