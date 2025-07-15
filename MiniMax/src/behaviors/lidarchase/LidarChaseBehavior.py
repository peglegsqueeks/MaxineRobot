import py_trees
import math
import time
from typing import List, Optional, Tuple
from queue import LifoQueue

from ..action_managers.VelocityManager import VelocityConfig
from ..types.MovementDirection import MovementDirection
from ..path_finding.Position import Position
from ..path_finding.new_a_star import a_star
from ..sensors.CameraSensor import CameraSensor
from ..sensors.LidarSensor import LidarSensor  # Assuming you have this


class LidarChaseBehavior(py_trees.behaviour.Behaviour):
    """
    LiDAR Chase behavior that moves the robot towards the closest person while avoiding obstacles
    """
    
    def __init__(self, 
                 velocity_manager,
                 camera_sensor: CameraSensor,
                 lidar_sensor,  # Your LiDAR sensor
                 head_manager=None,
                 name="LiDAR Chase"):
        super().__init__(name)
        
        self.velocity_manager = velocity_manager
        self.camera_sensor = camera_sensor
        self.lidar_sensor = lidar_sensor
        self.head_manager = head_manager
        
        # Path following state
        self.current_path = []
        self.current_waypoint_index = 0
        self.target_person = None
        
        # Timing and thresholds
        self.last_path_calculation = 0
        self.path_recalc_interval = 0.5  # Recalculate path every 500ms
        self.waypoint_reached_threshold = 300  # 30cm tolerance
        self.person_lost_timeout = 3.0  # Stop if no person for 3 seconds
        self.last_person_seen = 0
        
        # Movement parameters
        self.base_speed = 0.8  # Moderate speed for safety
        self.turn_speed = 0.6
        self.approach_distance = 1000  # Stop 1m from person
        
        # Head scanning for person search
        self.head_scan_positions = [-45, -20, 0, 20, 45]  # Degrees
        self.current_scan_index = 0
        self.last_head_move = 0
        self.head_scan_interval = 1.0  # Move head every 1 second when searching
        
        self.logger.debug("LiDAR Chase behavior initialized")

    def setup(self, **kwargs):
        """Setup behavior"""
        self.current_path = []
        self.current_waypoint_index = 0
        self.target_person = None
        self.last_person_seen = time.time()
        return True

    def initialise(self):
        """Initialize behavior when it starts"""
        self.logger.debug("Starting LiDAR Chase mode")
        self.current_path = []
        self.current_waypoint_index = 0
        
        # Stop any current movement
        self.velocity_manager.perform_action(VelocityConfig(MovementDirection.NONE, 0))

    def update(self):
        """Main behavior update loop"""
        current_time = time.time()
        
        try:
            # 1. Get person detection from camera
            person_detected = self.get_closest_person()
            
            if person_detected:
                self.target_person = person_detected
                self.last_person_seen = current_time
                self.logger.debug(f"Person detected at distance: {person_detected.distance:.0f}mm")
                
                # Check if close enough to person
                if person_detected.distance < self.approach_distance:
                    self.logger.info("Reached person - stopping")
                    self.stop_robot()
                    return py_trees.common.Status.SUCCESS
                
            else:
                # No person detected - scan with head if available
                if self.head_manager:
                    self.scan_for_person(current_time)
                
                # Check if person lost for too long
                if current_time - self.last_person_seen > self.person_lost_timeout:
                    self.logger.info("Person lost - stopping chase")
                    self.stop_robot()
                    return py_trees.common.Status.FAILURE
                
                # Continue with last known target if recently seen
                if not self.target_person:
                    self.stop_robot()
                    return py_trees.common.Status.RUNNING

            # 2. Get obstacles from LiDAR
            obstacles = self.get_lidar_obstacles()
            
            # 3. Calculate or update path if needed
            if (current_time - self.last_path_calculation > self.path_recalc_interval or 
                not self.current_path):
                
                if self.target_person:
                    self.calculate_path_to_person(obstacles, self.target_person)
                    self.last_path_calculation = current_time

            # 4. Execute movement along path
            if self.current_path and self.current_waypoint_index < len(self.current_path):
                movement_result = self.execute_path_movement()
                
                if movement_result == "waypoint_reached":
                    self.current_waypoint_index += 1
                    
                elif movement_result == "path_completed":
                    self.logger.info("Path completed")
                    return py_trees.common.Status.SUCCESS
                    
            else:
                # No valid path - stop and try again
                self.stop_robot()
                
            return py_trees.common.Status.RUNNING
            
        except Exception as e:
            self.logger.error(f"Error in LiDAR chase: {e}")
            self.stop_robot()
            return py_trees.common.Status.FAILURE

    def get_closest_person(self) -> Optional[Position]:
        """Get closest person from camera sensor"""
        try:
            person_data = self.camera_sensor.get_closest_person_fast()
            
            if not person_data:
                return None
            
            # Convert camera coordinates to robot position
            # Camera is offset from robot center
            camera_offset_x = 0      # Camera centered left-right
            camera_offset_y = 200    # Camera 20cm forward from lidar
            
            # Get person position relative to camera
            x_mm = person_data.get('x_mm', 0)
            y_mm = person_data.get('z_mm', 0)  # z_mm is forward distance
            
            # Adjust for camera offset to get position relative to robot center
            robot_x = x_mm - camera_offset_x
            robot_y = y_mm - camera_offset_y
            
            # Convert to polar coordinates for pathfinding
            distance = math.sqrt(robot_x**2 + robot_y**2)
            angle = math.atan2(robot_x, robot_y)
            
            return Position(angle=angle, distance=distance)
            
        except Exception as e:
            self.logger.error(f"Error getting person position: {e}")
            return None

    def get_lidar_obstacles(self) -> List[Position]:
        """Get obstacles from LiDAR sensor"""
        try:
            # Get LiDAR readings - adjust this based on your LiDAR sensor interface
            lidar_readings = self.lidar_sensor.get_reading()
            
            if not lidar_readings:
                return []
            
            obstacles = []
            
            # Convert LiDAR readings to Position objects
            # Adjust this based on your LiDAR data format
            for reading in lidar_readings:
                # Assuming reading has angle and distance
                angle = reading.get('angle', 0)  # radians
                distance = reading.get('distance', 0)  # mm
                
                # Filter out invalid readings
                if 100 < distance < 8000:  # 10cm to 8m range
                    obstacles.append(Position(angle=angle, distance=distance))
            
            return obstacles
            
        except Exception as e:
            self.logger.error(f"Error getting LiDAR obstacles: {e}")
            return []

    def calculate_path_to_person(self, obstacles: List[Position], target: Position):
        """Calculate path to person using A* algorithm"""
        try:
            self.logger.debug(f"Calculating path to person at {target}")
            
            # Use your existing A* pathfinding
            path, debug_info = a_star(obstacles, target)
            
            if path and len(path) > 1:
                self.current_path = path[1:]  # Skip current position
                self.current_waypoint_index = 0
                
                self.logger.debug(f"Path calculated with {len(self.current_path)} waypoints")
                self.logger.debug(f"Debug info: {debug_info}")
            else:
                self.logger.warning("No valid path found to person")
                self.current_path = []
                
        except Exception as e:
            self.logger.error(f"Error calculating path: {e}")
            self.current_path = []

    def execute_path_movement(self) -> str:
        """Execute movement towards current waypoint"""
        if (self.current_waypoint_index >= len(self.current_path)):
            return "path_completed"
        
        current_waypoint = self.current_path[self.current_waypoint_index]
        
        # Calculate movement direction and speed
        direction, speed = self.calculate_movement_to_waypoint(current_waypoint)
        
        # Send movement command
        config = VelocityConfig(direction, speed)
        self.velocity_manager.perform_action(config)
        
        # Check if waypoint reached
        if current_waypoint.distance < self.waypoint_reached_threshold:
            self.logger.debug(f"Waypoint {self.current_waypoint_index} reached")
            return "waypoint_reached"
        
        return "moving"

    def calculate_movement_to_waypoint(self, waypoint: Position) -> Tuple[MovementDirection, float]:
        """Calculate movement direction and speed to reach waypoint"""
        
        # Convert angle to movement direction
        angle_deg = math.degrees(waypoint.angle)
        distance = waypoint.distance
        
        # Determine primary movement direction based on angle
        if abs(angle_deg) < 15:  # Forward
            if distance > 1000:  # Far away
                return MovementDirection.FORWARDS, self.base_speed
            else:  # Close - slow down
                return MovementDirection.FORWARDS, self.base_speed * 0.6
                
        elif 15 <= angle_deg < 45:  # Forward-right
            return MovementDirection.FORWARDS_RIGHT, self.base_speed * 0.8
            
        elif angle_deg >= 45:  # Right
            return MovementDirection.RIGHT, self.turn_speed
            
        elif -45 < angle_deg <= -15:  # Forward-left
            return MovementDirection.FORWARDS_LEFT, self.base_speed * 0.8
            
        elif angle_deg <= -45:  # Left
            return MovementDirection.LEFT, self.turn_speed
            
        else:
            # Default to forward
            return MovementDirection.FORWARDS, self.base_speed * 0.5

    def scan_for_person(self, current_time: float):
        """Scan head to search for person"""
        if not self.head_manager:
            return
            
        if current_time - self.last_head_move > self.head_scan_interval:
            # Move to next scan position
            angle = self.head_scan_positions[self.current_scan_index]
            self.head_manager.set_head_angle_degrees(angle)
            
            self.current_scan_index = (self.current_scan_index + 1) % len(self.head_scan_positions)
            self.last_head_move = current_time
            
            self.logger.debug(f"Scanning head to {angle} degrees")

    def stop_robot(self):
        """Stop robot movement"""
        self.velocity_manager.perform_action(VelocityConfig(MovementDirection.NONE, 0))
        self.logger.debug("Robot stopped")

    def terminate(self, new_status):
        """Cleanup when behavior terminates"""
        self.stop_robot()
        
        # Center head if available
        if self.head_manager:
            self.head_manager.center_head()
            
        self.logger.debug("LiDAR Chase behavior terminated")

    def get_status_info(self) -> dict:
        """Get status information for debugging"""
        return {
            "target_person": str(self.target_person) if self.target_person else None,
            "current_path_length": len(self.current_path),
            "current_waypoint": self.current_waypoint_index,
            "last_person_seen": time.time() - self.last_person_seen,
            "behavior_status": str(self.status)
        }