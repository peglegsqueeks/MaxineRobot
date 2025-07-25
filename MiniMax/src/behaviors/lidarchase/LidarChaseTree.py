"""
LiDAR Chase Behavior Tree Integration
This file provides the behavior tree that handles LIDARCHASE mode
"""

import py_trees
from py_trees.common import Status
from py_trees.composites import Selector, Sequence
from src.behaviors.MaxineBehavior import MaxineBehavior
from src.behaviors.SetRobotMode import SetRobotMode
from src.behaviors.lidarchase.UpdateLidarPlot import UpdateLidarPlot
from src.types.RobotModes import RobotMode
from src.types.MovementDirection import MovementDirection
from src.action_managers.VelocityManager import VelocityConfig
import depthai as dai
import time


class PersonDetectionBehavior(MaxineBehavior):
    """Person detection for LiDAR chase"""
    
    def __init__(self):
        super().__init__("Person Detection")
        self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.WRITE)
    
    def update(self) -> Status:
        try:
            robot = self.get_robot()
            
            camera_reading = robot.camera_sensor.get_reading()
            
            if camera_reading:
                people = camera_reading.get_people_locations()
                
                if people:
                    target_person = people[0]
                    self.blackboard.set("TARGET_PERSON", target_person)
                    return Status.SUCCESS
                else:
                    try:
                        self.blackboard.unset("TARGET_PERSON")
                    except:
                        pass
                    return Status.FAILURE
            else:
                return Status.FAILURE
                
        except Exception as e:
            return Status.FAILURE


class TargetReachedChecker(MaxineBehavior):
    """Check if target is reached"""
    
    def __init__(self, threshold_mm=500):
        super().__init__("Target Reached Checker")
        self.threshold_mm = threshold_mm
        self.blackboard.register_key("TARGET_PERSON", access=py_trees.common.Access.READ)
    
    def update(self) -> Status:
        try:
            target_person: dai.SpatialImgDetection = self.blackboard.get("TARGET_PERSON")
            
            if target_person:
                spatial_coords = getattr(target_person, 'spatialCoordinates', None)
                if spatial_coords:
                    z_distance = getattr(spatial_coords, 'z', float('inf'))
                    
                    if z_distance <= self.threshold_mm:
                        robot = self.get_robot()
                        
                        if robot.velocity_manager:
                            stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
                            robot.velocity_manager.perform_action(stop_config)
                        
                        if robot.speech_manager:
                            robot.speech_manager.perform_action("Person reached!")
                        
                        time.sleep(2.0)
                        robot.set_mode(RobotMode.IDLE)
                        
                        return Status.SUCCESS
                    else:
                        return Status.FAILURE
                else:
                    return Status.FAILURE
            else:
                return Status.FAILURE
                
        except KeyError:
            return Status.FAILURE
        except Exception as e:
            return Status.FAILURE


def create_lidar_chase_behavior_tree():
    """
    Create the complete LiDAR chase behavior tree
    
    Returns UpdateLidarPlot which handles display creation and all chase logic.
    """
    return UpdateLidarPlot()


def create_simple_lidar_chase_behavior():
    """
    Create a simple LiDAR chase behavior (just the main display behavior)
    """
    return UpdateLidarPlot()


def create_lidar_chase_sequence():
    """
    Create the LiDAR chase sequence with person detection and main behavior
    """
    try:
        person_detection = PersonDetectionBehavior()
        
        main_lidar_behavior = UpdateLidarPlot()
        
        return Sequence(
            "LiDAR Chase Sequence",
            memory=True,
            children=[
                person_detection,
                main_lidar_behavior
            ]
        )
    except Exception as e:
        return UpdateLidarPlot()


def create_lidar_chase_with_target_check():
    """
    Create LiDAR chase with target reached checking
    """
    try:
        target_reached = TargetReachedChecker(500)
        
        chase_sequence = create_lidar_chase_sequence()
        
        return Selector(
            "LiDAR Chase with Target Check",
            memory=True,
            children=[
                target_reached,
                chase_sequence
            ]
        )
    except Exception as e:
        return UpdateLidarPlot()


# Export the main functions
__all__ = [
    "create_lidar_chase_behavior_tree",
    "create_simple_lidar_chase_behavior", 
    "create_lidar_chase_sequence",
    "create_lidar_chase_with_target_check",
    "UpdateLidarPlot",
    "PersonDetectionBehavior",
    "TargetReachedChecker"
]