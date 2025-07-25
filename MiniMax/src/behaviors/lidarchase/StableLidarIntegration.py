"""
Stable LiDAR Integration Module
Provides integration functions for the stable LiDAR system with head calibration
"""

from src.behaviors.lidarchase.UpdateLidarPlot import StableLidarChase, IsolatedLidarChase
from src.types.RobotModes import RobotMode
import py_trees


def create_stable_lidar_chase_behavior():
    """
    Create a stable LiDAR chase behavior using the ultra-stable LiDAR system with head calibration
    
    Returns:
        StableLidarChase behavior instance with head calibration
    """
    return StableLidarChase()


def create_lidar_chase_behavior_tree():
    """
    Create the complete stable LiDAR chase behavior tree with head calibration
    
    Returns:
        Stable LiDAR chase behavior with calibration
    """
    return create_stable_lidar_chase_behavior()


def make_stable_lidar_chase_sub_tree():
    """
    Main factory function for creating stable LiDAR chase behavior tree with head calibration
    This replaces the previous isolated/unstable implementations
    
    Returns:
        StableLidarChase behavior instance with head calibration
    """
    return create_stable_lidar_chase_behavior()


def validate_robot_for_stable_lidar(robot):
    """
    Validate that the robot has the necessary components for stable LiDAR operation with head calibration
    
    Args:
        robot: Robot instance to validate
        
    Returns:
        dict: Validation results with status and missing components
    """
    validation_results = {
        'valid': True,
        'missing_components': [],
        'warnings': [],
        'robot_dimensions': '660mm x 550mm',  # Correct dimensions
        'lidar_position': 'center front (330mm from center)',
        'camera_position': '130mm back from lidar, 510mm above',
        'head_calibration': 'enabled'
    }
    
    # Check for required robot components
    required_components = [
        ('camera_sensor', 'Camera sensor for person detection'),
        ('velocity_manager', 'Velocity manager for robot movement'),
        ('keyboard_sensor', 'Keyboard sensor for head calibration'),
    ]
    
    for component, description in required_components:
        if not hasattr(robot, component) or getattr(robot, component) is None:
            validation_results['valid'] = False
            validation_results['missing_components'].append({
                'component': component,
                'description': description
            })
    
    # Check for head control (at least one required)
    head_controllers = [
        ('head_velocity_manager', 'Head velocity manager'),
        ('servo_controller', 'Servo controller for head movement')
    ]
    
    has_head_control = False
    for component, description in head_controllers:
        if hasattr(robot, component) and getattr(robot, component) is not None:
            has_head_control = True
            break
    
    if not has_head_control:
        validation_results['valid'] = False
        validation_results['missing_components'].append({
            'component': 'head_controller',
            'description': 'At least one head controller required for calibration'
        })
    
    # Check for optional but recommended components
    optional_components = [
        ('speech_manager', 'Speech manager for calibration feedback')
    ]
    
    for component, description in optional_components:
        if not hasattr(robot, component) or getattr(robot, component) is None:
            validation_results['warnings'].append({
                'component': component,
                'description': f"Optional: {description}"
            })
    
    return validation_results


def get_stable_lidar_configuration():
    """
    Get configuration information for the stable LiDAR system with head calibration
    
    Returns:
        dict: Configuration details
    """
    return {
        'lidar_system': 'UltraStablePyRPLidarA3',
        'stability_mode': 4,
        'fallback_mode': 'force_scan',
        'motor_pwm': 600,
        'confidence_threshold': 0.15,
        'coordinate_transformer': 'enabled',
        'head_calibration': {
            'enabled': True,
            'calibration_keys': 'O (left), P (right), C (save)',
            'min_lidar_points': 50,
            'position_storage': 'blackboard'
        },
        'robot_dimensions': {
            'length': 660,  # mm
            'width': 550,   # mm
        },
        'sensor_positions': {
            'lidar_x_offset': 330,  # mm from center
            'camera_x_offset': 200, # mm from center (130mm back from lidar)
            'camera_z_offset': 510  # mm above base
        },
        'head_rotation_limits': '±90 degrees',
        'pathfinding': {
            'algorithm': 'A* with robot-aware obstacle buffering',
            'safety_buffer': '100mm (10cm)',
            'grid_resolution': '100mm (10cm)'
        }
    }


def transition_to_stable_lidar_mode(robot):
    """
    Safely transition robot to stable LiDAR chase mode with head calibration
    
    Args:
        robot: Robot instance to transition
        
    Returns:
        bool: True if transition successful
    """
    try:
        # Validate robot components
        validation = validate_robot_for_stable_lidar(robot)
        
        if not validation['valid']:
            return False
        
        # Stop any current movement
        if hasattr(robot, 'velocity_manager') and robot.velocity_manager:
            from src.types.MovementDirection import MovementDirection
            from src.action_managers.VelocityManager import VelocityConfig
            stop_config = VelocityConfig(MovementDirection.NONE, 0.0)
            robot.velocity_manager.perform_action(stop_config)
        
        # Note: Don't center head here - let calibration handle initial positioning
        
        # Set robot mode
        robot.set_mode(RobotMode.LIDARCHASE)
        
        return True
        
    except Exception:
        return False


def get_stable_lidar_status(robot):
    """
    Get current status of stable LiDAR systems with head calibration info
    
    Args:
        robot: Robot instance to check
        
    Returns:
        dict: Status information
    """
    status = {
        'robot_mode': robot.current_mode if hasattr(robot, 'current_mode') else 'unknown',
        'camera_sensor': 'available' if hasattr(robot, 'camera_sensor') and robot.camera_sensor else 'unavailable',
        'velocity_manager': 'available' if hasattr(robot, 'velocity_manager') and robot.velocity_manager else 'unavailable',
        'keyboard_sensor': 'available' if hasattr(robot, 'keyboard_sensor') and robot.keyboard_sensor else 'unavailable',
        'head_controller': 'none',
        'head_calibration': {
            'enabled': True,
            'calibrated_center': 'not_set'
        },
        'configuration': get_stable_lidar_configuration()
    }
    
    # Check head controller type
    if hasattr(robot, 'servo_controller') and robot.servo_controller:
        status['head_controller'] = 'servo_controller'
    elif hasattr(robot, 'head_velocity_manager') and robot.head_velocity_manager:
        status['head_controller'] = 'head_velocity_manager'
    
    # Check for calibrated center in blackboard (if behavior is running)
    try:
        from src.behaviors.MaxineBehavior import ROBOT_BLACKBOARD
        import py_trees
        
        blackboard = py_trees.blackboard.Client(name=ROBOT_BLACKBOARD)
        blackboard.register_key("HEAD_CALIBRATED_CENTER", access=py_trees.common.Access.READ)
        
        if blackboard.exists("HEAD_CALIBRATED_CENTER"):
            calibrated_center = blackboard.get("HEAD_CALIBRATED_CENTER")
            status['head_calibration']['calibrated_center'] = f"{calibrated_center:.3f}"
        
    except Exception:
        pass
    
    return status


# Export main functions for behavior tree integration
__all__ = [
    'create_stable_lidar_chase_behavior',
    'create_lidar_chase_behavior_tree', 
    'make_stable_lidar_chase_sub_tree',
    'validate_robot_for_stable_lidar',
    'get_stable_lidar_configuration',
    'transition_to_stable_lidar_mode',
    'get_stable_lidar_status',
    'StableLidarChase'
]