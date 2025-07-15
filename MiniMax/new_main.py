import py_trees
import pygame
from py_trees.trees import BehaviourTree
from src.behaviors.game_mode import make_game_sub_tree
from src.behaviors.diagnostic_mode import make_diagnostic_sub_tree
from src.behaviors.lidarchase import make_lidar_chase_sub_tree
from src.multithreading import graceful_thread_exit
from src.behaviors.MaxineBehavior import ROBOT_BLACKBOARD, ROBOT_KEY
from src.behaviors.utils import make_mode_sub_tree
from src.behaviors.idle_mode import make_idle_mode_sub_tree
from src.behaviors.keyboard_mode import make_keyboard_mode_sub_tree
from src.behaviors.chase_mode import make_chase_mode_sub_tree
from src.behaviors.discovery_mode import make_discovery_sub_tree
from src.behaviors.lidar_test_mode import make_lidar_test_sub_tree
from src.types.RobotModes import RobotMode
from src.robot.Robot import Robot
from src.robot.RobotFactory import RobotFactory

from src.action_managers.ServoController import ServoController
from src.action_managers.HeadMoveManager import HeadVelocityManager
from src.action_managers.DirectVelocityManager import DirectVelocityManager

CONFIG_FILE = "src/configs/config_maxine.yaml"


def initialize_pygame_for_all_displays():
    """
    Initialize pygame for ALL display output
    """
    try:
        if not pygame.get_init():
            pygame.init()
            pygame.display.init()
            display_info = pygame.display.Info()
            return True
        else:
            return True
    except Exception:
        return False


def initialize_servo_system():
    """
    Initialize the servo system with error handling
    """
    try:
        servo_controller = ServoController(i2c_address=11)
        
        if not servo_controller.initialize():
            return None, None
        
        head_velocity_manager = HeadVelocityManager(servo_controller)
        
        return servo_controller, head_velocity_manager
        
    except Exception:
        return None, None


def initialize_velocity_system(simulate=False):
    """
    Initialize the DirectVelocityManager for robot movement
    """
    try:
        # Initialize DirectVelocityManager with parameters from DirectVelocityManager.py
        # These values match what's in the DirectVelocityManager class
        velocity_manager = DirectVelocityManager(
            version="24V16",           # Motor controller version
            maxaccel=50,               # MAX_ACCELERATION from the class
            maxdecel=40,               # MAX_DECELERATION from the class  
            current_limit=15000,       # CURRENT_LIMIT from the class
            simulate=simulate          # Set to False for real robot, True for testing
        )
        
        return velocity_manager
        
    except Exception as e:
        return None


def build_robot() -> Robot:
    """
    Build robot with servo system, DirectVelocityManager, and pygame display support
    """
    try:
        initialize_pygame_for_all_displays()
        
        robot_factory = RobotFactory(CONFIG_FILE)
        robot = robot_factory.build_robot()
        
        if robot is None:
            return None
        
        # Initialize servo system
        servo_controller, head_velocity_manager = initialize_servo_system()
        
        if servo_controller is not None and head_velocity_manager is not None:
            robot.servo_controller = servo_controller
            robot.head_velocity_manager = head_velocity_manager
            robot.head_move_manager = head_velocity_manager
            robot.head_manager = head_velocity_manager
            robot.head_servo = servo_controller
        else:
            robot.servo_controller = None
            robot.head_velocity_manager = None
            robot.head_move_manager = None
            robot.head_manager = None
            robot.head_servo = None
        
        # Initialize DirectVelocityManager for robot movement
        # Set simulate=False for real robot operation
        # Set simulate=True for testing without hardware
        direct_velocity_manager = initialize_velocity_system(simulate=False)
        
        if direct_velocity_manager is not None:
            # Set up both possible names for compatibility
            robot.direct_velocity_manager = direct_velocity_manager
            robot.velocity_manager = direct_velocity_manager
        else:
            robot.direct_velocity_manager = None
            robot.velocity_manager = None
        
        if hasattr(robot, 'i2c_sensor_thread'):
            robot.i2c_sensor_thread = None
        
        return robot
        
    except Exception:
        return None


def build_robot_behavior_tree() -> BehaviourTree:
    """
    Build the behavior tree for the robot
    """
    mode_to_sub_tree = {
        RobotMode.IDLE: make_idle_mode_sub_tree(),
        RobotMode.KEYBOARD_CONTROL: make_keyboard_mode_sub_tree(),
        RobotMode.CHASE: make_chase_mode_sub_tree(),
        RobotMode.LIDARCHASE: make_lidar_chase_sub_tree(),
        RobotMode.DISCOVERY: make_discovery_sub_tree(),
        RobotMode.DIAGNOSTIC: make_diagnostic_sub_tree(),
        RobotMode.PLAYGAME: make_game_sub_tree(),
        RobotMode.LIDAR_TEST: make_lidar_test_sub_tree(),
    }

    sub_trees = []
    for robot_mode, sub_tree in mode_to_sub_tree.items():
        wrapped_sub_tree = make_mode_sub_tree(robot_mode, sub_tree)
        sub_trees.append(wrapped_sub_tree)

    robot_root = py_trees.composites.Selector("root", memory=True, children=sub_trees)

    robot_root.blackboard = robot_root.attach_blackboard_client(name=ROBOT_BLACKBOARD)
    robot_root.blackboard.register_key(ROBOT_KEY, access=py_trees.common.Access.WRITE)
    robot_root.setup_with_descendants()
    return BehaviourTree(robot_root)


def initialise_black_board(robot):
    """
    Initialize the blackboard
    """
    configuration = py_trees.blackboard.Client(name=ROBOT_BLACKBOARD)
    configuration.register_key(ROBOT_KEY, access=py_trees.common.Access.WRITE)
    configuration.set(ROBOT_KEY, robot)


def shutdown_robot(robot):
    """
    Shutdown robot systems including DirectVelocityManager
    """
    try:
        if robot:
            # Shutdown DirectVelocityManager first to stop motors safely
            if hasattr(robot, 'direct_velocity_manager') and robot.direct_velocity_manager:
                robot.direct_velocity_manager.shutdown()
            
            if hasattr(robot, 'velocity_manager') and robot.velocity_manager and robot.velocity_manager != robot.direct_velocity_manager:
                robot.velocity_manager.shutdown()
            
            if hasattr(robot, 'current_mode'):
                try:
                    if pygame.get_init():
                        pygame.event.clear()
                        
                        if pygame.display.get_init():
                            try:
                                screen = pygame.display.get_surface()
                                if screen:
                                    screen.fill((0, 0, 0))
                                    pygame.display.flip()
                            except Exception:
                                pass
                        
                except Exception:
                    pass
            
            if hasattr(robot, 'servo_controller') and robot.servo_controller:
                robot.servo_controller.shutdown()
            
            robot.shutdown()
            
    except Exception:
        pass


@graceful_thread_exit
def run():
    """
    Main execution function
    """
    robot = None
    
    try:
        robot = build_robot()
        
        if robot is None:
            return
        
        initialise_black_board(robot)
        behavior_tree = build_robot_behavior_tree()

        def stop_condition(tree: BehaviourTree):
            robot = tree.root.blackboard.get(ROBOT_KEY)
            if robot.current_mode == RobotMode.EXIT:
                exit()
        
        behavior_tree.tick_tock(50, post_tick_handler=stop_condition)
        
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    finally:
        shutdown_robot(robot)


if __name__ == "__main__":
    run()