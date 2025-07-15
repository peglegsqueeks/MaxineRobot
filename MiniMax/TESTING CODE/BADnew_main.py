import py_trees
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
from src.types.RobotModes import RobotMode
from src.robot.Robot import Robot
from src.robot.RobotFactory import RobotFactory

# Import UltraBorg for servo control
import UltraBorg

CONFIG_FILE = "src/configs/config_maxine.yaml"

def initialize_board():
    """
    Initialize the UltraBorg board for servo control
    Configured for I2C bus 7, address 11, servo on channel 3
    """
    try:
        print("Initializing UltraBorg board for servo control...")
        print("Configuration: I2C bus=7, address=11, servo channel=3")
        
        # Create UltraBorg instance for servo control
        board = UltraBorg.UltraBorg()
        
        # Set I2C address to 11 (0x0B in hex)
        board.i2cAddress = 11
        
        # Set I2C bus to 7 if the library supports it
        if hasattr(board, 'i2cBus'):
            board.i2cBus = 7
            print(f"Set I2C bus to: {board.i2cBus}")
        
        # Initialize the board
        '''if not board.Init():
            print("ERROR: Failed to initialize UltraBorg board!")
            print("Check:")
            print("  - I2C bus 7 is available and accessible")
            print("  - UltraBorg is connected to I2C address 11")
            print("  - No I2C address conflicts")
            print("  - Proper I2C permissions (try running with sudo if needed)")
            return None
        '''
        print(f"UltraBorg board initialized successfully at address {board.i2cAddress} on bus 7")
        
        # Test servo functionality by checking if servo methods exist
        '''if hasattr(board, 'SetServoPosition3') and hasattr(board, 'GetServoPosition3'):
            print("Servo control methods available for channel 3")
           
            # Test servo communication
            try:
                # Try to read current servo position to test communication
                current_pos = board.GetServoPosition3()
                print(f"Current servo 3 position: {current_pos}")
                
                # Optional: Set servo to center position (0.0) as initialization test
                print("Setting servo to center position (0.0) for initialization test...")
                board.SetServoPosition3(0.0)
                
                # Verify the position was set
                import time
                time.sleep(0.1)  # Small delay for servo to respond
                new_pos = board.GetServoPosition3()
                print(f"Servo position after centering: {new_pos}")
                
            except Exception as e:
                print(f"Warning: Servo communication test failed: {e}")
                print("Servo may still work, but communication is unreliable")
                
        else:
            print("ERROR: UltraBorg board missing required servo methods!")
            print("Check UltraBorg_old library version and servo channel support")
            return None
        '''    
        print("UltraBorg servo board initialization complete!")
        return board
        
    except Exception as e:
        print(f"Failed to initialize UltraBorg board: {e}")
        print("Troubleshooting steps:")
        print("  - Check I2C bus 7 availability: ls /dev/i2c-*")
        print("  - Check I2C device detection: i2cdetect -y 7")
        print("  - Verify UltraBorg address 11 or 0b(hex) appears in scan")
        print("  - Ensure proper I2C permissions")
        print("  - Check UltraBorg_old library installation")
        return None

def build_robot() -> Robot:
    # 🔴 THIS IS WHERE YOU NEED TO ADD THE BOARD! 🔴
    
    # Initialize the servo board first
    board = initialize_board()
    
    # Pass the board to the RobotFactory
    robot_factory = RobotFactory(CONFIG_FILE, board=board)
    return robot_factory.build_robot()

def build_robot_behavior_tree() -> BehaviourTree:
    """
    Builds the behavior tree for the robot.
    The tree is composed of sub trees for each mode.
    """
    # mapping of mode to sub tree
    mode_to_sub_tree = {
        RobotMode.IDLE: make_idle_mode_sub_tree(),
        RobotMode.KEYBOARD_CONTROL: make_keyboard_mode_sub_tree(),
        RobotMode.CHASE: make_chase_mode_sub_tree(),
        RobotMode.LIDARCHASE: make_lidar_chase_sub_tree(),
        RobotMode.DISCOVERY: make_discovery_sub_tree(),
        RobotMode.DIAGNOSTIC: make_diagnostic_sub_tree(),
        RobotMode.PLAYGAME: make_game_sub_tree()
    }
    # wrap each sub tree with a condition on current robot mode
    sub_trees = []
    for robot_mode, sub_tree in mode_to_sub_tree.items():
        wrapped_sub_tree = make_mode_sub_tree(robot_mode, sub_tree)
        sub_trees.append(wrapped_sub_tree)
    # root node is a selector (will run first passing sub tree)
    # ie will run sub tree of the current mode
    robot_root = py_trees.composites.Selector("root", memory=True, children=sub_trees)
    # add blackboard to root (so that stop condition can access robot)
    robot_root.blackboard = robot_root.attach_blackboard_client(name=ROBOT_BLACKBOARD)
    robot_root.blackboard.register_key(ROBOT_KEY, access=py_trees.common.Access.WRITE)
    robot_root.setup_with_descendants()
    return BehaviourTree(robot_root)

def initialise_black_board(robot):
    """
    Initialises the black board which gives all nodes in the tree access to the robot class
    arguments:
        - robot: the robot
    """
    configuration = py_trees.blackboard.Client(name=ROBOT_BLACKBOARD)
    configuration.register_key(ROBOT_KEY, access=py_trees.common.Access.WRITE)
    configuration.set(ROBOT_KEY, robot)

@graceful_thread_exit
def run():
    robot = build_robot()
    
    # Print any initialization errors
    if robot.errors_ocurred:
        print("Robot initialization errors:")
        for error in robot.errors_ocurred:
            print(f"  - {error}")
    
    # Print board status for debugging
    board_status = robot.get_board_status()
    print(f"Board status: {board_status}")
    
    # build tree
    initialise_black_board(robot)
    behavior_tree = build_robot_behavior_tree()
    
    # exits the program if the robot is in exit mode
    def stop_condition(tree: BehaviourTree):
        robot = tree.root.blackboard.get(ROBOT_KEY)
        if robot.current_mode == RobotMode.EXIT:
            exit()
    
    try:
        # continuously tick the behavior tree, checking robot is not in exit mode each time
        behavior_tree.tick_tock(50, post_tick_handler=stop_condition)
    finally:
        robot.shutdown()
        raise

if __name__ == "__main__":
    run()