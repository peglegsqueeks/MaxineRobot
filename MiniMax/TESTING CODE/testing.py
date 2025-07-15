
import time
import UltraBorg
from src.robot.Robot import Robot
from src.robot.RobotFactory import RobotFactory
from src.types.HeadMovementDirection import HeadMovementDirection


CONFIG_FILE = "src/configs/config_maxine.yaml"

board = UltraBorg.UltraBorg()
board.i2cAddress = 10
board.Init()


def build_robot() -> Robot:
    robot_factory = RobotFactory(CONFIG_FILE)
    return robot_factory.build_robot()


robot = build_robot()

for t in range (50):
    robot.head_move_manager.perform_action(HeadMovementDirection.LEFT)
    time.sleep(0.1)


time.sleep(0.5)

for t in range (50):
    robot.head_move_manager.perform_action(HeadMovementDirection.RIGHT)
    time.sleep(0.1)



