from py_trees.common import Status
from src.behaviors.MaxineBehavior import MaxineBehavior
from src.types.RobotModes import RobotMode


class CheckRobotMode(MaxineBehavior):
    """
    Behavior that checks if the robot is in a specific mode
    Returns SUCCESS if robot is in the specified mode, FAILURE otherwise
    """
    
    def __init__(self, expected_mode: RobotMode):
        super().__init__(f"Check Robot Mode ({expected_mode.name})")
        self.expected_mode = expected_mode
        
    def update(self) -> Status:
        """
        Check if robot is in the expected mode
        
        Returns:
            SUCCESS if robot is in expected mode
            FAILURE if robot is in different mode
        """
        try:
            robot = self.get_robot()
            current_mode = robot.current_mode
            
            if current_mode == self.expected_mode:
                return Status.SUCCESS
            else:
                return Status.FAILURE
                
        except Exception as e:
            print(f"❌ CheckRobotMode error: {e}")
            return Status.FAILURE
    
    def __str__(self):
        return f"CheckRobotMode({self.expected_mode.name})"