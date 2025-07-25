import pygame
from src.behaviors.MaxineBehavior import MaxineBehavior
from py_trees.common import Status
from src.types.RobotModes import RobotMode


class KeyboardHandler(MaxineBehavior):
    """
    Handles keyboard input for lidar chase mode, especially escape key
    """
    
    def __init__(self):
        super().__init__("Keyboard Handler")

    def update(self) -> Status:
        """
        Check for keyboard events, especially escape key to exit mode
        """
        try:
            # Process all pygame events
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        # Escape key pressed - exit to IDLE mode
                        robot = self.get_robot()
                        robot.set_mode(RobotMode.IDLE)
                        return Status.SUCCESS  # Signal to exit
                elif event.type == pygame.QUIT:
                    # Window close button pressed - exit to IDLE mode
                    robot = self.get_robot()
                    robot.set_mode(RobotMode.IDLE)
                    return Status.SUCCESS
            
            # No exit condition met, continue running
            return Status.FAILURE
            
        except Exception as e:
            return Status.FAILURE