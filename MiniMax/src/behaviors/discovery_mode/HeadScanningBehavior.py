import py_trees
from ..MaxineBehavior import MaxineBehavior
from ...types.FacialAnimation import FacialAnimation
from ...types.Sound import Sound
from py_trees.common import Status
import depthai as dai
from src.behaviors.HeadTurnBehavior import HeadTurn
from src.types.HeadMovementDirection import HeadMovementDirection
from ..MoveBehavior import MoveBehavior


class HeadScanningBehavior(MaxineBehavior):
    """
    Move head a little bit to the left then right
    """

    def __init__(self):
        super().__init__(f"Announce found person")
        # start off moving left
        self.direction=HeadMovementDirection.LEFT

    def switch_direction(self):
        if self.direction ==  HeadMovementDirection.LEFT:
            self.direction=HeadMovementDirection.RIGHT
        else:
            self.direction = HeadMovementDirection.LEFT

    def update(self) -> Status:
        # switch head direction if reached the end
        robot = self.get_robot()
        head_move_manager = robot.head_move_manager
        
        
        headposition = head_move_manager.get_head_position()
        if abs(headposition) >= 0.92:
            self.switch_direction()
    
        head_move_manager.perform_action(self.direction)
        head_move_manager.perform_action(self.direction)


        return Status.SUCCESS

       