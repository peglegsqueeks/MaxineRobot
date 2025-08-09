import py_trees
from py_trees.common import Status

from src.types.HeadMovementDirection import HeadMovementDirection
from ..action_managers.VelocityManager import VelocityConfig
from .MaxineBehavior import MaxineBehavior


class CountBehavior(MaxineBehavior):
    """
    counts amount of ticks until a certain threshold is reached
    Returns success if count has been increased, and returns failure when count is reached
    """

    def __init__(self, target):
        """
        Initialises the direction

        arguments:
            - direction: the direction to move in
        """
        super().__init__(name=f"count behavoir")
        #self.blackboard.register_key("TICK_COUNTER", access=py_trees.common.Access.WRITE)
        self.target=target
        self.tick_counter=0
        #self.blackboard.set("TICK_COUNTER", tick_counter)


    def update(self) -> Status:
        if self.tick_counter>=self.target:
            self.tick_counter=0
            return Status.FAILURE

        self.tick_counter+=1
        return Status.SUCCESS