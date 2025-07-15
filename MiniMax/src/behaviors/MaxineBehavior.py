from typing import Any, Callable
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from ..robot.Robot import Robot

ROBOT_BLACKBOARD = "ROBOT_BLACKBOARD"
ROBOT_KEY = "ROBOT_KEY"


class MaxineBehavior(Behaviour):
    """
    The base class for all Behaviors in our trees.
    This is an abstract class that cannot be instantiated.
    It just allows all tree nodes to acess the robot object via the get_robot() function.
    This allows all nodes to check sensors and perform actions
    """

    def __init__(self, name: str):
        """
        Initialises the behavior
        """
        super().__init__(name)

        # use py_trees blackboard functionality to share robot across all nodes in the tree
        self.blackboard = self.attach_blackboard_client(name=ROBOT_BLACKBOARD)
        self.blackboard.register_key(ROBOT_KEY, access=py_trees.common.Access.WRITE)

    def get_robot(self) -> Robot:
        """
        Returns the robot object
        """
        return self.blackboard.get(ROBOT_KEY)
