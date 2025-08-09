from py_trees.common import Status
from ..MaxineBehavior import MaxineBehavior

from abc import ABC, abstractmethod


class Condition(MaxineBehavior, ABC):
    """
    The base class for all conditions.
    A condition is a Node in the tree that will return:
        - SUCESS if a condition is met
        - FAILURE if it is not

    This can be combined with Selector nodes to conditionally follow paths in the tree
    This is an abstract class that cannot be instantated
    """

    def __init__(self, name: str):
        """
        Initialises the condition
        """
        super().__init__(name)

    @abstractmethod
    def condition_met(self) -> bool:
        """
        Checks if the condition is met
        Children must implement this
        """
        pass

    def update(self) -> Status:
        """
        Tree Behavior update function
        Returns SUCESS or FAILURE depending on condition
        """
        if self.condition_met():
            return Status.SUCCESS

        return Status.FAILURE
