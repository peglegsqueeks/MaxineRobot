from abc import ABC, abstractmethod
from typing import Generic, TypeVar

# the type of readings from this sensor
ActionConfig = TypeVar("ActionConfig")


class  ActionManager(ABC, Generic[ActionConfig]):
    """
    Base class for an Action Manager.
    An ActionManager handles a capability that the robot has.
    This is an abstract class that cannot be instantiated
    """

    def __init__(self, name) -> None:
        """ "
        Initialises the Action Manager

        arguments:
            - name: the name of the manager
        """
        self.name = name

    @abstractmethod
    def perform_action(self, config: ActionConfig):
        """
        Perform an action from this manager
        Children of this class must implement this method

        arguments:
            - config: The configuration for this action specific to this class
        """
        pass
