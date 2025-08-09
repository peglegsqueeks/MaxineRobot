from abc import ABC, abstractmethod
from typing import Generic, TypeVar
from ..types.MovementDirection import MovementDirection

# the type of readings from this sensor
ActionConfig = TypeVar("ActionConfig")


class VelocityConfig:
    """
    Configuration for velocity actions
    """
    def __init__(self, direction: MovementDirection, speed: float):
        self.direction = direction
        self.speed = speed
    
    def __str__(self):
        return f"VelocityConfig(direction={self.direction.value}, speed={self.speed})"


class VelocityManager(ABC, Generic[ActionConfig]):
    """
    Base class for velocity management
    """
    
    def __init__(self, name) -> None:
        self.name = name
    
    @abstractmethod
    def perform_action(self, config: VelocityConfig):
        """
        Perform velocity action
        
        arguments:
            - config: VelocityConfig with direction and speed
        """
        pass