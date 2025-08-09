import time
import py_trees
from py_trees.common import Status
from ..action_managers.VelocityManager import VelocityConfig
from ..types.MovementDirection import MovementDirection
from .MaxineBehavior import MaxineBehavior


class MoveBehavior(MaxineBehavior):
    """
    Moves the robot in a particular direction
    """

    def __init__(self, direction: MovementDirection):
        """
        Initialises the direction

        arguments:
            - direction: the direction to move in
        """
        super().__init__(name=f"Move in {direction.value} direction")
        self.direction = direction
        self.blackboard.register_key("SPEED", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        speed_exists = self.blackboard.exists("SPEED")
        if speed_exists:
            speed= self.blackboard.get("SPEED")
        else:
            speed= 1
        # tell velocity manager to move in direction
        velocity_manager = self.get_robot().velocity_manager
        velocity_manager.perform_action(VelocityConfig(self.direction,speed))

        # always returns sucess
        return Status.SUCCESS


class IncreaseSpeedBehavior(MaxineBehavior):
    def __init__(self):
        super().__init__(name=f"Increase Behavior")
        self.blackboard.register_key("SPEED", access=py_trees.common.Access.WRITE)

    def update(self):
        speed_exists = self.blackboard.exists("SPEED")
        if speed_exists:
            speed= self.blackboard.get("SPEED")
        else:
            speed= 1

        speed = speed + 0.1
        speed = min(speed, 2)

        self.get_robot().speech_manager.perform_action(f"Speed {int(speed*100)} percent")
        time.sleep(1.5)

        self.blackboard.set("SPEED", speed)
        return Status.SUCCESS


class DecreaseSpeedBehavior(MaxineBehavior):
    def __init__(self):
        super().__init__(name=f"Decrease Behavior")
        self.blackboard.register_key("SPEED", access=py_trees.common.Access.WRITE)

    def update(self):
        speed_exists = self.blackboard.exists("SPEED")
        if speed_exists:
            speed= self.blackboard.get("SPEED")
        else:
            speed= 1
                
        speed = speed - 0.1
        speed = max(speed, 0.4)

        self.get_robot().speech_manager.perform_action(f"Speed {int(speed*100)} percent")
        time.sleep(1.5)
        
        self.blackboard.set("SPEED", speed)

        return Status.SUCCESS