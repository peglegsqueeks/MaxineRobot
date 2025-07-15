from ...behaviors.MaxineBehavior import MaxineBehavior
from ...types.DirectionSensorLocation import DirectionSensorLocation
from ...types.FacialAnimation import FacialAnimation
from ...types.Sound import Sound


from py_trees.common import Status


class AnnounceObstacleDetection(MaxineBehavior):
    """
    Accounces and obstacle has been detected
    """

    def __init__(self, direction: DirectionSensorLocation):
        super().__init__(f"Announce obstacle in {direction.value} direction")
        self.direction = direction

    def update(self) -> Status:
        robot = self.get_robot()

        # play sound
        sound_manager = robot.sound_manager
        sound_manager.perform_action(Sound.inmyway)

        # play animation
        animation_manager = robot.facial_animation_manager
        animation_manager.perform_action(FacialAnimation.inmyway)

        # always passes
        return Status.SUCCESS
