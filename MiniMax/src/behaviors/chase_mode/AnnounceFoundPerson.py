from ...behaviors.MaxineBehavior import MaxineBehavior
from ...types.FacialAnimation import FacialAnimation
from ...types.Sound import Sound


from py_trees.common import Status


class AnnounceFoundPerson(MaxineBehavior):
    """
    Announce that the robot has found a person
    """

    def __init__(self):
        super().__init__(f"Announce found person")

    def update(self) -> Status:
        robot = self.get_robot()

        # play sound
        sound_manager = robot.sound_manager
        sound_manager.perform_action(Sound.found_person)

        # facial animation
        animation_manager = robot.facial_animation_manager
        animation_manager.perform_action(FacialAnimation.found_person)

        # always sucessed
        return Status.SUCCESS
