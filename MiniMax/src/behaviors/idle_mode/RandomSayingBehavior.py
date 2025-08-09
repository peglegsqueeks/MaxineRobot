import random
from py_trees.common import Status
from ...types.FacialAnimation import FacialAnimation
from ...types.Sound import Sound
from ..MaxineBehavior import MaxineBehavior

SAYINGS = [
    "better",
    "compute",
    "cross",
    "danger",
    "directive",
    "humans",
    "cautionroguerobots",
    "chess",
    "dangerwillrobinson",
    "malfunction2",
    "nicesoftware2",
    "no5alive",
    "program",
    "selfdestruct",
    "shallweplayagame",
    "silly",
    "stare",
    "world",
    "comewithme",
    "gosomewhere2",
    "lowbattery",
    "robotnotoffended",
    "satisfiedwithmycare",
    "waitbeforeswim",
    "helpme",
]


class RandomSayingBehavior(MaxineBehavior):
    """
    Behavior that animates face and plays sound of a random saying
    """

    def __init__(self):
        super().__init__("Random Saying")
        self.sampled_saying = None

    def select_random_saying(self):
        """
        Randomly samples a saying
        """
        self.sampled_saying = random.randint(0, len(SAYINGS) - 1)

    def initialise(self) -> None:
        """
        When initialising the node, sample a saying
        """
        self.select_random_saying()

    def update(self) -> Status:
        """
        When running the node, make animation and sound managers play the saying
        """
        # get saying
        saying = SAYINGS[self.sampled_saying]

        # get animation and sound managers
        robot = self.get_robot()
        animation_manager = robot.facial_animation_manager
        sound_manager = robot.sound_manager

        # perform the animation
        sound_manager.perform_action(Sound.from_name(saying))
        animation_manager.perform_action(FacialAnimation.from_name(saying))

        # the node always passses
        return Status.SUCCESS

    def terminate(self, new_status: Status) -> None:
        """
        When finished, reset sampled saying
        """
        self.sampled_saying = None
