import pygame

from ..types.Sound import Sound
from .ActionManager import ActionManager


class SoundManager(ActionManager[Sound]):
    """
    The Action Manager for playing sounds from file
    """

    def __init__(self, channel: int) -> None:
        """
        Initialises the manager.

        arguments:
            - channel: the audio channel to play on
        """
        self.channel = pygame.mixer.Channel(channel)

    def perform_action(self, config: Sound):
        """
        The manager's implementation of perform action.
        Will load a sound from file and play it.
        """
        sound = config.load_sound()
        self.channel.play(sound)
