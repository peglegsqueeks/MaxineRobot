from ..multithreading.TextToSpeechThread import TextToSpeechThread
from .ActionManager import ActionManager
import queue

class SpeechManager(ActionManager[str]):
    """
    The Manager for Text To Speech.
    This will run on a separate thread to prevent blocking.
    """

    def __init__(self, rate, voice, volume) -> None:
        """
        Initilises the manager

        arguments:
            - rate: the bit rate
            - voice: the voice to use
            - volumne: the volume to play at
        """
        super().__init__("Speech Manager")

        # build and start TTS thread
        self.speech_queue = queue.Queue()
        self.thread = TextToSpeechThread(self.speech_queue, rate, voice, volume)
        self.thread.start()

    def perform_action(self, sentence: str):
        """
        Converts text to speech and plays it.
        """
        self.speech_queue.put(sentence)
