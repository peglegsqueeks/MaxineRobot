from queue import Queue, Empty
from .LoopingThread import LoopingThread
import pyttsx3


class TextToSpeechThread(LoopingThread):
    """
    Thread running the Text to Speech engine
    """

    def __init__(self, queue: Queue, rate: int, voice: str, volume: int):
        """
        Initializes the thread

        arguments:
            - queue: the queue where text to say will be passed
            - rate: the engine rate
            - voice: the engine voice
            - volume: the engine volume
        """
        super().__init__(ms_delay=50)
        self.queue = queue
        self.rate = rate
        self.voice = voice
        self.volume = volume
        self.engine = None

    def on_start(self):
        # Initialize the pyttsx3 engine
        self.engine = pyttsx3.init()
        self.engine.setProperty("rate", self.rate)
        self.engine.setProperty("voice", self.voice)
        self.engine.setProperty("volume", self.volume)

        # Start the non-blocking event loop
        self.engine.startLoop(False)

    def tick(self):
        try:
            # Attempt to get something to say
            phrase = self.queue.get_nowait()
            print(f"TTS: Speaking -> {phrase}")
            self.engine.say(phrase)
        except Empty:
            pass

        # Always iterate the engine in each tick
        self.engine.iterate()

    def on_finish(self):
        self.engine.endLoop()