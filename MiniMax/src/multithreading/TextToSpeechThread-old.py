from queue import Queue
from .LoopingThread import LoopingThread
import pyttsx3


class TextToSpeechThread(LoopingThread):
    """
    Thread running the Text to Speech engine
    """

    def __init__(self, queue: Queue, rate: int, voice: str, volume: int):
        """
        Initialises the thread

        arguments:
            - queue: the queue where text to say will be passed
            - rate: the engine rate
            - voice: the engine voice
            - volume: the engine volume
        """
        super().__init__(ms_delay=50)
        self.rate = rate
        self.queue = queue
        self.voice = voice
        self.volume = volume
        #self.engine = None

    def on_start(self):
        # initialise engine
        self.engine = pyttsx3.init()
        self.engine.setProperty("rate", self.rate)
        self.engine.setProperty("voice", self.voice)
        self.engine.setProperty("volume", self.volume)
        self.engine.startLoop(False)

    def tick(self):
        # if nothing in queue
        if self.queue.empty() and self.engine._inLoop:
        # Inside your tick() or thread loop:
            if self.queue!=None:
                self.engine.say('hi')
                self.engine.runAndWait()


            

        # add queue saying
        else:
            data = self.queue.get()
            self.engine.say(data)

    def on_finish(self):
        self.engine.endLoop()
