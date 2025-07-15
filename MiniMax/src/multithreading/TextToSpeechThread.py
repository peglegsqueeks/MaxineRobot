from queue import Queue, Empty
from .LoopingThread import LoopingThread
import pyttsx3


class TextToSpeechThread(LoopingThread):
    """
    Thread running the Text to Speech engine using pyttsx3
    """

    def __init__(self, queue: Queue, rate: int, voice: str, volume: int):
        super().__init__(ms_delay=100)  # Slightly slower tick to reduce CPU use
        self.queue = queue
        self.rate = rate
        self.voice = voice
        self.volume = volume
        self.engine = None
        self.speaking = False

    def on_start(self):
        self.engine = pyttsx3.init()
        self.engine.setProperty("rate", self.rate)
        self.engine.setProperty("voice", self.voice)
        self.engine.setProperty("volume", self.volume)

    def tick(self):
        if not self.speaking:
            try:
                phrase = self.queue.get_nowait()
                print(f"[TTS] Speaking: {phrase}")
                self.speaking = True
                self.engine.say(phrase)
                self.engine.runAndWait()
                self.speaking = False
            except Empty:
                pass

    def on_finish(self):
        # No need to stop loop; runAndWait handles everything
        pass
