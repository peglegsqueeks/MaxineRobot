from threading import Thread
import time
from abc import ABC, abstractmethod

from . import THREAD_STOP_EVENT


class LoopingThread(Thread, ABC):
    """
    Base class for all looping threads.
    This is an abstract class that cannot be instantiated.
    """

    def __init__(self, ms_delay: int = 0) -> None:
        """
        Initialises the thread

        arguments:
            - ms_delay: the miliseconds to wait in between the polling loop
        """
        super().__init__(daemon=True)
        self.ms_delay = ms_delay

    def run(self):
        """
        Runs the thread
        """

        self.on_start()

        while True:
            # do work
            self.tick()

            # await delay
            time.sleep(self.ms_delay/1000)

            # stop if master event is set
            if THREAD_STOP_EVENT.is_set():
                break

        self.on_finish()

    @abstractmethod
    def on_finish(self):
        """
        Will be run upon finishing the thread.
        Children must implement this class
        """
        pass

    @abstractmethod
    def tick(self):
        """
        Is called at every loop of the thread.
        Children must implement this class
        """
        pass

    @abstractmethod
    def on_start(self):
        """
        Runs before the thread starts.
        Children must implement this class
        """
        pass
