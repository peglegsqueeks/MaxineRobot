# This file is kept for compatibility but is no longer needed
# since all ultrasonic sensors have been removed.
# 
# The original I2CSensorThread was used to poll multiple ultrasonic sensors
# across multiple UltraBorg boards. Since you now only have one UltraBorg 
# board for servo control (no sensors), this threading system is no longer needed.
#
# If you have other code that imports this file, you can either:
# 1. Remove those imports, or 
# 2. Keep this placeholder file to prevent import errors

from typing import Dict, List
from .LoopingThread import LoopingThread


class I2CSensorThread(LoopingThread):
    """
    DEPRECATED: This thread was used to poll I2C ultrasonic sensors
    Since ultrasonic sensors have been removed, this is now a placeholder
    """

    def __init__(self, sensors: List = None, ms_delay: int = 50) -> None:
        """
        Placeholder initialization - does nothing since sensors are removed
        """
        super().__init__(ms_delay)
        self.sensors = sensors or []
        print("WARNING: I2CSensorThread is deprecated - no ultrasonic sensors to poll")

    def init_boards(self) -> Dict:
        """
        Placeholder - no boards to initialize since sensors are removed
        """
        return {}

    def on_start(self):
        """
        Placeholder - nothing to start
        """
        pass

    def tick(self):
        """
        Placeholder - no sensors to update
        """
        pass

    def on_finish(self):
        """
        Placeholder - nothing to finish
        """
        pass