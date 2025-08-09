import random
import time
from py_trees.common import Status
from ...types.FacialAnimation import FacialAnimation
from ...types.Sound import Sound
from ..MaxineBehavior import MaxineBehavior

class ReportErrorBehavior(MaxineBehavior):
    """
    Report errors
    """

    def __init__(self):
        super().__init__("Report Errors")

    def update(self):
        # first, tell the user we are reporting errors occured
        robot = self.get_robot()
        robot.speech_manager.perform_action('Stand by for error report')
        time.sleep(2)
        if len(robot.errors_ocurred) <1:
            robot.speech_manager.perform_action('No errors to report')
            return Status.SUCCESS
        
        for error in range(len(robot.errors_ocurred)):
            robot.speech_manager.perform_action(robot.errors_ocurred[error])
            time.sleep(0.75)
        # for each error occured, read it 
        # out
        
        return Status.SUCCESS