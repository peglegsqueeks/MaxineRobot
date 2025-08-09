import py_trees
from ...behaviors.MaxineBehavior import MaxineBehavior
from ...types.FacialAnimation import FacialAnimation
from ...types.Sound import Sound
from py_trees.common import Status
import depthai as dai

class AnnounceFoundObject(MaxineBehavior):
    """
    Announce that the robot has found a person
    """

    def __init__(self):
        super().__init__(f"Announce found person")
        #print("start")
        self.blackboard.register_key("TARGET_OBJECT", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("UPDATE_COUNTER", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        robot = self.get_robot()
        target_object="Test"
        # play sound
        target_object = self.blackboard.get("TARGET_OBJECT")
        target_object=list(target_object)
        #print(len(target_object))
        update_count = self.blackboard.get("UPDATE_COUNTER")
        #print("Counter .... " +str(update_count))
        if update_count>19:
            robot.speech_manager.perform_action(str(target_object[0]))
            update_count=1
            self.blackboard.set("UPDATE_COUNTER", update_count)
        # always sucessed
        return Status.SUCCESS
