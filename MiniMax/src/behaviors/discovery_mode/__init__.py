import py_trees
from src.behaviors.CountBehavior import CountBehavior
from src.behaviors.chase_mode.SelectPerson import SelectPerson
from typing import Tuple
from py_trees.composites import Sequence, Selector

from src.behaviors.discovery_mode import HeadScanningBehavior
from src.behaviors.discovery_mode.HeadScanningBehavior import HeadScanningBehavior
from ...behaviors.MoveBehavior import MoveBehavior
from ...behaviors.SetRobotMode import SetRobotMode
from ...behaviors.discovery_mode.AnnounceFoundObject import AnnounceFoundObject
from ...behaviors.discovery_mode.RecogniseObject import RecogniseObject
from ...types.DirectionSensorLocation import DirectionSensorLocation
from ...types.MovementDirection import MovementDirection
from ...behaviors.ShowCameraFrame import ShowCameraFrame
from ...behaviors.utils import make_press_esc_to_exit_behavior
from ...types.RobotModes import RobotMode
from src.behaviors.HeadTurnBehavior import HeadTurn
from src.types.HeadMovementDirection import HeadMovementDirection
from ...behaviors.MoveBehavior import MoveBehavior
from py_trees.common import Status
from py_trees.behaviour import Behaviour
from src.behaviors.HeadTurnBehavior import HeadTurn
from src.types.HeadMovementDirection import HeadMovementDirection
from ...behaviors.MoveBehavior import MoveBehavior

def make_discovery_sub_tree():
    
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)
    show_frame = ShowCameraFrame()
    recognise_object = RecogniseObject()
    announce_found_object = AnnounceFoundObject()
    turn_behavior= HeadScanningBehavior()

    #Force head to turn after 20 ticks by count behavior returning FALSE when it reaches it's target (20)
    count_behavior= CountBehavior(20)
    
    recog_sequence = Sequence("discovery mode behavior",
        memory=True,children=[count_behavior,recognise_object,announce_found_object])

    recog_turn_selector=Selector("recognise or turn",
        memory=True,children=[recog_sequence,turn_behavior])
    
    discovery_seq = Sequence(
        "discovery mode behavior",
        memory=True,
        children=[
            show_frame,
            recog_turn_selector,    
        ],
    )

    return Selector("Discovery mode", memory=True,children=[exit_mode_behavior,discovery_seq])

