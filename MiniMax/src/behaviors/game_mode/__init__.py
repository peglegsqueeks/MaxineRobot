import py_trees
from src.behaviors.game_mode.PlayGameBehavior import PlayGameBehavior
from src.behaviors.SetRobotMode import SetRobotMode
from src.behaviors.diagnostic_mode.ReportErrorBehavior import ReportErrorBehavior
from src.behaviors.utils import make_press_esc_to_exit_behavior
from src.types.RobotModes import RobotMode
from src.behaviors.CountBehavior import CountBehavior
from src.behaviors.chase_mode.SelectPerson import SelectPerson
from typing import Tuple
from py_trees.composites import Sequence, Selector

def make_game_sub_tree():
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)

    play_game_behavior = PlayGameBehavior()

    set_robot_mode = SetRobotMode(RobotMode.IDLE)

     
    play_game_sequence = Sequence("Game sequence", 
                                   memory=True,
                                    children=[play_game_behavior, set_robot_mode]
                                    )

    # Diagnostic mode:
    #   Exit if esc pressed 
    #   OR 
    #   run diagnostic mode
    #       - Report Errore AND
    #       - Set robot mode to idle
    return Selector("Game mode", memory=True,children=[exit_mode_behavior, play_game_sequence])