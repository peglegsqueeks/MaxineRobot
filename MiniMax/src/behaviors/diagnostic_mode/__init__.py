import py_trees
from src.behaviors.SetRobotMode import SetRobotMode
from src.behaviors.diagnostic_mode.ReportErrorBehavior import ReportErrorBehavior
from src.behaviors.utils import make_press_esc_to_exit_behavior
from src.types.RobotModes import RobotMode
from src.behaviors.CountBehavior import CountBehavior
from src.behaviors.chase_mode.SelectPerson import SelectPerson
from typing import Tuple
from py_trees.composites import Sequence, Selector

def make_diagnostic_sub_tree():
    exit_mode_behavior = make_press_esc_to_exit_behavior(RobotMode.IDLE)

    report_error_behavior = ReportErrorBehavior()

    set_robot_mode = SetRobotMode(RobotMode.IDLE)

     
    diagnostic_sequence = Sequence("Diagnostic sequence", 
                                   memory=True,
                                    children=[report_error_behavior, set_robot_mode, ]
                                    )

    # Diagnostic mode:
    #   Exit if esc pressed 
    #   OR 
    #   run diagnostic mode
    #       - Report Errore AND
    #       - Set robot mode to idle
    return Selector("Diagnostic Mode", memory=True,children=[exit_mode_behavior, diagnostic_sequence])