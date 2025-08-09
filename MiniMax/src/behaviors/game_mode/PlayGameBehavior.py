import random
import time
import pygame
import os, sys
from pygame.locals import *
from os import path
sys.path.append("../")
defaultpaths = os.path.join('/home/raspberrypi/maxine/MiniMax/Explosion/', '/home/raspberrypi/maxine/MiniMax/Boss', '/home/raspberrypi/maxine/MiniMax/Enemy', '/home/raspberrypi/maxine/MiniMax/Powerups')
from .play_windows import play_windows
from parallax import parallax
from py_trees.common import Status
from ...types.FacialAnimation import FacialAnimation
from ...types.Sound import Sound
from ..MaxineBehavior import MaxineBehavior
from multiprocessing import Process
from .play_game import play_game

class PlayGameBehavior(MaxineBehavior):
    """
    Play Game
    """

    def __init__(self):
        super().__init__("Play Game")
        
    def update(self):
        robot = self.get_robot()
        robot.facial_animation_manager.close_window()
        
        robot.speech_manager.perform_action('Stand by for Game Mode')
        time.sleep(0.5)
        
        play_game()

               
        # encapsulate into function
        # update() -> start new process 
        # play_windows()
        #self.process.start()
        #self.process.join()

        robot.facial_animation_manager.open_window()
        return Status.SUCCESS
        #    
        # Import libraries
        
        # from pygame.locals import *
        
        

       
        # 
        #    
        #     
        # 
        # 
        
        
        return Status.SUCCESS