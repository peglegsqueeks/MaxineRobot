import os
import time

import pygame

from ..types.FacialAnimation import FacialAnimation
from .ActionManager import ActionManager


class FacialAnimationManager(ActionManager[FacialAnimation]):
    """
    Action Manager for Facial Animations
    """

    # constant variables
    ANIMATION_FOLDER = "Animations"
    DEFAULT_ANIMATION = FacialAnimation.advised

    def __init__(self) -> None:
        """
        Initialises the manager

        arguments:
            - display: the pygram display object
        """
        super().__init__("Facial Animation Manager")
        self.resting_face_img = self.load_default_face()
        self.resting_face_img =pygame.transform.scale(self.resting_face_img,(1920, 1080))    
                
           
        
        self.open_window()
        # show resting face
        self.display.blit(self.resting_face_img, (0, 0))
        pygame.display.flip()

    def close_window(self):
        pygame.display.quit()


    def open_window(self):
        display_flags = pygame.FULLSCREEN
        display = pygame.display.set_mode((0, 0), display_flags)
        self.display  = display

    def perform_action(self, config: FacialAnimation):
        """
        Implementation of the managers perform action function
        Will load the relevant images for the animation
        And animate the face
        """
        # load images
        animation_images = self.load_animation_images(config)

        # run animation
        for image in animation_images:
            image =pygame.transform.scale(image, (1920, 1080)) 
            self.display.blit(image, (0, 0))
            pygame.display.flip()
            # time.sleep(0.01)

        # end animation with resting face
        self.display.blit(self.resting_face_img, (0, 0))
        pygame.display.flip()

    def load_default_face(self):
        """
        Loads the default face image to be stored
        """
        animation_dir = self.get_animation_dir(self.DEFAULT_ANIMATION)
        image_filename = list(sorted(os.listdir(animation_dir)))[0]
        return self.load_image(animation_dir, image_filename)

    def get_animation_dir(self, animation: FacialAnimation) -> str:
        """
        Returns the directory of an animation type
        """
        return os.path.join(self.ANIMATION_FOLDER, animation.value)

    def load_image(self, animation_dir: str, image_filename: str):
        """
        Loads an animation image
        """
        return pygame.image.load(os.path.join(animation_dir, image_filename))

    def load_animation_images(self, animation: FacialAnimation):
        """
        Loads all the images for a particular animation
        """
        animation_dir = self.get_animation_dir(animation)
        animation_images = os.listdir(animation_dir)
        animation_images = sorted(animation_images)

        return [
            self.load_image(animation_dir, image_name)
            for image_name in animation_images[::1]
        ]
