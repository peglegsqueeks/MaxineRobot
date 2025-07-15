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

    def bring_to_front(self):
        """
        Brings the pygame window back to the front/focus
        """
        try:
            # Try to bring pygame window to front
            pygame.display.flip()
            
            # For systems that support it, try to raise the window
            if hasattr(pygame.display, 'get_surface'):
                surface = pygame.display.get_surface()
                if surface:
                    # Refresh the display to potentially bring it forward
                    pygame.event.pump()
                    pygame.display.update()
                    
            # Set window position environment variable for future windows
            os.environ['SDL_VIDEO_WINDOW_POS'] = '0,0'
            
        except Exception as e:
            print(f"Warning: Could not bring pygame window to front: {e}")

    def open_window(self):
        # Set pygame window to stay on top and position it
        os.environ['SDL_VIDEO_WINDOW_POS'] = '0,0'  # Position at top-left
        
        # Initialize pygame if not already done
        if not pygame.get_init():
            pygame.init()
            
        display_flags = pygame.FULLSCREEN
        display = pygame.display.set_mode((0, 0), display_flags)
        
        # Set window caption for identification
        pygame.display.set_caption("Maxine Avatar")
        
        self.display = display
        
        # Try to ensure window stays active
        pygame.event.pump()
        pygame.display.flip()

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