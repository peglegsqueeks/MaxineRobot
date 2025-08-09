import os
import pygame
import multiprocessing
import time

def play_windows():
    pygame.quit()
    time.sleep(1)
    # Set the window position BEFORE initializing the display.
    print('still workin')
    pygame.init()
    os.environ['SDL_VIDEO_WINDOW_POS'] = f"{100},{100}"
    print('hmmm still going')
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    print('set mode')
    pygame.display.set_caption('hello there')
    clock = pygame.time.Clock()
    print('NOT anymore!')
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            # Draw a simple background (black)
            screen.fill((0, 0, 0))
            pygame.display.flip()
            clock.tick(30)
        
        #pygame.quit()
    play_windows()



