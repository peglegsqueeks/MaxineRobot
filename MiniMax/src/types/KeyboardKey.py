import pygame


from enum import Enum


class KeyboardKey(Enum):
    """
    Represents keyboard keys
    """

    # velocity movement
    Q = pygame.K_q
    E = pygame.K_e
    W = pygame.K_w
    A = pygame.K_a
    S = pygame.K_s
    D = pygame.K_d
    Z = pygame.K_z
    X = pygame.K_x
    
    # head control
    O = pygame.K_o
    P = pygame.K_p
    
    # mode control
    ESC = pygame.K_ESCAPE
    K = pygame.K_k
    C = pygame.K_c
    L = pygame.K_l
    R = pygame.K_r
    H = pygame.K_h
    G = pygame.K_g
    T = pygame.K_t
    U = pygame.K_u
    
    Plus = pygame.K_EQUALS
    Minus = pygame.K_MINUS