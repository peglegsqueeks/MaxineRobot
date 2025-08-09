# -*- coding: utf-8 -*-

#    Copyright (C) , 2012 Åke Forslund (ake.forslund@gmail.com)
#
#    Permission to use, copy, modify, and/or distribute this software for any
#    purpose with or without fee is hereby granted, provided that the above
#    copyright notice and this permission notice appear in all copies.
#
#    THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#    WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#    MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#    ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#    WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#    ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#    OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


import os, sys
import pygame
from pygame.locals import *
import time

sys.path.append ("../")
from parallax import parallax

pygame.init()
screen = pygame.display.set_mode((1024, 768), pygame.DOUBLEBUF)
pygame.display.set_caption('Parallax-test')
pygame.mouse.set_visible(0)

orientation = 'vertical'

bg = parallax.ParallaxSurface((1024, 768), pygame.RLEACCEL)
bg.add('bkgd_0.png', 6)
bg.add('bkgd_1.png', 5)
bg.add('bkgd_2.png', 4)
bg.add('bkgd_4.png', 3)
bg.add('bkgd_6.png', 2)
bg.add('bkgd_7.png', 1)

run = True
speed = 0
t_ref = 0
while run:
    for event in pygame.event.get():
        if event.type == QUIT:
            run = False
        if event.type == KEYDOWN and event.key == K_RIGHT:
            speed += 4
        if event.type == KEYUP and event.key == K_RIGHT:
            speed -= 4
        if event.type == KEYDOWN and event.key == K_LEFT:
            speed -= 4
        if event.type == KEYUP and event.key == K_LEFT:
            speed += 4
        if event.type == KEYDOWN and event.key == K_UP:
            orientation = 'vertical'
        if event.type == KEYDOWN and event.key == K_DOWN:
            orientation = 'horizontal'
        
    bg.scroll(speed, orientation)
    t = pygame.time.get_ticks()
    if (t - t_ref) > 60:
        bg.draw(screen)
        pygame.display.flip()
        time.sleep(0.01)

