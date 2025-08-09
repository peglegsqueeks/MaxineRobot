import pygame
import os, sys
from pygame.locals import *
from os import path
sys.path.append("../")
from parallax import parallax
from multiprocessing import Process
import random
from pygame import *
import time


def play_game():
    WIDTH = 800     
    HEIGHT = 600
    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.DOUBLEBUF)

    # screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    # Initializing
    pygame.mixer.pre_init(48000, -16, 8, 8192)# initialise the music and sound mixer
    # pygame.init() # initialise pygame
    pygame.mouse.set_visible(0)

    entity=None

    # Create Graphics Window of Width and Height call it screen
                                      
    music_volume=0.5
    pygame.mixer.music.set_volume(music_volume)
    clock = pygame.time.Clock()
    explosion_finished=False
    shield_kill=False

    POWERUP_TIME =6000
    Boss_died_anim_finished=False
    boss_explode_inprogress=False
    Boss_level_triggered=False
    explosion_time=0

    orientation = 'vertical'
    speed=4

    bg = parallax.ParallaxSurface((1024, 768), pygame.RLEACCEL)
    bg.add('/home/jetson/maxine/MiniMax/src/behaviors/game_mode/bkgd_0.png', 6)
    bg.add('/home/jetson/maxine/MiniMax/src/behaviors/game_mode/bkgd_1.png', 5)
    bg.add('/home/jetson/maxine/MiniMax/src/behaviors/game_mode/bkgd_2.png', 4)
    bg.add('/home/jetson/maxine/MiniMax/src/behaviors/game_mode/bkgd_4.png', 3)
    bg.add('/home/jetson/maxine/MiniMax/src/behaviors/game_mode/bkgd_6.png', 2)
    bg.add('/home/jetson/maxine/MiniMax/src/behaviors/game_mode/bkgd_7.png', 1)

    # Creating colors
    BLUE  = (0, 0, 255)
    RED   = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)

    # Initialise variables for zoom_player function
    x=0
    y=0
    zoom_speed=8
    t_ref = 0
    col1 =(255,244,11)
    col2 =(0,255,3)
    col3 =(255,153,33)
    col4 =(58,0,239)
    col5 =(175,0,231)
    cols = [col1,col2,col3,col4,col5]

    # Define sound objects
    bullet_sound = pygame.mixer.Sound("Sound/Laser.wav")
    crash_sound = pygame.mixer.Sound("Sound/Explosion.wav")  
    boss_hit_sound = pygame.mixer.Sound("Sound/Boss_hit_sound.wav")
    enemy_hit = pygame.mixer.Sound("Sound/Hit_Enemy.wav")
    player_hit = pygame.mixer.Sound("Sound/expl6.wav")
    boss_bullet_sound = pygame.mixer.Sound("Sound/Boss_bullet.wav")
    game_over_message = pygame.mixer.Sound("Sound/Game_over.wav")
    powerup_shields = pygame.mixer.Sound("Sound/Powerup_shields.wav")
    bolt_powerup = pygame.mixer.Sound("Sound/Bolt_powerup.wav")
    boss_explode_sound = pygame.mixer.Sound("Sound/BIG_explosion.wav")
    chris_getready = pygame.mixer.Sound("Sound/Chris_getready.wav")
    owwyeah = pygame.mixer.Sound("Sound/Oh_yeah.wav")
    evil_laugh = pygame.mixer.Sound("Sound/Evil_laugh.wav")

    # Define graphic objects
    player_ship = pygame.image.load("Player/Ship2.png")
    enemy_ships = pygame.image.load("Enemy/Zorg.png")
    boss_monster = pygame.image.load("Boss/Boss1.png")
    player_shield = pygame.image.load("Player/shield.png")
    sheet = pygame.image.load('Explosion/Explosion.png')
    Background = pygame.image.load('/home/jetson/maxine/MiniMax/src/behaviors/game_mode/bkgd_0.png')
    
    img_dir=("Powerups")
    powerup_images = {}
    powerup_images['shield'] = pygame.image.load('Powerups/shield_powerup.png')
    powerup_images['power'] = pygame.image.load('Powerups/bolt.png')
    player_mini_img = pygame.transform.scale(player_ship, (35, 29))


    # -----------------------------------------------------------------------------------------------------------------------------
    # --------------------------------- Load Animated sprites into lists ----------------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------------------

    # loading the explosion images into lists using filename of images with 0-8 on the end

    img_dir = ("Explosion") 
    explosion_anim = {}
    explosion_anim['lg'] = []
    explosion_anim['sm'] = []
    explosion_anim['player'] = []
    for i in range(9):
        filename = 'regularExplosion0{}.png'.format(i)
        img = pygame.image.load(path.join(img_dir, filename)).convert()
        img.set_colorkey(BLACK)
        img_lg = pygame.transform.scale(img, (75, 75))
        explosion_anim['lg'].append(img_lg)
        img_sm = pygame.transform.scale(img, (32, 32))
        explosion_anim['sm'].append(img_sm)
        filename = 'sonicExplosion0{}.png'.format(i)
        img = pygame.image.load(path.join(img_dir, filename)).convert()
        img.set_colorkey(BLACK)
        explosion_anim['player'].append(img)

    # loading the BOSS images into lists using filename of images with 0-7 on the end

    boss_dir = ("Boss")  
    eye_anim = {}
    eye_anim['boss'] = []
    for i in range(8):
        filename = 'Boss{}.png'.format(i)
        img = pygame.image.load(path.join(boss_dir, filename))
        eye_anim['boss'].append(img)

    # loading BIG BOSS EXPLOSION into lists

    size = 192,192
    pos = 0,0
    len_sprt_x,len_sprt_y = size                                         #sprite size
    sprt_rect_x,sprt_rect_y = pos                                        #where to find first sprite on sheet
    sheet = pygame.image.load('Explosion/Explosion.png').convert_alpha() #Load the sheet
    sheet_rect = sheet.get_rect()
    msprites = []   

    for i in range(0,22):#columns
        sheet.set_clip(pygame.Rect(sprt_rect_x, sprt_rect_y, len_sprt_x, len_sprt_y)) #find sprite you want
        sprite = sheet.subsurface(sheet.get_clip())                                   #grab the sprite you want
        msprites.append(sprite)
        sprt_rect_x += len_sprt_x
    bexplosion = msprites

    #Setting up Frames Per Second 
    FPS = 60
    FramePerSec = pygame.time.Clock()
    
    # Other Variables for use in the program
    score = 0
    enemy_speedx = 1.2
    enemy_speedy = 1
    boss_speedx = 1
    boss_speedy = 1
    direction = 1
    scroll_speed = 2

    # Bullet settings
    bullet_speed = -20
    boss_bullet_speed = 12
    bullet_width = 4
    bullet_height = 16
    bullet_color = (255, 255, 255)
    bullets_allowed = 3

    #Setting up Fonts
    font = pygame.font.SysFont("Verdana", 60)
    font_small = pygame.font.SysFont("Verdana", 20)
    game_over_text = font.render("Game Over", True, BLACK)

    # Set screen caption
    pygame.display.set_caption("Chris's Crazy Space Game")

    # Setting up the background for vertical scrolling space effect so it looks like your moving through space! TRICKY TRICKY!
    # bg = pygame.image.load(os.path.join("Background",'bg2.png')).convert()
    # bg_rect = bg.get_rect()
    # bgY = 0 # bgY cordinate is at the top of screen
    # bgY2 = bg.get_height() * -1 # the bgY2 coordinate is a whole screen above the visable area

    # -----------------------------------------------------------------------------------------------------------------------------
    # ------------------------------------ Class Definitions for all the major sprites --------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------------------

    class Powerups(pygame.sprite.Sprite):
        def __init__(self, center):
            pygame.sprite.Sprite.__init__(self)
            self.type = random.choice(['shield', 'power'])
            self.image = powerup_images[self.type]
            self.rect = self.image.get_rect()
            self.rect.center = center
            self.speedy = 5

        def move(self):
            self.rect.y += self.speedy
            # kill if it moves off the bottom of the screen
            if self.rect.top > HEIGHT:
                self.kill()    
        
    # Class function to draw 1st BOSS bullet and move them
    class Boss_Bullet1(pygame.sprite.Sprite):
        def __init__(self, x, y):
            pygame.sprite.Sprite.__init__(self)
            self.image = pygame.Surface((bullet_width, bullet_height))
            self.image.fill(bullet_color)
            self.rect = self.image.get_rect()
            self.rect.bottom = y
            self.rect.centerx = x 
            self.speedy = boss_bullet_speed # boss bullet speed is positive int so bullet moves DOWN the screen

        def move(self):
            self.rect.y += self.speedy
            # kill if it moves off the BOTTOM of the screen
            if self.rect.bottom > HEIGHT:
                self.kill()
                
    # Class function to draw 2nd BOSS bullet and move them
    class Boss_Bullet2(pygame.sprite.Sprite):
        def __init__(self, x, y):
            pygame.sprite.Sprite.__init__(self)
            self.image = pygame.Surface((bullet_width, bullet_height))
            self.image.fill(bullet_color)
            self.rect = self.image.get_rect()
            self.rect.bottom = y + 64   # move start position of second boss bullet to the LEFT WING of the BOSS
            self.rect.centerx = x -116  # move start position of second boss bullet to the LEFT WING of the BOSS
            self.speedy = boss_bullet_speed # boss bullet speed is positive int so bullet moves DOWN the screen

        def move(self):
            self.rect.y += self.speedy
            # kill if it moves off the BOTTOM of the screen
            if self.rect.bottom > HEIGHT:
                self.kill()            
                
    # Class function to draw 3rd BOSS bullet and move them
    class Boss_Bullet3(pygame.sprite.Sprite):
        def __init__(self, x, y):
            pygame.sprite.Sprite.__init__(self)
            self.image = pygame.Surface((bullet_width, bullet_height))
            self.image.fill(bullet_color)
            self.rect = self.image.get_rect()
            self.rect.bottom = y + 64   # move start position of third boss bullet to the RIGHT WING of the BOSS
            self.rect.centerx = x +116  # move start position of third boss bullet to the RIGHT WING of the BOSS
            self.speedy = boss_bullet_speed # boss bullet speed is positive int so bullet moves DOWN the screen

        def move(self):
            self.rect.y += self.speedy
            # kill if it moves off the BOTTOM of the screen
            if self.rect.bottom > HEIGHT:
                self.kill()                        
    
    # Class function to draw bullets and move them
    class Bullet(pygame.sprite.Sprite):
        def __init__(self, x, y):
            pygame.sprite.Sprite.__init__(self)
            self.image = pygame.Surface((bullet_width, bullet_height))
            self.image.fill(bullet_color)
            self.rect = self.image.get_rect()
            self.rect.bottom = y
            self.rect.centerx = x 
            self.speedy = bullet_speed  # bullet speed is negative int so bullet moves UP the screen

        def move(self):
            self.rect.y += self.speedy
            # kill if it moves off the TOP of the screen
            if self.rect.bottom < 0:
                self.kill()
                
    # a class  function to draw the ememies, move them
    class Enemy(pygame.sprite.Sprite):
        def __init__(self):
            pygame.sprite.Sprite.__init__(self)
            self.image = enemy_ships
            self.rect = self.image.get_rect()
            self.radius = int(self.rect.width * .75 / 2) # trying to make a hit box of correct size for enemy need HELP with this
            #pygame.draw.circle(self.image, RED, self.rect.center, self.radius)
            self.rect.x = random.randrange(WIDTH - self.rect.width)
            self.rect.y = random.randrange(-100, -40)
            self.speedy = random.randrange(1, 8)
            self.speedx = random.randrange(-3, 3)
            self.last_update = pygame.time.get_ticks()

        def move(self): # moves the ememies down and randomly to the left or right
            self.rect.x += self.speedx
            self.rect.y += self.speedy + enemy_speedx
            if self.rect.top > HEIGHT + 10 or self.rect.left < -25 or self.rect.right > WIDTH + 20:
                self.rect.x = random.randrange(WIDTH - self.rect.width)
                self.rect.y = random.randrange(-100, -40)
                self.speedy = random.randrange(1, 8)

    # a class  function to draw the BOSS MONSTER, move it
    class Boss(pygame.sprite.Sprite):
        def __init__(self, center, size):     
            pygame.sprite.Sprite.__init__(self)
            self.size = size
            self.image = eye_anim[self.size][0]
            self.rect = self.image.get_rect()
            self.rect.center = center
            self.frame = 0
            self.radius = int(self.rect.width * .68 / 2) # trying to make a hit box of correct size for enemy need HELP with this
            #pygame.draw.circle(self.image, RED, self.rect.center, self.radius)
            self.rect.x = random.randrange(WIDTH - self.rect.width)
            self.rect.y = 100
            self.speedy = boss_speedy
            self.speedx = boss_speedx
            self.shoot_delay = 140
            self.last_shot = pygame.time.get_ticks()
            self.last_update = pygame.time.get_ticks()
            self.frame_rate = 25
            self.life = 100

        def move(self): # moves the ememies down and randomly to the left or right
            if not boss_explode_inprogress:
                now = pygame.time.get_ticks()
                if now - self.last_update > self.frame_rate:
                    self.last_update = now
                    self.frame += 1
                    if self.frame == len(eye_anim[self.size]):
                        self.frame=0
                    else:
                        #center = self.rect.center
                        self.image = eye_anim[self.size][self.frame]
                        #self.rect = self.image.get_rect()
                        #self.rect.center = center  
                self.rect.x += self.speedx
                self.rect.y += self.speedy 
                if self.rect.left < 0 or self.rect.right > WIDTH :
                    #self.rect.x = random.randrange(WIDTH - 100)
                    #self.rect.y = random.randrange(100, 200)
                    #self.speedy = 0
                    self.speedx = self.speedx * -1
                if self.rect.top < 14 or self.rect.bottom > HEIGHT -130:
                    self.speedy = self.speedy * -1
                if self.rect.top < 14:
                    self.rect.y +=1
                if self.rect.bottom > HEIGHT -130:
                    self.rect.y -=1
                reverse_chance =random.randrange(1,100)
                if reverse_chance >=99:
                    self.speedx = self.speedx * -1
                if reverse_chance <=1:
                    self.speedy = self.speedy * -1
                bullet_chance = random.randrange(1,100)
                if bullet_chance >93:   # fire a bullet!
                    boss_bullet_sound.set_volume(0.5)
                    #boss_bullet_sound.play()
                    Big_boss.shoot()
                
        def shoot(self):  # create a new bullet
            Bnow = pygame.time.get_ticks()
            if Bnow - self.last_shot > self.shoot_delay:
                self.last_shot = Bnow
                limit_fire=random.randrange(1,100)
                if limit_fire<31:
                    boss_bullet_sound.play()
                    B_bullet1 = Boss_Bullet1(self.rect.centerx, self.rect.centery)   #shoot 1st Bullet
                    all_sprites.add(B_bullet1)
                    boss_bullet_list.add(B_bullet1)
                if limit_fire>30 and limit_fire<61:
                    boss_bullet_sound.play()
                    B_bullet2 = Boss_Bullet2(self.rect.centerx, self.rect.centery)   #shoot 2nd bullet
                    all_sprites.add(B_bullet2)
                    boss_bullet_list.add(B_bullet2)
                if limit_fire>60 and limit_fire<93:
                    boss_bullet_sound.play() 
                    B_bullet3 = Boss_Bullet3(self.rect.centerx, self.rect.centery)   #shoot 3rd bullet
                    all_sprites.add(B_bullet3)
                    boss_bullet_list.add(B_bullet3)
                if limit_fire>93:
                    boss_bullet_sound.play()
                    evil_laugh.play()
                    B_bullet3 = Boss_Bullet3(self.rect.centerx, self.rect.centery)   #shoot 3rd bullet
                    B_bullet1 = Boss_Bullet1(self.rect.centerx, self.rect.centery)   #shoot 1st Bullet
                    all_sprites.add(B_bullet1)
                    boss_bullet_list.add(B_bullet1) 
                    all_sprites.add(B_bullet3)
                    boss_bullet_list.add(B_bullet3)
                
    # a class function to draw the player and move, as well as start the shoot bullet process if the space bar is pressed

    class Player(pygame.sprite.Sprite):
        def __init__(self):
            pygame.sprite.Sprite.__init__(self)
            self.image = player_ship
            self.surf = pygame.Surface((140, 98))
            self.rect = self.image.get_rect(center =((WIDTH/2)-46, HEIGHT - 40))
            self.radius = 40   # trying to make a hit box of correct size for player need HELP with this
            #pygame.draw.circle(screen, RED, self.rect.center, self.radius)
            self.speedx = 0
            self.shield = 100
            self.shoot_delay = 250
            self.last_shot = pygame.time.get_ticks()
            self.lives = 3
            self.hidden = False
            self.power = 1
            self.fire_now = pygame.time.get_ticks()
            self.power_time = pygame.time.get_ticks()
            self.hide_timer = pygame.time.get_ticks()
            
        def move(self): # move the player left and right
            # unhide if hidden
            if self.hidden and pygame.time.get_ticks() - self.hide_timer > 1000:
                self.hidden = False
                self.rect.centerx = ((WIDTH /2)-46)
                self.rect.bottom = HEIGHT
            self.speedx = 0
            pressed_keys = pygame.key.get_pressed()
                
            if self.rect.left > 0:
                if pressed_keys[K_LEFT]:
                    self.speedx = -5
            if self.rect.right < WIDTH:        
                if pressed_keys[K_RIGHT]:
                    self.speedx = 5
            if pressed_keys[K_SPACE] and (pygame.time.get_ticks() - self.fire_now) >300:   # fire a bullet!
                self.fire_now = pygame.time.get_ticks()
                P1.shoot()      
            self.rect.x += self.speedx
            # timeout for powerups
            if self.power >= 2 and pygame.time.get_ticks() - self.power_time > POWERUP_TIME:
                self.power -= 1
                self.power_time = pygame.time.get_ticks()
                
        def zoom(self):
            x=0
            y=0
            Dspeed=zoom_speed
            for i in range(0, int((HEIGHT/Dspeed)/5)):
                for i in range(len(cols)):
                    color=cols[i]
                    pygame.draw.line(screen, (color), ((WIDTH/2),HEIGHT), ((WIDTH/2),HEIGHT-y), 32)
                    self.rect.x = int(WIDTH/2)-46
                    if self.rect.y>-70:
                        screen.blit(P1.image, P1.rect)
                        time.sleep(0.009)
                        self.rect.y -=8
                    pygame.display.flip()
                    bg.draw(screen)
                    time.sleep(0.009)
                    y=y+Dspeed
            x=0
            y=0
            Dspeed =+10
            for i in range(0, int(WIDTH/Dspeed/2)):
                pygame.draw.line(screen, (color), (512-x,HEIGHT), (512-x,0), 5)
                pygame.draw.line(screen, (color), (512+x,HEIGHT), (512+x,0), 5)
                pygame.display.flip()
                bg.draw(screen)
                time.sleep(0.009)
                x=x+Dspeed
            x=0
            y=0
            for i in range(0, int(WIDTH/Dspeed/2)):
                pygame.draw.line(screen, (color), (0+x,HEIGHT), (0+x,0), 5)
                pygame.draw.line(screen, (color), (WIDTH-x,HEIGHT), (WIDTH-x,0), 5)
                pygame.display.flip()
                bg.draw(screen)
                time.sleep(0.009)
                x=x+Dspeed
            self.rect.y = HEIGHT - 76   
            x=0
            y=0
            Dspeed=zoom_speed
            
        def shoot(self):
            now = pygame.time.get_ticks()
            if now - self.last_shot > self.shoot_delay:
                self.last_shot = now
                if self.power == 1:
                    bullet = Bullet(self.rect.centerx, self.rect.top+26)
                    all_sprites.add(bullet)
                    bullet_list.add(bullet)
                    bullet_sound.set_volume(0.7) 
                    bullet_sound.play()
                if self.power >= 2:
                    bullet_sound.set_volume(0.7) 
                    bullet_sound.play()
                    bullet1 = Bullet(self.rect.left+7, self.rect.centery+26)
                    bullet2 = Bullet(self.rect.right-7, self.rect.centery+26)
                    all_sprites.add(bullet1)
                    all_sprites.add(bullet2)
                    bullet_list.add(bullet1)
                    bullet_list.add(bullet2)
                    #sound.play()
                if self.power >= 3:
                    bullet_sound.set_volume(0.7) 
                    bullet_sound.play()
                    bullet1 = Bullet(self.rect.left+7, self.rect.centery+26)
                    bullet2 = Bullet(self.rect.right-7, self.rect.centery+26)
                    bullet3 = Bullet(self.rect.centerx, self.rect.top+26)
                    all_sprites.add(bullet1)
                    all_sprites.add(bullet2)
                    all_sprites.add(bullet3)
                    bullet_list.add(bullet1)
                    bullet_list.add(bullet2)
                    bullet_list.add(bullet3)
    
        def powerup(self):
            self.power += 1
            self.power_time = pygame.time.get_ticks()

        def hide(self):
            # hide the player temporarily
            self.hidden = True
            self.hide_timer = pygame.time.get_ticks()
            self.rect.center = (WIDTH +250, HEIGHT + 250)
            
    # a class function to draw the player SHIELD and move 
    class Shield(pygame.sprite.Sprite):
        def __init__(self, center):
            pygame.sprite.Sprite.__init__(self)
            self.image = player_shield
            self.rect = self.image.get_rect()
            self.rect.center = center
            self.surf = pygame.Surface((140, 68))
            self.radius = 40   # trying to make a hit box of correct size for player need HELP with this
            #pygame.draw.circle(screen, RED, self.rect.center, self.radius)
            self.speedx = 0
            self.rect.x = (P1.rect.x -10 )
        
        def move(self): # move the shield left and right
            self.rect.x = (P1.rect.x -10 )    # -------------  make sure the shield appears in the same position as the player's ship
            if shield_kill:
                all_sprites.remove(P1_shield)
                shield_sprites.remove(P1_shield)
                self.kill()

    class Explosion(pygame.sprite.Sprite):
        def __init__(self, center, size):
            pygame.sprite.Sprite.__init__(self)
            self.size = size
            self.image = explosion_anim[self.size][0]      # set first frame in animation sequence
            self.rect = self.image.get_rect()
            self.rect.center = center
            self.frame = 0
            self.last_update = pygame.time.get_ticks()
            self.frame_rate = 40

        def move(self):
            now = pygame.time.get_ticks()
            if now - self.last_update > self.frame_rate:
                self.last_update = now
                self.frame += 1                                    # goto the next frame
                if self.frame == len(explosion_anim[self.size]):
                    # if the animation has reached the last frame the remove sprite from all lists
                    shield_kill = True
                    if self.size == 'sm' or self.size == 'player' and len(shield_sprites)>0:
                        all_sprites.remove(P1_shield)
                        P1_shield.kill()
                    self.kill()
                else:
                    center = self.rect.center
                    self.image = explosion_anim[self.size][self.frame]
                    self.rect = self.image.get_rect()
                    self.rect.center = center
                    
    class Boss_explosion(pygame.sprite.Sprite):
        def __init__(self, center):
            pygame.sprite.Sprite.__init__(self)
            self.image = bexplosion[0]      # set first frame in animation sequence
            self.rect = self.image.get_rect()
            self.rect.center = center
            self.frame = 0
            self.last_update = pygame.time.get_ticks()
            self.frame_rate = 30
            

        def move(self):
            now = pygame.time.get_ticks()
            if now - self.last_update > self.frame_rate:
                self.last_update = now
                self.frame += 1                                    # goto the next frame
                time.sleep(0.005)
                if self.frame == len(bexplosion):
                    # if the animation has reached the last frame the remove sprite from all lists
                    Boss_died_anim_finished=True
                    
                    self.kill()
                    all_sprites.remove(Big_boss)
                    enemy_boss.remove(Big_boss)
                else:
                    center = self.rect.center
                    Boss_died_anim_finished=False
                    self.image = bexplosion[self.frame]
                    self.rect = self.image.get_rect()
                    self.rect.center = center       

    # -----------------------------------------------------------------------------------------------------------------------------
    # -------------------------------------------------------- END of Class Definitions -------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------------------

    def do_big_explosion():
        boss_explode_sound.set_volume(1)
        boss_explode_sound.play()
        boss_explode_inprogress=True
        Big_boss.life = 100
        XX = hit.rect.center[0]
        YY = hit.rect.center[1]
        YY=YY-60
        ZZZ= (XX,YY)
        ZZ = (XX,YY+192)
        XX=XX-192                            # multiple explosions in middle and below 6 explosions in all
        AA= (XX,YY)
        BB= (XX,YY+192)
        XX=XX+384
        CC=(XX,YY)
        DD=(XX,YY+192)
        Huge_expl1 = Boss_explosion(ZZZ)
        Huge_expl2 = Boss_explosion(ZZ)
        Huge_expl3 = Boss_explosion(AA)
        Huge_expl4 = Boss_explosion(BB)
        Huge_expl5 = Boss_explosion(CC)
        Huge_expl6 = Boss_explosion(DD)
        all_sprites.add(Huge_expl1)
        all_sprites.add(Huge_expl2)
        all_sprites.add(Huge_expl3)
        all_sprites.add(Huge_expl4)
        all_sprites.add(Huge_expl5)
        all_sprites.add(Huge_expl6)
        myexpl.add(Huge_expl1)
        myexpl.add(Huge_expl2)
        myexpl.add(Huge_expl3)
        myexpl.add(Huge_expl4)
        myexpl.add(Huge_expl5)
        myexpl.add(Huge_expl6)
        myexpl.draw(screen)
                    

    # A Function to draw text
    font_name = pygame.font.match_font('arial')
    def draw_text(surf, text, size, x, y, text_color):
        font = pygame.font.Font(font_name, size)
        text_surface = font.render(text, True, text_color)
        text_rect = text_surface.get_rect()
        text_rect.midtop = (x, y)
        surf.blit(text_surface, text_rect)

    # function to draw 2 backgrounds one above the visible area            
    #def DrawBackground(): 
    #    screen.blit(bg, (0,bgY))   # draws our first bg image
    #    screen.blit(bg, (0,bgY2))  # draws the seconf bg image

    # function to show the start screen with instructions
    def show_go_screen():
        #screen.blit(bg, bg_rect)
        pygame.mixer.music.load('Sound/Background_music.mp3')   # loads BACKGROUND MUSIC
        pygame.mixer.music.set_volume(music_volume)
        pygame.mixer.music.play(-1, 1000)   # Plays background music in continous loop
        bg.draw(screen)
        draw_text(screen, "Chris' Crazy Space Game!", 84, WIDTH / 2, HEIGHT * 0.1, WHITE)
        draw_text(screen, "Arrow keys move, space to fire", 48, WIDTH / 2, HEIGHT * 0.4, WHITE)
        draw_text(screen, "PRESS  S  KEY TO START", 32, WIDTH / 2, HEIGHT * 0.6, WHITE)
        draw_text(screen, "PRESS  Q  KEY TO QUIT", 28, WIDTH / 2, HEIGHT * 0.85, WHITE)
        pygame.display.update()
        waiting = True
        while waiting:
            clock.tick(FPS)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    main_loop=False
                    pygame.display.quit()
                    pygame.mixer.music.stop()
                    return True
                    # sys.exit()
                if event.type == pygame.KEYUP and event.key == pygame.K_q :
                    main_loop=False
                    pygame.display.quit()
                    pygame.mixer.music.stop()
                    return True
                #pygame.quit()
                    #sys.exit()
                if event.type == pygame.KEYUP and event.key == pygame.K_s :
                    waiting = False

        return False
    def new_game():
        game_over = False
        boss_explode_inprogress=False
        enemy_speedx = 1.2
        enemy_speedy = 1
        boss_speedx = 1
        boss_speedy = 1
        enemies = pygame.sprite.Group()      # create a group of sprites that include all the ememies
        boss_bullet_list = pygame.sprite.Group()
        bullet_list = pygame.sprite.Group()  # create a group of sprites for the bullets
        all_sprites = pygame.sprite.Group()  # create a group of sprites that include all the sprites from player, enemies and bullets
        enemy_boss = pygame.sprite.Group()
        shield_sprites = pygame.sprite.Group()
        powerups = pygame.sprite.Group()
        myexpl = pygame.sprite.Group()
        P1 = Player()
        all_sprites.add(P1)
        P1_shield = Shield(P1.rect.center)
        for i in range(8):   # create 8 new enemies
            new_enemy()
        score = 0            # reset score back to 0 
        shield = 100         # reset shields back to 100
        P1.lives = 3         # set lives back to 3
        direction = 1        # set scroll direction back to normal
        scroll_speed = 2     # set scroll speed back to normal
        speed = scroll_speed * direction
        Boss_level_triggered=False
        musical_volume=0.5

    def get_ready():
        #screen.blit(bg, bg_rect)
        chris_getready.play()
        bg.draw(screen)
        pygame.display.update()
        draw_text(screen, "Get Ready", 84, WIDTH / 2, HEIGHT * 0.4, WHITE)
        waiting = True   
        pygame.display.update()
        time.sleep(1.5)
        
    def super_zoom():
        #screen.blit(bg, bg_rect)
        bg.draw(screen)
        pygame.display.update()
        P1.zoom()
        #bg.scroll(speed, orientation)
        #t = pygame.time.get_ticks()
        bg.draw(screen)
        pygame.display.update()
        time.sleep(0.5)    
        
    def show_boss_level():
        #screen.blit(bg, bg_rect)
        bg.draw(screen)
        draw_text(screen, "BOSS LEVEL", 84, WIDTH / 2, HEIGHT * 0.3, WHITE)
        draw_text(screen, "Get Ready", 48, WIDTH / 2, HEIGHT * 0.6, WHITE)
        waiting = True   
        pygame.display.update()
        pygame.mixer.music.load('Sound/Boss_music2.mp3')
        pygame.mixer.music.set_volume(music_volume)
        pygame.mixer.music.play(-1, 1000)
        time.sleep(2)
        
    # function to draw the shield bar at the top middle of the screen
    def draw_boss_bar(surf, x, y, pct):
        if pct < 0:
            pct = 0
        BAR_LENGTH = 100
        BAR_HEIGHT = 15
        fill = (pct / 100) * BAR_LENGTH
        outline_rect = pygame.Rect(x, y, BAR_LENGTH, BAR_HEIGHT)
        fill_rect = pygame.Rect(x, y, fill, BAR_HEIGHT)
        pygame.draw.rect(surf, RED, fill_rect)
        pygame.draw.rect(surf, WHITE, outline_rect, 2)

    # function to draw the shield bar at the top middle of the screen
    def draw_shield_bar(surf, x, y, pct):
        if pct < 0:
            pct = 0
        BAR_LENGTH = 100
        BAR_HEIGHT = 15
        fill = (pct / 100) * BAR_LENGTH
        outline_rect = pygame.Rect(x, y, BAR_LENGTH, BAR_HEIGHT)
        fill_rect = pygame.Rect(x, y, fill, BAR_HEIGHT)
        pygame.draw.rect(surf, GREEN, fill_rect)
        pygame.draw.rect(surf, WHITE, outline_rect, 2)
        
    # function to draw the player lives on the right of the screen
    def draw_lives(surf, x, y, lives, img):
        for i in range(lives-1):
            img_rect = img.get_rect()
            img_rect.x = x + 30 * i
            img_rect.y = y
            surf.blit(img, img_rect)

    # Setting up Sprites creating the player ship        
    P1 = Player()  # creates a player ship

    # Creating Sprites Groups
    enemies = pygame.sprite.Group() # create a group of sprites that include all the ememies
    boss_bullet_list = pygame.sprite.Group()
    bullet_list = pygame.sprite.Group()  # create a group of sprites for the bullets
    all_sprites = pygame.sprite.Group()  # create a group of sprites that include all the sprites from player, enemies and bullets
    enemy_boss = pygame.sprite.Group()
    shield_sprites = pygame.sprite.Group()
    powerups = pygame.sprite.Group()
    myexpl = pygame.sprite.Group()

    all_sprites.add(P1) # adds players ship to all sprites group
    P1_shield = Shield(P1.rect.center) # spawns shield so we don't have definition problems later.

    # a function to create a new ememy sprite    
    def new_enemy():
        e = Enemy()
        all_sprites.add(e)
        enemies.add(e)
            
    for i in range(8):          # create 8 new enemies and place them in the sprite groups
        new_enemy()

    # Adding a new User event 
    INC_SPEED = pygame.USEREVENT + 1 # creates new EVENT called INC_SPEED and gives it a new ID 
    pygame.time.set_timer(INC_SPEED, 2000)    # creates the pygame event called INC_SPEED every 2000 miliseconds this will allow us to slowly increase the speed of enemies



    #  ---------------------------------------------------------------------------------------------------------------------------

    # Main Game Loop
    should_exit = show_go_screen()
    if should_exit:
        return
    
    
    get_ready()
    game_over = False        
    main_loop = True
    shield=100

    while main_loop:
        
        if game_over:
            should_exit = show_go_screen()
            if should_exit:
                return
            
            game_over = False
            boss_explode_inprogress=False
            all_sprites = pygame.sprite.Group()
            enemies = pygame.sprite.Group()
            bullet_list = pygame.sprite.Group()
            powerups = pygame.sprite.Group()
            enemy_speedx = 1.2
            enemy_speedy = 1
            boss_speedx = 1
            boss_speedy = 1
            P1 = Player()
            all_sprites.add(P1)
            for i in range(8):   # create 8 new enemies
                new_enemy()
            score = 0        # reset score back to 0 
            shield = 100     # reset shields back to 100
            P1.lives = 3     # set lives back to 3
            direction = 1    # set scroll direction back to normal 
            scroll_speed = 2
            speed = scroll_speed * direction   # Set scroll speed back to normal
            Boss_level_triggered=False
            musical_volume=0.5
            get_ready()
        bg.scroll(speed, orientation)
        t = pygame.time.get_ticks()
        bg.draw(screen)
        #DrawBackground() # makes my moving background scroll he he he Tricky Tricky!
        # Cycles through all events that are occuring  
        for event in pygame.event.get():
            if event.type == INC_SPEED:
                enemy_speedx += 0.4
                enemy_speedy +=0.4
            elif event.type == pygame.QUIT:  # quit if x in window is clicked
                main_loop=False
                pygame.display.close()
                return
                break
                # screen = pygame.display.set_mode((0, 0), FULLSCREEN)
                # sys.exit()
                #pygame.quit()
                #sys.exit()
        #if not boss_explode_inprogress:      # STOP scrolling if BOSS DIES 
        # bgY += direction * scroll_speed  # Move both background images down
        #  bgY2 += direction * scroll_speed # Move both background images down
        #  if direction >0:
            #   if bgY > bg.get_height() * 1:  # If our 1st background has moved to the bottom of the screen then reset its position back to zero
            #       bgY = 0
            #   if bgY2 > 0:
            #       bgY2 = bg.get_height() * -1 # If our 2nd background has moved to the top of the screen then reset its position back to 1 screen above the visable area
            #  if direction <0:
            #    if bgY < 0:  
            #      bgY = bg.get_height() * 1
            # if bgY2 < bg.get_height() *-1:
            #    bgY2 = 0
    #                                                check the Shield powerup AND Bolt powerup for a colission
    # ----------------------------------------------------------------------------------------------------------------------------------------------------------------

        hits = pygame.sprite.spritecollide(P1, powerups, True)
        for hit in hits:
            if hit.type == 'shield':
                powerup_shields.play()
                P1.shield += random.randrange(10, 30)
                if P1.shield >= 100:
                    P1.shield = 100
            if hit.type == 'power':
                bolt_powerup.play()
                P1.powerup()
    #                                                Check for Boss bullet and Player collisions
    # ---------------------------------------------------------------------------------------------------------------------------------------------------------------   
        if len(enemy_boss) >0:
            # Calculate mechanics for each bossbullet
            boss_hit_list = pygame.sprite.spritecollide(P1, boss_bullet_list, True)   # collision detection FOR BOSS BULLETS and PLAYER SHIP
            if len(boss_hit_list)>0 and len(shield_sprites)<=0:
                    P1_shield = Shield(P1.rect.center)
                    all_sprites.add(P1_shield)
                    shield_sprites.add(P1_shield)
            for bullet_hit in boss_hit_list:
                player_hit.play()
                P1.shield -= 10
                
                expl = Explosion(bullet_hit.rect.center, 'sm')
                all_sprites.add(expl)
                if P1.shield <= 0: # if player has no shield left then play explosion sound & animation, subract 1 from lives and reset shield to 100
                    crash_sound.play()
                    death_explosion = Explosion(P1.rect.center, 'player')
                    all_sprites.add(death_explosion)
                    P1.hide()
                    P1.lives -=1
                    P1.shield = 100
    # ---------------------------------------------------------------------------------------------------------------------------------------------------------------------  
    # -------------------------------------------- Test for collision between PLAYER BULLET and BOSS MONSTER! -------------------------------------------------------------
    # ---------------------------------------------------------------------------------------------------------------------------------------------------------------------           
    # Calculate mechanics for each bullet
        if len(enemy_boss) >0:   # ONLY CHECK IF BOSS MONSTER EXISTS
            for bullet in bullet_list:
                # See if the bullet has hit BOSS MONSTER
                enemy_hit_list = pygame.sprite.spritecollide(bullet, enemy_boss, False, pygame.sprite.collide_circle )   # collision detection using the circle hit box method
                # For each Player hit, remove the bullet and add to the score
                for hit in enemy_hit_list:
                    boss_hit_sound.play()
                    bullet_list.remove(bullet)
                    all_sprites.remove(bullet)
                    tempx = bullet.rect.x
                    tempy = hit.rect.y+126
                    tempxy = tempx, tempy
                    expl = Explosion(tempxy, 'sm')
                    all_sprites.add(expl)
                    score += 5
                    Big_boss.life -= 1
                    if Big_boss.life<5:
                        pygame.mixer.music.fadeout(1400)
                    
                    if Big_boss.life<=0 and Boss_died_anim_finished==False and boss_explode_inprogress==False and len(enemy_boss)>0:
                        do_big_explosion()
                        explosion_time = pygame.time.get_ticks()
                        
        
        if len(enemy_boss)<=0 and Boss_level_triggered==True and pygame.time.get_ticks() - explosion_time >2500:
            owwyeah.play()
            time.sleep(0.9)
            Boss_died_anim_finished=False
            pygame.display.flip()
            game_over=True
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------   
    # -------------------------------------------------------- Test for collision between PLAYER bullet and enemies! ---------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------            
        # Calculate mechanics for each bullet
        for bullet in bullet_list:
            # See if the bullet has hit an enemy
            enemy_hit_list = pygame.sprite.spritecollide(bullet, enemies, True, pygame.sprite.collide_circle )   # collision detection using the circle hit box method
            # For each enemy hit, remove the bullet and add to the score
            for hit in enemy_hit_list:
                enemy_hit.play()
                bullet_list.remove(bullet)
                all_sprites.remove(bullet)
                expl = Explosion(hit.rect.center, 'lg')
                all_sprites.add(expl)
                score += 10
                if random.random() > 0.85:
                    Pups = Powerups(hit.rect.center)
                    all_sprites.add(Pups)
                    powerups.add(Pups)
                if score<200:
                    new_enemy()
    
            # Remove the bullet if it flies up off the screen
            if bullet.rect.y < -10:
                bullet_list.remove(bullet)
                all_sprites.remove(bullet)
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # -------------- Refreshes display and MOVES all Sprites  calls move function in all sprite classes----- VERY IMPORTANT -------------------------------------------- 
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------
        for entity in all_sprites:
            screen.blit(entity.image, entity.rect)
            entity.move()
        if score >200:
            number = len(enemies)  # the number of enemies left
            if number == 0 and len(enemy_boss)==0 and Boss_level_triggered==False:     #     if there are 0 enemies left and the score is more than 150 then CHANGE background scroll direction, speed create BOSS sprite, add to sprite groups!
                super_zoom()
                Boss_level_triggered=True
                direction = -1
                scroll_speed = 6
                speed = scroll_speed * direction
                musical_volume = 0.9 # turn up music for Boss Monster level
                show_boss_level()
                Big_boss = Boss(entity.rect.center,'boss')
                all_sprites.add(Big_boss)
                enemy_boss.add(Big_boss)
                enemy_boss.draw(screen)
    # ---------------------------------------------------------------------------------------------------------------------------------------- 
    # To be run if collision occurs between PLAYER and Enemy
    # ---------------------------------------------------------------------------------------------------------------------------------------- 
        hits = pygame.sprite.spritecollide(P1, enemies, True, pygame.sprite.collide_circle)
        if len(hits)>0 and len(shield_sprites)<=0:
            P1_shield = Shield(P1.rect.center)
            all_sprites.add(P1_shield)
            shield_sprites.add(P1_shield)
        for hit in hits:
            player_hit.play()
            P1.shield -= 20
            expl = Explosion(hit.rect.center, 'sm')
            all_sprites.add(expl)
            new_enemy()
        if P1.shield <= 0: # if player has no shield left then play explosion sound & animation, subract 1 from lives and reset shield to 100
            all_sprites.remove(P1_shield)
            shield_sprites.remove(P1_shield)
            P1_shield.kill()
            shield_kill=False
            crash_sound.play()
            death_explosion = Explosion(P1.rect.center, 'player')
            all_sprites.add(death_explosion)
            P1.hide()
            P1.lives -=1
            P1.shield = 100
    # ---------------------------------------------------------------------------------------------------------------------------------------
    # If the player has no more lives left and the explosion animation has finished playing then GAME OVER   
    # ---------------------------------------------------------------------------------------------------------------------------------------           
        if P1.lives == 0 and not death_explosion.alive():
            all_sprites.remove(P1_shield)
            shield_sprites.remove(P1_shield) 
            P1_shield.kill()
            draw_text(screen, "GAME OVER", 94, WIDTH / 2, (HEIGHT / 2)-70, RED)
            pygame.display.update()
            game_over_message.play()
            time.sleep(3)
            enemy_speedx=1
            enemy_speedy=1
            if len(enemy_boss)>0:
                enemy_boss.remove(Big_boss)
                all_sprites.remove(Big_boss)
            all_sprites.remove(P1)
            P1.kill()
            game_over=True
    # ---------------------------------------------------------------------------------------------------------------------------------------       
        draw_shield_bar(screen, WIDTH / 1.3, 7, P1.shield)
        draw_text(screen, 'SCORE ' +str(score), 22, WIDTH / 11, 3, WHITE) # display score on screen on far left
        draw_lives(screen, WIDTH - 100, 7, P1.lives, player_mini_img)
        if len(enemy_boss) >0:   # ONLY DRAW BOSS BAR IF BOSS MONSTER EXISTS
            draw_text(screen, 'BOSS HEATH', 20, WIDTH / 2.4, 3, WHITE) 
            draw_boss_bar(screen, WIDTH / 2.1, 7, Big_boss.life)    
        pygame.display.flip()
        FramePerSec.tick(FPS)
    # pygame.quit()
    pygame.display.quit()

