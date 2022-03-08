from dataclasses import dataclass
import math
import pygame
from PIL import Image

from HelperFunctions import (map_to_mask,validatePolar,polar,vector,asPolar,asCart,doCollision,collide_map)

PI90 = math.pi / 2
PI = math.pi
PI2 = math.pi * 2

INSET = 5 # playfeild inset

GRAVITY = 0.029
WALL_FRICTION = 0
ELASTICITY = 3

WORKING_DIR = 'C:\\Users\\Aedif\\git-projects\\alti_trajectory_predict\\'
BALL_IMG_PATH = WORKING_DIR + 'ball.png'
MAP_PATH = WORKING_DIR + 'ball_map.png'
MAP_MASK_PATH = WORKING_DIR + 'ball_map_mask.png'
TEST_MAP_MASK = WORKING_DIR + 'test_map.png'

LOOPY_EJECT_FORCE = 63

EJECT_FORCE = LOOPY_EJECT_FORCE

class Background(pygame.sprite.Sprite):
    def __init__(self, image_file, location):
        pygame.sprite.Sprite.__init__(self)  #call Sprite initializer
        self.image = pygame.image.load(image_file)
        self.rect = self.image.get_rect()
        self.rect.left, self.rect.top = location

    def getSize(self):
        return self.rect.w, self.rect.h

@dataclass
class Ball():
    x: int          # pos
    y: int          # pos
    radius: int     # radius
    img_path: str   # path to ball image
    r: float = 0.1  # its rotation AKA orientation or direction in radians
    dx: int = 0     # delta x  in pixels per frame 1/60th second
    dy: int = 0     # delta y
    dr: float = 0.0 # deltat rotation in radians  per frame 1/60th second
    mass: float = 10

    def __post_init__(self):
        self.img = pygame.image.load(self.img_path)

    def draw(self, surface):
        # draw image
        b_sur = pygame.transform.rotate(self.img, self.r)
        b_sur_rect = b_sur.get_rect()
        b_sur_rect.center = (self.x, self.y)
        surface.blit(b_sur, b_sur_rect)

        #
        pygame.draw.circle(surface, (0,0,255), (int(self.x), int(self.y)), 1, 0)
        pygame.draw.circle(surface, (255,0,255), (int(self.x), int(self.y)), self.radius, 1)

    def update(self):
        self.x += self.dx
        self.y += self.dy
        self.dy += GRAVITY # alittle gravity
        #self.r += self.dr

    def getPoint(self):
        dx = math.cos(self.r)
        dy = math.sin(self.r)
        x = 0
        y = 0
        xx = x * dx + y * -dy
        yy = x * dy + y * dx
        details = asPolar(vector(xx, yy))
        xx += self.x
        yy += self.y
        velocityA = polar(details['mag'] * self.dr, details['dir'] + PI90)
        velocity = vector(self.dx, self.dy)
        return {
            'velocity' : velocity,  # only directional
            'velocityA' :  velocityA, # angular only
            'pos' : vector(xx, yy),
            'radius' : details['mag'],
        }
    
    def applyForce(self, force, loc, print_debug=False): # force is a vector, loc is a coordinate
        validatePolar(force) # make sure the force is a valid polar
        l = asCart(loc) # make sure the location is in cartesian form

        toCenter = asPolar(vector(self.x - l['x'], self.y - l['y']))
        pheta = toCenter['dir'] - force['dir']
        
        Fv = math.cos(pheta) * force['mag']
        Fa = math.sin(pheta) * force['mag']
        accel = asPolar(toCenter) # copy the direction to center
        accel['mag'] = Fv / self.mass # now use F = m * a in the form a = F/m
        deltaV = asCart(accel) # convert it to cartesian 
        self.dx += deltaV['x'] # update the box delta V
        self.dy += deltaV['y']
        accelA = Fa / (toCenter['mag']  * self.mass) # for the angular component get the rotation
                                                    # acceleration
        self.dr += accelA # now add that to the box delta r

# initialise pygame
pygame.init()

# generate collision mask for the default map
mask_pixels = map_to_mask(MAP_PATH, MAP_MASK_PATH)

# define ball/balls to be placed on the map
ball = Ball(200, 200, 13, BALL_IMG_PATH)

# load background and set it's height and width as bounds for redering
background = Background(MAP_MASK_PATH, [0,0])
world_width, world_height = background.getSize()

# overwrite screen dimensions
screen_height = 700
screen_width = 700

screen = pygame.display.set_mode((screen_width,screen_height))
#screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

infoObject = pygame.display.Info()

background.image.convert_alpha()
pygame.display.set_caption('Trajectory prediction')

background_colour = (255,255,255)
screen.fill(background_colour) # clear screen

def update():
    ball.update()

    p = ball.getPoint()
    # only do one collision per frame or we will end up adding energy
    if p['pos']['x']  < INSET + ball.radius:
        ball.x += (INSET + ball.radius) - p['pos']['x']
        doCollision(ball,p,3)
    elif p['pos']['x'] > world_width-INSET-ball.radius:
        ball.x += (world_width-INSET-ball.radius) - p['pos']['x']
        doCollision(ball,p,1)
    elif p['pos']['y'] < INSET + ball.radius:
        ball.y += (INSET + ball.radius) -p['pos']['y']
        doCollision(ball,p,0)
    elif p['pos']['y'] > world_height-INSET-ball.radius:
        ball.y += (world_height-INSET-ball.radius) -p['pos']['y']
        doCollision(ball,p,2)

running = True
clock = pygame.time.Clock()
lx = 0
ly = 0
pressed = False

update_enabled = False


ball.x = 241
ball.y = 890
start_time = None

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONUP:
            start_time = pygame.time.get_ticks()

            # uncomment for determining ball position
            # (mouseX, mouseY) = pygame.mouse.get_pos()
            # mouseX += ball.x - screen_width/2
            # mouseY += ball.y - screen_height/2
            # ball.x = mouseX
            # ball.y = mouseY
            # print((mouseX,mouseY))

            # mouseX = ball.x
            # mouseY = ball.y - 500
            # force = asPolar(vector(mouseX - ball.x, mouseY-ball.y))
            # force['mag'] = EJECT_FORCE
            # ball.applyForce(force,vector(mouseX, mouseY), True)

            # uncomment for mouse force apply
            (mouseX, mouseY) = pygame.mouse.get_pos()
            mouseX += ball.x - screen_width/2
            mouseY += ball.y - screen_height/2
            force = asPolar(vector(mouseX - ball.x, mouseY-ball.y))
            force['mag'] = EJECT_FORCE
            ball.applyForce(force,vector(mouseX, mouseY), True)

    drawSurface = background.image.copy()

    # if start_time:
    #     update()
    update()
    ball.draw(drawSurface)

    colliding, c_x, c_y, col_img = collide_map(ball, mask_pixels, world_width, world_height)
    if col_img:
        py_img = pygame.image.fromstring(col_img.tobytes(), col_img.size, col_img.mode)
        drawSurface.blit(py_img, (0,0))
        pygame.draw.circle(drawSurface, (0,255,0), (int(c_x-ball.x+ball.radius), int(c_y-ball.y+ball.radius)), 5, 0)
    if colliding:
        # if start_time:
        #     print("Time: " + str((pygame.time.get_ticks() - start_time)/1000))
        #     start_time = None
        rad = math.atan2(c_y-ball.y, c_x-ball.x)
        p = ball.getPoint()

        #===
        v1 = pygame.math.Vector2(c_x - p['pos']['x'], c_y - p['pos']['y'])
        v2 = pygame.math.Vector2(c_x - p['pos']['x'], c_y - p['pos']['y'])
        v2.scale_to_length(ball.radius)

        diff_x = v2.x - v1.x
        diff_y = v2.y - v1.y
        #=====
        ball.x -= diff_x
        ball.y -= diff_y
        doCollision(ball, p, -1, rad)

    d_rect = drawSurface.get_rect()

    screen.blit(drawSurface, (-ball.x+screen_width/2, -ball.y+screen_height/2))

    pygame.display.update()
    clock.tick(60)


