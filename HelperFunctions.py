from PIL import Image
import math

PI90 = math.pi / 2
PI = math.pi
PI2 = math.pi * 2

WALL_NORMS = [PI90,PI,-PI90,0] # direction of the wall normals

GRAVITY = 0.005
WALL_FRICTION = 0.18
ELASTICITY = 1.3

def map_to_mask(map_img_path: str, map_mask_save_path:str):
    ''' Given an altitude map editor screenshot export of the map with
    collision geometry set to filled in; converts that map into a black
    and white mask, white representing collidable geometry.  '''
    img = Image.open(map_img_path)
    width, height = img.size
    pixels = img.load()

    for x in range(width):
        for y in range(height):
            r,g,b,a = pixels[x,y]
            if r == 255 and b != 255 and g != 255:
                pixels[x,y] = (255, 255, 255)
            else:
                pixels[x,y] = (0, 0 ,0)
    
    img.save(map_mask_save_path, 'PNG')
    return pixels

def collide_map(ball, map_pixels, width, height):
    ''' Returns an image showing pixels within the ball radius. If any of the
    pixels are white thhe ball is considered as colliding and the the average 
    position of the white pixels is caluclated and returned as x,y coordinates.'''
    
    ball_collide_view_image = None
    total_x = 0
    total_y = 0
    total_found = 0
    pixels_list = []
    for y in range(int(ball.y - ball.radius),int(ball.y + ball.radius)):
        for x in range(int(ball.x - ball.radius), int(ball.x + ball.radius)):
            if x >= 0 and x < width and y >= 0 and y < height:
                dx = x - ball.x
                dy = y - ball.y
                if dx*dx + dy*dy < ball.radius*ball.radius:
                    pixels_list.append(map_pixels[x,y])
                    r,g,b,a = map_pixels[x,y]
                    if (r,g,b) == (255,255,255):
                        total_x += x
                        total_y += y
                        total_found += 1
                else:
                    pixels_list.append((255,25,25,255))
    if len(pixels_list) == ball.radius*2*ball.radius*2:
        ball_collide_view_image = Image.new("RGB",(ball.radius*2,ball.radius*2))
        ball_collide_view_image.putdata(pixels_list)

    colliding = False
    dx = 0
    dy = 0
    if total_found > 0:
        dx = (total_x / total_found)
        dy = (total_y / total_found)
        colliding = True

    return (colliding, dx, dy, ball_collide_view_image)

# calculations can result in a negative magnitude though this is valide for some
# calculations this results in the incorrect vector (reversed)
# this simply validates that the polat vector has a positive magnitude
# it does not change the vector just the sign and direction
def validatePolar(vec):
    if isPolar(vec):
        if vec['mag'] < 0:
            vec['mag'] = - vec['mag']
            vec['dir'] += PI
    return vec

# converts a vector from polar to cartesian returning a new one
def polarToCart(pVec):
    retV = {'x' : 0, 'y' : 0}
    retV['x'] = math.cos(pVec['dir']) * pVec['mag']
    retV['y'] = math.sin(pVec['dir']) * pVec['mag']
    return retV

# converts a vector from cartesian to polar returning a new one
def cartToPolar(vec):
    retV  = {'dir' : 0, 'mag' : 0}
    retV['dir'] = math.atan2(vec['y'],vec['x'])
    retV['mag'] = math.hypot(vec['x'],vec['y'])
    return retV

def polar (mag = 1, dir_ = 0):
    return validatePolar({'dir' : dir_, 'mag' : mag}) # create a polar vector
def vector (x= 1, y= 0):
    return {'x': x, 'y': y} # create a cartesian vector
def isPolar (vec):
    # returns true if polar
    return 'mag' in vec and 'dir' in vec
def isCart (vec):
    # returns true if cartesian 
    return 'x' in vec and 'y' in vec

# copy and converts an unknown vec to polar if not already
def asPolar(vec):
    if isCart(vec):
        return cartToPolar(vec)
    if vec['mag'] < 0:
        vec['mag'] = - vec['mag']
        vec['dir'] += PI
    return { 'dir' : vec['dir'], 'mag' : vec['mag'] }

# copy and converts an unknown vec to cart if not already
def asCart(vec):
    if isPolar(vec):
        return polarToCart(vec)
    return { 'x' : vec['x'], 'y' : vec['y']}

# This splits the vector (polar or cartesian) into the components along  dir and the tangent to that dir
def vectorComponentsForDir(vec,dir_):
    v = asPolar(vec) # as polar
    pheta = v['dir'] - dir_
    Fv = math.cos(pheta) * v['mag']
    Fa = math.sin(pheta) * v['mag']

    d1 = dir_
    d2 = dir_ + PI90 
    if Fv < 0:
        d1 += PI
        Fv = -Fv

    if Fa < 0:
        d2 += PI
        Fa = -Fa

    return {
        'along' : polar(Fv,d1),
        'tangent' : polar(Fa,d2)
    }

def doCollision(obj, pointDetails, wallIndex, rad=None):
    vv = asPolar(pointDetails['velocity']) # Cartesian V make sure the velocity is in cartesian form
    va = asPolar(pointDetails['velocityA']) # Angular V make sure the velocity is in cartesian form
    vvc = vectorComponentsForDir(vv, WALL_NORMS[wallIndex])            
    vac = vectorComponentsForDir(va, WALL_NORMS[wallIndex])
    if rad:
        vvc = vectorComponentsForDir(vv, rad)            
        vac = vectorComponentsForDir(va, rad)             
    vvc['along']['mag'] *= ELASTICITY # Elastic collision requiers that the two equal forces from the wall
    vac['along']['mag'] *= ELASTICITY # against the box and the box against the wall be summed. 
                          # As the wall can not move the result is that the force is twice 
                          # the force the box applies to the wall (Yes and currently force is in 
                          # velocity form untill the next line)
    vvc['along']['mag'] *= obj.mass # convert to force
    #vac.along.mag/= pointDetails.radius
    vac['along']['mag'] *= obj.mass
    vvc['along']['dir'] += PI # force is in the oppisite direction so turn it 180
    vac['along']['dir'] += PI # force is in the oppisite direction so turn it 180
    # split the force into components based on the wall normal. One along the norm the 
    # other along the wall


    vvc['tangent']['mag'] *= WALL_FRICTION  # add friction along the wall 
    vac['tangent']['mag'] *= WALL_FRICTION
    vvc['tangent']['mag'] *= obj.mass  #
    vac['tangent']['mag'] *= obj.mass
    vvc['tangent']['dir'] += PI # force is in the oppisite direction so turn it 180
    vac['tangent']['dir'] += PI # force is in the oppisite direction so turn it 180

    # apply the force out from the wall
    obj.applyForce(vvc['along'], pointDetails['pos'])    
    # apply the force along the wall
    obj.applyForce(vvc['tangent'], pointDetails['pos'])    
    # apply the force out from the wall
    obj.applyForce(vac['along'], pointDetails['pos'])    
    # apply the force along the wall
    obj.applyForce(vac['tangent'], pointDetails['pos'])    


def rotate(p1,p2,theta): #rotate p1 around p2 by theta (rad)
    x, y = p1
    xo, yo = p2
    xr=math.cos(theta)*(x-xo)-math.sin(theta)*(y-yo)   + xo
    yr=math.sin(theta)*(x-xo)+math.cos(theta)*(y-yo)  + yo
    return (xr,yr)