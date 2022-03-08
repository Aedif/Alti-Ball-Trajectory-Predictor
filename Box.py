

@dataclass
class Box():
    x: int          # pos
    y: int          # pos
    h: int          # its height, and I will assume that its depth is always equal to its height
    w: int          # its width
    r: float = 0.1  # its rotation AKA orientation or direction in radians
    dx: int = 0     # delta x  in pixels per frame 1/60th second
    dy: int = 0     # delta y
    dr: float = 0.0 # deltat rotation in radians  per frame 1/60th second
    mass: float = 0 


    def getDesc(self):
        vel = math.hypot(self.dx ,self.dy)
        radius = math.hypot(self.w,self.h)/2
        rVel = math.fabs(self.dr * radius)
        desc = "V " + (vel*60).toFixed(0) + "pps "
        desc += math.fabs(self.dr * 60 * 60).toFixed(0) + "rpm "
        desc += "Va " + (rVel*60).toFixed(0) + "pps "
        return desc

    def calcMass(self):
        # mass in K things
        return (self.w * self.h * self.h)/1000

    def draw(self, screen):
        pygame.draw.circle(screen, (255,0,0), (int(self.x), int(self.y)), 5, 0)

        rect = pygame.Rect(0, 0, self.w, self.h)
        rect.center = (self.x, self.y)
        r_points = []
        r_points.append(rotate((rect.x, rect.y), rect.center, self.r))
        r_points.append(rotate((rect.x+self.w, rect.y), rect.center, self.r))
        r_points.append(rotate((rect.x+self.w, rect.y+self.h), rect.center, self.r))
        r_points.append(rotate((rect.x, rect.y+self.h), rect.center, self.r))
        pygame.draw.lines(screen, (0,0,0), True, r_points)

    def update(self):
        self.x += self.dx
        self.y += self.dy
        self.dy += GRAVITY # alittle gravity
        self.r += self.dr

    def getPoint(self, which:int):
        dx = math.cos(self.r)
        dy = math.sin(self.r)
        if which is 0:
            x = -self.w /2
            y = -self.h /2
        elif which is 1:
            x = self.w /2
            y = -self.h /2
        elif which is 2:
            x = self.w /2
            y = self.h /2
        elif which is 3:
            x = -self.w /2
            y = self.h /2
        elif which is 4:
            x = self.x
            y = self.y

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
        # addTempVec(loc, force,"White", LIFE, SCALE_FORCE) # show the force
        l = asCart(loc) # make sure the location is in cartesian form
        toCenter = asPolar(vector(self.x - l['x'], self.y - l['y']))
        if print_debug:
            print(f'toCenter = {toCenter}')
            print(f'toCenter["dir"] = {toCenter["dir"]}')
            print(f'force["dir"] = {force["dir"]}')
        pheta = toCenter['dir'] - force['dir']
        if print_debug:
            print(f'pheta = {math.degrees(pheta)}')
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