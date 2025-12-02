import random
import math
from ..utils.types import Point

class Map:
    def __init__(self, config):
        self.cfg = config['map']
        self.width = self.cfg['width']
        self.height = self.cfg['height']
        self.start = Point(0,0)
        self.goal = Point(0,0)
        self.obstacles = [] # [('c',x,y,r)|('r',x,y,w,h)]

    def generate_obstacles(self):
        self.obstacles = []
        num = random.randint(self.cfg['obs_num_min'], self.cfg['obs_num_max'])
        for _ in range(num):
            if random.random() < self.cfg['rect_ratio']:
                w = random.uniform(self.cfg['rect_w_min'], self.cfg['rect_w_max'])
                h = random.uniform(self.cfg['rect_h_min'], self.cfg['rect_h_max'])
                x = random.uniform(w/2, self.width - w/2)
                y = random.uniform(h/2, self.height - h/2)
                self.obstacles.append(('r', x, y, w, h))
            else:
                r = random.uniform(self.cfg['circle_r_min'], self.cfg['circle_r_max'])
                x = random.uniform(r, self.width - r)
                y = random.uniform(r, self.height - r)
                self.obstacles.append(('c', x, y, r))
        self.random_start_goal()
        return self.obstacles

    def check_collision(self, x, y, r_safe=0.0):
        for o in self.obstacles:
            if o[0]=='c':
                _,ox,oy,r=o
                if (x-ox)**2+(y-oy)**2<=(r+r_safe)**2: return True
            else:
                _,ox,oy,w,h=o; dx=abs(x-ox)-w/2; dy=abs(y-oy)-h/2
                if dx<=r_safe and dy<=r_safe and (dx<=0 and dy<=0 or (max(dx,0)**2+max(dy,0)**2<=r_safe**2)): return True
        return False

    def near_obstacle(self, x, y, d):
        for o in self.obstacles:
            if o[0]=='c':
                _,ox,oy,r=o
                if (x-ox)**2+(y-oy)**2<=(r+d)**2: return True
            else:
                _,ox,oy,w,h=o; dx=max(abs(x-ox)-w/2,0); dy=max(abs(y-oy)-h/2,0)
                if math.hypot(dx,dy)<=d: return True
        return False

    def random_start_goal(self):
        if not self.cfg.get('random_start_goal'): return
        dmin=self.cfg['min_start_goal_dist']
        while True:
            sx,sy=random.uniform(0,self.width),random.uniform(0,self.height)
            gx,gy=random.uniform(0,self.width),random.uniform(0,self.height)
            if math.hypot(sx-gx,sy-gy)>=dmin and not self.check_collision(sx,sy) and not self.check_collision(gx,gy):
                self.start=Point(sx,sy); self.goal=Point(gx,gy); break
