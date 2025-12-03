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
        self.start_yaw = 0.0 # 初始朝向，仅在 road 模式下生效
        self.obstacles = [] # [('c',x,y,r)|('r',x,y,w,h)]

    def generate_obstacles(self):
        self.obstacles = []
        t=self.cfg.get('type','normal')
        if t=='road': self._generate_road()
        elif t=='maze': self._generate_maze()
        else:
            self._generate_normal()
        return self.obstacles

    def _generate_normal(self):
        if not self.cfg.get('random_obstacles', False):
            self.random_start_goal()
            return

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

    def _generate_road(self):
        cx=self.width/2; cy=self.height/2; self.start=Point(cx,cy); self.goal=Point(cx,cy); self.ref_path=[]; r=self.cfg['road']; A=r.get('eight_A',self.width/4); B=r.get('eight_B',self.height/6); L=r.get('eight_loops',3); N=r.get('eight_points',600)
        for i in range(N):
            t=i/N*L*2*math.pi
            x=cx+A*math.sin(t); y=cy+B*math.sin(2*t)
            self.ref_path.append(Point(x,y))

        # 计算起始朝向 (使用前两个点的切线)
        if len(self.ref_path) >= 2:
            p0 = self.ref_path[0]
            p1 = self.ref_path[1]
            self.start_yaw = math.atan2(p1.y - p0.y, p1.x - p0.x)

        self.obstacles=[]

    def _generate_maze(self):
        m=self.cfg['maze']; c=m.get('cell',8.0); w=m.get('wall',2.0); gx=int(self.width/c); gy=int(self.height/c); V=[[0]*gy for _ in range(gx)]; E=set(); S=[(0,0)]; V[0][0]=1
        import random
        def Nbs(x,y):
            R=[]
            for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                nx,ny=x+dx,y+dy
                if 0<=nx<gx and 0<=ny<gy and not V[nx][ny]: R.append((nx,ny))
            random.shuffle(R); return R
        while S:
            x,y=S[-1]; nb=Nbs(x,y)
            if not nb: S.pop(); continue
            nx,ny=nb[0]; V[nx][ny]=1; S.append((nx,ny)); E.add(((x,y),(nx,ny)))
        self.obstacles=[]
        for x in range(gx):
            for y in range(gy):
                if x<gx-1 and ((x,y),(x+1,y)) not in E and ((x+1,y),(x,y)) not in E:
                    ox=(x+1)*c; oy=y*c+c/2; self.obstacles.append(('r',ox,oy,w,c))
                if y<gy-1 and ((x,y),(x,y+1)) not in E and ((x,y+1),(x,y)) not in E:
                    ox=x*c+c/2; oy=(y+1)*c; self.obstacles.append(('r',ox,oy,c,w))
        self.start=Point(c/2,c/2); self.goal=Point(self.width-c/2,self.height-c/2); self.ref_path=[]; g=(0,0); self.start_yaw=self.cfg['maze'].get('start_yaw',0.0)
        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            if ((g,(g[0]+dx,g[1]+dy)) in E) or (((g[0]+dx,g[1]+dy),g) in E): self.start_yaw=math.atan2(dy,dx); break

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
