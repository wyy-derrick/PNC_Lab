from .planner_base import PlannerBase
from ..utils.types import Point
import math

class BFSPlanner(PlannerBase):
    def __init__(self, config):
        self.gs = config['planning']['grid_size']
        self.w = int(config['map']['width'] / self.gs)
        self.h = int(config['map']['height'] / self.gs)

    def plan(self, start, goal, obstacles):
        to_grid = lambda p: (int(p.x/self.gs), int(p.y/self.gs))
        to_world = lambda gx, gy: Point((gx+0.5)*self.gs, (gy+0.5)*self.gs)
        
        sg, gg = to_grid(start), to_grid(goal)
        q, visited, parent = [sg], {sg}, {}
        
        def is_collision(gx, gy):
            wx, wy = (gx+0.5)*self.gs, (gy+0.5)*self.gs
            for o in obstacles:
                if o[0]=='c':
                    _,ox,oy,r=o
                    if (wx-ox)**2+(wy-oy)**2<=(r+self.gs)**2: return True
                else:
                    _,ox,oy,w,h=o; dx=abs(wx-ox)-w/2; dy=abs(wy-oy)-h/2
                    if dx<=self.gs and dy<=self.gs and (dx<=0 and dy<=0 or (max(dx,0)**2+max(dy,0)**2<=self.gs**2)): return True
            return False

        while q:
            curr = q.pop(0)
            if curr == gg: break
            
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]: # 8-conn
                nx, ny = curr[0]+dx, curr[1]+dy
                if 0<=nx<self.w and 0<=ny<self.h and (nx,ny) not in visited and not is_collision(nx,ny):
                    visited.add((nx,ny)); parent[(nx,ny)] = curr; q.append((nx,ny))
                    
        if gg not in parent and sg != gg: return [] # No path
        
        path, curr = [goal], gg if gg in parent else sg # 包含终点
        while curr != sg:
            path.append(to_world(*curr))
            curr = parent[curr]
        return path[::-1]
