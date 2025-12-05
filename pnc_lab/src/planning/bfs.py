from .planner_base import PlannerBase
from ..utils.types import Point
import math
# BFS 网格路径规划器：在离散网格图上用广度优先搜索最短路径（最少步数）

class BFSPlanner(PlannerBase):
    def __init__(self, config):
        # 网格边长（米），将连续空间按 gs 划分为 w×h 个格子   gs就是网格边长 默认是1
        self.gs = config['planning']['grid_size']
        # 地图尺寸除以 gs 得到网格维度（格子个数）   默认是120x120个格子
        self.w = int(config['map']['width'] / self.gs)
        self.h = int(config['map']['height'] / self.gs)
        self.sm = config['planning'].get('safe_margin',0.0)

    def plan(self, start, goal, obstacles):
        # 坐标→网格索引（将点映射到其所在格子）；网格索引→坐标（取格子中心点）
        #lambda是匿名函数的意思 本质上就是定义to_grid和to_world这两个函数 后面连着的就是传入的参数
        to_grid = lambda p: (int(p.x/self.gs), int(p.y/self.gs))
        to_world = lambda gx, gy: Point((gx+0.5)*self.gs, (gy+0.5)*self.gs)
        
        # 起点/终点的网格索引
        sg, gg = to_grid(start), to_grid(goal)
        #to_grid输出的就是起点和终点的网格索引类似（0， 0）网格坐标
        # 队列（先进先出） 是一个正常的列表 每次append从尾部加入新的节点 每次pop出队头的节点
        q = [sg]
        #已经访问字典 父节点字典
        visited, parent = {sg}, {}
        
        # 碰撞检测：若格子中心落入圆/矩形障碍膨胀范围，则视为不可通行节点
        def is_collision(gx, gy):
            wx, wy = (gx+0.5)*self.gs, (gy+0.5)*self.gs; R=self.gs+self.sm
            for o in obstacles:
                if o[0]=='c':
                    _,ox,oy,r=o
                    if (wx-ox)**2+(wy-oy)**2<=(r+R)**2: return True
                else:
                    _,ox,oy,w,h=o; dx=abs(wx-ox)-w/2; dy=abs(wy-oy)-h/2
                    if dx<=R and dy<=R and (dx<=0 and dy<=0 or (max(dx,0)**2+max(dy,0)**2<=R**2)): return True
            return False

        # BFS 主循环：逐层展开邻居（八连通），遇到终点立即停止（最少步数）
        while q:
            curr = q.pop(0)
            if curr == gg: break
            # 邻居方向：4/8 连通；此处启用 8 连通以提高可达性与路径平滑度
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]:
                nx, ny = curr[0]+dx, curr[1]+dy
                # 边界、未访问、非碰撞 三条件同时满足则入队并记录父指针
                if 0<=nx<self.w and 0<=ny<self.h and (nx,ny) not in visited and not is_collision(nx,ny):
                    visited.add((nx,ny)); parent[(nx,ny)] = curr; q.append((nx,ny))
                    
        # 若终点不可达且起终点不重合，则返回空路径
        if gg not in parent and sg != gg: return []
        
        # 回溯路径：从终点沿 parent 回到起点；再按世界坐标序列返回给控制器
        path, curr = [goal], gg if gg in parent else sg
        while curr != sg:
            path.append(to_world(*curr))
            curr = parent[curr]
        return path[::-1]
