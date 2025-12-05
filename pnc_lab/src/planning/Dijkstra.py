from .planner_base import PlannerBase
from ..utils.types import Point
import math, heapq

# Dijkstra 路径规划器：使用优先队列（小根堆）在加权网格图上寻找最短路径
# 相比 BFS，Dijkstra 考虑了移动的代价（权重），能区分直线（代价1）和斜线（代价√2）

class DijkstraPlanner(PlannerBase):
    def __init__(self, config):
        # 初始化网格参数
        # gs: 网格单元的大小（米），将连续的世界坐标离散化
        self.gs = config['planning']['grid_size']
        # w, h: 计算地图网格的宽度和高度（即横纵方向的格子数量）
        self.w = int(config['map']['width'] / self.gs)
        self.h = int(config['map']['height'] / self.gs)
        # sm: 安全边距（Safe Margin），用于在障碍物周围增加一圈缓冲区
        self.sm = config['planning'].get('safe_margin', 0.0)

    def plan(self, start, goal, obstacles):
        print('Dijkstra规划中...')
        
        # 定义坐标转换的匿名函数（Lambda）：
        # tg (to_grid): 将世界坐标(float)转换为网格索引(int, int)
        tg = lambda p: (int(p.x/self.gs), int(p.y/self.gs))
        # tw (to_world): 将网格索引(int, int)转换回世界坐标中心点(float)
        tw = lambda x, y: Point((x+0.5)*self.gs, (y+0.5)*self.gs)
        
        # 获取起点和终点的网格坐标
        sg, gg = tg(start), tg(goal)
        # 计算总膨胀半径 R = 网格大小 + 安全边距
        R = self.gs + self.sm

        # 内部函数：碰撞检测
        # 判断网格坐标 (x, y) 是否与任何障碍物发生冲突
        def col(x, y):
            # 将网格索引转换为世界坐标中心，用于几何计算
            wx, wy = (x+0.5)*self.gs, (y+0.5)*self.gs
            for o in obstacles:
                if o[0] == 'c': # 处理圆形障碍物
                    _, ox, oy, r = o
                    # 判断网格中心到圆心的距离是否小于 (障碍物半径 + 膨胀半径)
                    if (wx-ox)**2 + (wy-oy)**2 <= (r+R)**2: return True
                else: # 处理矩形障碍物
                    _, ox, oy, w, h = o
                    # 计算网格中心到矩形中心的相对距离（绝对值）
                    dx = abs(wx-ox) - w/2; dy = abs(wy-oy) - h/2
                    # 判断逻辑：如果在矩形内部或边缘膨胀范围内，则认为碰撞
                    if dx <= R and dy <= R and (dx <= 0 and dy <= 0 or (max(dx, 0)**2 + max(dy, 0)**2 <= R**2)): return True
            return False

        # 预先检查：如果起点或终点本身就在障碍物内，直接返回空路径
        if col(*sg) or col(*gg): return []

        # 定义节点的邻居方向及其移动代价（权重）
        # 格式: (dx, dy, weight)
        # 直线移动（上下左右）代价为 1
        # 对角线移动代价为 sqrt(2) ≈ 1.414，符合物理距离规律
        nb = [
            (0, 1, 1), (1, 0, 1), (0, -1, 1), (-1, 0, 1),             # 4个直线方向
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),              # 4个对角线方向
            (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))
        ]
        
        # dist: 记录从起点到每个点的当前已知最短距离，初始化起点距离为0
        dist = {sg: 0}
        # par: Parent 字典，记录路径上每个节点的前驱节点，用于最后回溯路径
        par = {}
        # pq: Priority Queue（优先队列），存储元组 (当前距离, 节点坐标)
        # heapq 实现的是小根堆，每次 pop 都会弹出距离最小的节点
        pq = [(0, sg)]
        # seen: 记录已经处理过（即已经作为最短路径基准扩展过）的节点
        seen = set()

        # Dijkstra 算法主循环
        while pq:
            # 弹出当前队列中距离起点最近的节点 u，及其距离 d
            d, u = heapq.heappop(pq)
            
            # 如果该节点已经被处理过（意味着我们之前已经通过更短或相等的路径访问过它），则跳过
            if u in seen: continue
            seen.add(u)
            
            # 如果当前节点就是终点，说明最短路径已找到，提前结束搜索
            if u == gg: break
            
            x, y = u
            # 遍历当前节点的 8 个邻居
            for dx, dy, w in nb:
                nx, ny = x + dx, y + dy
                
                # 检查邻居节点是否在地图范围内，且没有碰撞
                if 0 <= nx < self.w and 0 <= ny < self.h and not col(nx, ny):
                    # 计算经由当前节点 u 到达邻居 (nx, ny) 的新距离
                    nd = d + w
                    
                    # 如果新距离比之前记录的距离更短（或者该邻居从未被访问过）
                    # get((nx, ny), 1e18) 表示如果未访问过则距离设为无穷大
                    if nd < dist.get((nx, ny), 1e18):
                        dist[(nx, ny)] = nd       # 更新最短距离
                        par[(nx, ny)] = u         # 更新父节点指针
                        heapq.heappush(pq, (nd, (nx, ny))) # 将新的更优路径加入优先队列

        # 路径重建
        # 如果终点不在 par 字典中（且起点不等于终点），说明无法到达终点
        if gg not in par and sg != gg: return []

        # 从终点开始回溯
        path = [goal] # 路径包含精确的终点坐标
        cur = gg if gg in par else sg
        
        # 沿着 par 字典中的父节点指针一直回溯到起点
        while cur != sg:
            path.append(tw(*cur)) # 将网格中心转换为世界坐标并加入路径
            cur = par[cur]
        
        # 因为是从终点往回找，所以最后需要将路径反转，变成 起点 -> 终点 的顺序
        return path[::-1]
