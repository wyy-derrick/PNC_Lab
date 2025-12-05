# PNC 实验室教学笔记（teach.md）

面向将 Maze 的路径搜索替换为 Dijkstra 算法，提供接口与集成指引；你只需按本文说明新增文件与最小改动，即可插拔算法。函数实现由你完成。

## 总览与流程
- 入口与模块：`d:\桌面\wyy\Algorithm_lab\main.py:9, 11-15`
- 环境生成：`main.py:17-19` → `Map.generate_obstacles`（`pnc_lab/src/env/map.py:15-22`）
- 路径规划：Road 用参考线，Normal/Maze 用 `planner.plan(...)`（`main.py:21-26`）
- 起始朝向：Road/Maze 用 `map_env.start_yaw`；Normal 指向终点（`main.py:33`）
- 仿真循环：`main.py:39-56`

## 三种模式速览
- Normal：随机障碍与起终点（`pnc_lab/src/env/map.py:24-43, 101-108`），BFS 规划。
- Road（八字）：参考线与起始切线（`pnc_lab/src/env/map.py:44-56`），三次回中心结束（`main.py:43-49`）。
- Maze：DFS 挖掘墙体与自动朝向（`pnc_lab/src/env/map.py:60-80`），到达终点/近障失败结束（`main.py:49-51`）。

## 接口规范（保持不变）
- 规划接口：`PlannerBase.plan(start, goal, obstacles)`（`pnc_lab/src/planning/planner_base.py:3-12`）
- 输入输出：`start/goal` 为 `Point`；`obstacles` 为 `('c'|'r', x, y, r|w,h)` 列表；返回 `Point` 列表路径。
- 控制接口：`ControllerBase.compute_command(state, path)`（`pnc_lab/src/control/controller_base.py:3-11`）
- 统一配置：`pnc_lab/config/default_config.yaml`

## 将 Maze 切换到 Dijkstra（最小改动方案）
- 新增配置项（建议）：
  - 在 `planning` 段增加 `algorithm: dijkstra|bfs`（默认 `bfs`），可选 `diagonal: true|false` 控制 4/8 连通；保留现有 `grid_size`。
- 新建文件：`pnc_lab/src/planning/dijkstra.py`
  - 类名：`DijkstraPlanner(PlannerBase)`，构造入参为全量 `config`（对齐 BFS）。
  - 方法：`plan(self, start, goal, obstacles)`，与 BFS 完全一致的签名与返回类型。
- 修改入口选择：`main.py` 依据 `cfg['planning'].get('algorithm','bfs')` 选择 Planner；不改其他流程。

## Dijkstra 规划器实现要点（你来写代码）
- 网格映射：
  - `to_grid = lambda p: (int(p.x/gs), int(p.y/gs))`
  - `to_world = lambda gx,gy: Point((gx+0.5)*gs,(gy+0.5)*gs)`
- 邻居定义：
  - 若 `diagonal=true`：`[(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]`
  - 否则 4 连通。
- 邻居代价：
  - 直移代价 `1*gs`，斜移代价 `sqrt(2)*gs`（或直接用 `1` 与 `sqrt(2)`，最终再映射到世界坐标）。
- 碰撞判定：复用 BFS 的 `is_collision(gx,gy)` 逻辑（`pnc_lab/src/planning/bfs.py:18-27`）。
- 主要数据结构：
  - `dist[(gx,gy)]` 记录起点到该格的最小累积代价；初始起点为 `0`。
  - `parent[(gx,gy)]` 记录回溯父节点。
  - 最小堆：`heapq` 存 `(cost,(gx,gy))`。
- 终止与回溯：
  - 取出最小代价节点，松弛所有合法邻居（越界/碰撞/已更优则跳过）。
  - 当取出节点为终点格或堆空结束；若无路径返回空列表。
  - 用 `parent` 从终点回溯到起点，映射到 `Point` 列表并反转。

## Dijkstra 代码框架（最小行数示例，供你照写）
```python
# pnc_lab/src/planning/dijkstra.py
from .planner_base import PlannerBase
from ..utils.types import Point
import math, heapq
class DijkstraPlanner(PlannerBase):
    def __init__(self,cfg): self.gs=cfg['planning']['grid_size']; self.w=int(cfg['map']['width']/self.gs); self.h=int(cfg['map']['height']/self.gs); self.diag=cfg['planning'].get('diagonal',True)
    def plan(self,start,goal,obs):
        tg=lambda p:(int(p.x/self.gs),int(p.y/self.gs)); tw=lambda x,y:Point((x+.5)*self.gs,(y+.5)*self.gs)
        sg,gg=tg(start),tg(goal); iscol=lambda x,y: any(( (o[0]=='c' and ( (x+.5)*self.gs-o[1])**2+((y+.5)*self.gs-o[2])**2<=(o[3]+self.gs)**2 ) ) or ( o[0]=='r' and (abs((x+.5)*self.gs-o[1])-o[3]/2<=self.gs and abs((y+.5)*self.gs-o[2])-o[4]/2<=self.gs and ( (abs((x+.5)*self.gs-o[1])-o[3]/2<=0 and abs((y+.5)*self.gs-o[2])-o[4]/2<=0) or (max(abs((x+.5)*self.gs-o[1])-o[3]/2,0)**2+max(abs((y+.5)*self.gs-o[2])-o[4]/2,0)**2<=self.gs**2) ) ) ) for o in obs)
        N=[(0,1),(1,0),(0,-1),(-1,0)]+([(1,1),(1,-1),(-1,1),(-1,-1)] if self.diag else [])
        dist={sg:0}; parent={}; pq=[(0,sg)]; vis=set()
        while pq:
            c,u=heapq.heappop(pq)
            if u in vis: continue; vis.add(u)
            if u==gg: break
            for dx,dy in N:
                v=(u[0]+dx,u[1]+dy)
                if not(0<=v[0]<self.w and 0<=v[1]<self.h) or iscol(*v): continue
                w=self.gs*(math.sqrt(2) if dx and dy else 1)
                nc=c+w
                if nc<dist.get(v,float('inf')): dist[v]=nc; parent[v]=u; heapq.heappush(pq,(nc,v))
        if gg not in parent and sg!=gg: return []
        path=[goal]; cur=gg if gg in parent else sg
        while cur!=sg: path.append(tw(*cur)); cur=parent[cur]
        return path[::-1]
```

## 入口选择修改（仅几行）
- 在 `main.py` 顶部引入你的 Planner 文件：`from src.planning.dijkstra import DijkstraPlanner`
- 用配置切换：
```python
# d:\桌面\wyy\Algorithm_lab\main.py:14-15
algo=cfg.get('planning',{}).get('algorithm','bfs')
planner = DijkstraPlanner(cfg) if algo=='dijkstra' else BFSPlanner(cfg)
```

## 配置示例（不修改其它参数）
```yaml
planning:
  algorithm: dijkstra # bfs|dijkstra
  grid_size: 1.0
  diagonal: true
```

## 兼容性与注意事项
- 保持接口与返回类型一致，Road/Normal/Maze 的主流程无需变更。
- 统一配置管理，不重复定义变量；仅在 `planning` 段新增键。
- 迷宫墙体判定沿用 BFS 的近似方法，避免引入新依赖。
- 性能：`grid_size` 越小越慢；`diagonal=false` 会更保守但路径可能更长。

## 速查索引
- 入口与流程：`d:\桌面\wyy\Algorithm_lab\main.py:9, 17-26, 32-34, 39-56`
- 地图生成：`pnc_lab/src/env/map.py:15-22, 24-58, 60-80, 81-99, 101-108`
- 规划接口：`pnc_lab/src/planning/planner_base.py:3-12`
- BFS 参考：`pnc_lab/src/planning/bfs.py:11-44`
- 控制接口与纯跟踪：`pnc_lab/src/control/controller_base.py:3-11`, `pnc_lab/src/control/pure_pursuit.py:15-42`

> 实践建议：先在 Maze 模式验证 Dijkstra，再按同样接口扩展到 Normal；如需多算法共存，仅在入口根据配置选择实例即可。
