import yaml
import os
import time
from pnc_lab.src.env.map import Map
from pnc_lab.src.env.vehicle import Vehicle
from pnc_lab.src.env.visualizer import Visualizer
from pnc_lab.src.planning.bfs import BFSPlanner
from pnc_lab.src.control.pure_pursuit import PurePursuitController
from pnc_lab.src.utils.math_utils import dist

def load_config():
    paths = [
        'pnc_lab/config/default_config.yaml',
        os.path.join(os.path.dirname(__file__), 'pnc_lab/config/default_config.yaml')
    ]
    for p in paths:
        if os.path.exists(p):
            with open(p, 'r', encoding='utf-8') as f: return yaml.safe_load(f)
    raise FileNotFoundError("Config file not found")

def main():
    print("初始化...")
    cfg = load_config()
    
    map_env = Map(cfg)
    vehicle = Vehicle(cfg)
    vis = Visualizer(cfg)
    planner = BFSPlanner(cfg)
    controller = PurePursuitController(cfg)
    
    print("生成环境...")
    obs = map_env.generate_obstacles(); vis.obs=obs
    vehicle.set_state(map_env.start.x, map_env.start.y, 0, 0)
    
    print("规划路径...")
    path = planner.plan(map_env.start, map_env.goal, obs)
    
    if not path:
        print("无法找到路径！请重试。")
        return

    print(f"路径已找到，步数: {len(path)}")
    
    try:
        print("开始仿真 (按 Ctrl+C 停止)...")
        max_steps = 3000
        for step in range(max_steps):
            # 检查是否到达
            if dist(vehicle.state, map_env.goal) < 2.0:
                print("到达终点！")
                break
                
            # 计算控制
            acc, steer, _ = controller.compute_command(vehicle.state, path)
            
            # 更新状态
            vehicle.update(acc, steer)
            
            # 渲染
            vis.render(path, vehicle.state, acc, steer)

            # 失败判定：接近障碍物
            if map_env.near_obstacle(vehicle.state.x, vehicle.state.y, cfg['map']['near_fail_dist']):
                print("接近障碍物，任务失败！")
                break
            
            # 简单的超时机制
            if step == max_steps - 1:
                print("超时未到达。")
                
    except KeyboardInterrupt:
        print("仿真停止。")
    finally:
        vis.close()

if __name__ == '__main__':
    main()
