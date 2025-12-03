import yaml, sys, os, math
sys.path.append('pnc_lab')
from src.env.map import Map
from src.env.vehicle import Vehicle
from src.env.visualizer import Visualizer
from src.planning.bfs import BFSPlanner
from src.control.pure_pursuit import PurePursuitController

cfg = yaml.safe_load(open('pnc_lab/config/default_config.yaml', encoding='utf-8'))
print("初始化...")
map_env = Map(cfg)
vehicle = Vehicle(cfg)
vis = Visualizer(cfg)
planner = BFSPlanner(cfg)
controller = PurePursuitController(cfg)

print("生成环境...")
obs = map_env.generate_obstacles()
vis.obs = obs

print("规划路径...")
if cfg['map'].get('type') == 'road':
    path = map_env.ref_path
else:
    path = planner.plan(map_env.start, map_env.goal, obs)

if not path:
    print("无法找到路径！请重试。")
    sys.exit(1)

print("开始仿真 (按 Ctrl+C 停止)...")
t=cfg['map'].get('type')
init_yaw = map_env.start_yaw if t in ('road','maze') else math.atan2(map_env.goal.y-map_env.start.y,map_env.goal.x-map_env.start.x)
vehicle.set_state(map_env.start.x, map_env.start.y, 0, init_yaw)

target_speed = cfg['control']['target_speed']
step = 0
laps=0;near=False;fr=cfg['map'].get('road',{}).get('finish_radius',2.0);need=cfg['map'].get('road',{}).get('finish_laps',3)
while True:
    try:
        acc, delta, _ = controller.compute_command(vehicle.state, path)
        vehicle.update(acc, delta)
        if cfg['map'].get('type')=='road':
            d=math.hypot(vehicle.state.x-map_env.start.x,vehicle.state.y-map_env.start.y)
            if d<fr:
                if not near: laps+=1; near=True; print(f"回到起点第{laps}次")
            else: near=False
            if laps>=need: print("完成任务"); break
        else:
            if map_env.check_collision(vehicle.state.x, vehicle.state.y, cfg['map']['near_fail_dist']): print("接近障碍物，任务失败！"); break
            if math.hypot(vehicle.state.x - map_env.goal.x, vehicle.state.y - map_env.goal.y) < 2.0: print("到达终点！"); break
        vis.render(path, vehicle.state, acc, delta); step+=1
    except KeyboardInterrupt:
        print("仿真中断"); break

vis.close()
