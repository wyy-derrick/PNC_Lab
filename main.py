import yaml, sys, os, math
# 将项目路径添加到系统路径，确保可以正确导入模块
sys.path.append('pnc_lab')
# 导入地图、车辆、可视化、规划器和控制器模块
from src.env.map import Map
from src.env.vehicle import Vehicle
from src.env.visualizer import Visualizer
from src.planning.bfs import BFSPlanner
from src.planning.Dijkstra import DijkstraPlanner

from src.control.pure_pursuit import PurePursuitController

# 加载配置文件，配置文件中包含地图、车辆、控制器等参数
cfg = yaml.safe_load(open('pnc_lab/config/default_config.yaml', encoding='utf-8'))

# 初始化各模块
print("初始化...")
map_env = Map(cfg)  # 地图环境对象，用于生成地图和障碍物
vehicle = Vehicle(cfg)  # 车辆对象，包含车辆状态和运动学模型
vis = Visualizer(cfg)  # 可视化对象，用于实时显示仿真过程
planner = DijkstraPlanner(cfg)
bfs=BFSPlanner(cfg)
controller = PurePursuitController(cfg)  # 控制器，使用纯跟踪算法控制车辆

# 生成环境
print("生成环境...")
obs = map_env.generate_obstacles()  # 生成障碍物
vis.obs = obs  # 将障碍物传递给可视化模块

# 规划路径
print("规划路径...")
if cfg['map'].get('type')=='road':
    path=map_env.ref_path; path2=None
else:
    path=planner.plan(map_env.start,map_env.goal,obs)
    path2=bfs.plan(map_env.start,map_env.goal,obs)
    # path2=[]

# 如果路径为空，说明无法找到路径，程序退出
if not path:
    print("无法找到路径！请重试。")
    sys.exit(1)

# 开始仿真
print("开始仿真 (按 Ctrl+C 停止)...")
t = cfg['map'].get('type')  # 获取地图类型
# 根据地图类型设置车辆初始朝向
init_yaw = map_env.start_yaw if t in ('road', 'maze') else math.atan2(
    map_env.goal.y - map_env.start.y, map_env.goal.x - map_env.start.x)
# 设置车辆初始状态（位置、速度、朝向）
vehicle.set_state(map_env.start.x, map_env.start.y, 0, init_yaw)

# 获取目标速度
target_speed = cfg['control']['target_speed']
step = 0  # 仿真步数计数器
laps = 0  # 记录车辆在 road 模式下回到起点的次数
near = False  # 标记车辆是否接近起点
# 获取 road 模式下的完成任务判定参数
fr = cfg['map'].get('road', {}).get('finish_radius', 2.0)  # 起点附近的判定半径
need = cfg['map'].get('road', {}).get('finish_laps', 3)  # 完成任务所需的圈数

# 仿真主循环
while True:
    try:
        # 计算控制指令（加速度和转向角）
        acc, delta, _ = controller.compute_command(vehicle.state, path)
        # 更新车辆状态
        vehicle.update(acc, delta)
        # 如果是 road 模式，检查是否完成任务
        if cfg['map'].get('type') == 'road':
            # 计算车辆与起点的距离
            d = math.hypot(vehicle.state.x - map_env.start.x, vehicle.state.y - map_env.start.y)
            if d < fr:  # 如果车辆接近起点
                if not near:  # 如果之前未接近过
                    laps += 1  # 圈数加一
                    near = True
                    print(f"回到起点第{laps}次")
            else:
                near = False  # 离开起点范围
            if laps >= need:  # 如果圈数达到要求，任务完成
                print("完成任务")
                break
        else:
            # 如果是其他模式，检查是否碰撞或到达终点
            if map_env.check_collision(vehicle.state.x, vehicle.state.y, cfg['map']['near_fail_dist']):
                print("接近障碍物，任务失败！")
                break
            if math.hypot(vehicle.state.x - map_env.goal.x, vehicle.state.y - map_env.goal.y) < 2.0:
                print("到达终点！")
                break
        # 渲染当前仿真状态
        vis.render(path, vehicle.state, acc, delta, path2,'g--')
        step += 1  # 仿真步数加一
    except KeyboardInterrupt:  # 捕捉键盘中断信号
        print("仿真中断")
        break

# 关闭可视化窗口
vis.close()
