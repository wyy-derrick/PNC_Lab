# 自动驾驶 PNC 算法实验室

这是一个轻量级的自动驾驶规划与控制 (Planning and Control) 算法验证平台。
专为学习设计，代码结构清晰，模块解耦。

## 🚀 快速开始

### 1. 环境配置 (Conda)

如果你是初学者，推荐使用 Conda 来管理环境。

```bash
# 1. 创建并激活新环境 (Python 3.9)
conda create -n pnc_lab python=3.9 -y
conda activate pnc_lab

# 2. 安装依赖
pip install -r requirements.txt
```

`requirements.txt` 中包含：
- `numpy`: 虽然本简版代码尽量减少了依赖，但它是科学计算的基础。
- `matplotlib`: 用于实时绘图与可视化。
- `pyyaml`: 用于读取 `.yaml` 配置文件。

### 2. 运行代码

在项目根目录下运行：

```bash
python main.py
```

你应该能看到：
- 左侧：只显示路径与四轮小车车体（含黄色航向箭头）。
- 右侧：实时的控制指令波形（`Acc`,`Steer`）。

## 📂 项目结构

```text
pnc_lab/
├── config/
│   └── default_config.yaml  # ✨ 核心配置文件：所有参数都在这里修改，无需改代码
├── src/
│   ├── env/                 # 仿真环境 (车辆模型、地图、可视化)
│   ├── planning/            # 路径规划 (目前实现了 BFS)
│   ├── control/             # 车辆控制 (目前实现了 Pure Pursuit + PID)
│   └── utils/               # 通用工具
└── main.py                  # 程序入口
```

## 🛠️ 核心功能说明

1.  **统一配置**: 修改 `config/default_config.yaml` 可以调整地图大小、障碍物数量、车辆物理参数、PID 参数等。
2.  **模块化设计**:
    *   `planning`: 实现了 `PlannerBase` 接口，方便未来添加 A* 等新算法。
    *   `control`: 实现了 `ControllerBase` 接口，方便未来添加 Stanley, MPC 等新算法。
3.  **可视化**:
   - 主图：仅绘制路径和四轮车（多边形+四轮），新增黄色箭头指示车头航向。
   - 辅图：实时绘制控制器输出指令（纵向加速度 `Acc`，横向转角 `Steer`）。
4.  **地图与障碍物**:
   - **Normal 模式**: 随机起终点，散布圆形/矩形障碍物；起始朝向指向终点方向。
   - **Road 模式（八字弯）**: 起点在地图中心，参考线为可调八字；不生成车道障碍，仅用于路径跟踪评测；车辆三次回到中心（小于阈值半径）判定完成。

5.  **Maze 模式**
   - 固定起终点在对角线位置，使用 BFS 规划，控制器追踪。
   - 迷宫由规则网格随机挖掘生成，可通过配置调整单元尺寸与墙厚。

## 🧱 Maze 模式
- 设置 `map.type: maze` 启用迷宫地图；起点为左上角单元中心，终点为右下角单元中心。
- 路径由 `BFSPlanner` 在网格上规划，车辆用纯跟踪控制追踪路径。
- 起始朝向：自动对齐到首段路径方向（起点到第一个可通行邻居）；无邻居时回退到 `map.maze.start_yaw`（默认 0 弧度，向右）。
- 代码位置：
  - 迷宫生成：`pnc_lab/src/env/map.py`
  - 规划与控制：`pnc_lab/src/planning/bfs.py`, `pnc_lab/src/control/pure_pursuit.py`

## 🧩 三种模式总览
- 切换模式：在 `config/default_config.yaml` 设置 `map.type: normal | road | maze`。
- Normal：随机障碍+随机起终点；起始朝向指向终点；使用 BFS 规划、纯跟踪控制。
- Road（八字弯）：中心起终点；Lissajous 参考线；三次回中心完成；无障碍；起始朝向为参考线首段切线。
- Maze：规则网格随机挖掘；BFS 规划；起始朝向自动对齐首段路径，异常时回退配置值。

## 📝 学习建议 (给初学者)

*   **如果想改地图**: 去 `config/default_config.yaml` 修改 `map` 部分（包括矩形比例、尺寸范围、近障失败距离、随机起终点最小距离）。
*   **如果想改控制效果**: 调节 `control` 下的 `pid` 参数或 `pure_pursuit` 的预瞄系数 `k`。
*   **代码阅读顺序**:
    1.  `main.py`: 了解整个流程是如何串联的。
    2.  `pnc_lab/src/env/vehicle.py`: 了解自行车模型是如何运动的。
    3.  `pnc_lab/src/control/pure_pursuit.py`: 学习最经典的纯跟踪控制算法。
## 🧭 Road 模式（八字弯）
- 起点/终点：地图中心；参考线为 Lissajous 曲线：`x=cx+A·sin(t)`, `y=cy+B·sin(2t)`。
- 结束条件：每次车辆位置距中心小于 `finish_radius` 记为一次回中心；累计达到 `finish_laps` 即完成。
- 代码位置：
  - 参考线生成：`pnc_lab/src/env/map.py:43–57`
  - 起始朝向计算：`pnc_lab/src/env/map.py:51–55`
  - Road 完成判定：`main.py:37–47`
- 配置项（`config/default_config.yaml`）：
  - `map.type: road`
  - `map.road.eight_A`（横向振幅），`map.road.eight_B`（纵向振幅）
  - `map.road.eight_loops`（回环次数），`map.road.eight_points`（采样点数）
  - `map.road.finish_laps`（完成所需次数），`map.road.finish_radius`（中心半径阈值）

## 🚗 车辆与控制
- 车辆模型：自行车模型更新在 `pnc_lab/src/env/vehicle.py:23–34`。
- 纯跟踪控制：`pnc_lab/src/control/pure_pursuit.py:17–45`
  - 预瞄距离：`ld = k·v + L_min`（`control.pure_pursuit.k`, `L_min`）
  - PID 纵向：以 `target_speed` 为目标速度进行闭环控制（`control.pid`）

## 🔧 调参建议
- 更难的路况：增大 `eight_A/eight_B`（弯更急、曲率更大）。
- 稳健性：适当增大 `pure_pursuit.k` 提升预瞄距离，或调小 `pid.kp` 减少纵向振荡。

## 📑 关键参数总览（统一配置）
- `map.type`: `normal | road | maze`
- `map.road`: `eight_A`, `eight_B`, `eight_loops`, `eight_points`, `finish_laps`, `finish_radius`
- `map.maze`: `cell`, `wall`, `start_yaw`
- `vehicle`: `L`, `max_steer`, `max_acc`, `dt`, `car_length`, `car_width`
- `control`: `target_speed`, `pure_pursuit.k`, `pure_pursuit.L_min`, `pid.kp/ki/kd`

## 🖥️ 性能提示
- 关闭障碍显示：`vis.show_obstacles: false`（road 模式默认无障碍）。
- 调整渲染速度：`vis.pause` 越小刷新越快，CPU占用更高。

## ❓常见问题
无
