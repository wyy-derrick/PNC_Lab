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
- 左侧：只显示路径与四轮小车车体（不再显示箭头/其它图层）。
- 右侧：实时的横纵向速度曲线（`vx`,`vy`）。

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
   - 主图：仅绘制路径和四轮车（多边形+四轮）。
   - 辅图：实时绘制速度波形 `vx, vy`。
4.  **地图与障碍物**:
   - 更大地图，起终点随机，保证最小距离；支持圆形与轴对齐矩形“墙”。
   - 失败条件：若车辆与障碍物距离小于 `map.near_fail_dist`，仿真结束并判定失败。

## 📝 学习建议 (给初学者)

*   **如果想改地图**: 去 `config/default_config.yaml` 修改 `map` 部分（包括矩形比例、尺寸范围、近障失败距离、随机起终点最小距离）。
*   **如果想改控制效果**: 调节 `control` 下的 `pid` 参数或 `pure_pursuit` 的预瞄系数 `k`。
*   **代码阅读顺序**:
    1.  `main.py`: 了解整个流程是如何串联的。
    2.  `pnc_lab/src/env/vehicle.py`: 了解自行车模型是如何运动的。
    3.  `pnc_lab/src/control/pure_pursuit.py`: 学习最经典的纯跟踪控制算法。
