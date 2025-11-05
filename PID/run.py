
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from simulation import BicycleModel, Path

# --- 1. 设置中文显示 ---
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False  

# --- 2. 控制器 (包含所有算法逻辑) ---
class Controller:
    """
    实现纵向P控制器 + 横向纯跟踪(Pure Pursuit)控制器
    这个控制器现在"拥有"所有算法逻辑
    """
    def __init__(self, Kp_v, target_speed, L, K_lookahead, L_lookahead_base, path_data):
        # 纵向P控制器（速度）
        self.Kp_v = Kp_v
        self.target_speed = target_speed

        # 横向纯跟踪控制器（转向）
        self.L = L  # 车辆轴距 (m)
        self.K_lookahead = K_lookahead # 前瞻距离的速度增益
        self.L_lookahead_base = L_lookahead_base # 基础前瞻距离 (m)
        
        # --- 核心修改 ---
        # 控制器存储它需要的所有路径信息
        self.path_x_raw = path_data.x
        self.path_y_raw = path_data.y
        self.path_yaw_raw = path_data.yaw
        self.path_x_wrapped = path_data.x_wrapped
        self.path_y_wrapped = path_data.y_wrapped

    def normalize_angle(self, angle):
        """将角度归一化到 [-pi, pi]"""
        while angle > np.pi: angle -= 2 * np.pi
        while angle < -np.pi: angle += 2 * np.pi
        return angle

    # --- 新增的内部方法 ---
    def _find_closest_point(self, x, y):
        """
        (从 Path 类移入)
        找到路径上离车辆当前位置最近的点
        """
        dx = self.path_x_raw - x
        dy = self.path_y_raw - y
        distances = np.sqrt(dx**2 + dy**2)
        closest_index = np.argmin(distances)
        return closest_index, distances[closest_index]

    # --- 新增的内部方法 ---
    def _find_lookahead_point(self, x, y, Ld):
        """
        (从 Path 类移入)
        找到距离车辆 (x,y) 约 Ld 远的前瞻点
        """
        # 1. 找到最近点 (调用自己的内部方法)
        closest_index, _ = self._find_closest_point(x, y)
        
        # 2. 从最近点开始，沿着 "环形" 路径搜索
        target_idx = closest_index
        current_dist = 0.0
        while current_dist < Ld and target_idx < len(self.path_x_wrapped) - 1:
            # 计算车辆(x,y)到路径点(target_idx)的直线距离
            current_dist = np.sqrt((self.path_x_wrapped[target_idx] - x)**2 + (self.path_y_wrapped[target_idx] - y)**2)
            
            if current_dist < Ld:
                target_idx += 1
        
        # 3. 找到了目标点
        return self.path_x_wrapped[target_idx], self.path_y_wrapped[target_idx]

    def control(self, state):
        """
        (修改：不再需要 path 参数)
        计算控制量：油门（加速度）和前轮转角
        """
        x, y, yaw, v = state
        
        # --- 1. 纵向控制 (P-Controller for Velocity) ---
        v_error = self.target_speed - v
        throttle = self.Kp_v * v_error
        
        # --- 2. 横向控制 (Pure Pursuit) ---
        Ld = self.L_lookahead_base + self.K_lookahead * v
        
        # 调用自己的内部方法
        tx, ty = self._find_lookahead_point(x, y, Ld)
        
        alpha = np.arctan2(ty - y, tx - x) - yaw
        alpha = self.normalize_angle(alpha)
        
        delta = np.arctan2(2.0 * self.L * np.sin(alpha), Ld)
        
        return throttle, delta

# --- 3. 仿真和动画设置 ---

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# ！！！ 调整参数区 ！！！
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
KP_V = 2.0
TARGET_SPEED = 10.0
L_LOOKAHEAD_BASE = 4.0 
K_LOOKAHEAD = 0.5 

# --- 初始化环境和智能体 ---
# 1. 创建"世界"和"地图"
vehicle = BicycleModel(x=0.0, y=-10.0, yaw=np.radians(90), v=0.0, L=2.5, dt=0.1)
path = Path(amplitude=50.0, num_points=1000)

# 2. 创建"控制器"，并把"地图" (path) 交给它
controller = Controller(Kp_v=KP_V, 
                        target_speed=TARGET_SPEED, 
                        L=vehicle.L,
                        K_lookahead=K_LOOKAHEAD, 
                        L_lookahead_base=L_LOOKAHEAD_BASE,
                        path_data=path) # <--- 在这里把路径数据传入

# (后续的仿真和绘图设置与之前相同)
T_SIM = 150 
max_steps = int(T_SIM / vehicle.dt)
history = {'x': [], 'y': [], 'v': [], 'cte': [], 'yaw_error': [], 'time': []}

# --- 4. 设置画布 ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12))
# 图1: 路径图
ax1.plot(path.x, path.y, 'b--', label='参考路径 (8字形)')
ax1.plot(0.0, -10.0, 'go', markersize=10, label='起点')
trajectory_line, = ax1.plot([], [], 'r-', label='车辆实际轨迹')
vehicle_marker, = ax1.plot([], [], 'ro', label='车辆当前位置')
lookahead_marker, = ax1.plot([], [], 'mo', markersize=8, label='前瞻点')
ax1.set_title('纯跟踪 (Pure Pursuit) 动态仿真 (模块化)')
ax1.set_xlabel('X 坐标 (m)')
ax1.set_ylabel('Y 坐标 (m)')
ax1.legend(loc='upper right')
ax1.axis('equal')
ax1.grid(True)
# 图2: 误差图
cte_line, = ax2.plot([], [], 'g-', label='横向跟踪误差 (m)')
yaw_error_line, = ax2.plot([], [], 'm-', label='航向误差 (rad)')
ax2.set_title('跟踪误差-时间图')
ax2.set_xlabel('时间 (s)')
ax2.set_ylabel('误差')
ax2.legend(loc='upper right')
ax2.grid(True)
ax2.set_xlim(0, T_SIM)
ax2.set_ylim(-3.0, 3.0) 
plt.tight_layout()

# --- 5. 动画更新函数 ---
def animate(i):
    """
    这个函数在动画的每一帧都会被调用
    """
    current_state = vehicle.get_state()
    current_x, current_y, current_yaw, current_v = current_state
    
    # 1. 运行一步控制 (不再需要传入 path)
    throttle, delta = controller.control(current_state)
    
    # 2. 更新车辆状态
    vehicle.update(throttle, delta)
    
    # --- 3. 计算误差 (仅用于绘图) ---
    #    (现在通过 controller 的方法来获取路径信息)
    target_idx, _ = controller._find_closest_point(current_x, current_y)
    target_x = controller.path_x_raw[target_idx]
    target_y = controller.path_y_raw[target_idx]
    target_yaw = controller.path_yaw_raw[target_idx]
    
    error_vec_x = current_x - target_x
    error_vec_y = current_y - target_y
    path_normal_x = -np.sin(target_yaw)
    path_normal_y = np.cos(target_yaw)
    cte = error_vec_x * path_normal_x + error_vec_y * path_normal_y
    yaw_error = controller.normalize_angle(target_yaw - current_yaw)
    
    # 4. 记录数据
    history['x'].append(current_x)
    history['y'].append(current_y)
    history['v'].append(current_v)
    history['cte'].append(cte)
    history['yaw_error'].append(yaw_error)
    history['time'].append(i * vehicle.dt)
    
    # 5. 更新绘图数据
    trajectory_line.set_data(history['x'], history['y'])
    vehicle_marker.set_data([current_x], [current_y])
    
    # (通过 controller 的方法来获取前瞻点)
    Ld = controller.L_lookahead_base + controller.K_lookahead * current_v
    tx, ty = controller._find_lookahead_point(current_x, current_y, Ld)
    lookahead_marker.set_data([tx], [ty])
    
    cte_line.set_data(history['time'], history['cte'])
    yaw_error_line.set_data(history['time'], history['yaw_error'])
    
    if i > 10:
        all_errors = history['cte'] + history['yaw_error']
        min_err = min(all_errors)
        max_err = max(all_errors)
        ax2.set_ylim(min_err - 0.5, max_err + 0.5)

    return trajectory_line, vehicle_marker, cte_line, yaw_error_line, lookahead_marker

# --- 6. 主函数 ---
def main():
    print("开始路径跟踪 (Pure Pursuit) 动态仿真 [模块化架构]...")
    
    ani = animation.FuncAnimation(
        fig, 
        animate, 
        frames=max_steps, 
        interval=10,  # 10ms (10倍速)
        blit=True, 
        repeat=False
    )
    
    plt.show()
    print("仿真结束。")

if __name__ == '__main__':
    main()