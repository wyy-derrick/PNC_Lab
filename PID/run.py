import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from simulation import BicycleModel, Path

# --- 1. 设置中文显示 ---
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False  

# --- 2. 控制器类 (包含所有算法逻辑) ---
class Controller:
    """s
    实现纵向P控制器 + 横向纯跟踪(Pure Pursuit)控制器
    """
    def __init__(self, Kp_v, Ki_v, Kd_v, Kp_v_hen, target_speed, L, L_lookahead, path_data):
        # 纵向PI控制器（速度）
        self.Kp_v = Kp_v
        self.Ki_v = Ki_v
        self.Kd_v = Kd_v
        self.target_speed = target_speed

        # 横向纯跟踪控制器（转向）
        self.Kp_v_hen = Kp_v_hen  # 横向控制增益
        self.L = L  # 车辆轴距 (m)
        self.L_lookahead = L_lookahead # 基础前瞻距离 (m)
        
        # --- 核心修改 ---
        # 控制器存储它需要的所有路径信息
        self.path_x_raw = path_data.x
        self.path_y_raw = path_data.y
        self.path_yaw_raw = path_data.yaw
        self.path_x_wrapped = path_data.x_wrapped
        self.path_y_wrapped = path_data.y_wrapped

        # 积分项初始化
        self.v_error_integral = 0.0
        self.previous_v_error = 0.0
        self.dt = 0.02

        

    def normalize_angle(self, angle):
        """将角度归一化到 [-pi, pi]"""
        while angle > np.pi: angle -= 2 * np.pi
        while angle < -np.pi: angle += 2 * np.pi
        return angle

    # --- 新增的内部方法 ---
    def _find_closest_point(self, x, y):
        """
        找到路径上离车辆当前位置最近的点
        """
        dx = self.path_x_raw - x
        dy = self.path_y_raw - y
        distances = np.sqrt(dx**2 + dy**2)
        closest_index = np.argmin(distances)
        return closest_index, distances[closest_index]

    def _find_lookahead_point(self, x, y, Ld):
        """
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
        计算控制量：油门（加速度）和前轮转角
        """
        x, y, yaw, v = state
        
        # --- 1. 纵向控制 (P-Controller for Velocity) ---

        # 计算速度误差
        v_error = self.target_speed - v
        # 计算误差积分
        self.v_error_integral += v_error * self.dt

        # 计算误差微分 
        self.v_error_derivative = (v_error - self.previous_v_error) / self.dt
        self.previous_v_error = v_error

         
        throttle = self.Kp_v * v_error + self.Ki_v * self.v_error_integral + self.Kd_v * self.v_error_derivative
        
        # --- 2. 横向控制 (Pure Pursuit) ---
        Ld = self.L_lookahead
        
        # 调用自己的内部方法
        tx, ty = self._find_lookahead_point(x, y, Ld)
        
        alpha = np.arctan2(ty - y, tx - x) - yaw  #计算即车辆需要转过的角度
        alpha = self.normalize_angle(alpha)   #归一化角度
        #用p控，计算转角
        delta = self.Kp_v_hen * alpha
        
        return throttle, delta






# --- 3. use the simulation env ---


#parameters   
KP_V = 0.5
KI_V = 0.0
KD_V = 0.0

KP_V_HEN = 2.0
KI_V_HEN = 0.0
KD_V_HEN = 0.0

TARGET_SPEED = 10.0
# --- MODIFIED ---
# L_LOOKAHEAD 必须大于 L (5.5)。 3.0 太小会导致震荡。
L_LOOKAHEAD = 10.0 

# --- 初始化环境和智能体 ---
# --- MODIFIED --- 
# 1. 必须先创建"地图"，才能获取地图的起始点信息
path = Path(amplitude=50.0, num_points=500) # MODIFIED: 增加点数 (100 -> 500) 提高路径平滑度

# 2. 创建"世界"和"车辆"，使用地图的起始朝向
vehicle = BicycleModel(x=0.0, y=0.0, yaw=path.yaw[0], v=0.0, L=5.5, dt=0.02) # MODIFIED: yaw=np.radians(90) -> yaw=path.yaw[0]


# 3. 创建"控制器"，并把"地图" (path) 交给它
controller = Controller(Kp_v=KP_V, 
                        Ki_v=KI_V,
                        Kd_v=KD_V,
                        Kp_v_hen=KP_V_HEN,
                        target_speed=TARGET_SPEED, 
                        L=vehicle.L,
                        L_lookahead=L_LOOKAHEAD,
                        path_data=path) 
# (后续的仿真和绘图设置与之前相同)
T_SIM = 200  
max_steps = int(T_SIM / vehicle.dt)
history = {'x': [], 'y': [], 'v': [], 'cte': [], 'yaw_error': [], 'time': []}

# --- 4. 设置画布 ---
fig, ((ax1, ax3), (ax2, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
# 图1: 路径图
ax1.plot(path.x, path.y, 'b--', label='参考路径 (8字形)')
ax1.plot(0.0, 0.0, 'go', markersize=10, label='起点')
trajectory_line, = ax1.plot([], [], 'r-', label='车辆实际轨迹')
vehicle_marker, = ax1.plot([], [], 'ro', label='车辆当前位置')
lookahead_marker, = ax1.plot([], [], 'mo', markersize=8, label='前瞻点')
ax1.set_title('纯跟踪 (Pure Pursuit) 动态仿真 (模块化)')
ax1.set_xlabel('X 坐标 (m)')
ax1.set_ylabel('Y 坐标 (m)')
ax1.legend(loc='upper right')
ax1.axis('equal')
ax1.grid(True)

# 图2: 横向跟踪误差图
cte_line, = ax2.plot([], [], 'g-', label='横向跟踪误差 (m)')
ax2.set_title('横向跟踪误差-时间图')
ax2.set_xlabel('时间 (s)')
ax2.set_ylabel('横向跟踪误差 (m)')
ax2.legend(loc='upper right')
ax2.grid(True)
ax2.set_xlim(0, T_SIM)
# --- MODIFIED --- (解决 "画的太慢" 问题)
# 设定一个固定的Y轴范围。-5到+5米的误差范围足够调试了。
# 动态修改ylim会导致blit=True失效，使动画变慢。
ax2.set_ylim(-5.0, 5.0) 

# --- MODIFIED --- 
# 图3: 航向误差图 (修正标题和标签，原为"纵向误差")
yaw_error_line, = ax3.plot([], [], 'm-', label='航向误差 (rad)') # MODIFIED: 标签文本
ax3.set_title('航向误差-时间图') # MODIFIED: 标题
ax3.set_xlabel('时间 (s)')
ax3.set_ylabel('航向误差 (rad)') # MODIFIED: Y轴标签
ax3.legend(loc='upper right')
ax3.grid(True)
ax3.set_xlim(0, T_SIM) # <--- MODIFIED: 修正拼写错误 (T_SAM -> T_SIM)
# --- MODIFIED --- (解决 "画的太慢" 问题)
# 设定一个固定的Y轴范围。-pi/2 到 +pi/2 (即-90°到+90°) 足够调试了。
ax3.set_ylim(-np.pi/2, np.pi/2)

# --- MODIFIED ---
# 图4: 速度图 (添加目标速度参考线，并调整Y轴范围)
speed_line, = ax4.plot([], [], 'b-', label='车辆速度 (m/s)')
ax4.axhline(y=TARGET_SPEED, color='g', linestyle='--', label=f'目标速度 ({TARGET_SPEED} m/s)') # NEW: T
ax4.set_title('车辆速度-时间图')
ax4.set_xlabel('时间 (s)')
ax4.set_ylabel('车辆速度 (m/s)')
ax4.legend(loc='upper right')
ax4.grid(True)
ax4.set_xlim(0, T_SIM) # <--- MODIFIED: 修正拼写错误 (T_SAM -> T_SIM)
# --- MODIFIED --- (解决 "速度图不画了" 问题)
# 你的车辆模型 (simulation.py) 中 clip 上限是 10.0
# 你之前的 ylim (0, 5.0) 太小了，导致速度超过5.0时"看不见"
ax4.set_ylim(0, 10.5) # MODIFIED: 调整Y轴范围 (0-5.0 -> 0-10.5)
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
    Ld = controller.L_lookahead
    tx, ty = controller._find_lookahead_point(current_x, current_y, Ld)
    lookahead_marker.set_data([tx], [ty])
    
    cte_line.set_data(history['time'], history['cte'])
    yaw_error_line.set_data(history['time'], history['yaw_error'])
    speed_line.set_data(history['time'], history['v'])
    
    # --- MODIFIED --- (解决 "画的太慢" 问题)
    # 删除了动态更新 ax2 和 ax3 ylim 的代码块
    if i > 10:
        # 动态更新 ylim 会导致 blit=True 性能下降，使动画变慢
        # 我们在上面设置画布时已指定了固定的Y轴范围
        pass
        
        # # (已删除) 更新横向误差图的y轴范围
        # min_cte = min(history['cte'])
        # ...
        
        # # (已删除) 更新航向误差图的y轴范围
        # min_yaw_error = min(history['yaw_error'])
        # ...

    # 检查是否是最后一帧
    if i == max_steps - 1:
        print("仿真完成，自动关闭窗口...")
        plt.close()

    # --- 唯一修改 ---
    # 必须返回所有被 blit=True 更新的 "艺术家" (Artist)
    # 之前你忘记了 speed_line, 导致 ax4 不更新
    return trajectory_line, vehicle_marker, cte_line, yaw_error_line, lookahead_marker, speed_line

# --- 6. 主函数 ---
def main():
    print("开始路径跟踪 (Pure Pursuit) 动态仿真...")


    
    ani = animation.FuncAnimation(
        fig, 
        animate, 
        frames=max_steps, 
        # --- MODIFIED --- 
        # 500ms 太慢了 (25倍慢放)。 
        # dt=0.02s = 20ms, 所以 interval=20 对应 1x 实时播放
        interval=2,  # 你的 interval=2, 保持不变
        blit=True, 
        repeat=False
    )
    
    
    plt.show()
    print("仿真结束。")

if __name__ == '__main__':
    main()