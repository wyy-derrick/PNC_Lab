import numpy as np

# --- 仿真器和车辆模型 ---
class BicycleModel:
    """
    实现一个运动学自行车模型
    """
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, L=10.5, dt=0.1):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.L = L  # 车辆轴距 (m)
        self.dt = dt # 时间步长 (s)

    def update(self, throttle, delta):
        delta = np.clip(delta, -np.radians(30), np.radians(30))
        self.x += self.v * np.cos(self.yaw) * self.dt
        self.y += self.v * np.sin(self.yaw) * self.dt
        self.yaw += (self.v / self.L) * np.tan(delta) * self.dt
        self.v += throttle * self.dt
        self.v = np.clip(self.v, 0, 10) 

    def get_state(self):
        return self.x, self.y, self.yaw, self.v

# --- 路径定义 ---
class Path:
    """
    
    """
    def __init__(self, amplitude=50.0, num_points=500):
        t = np.linspace(0, 2 * np.pi, num_points)
        
        # 基础路径点
        self.x = amplitude * np.sin(t)
        self.y = amplitude * np.sin(t) * np.cos(t)
        
        # 路径航向
        dx_dt = amplitude * np.cos(t)
        dy_dt = amplitude * (np.cos(t)**2 - np.sin(t)**2)
        self.yaw = np.arctan2(dy_dt, dx_dt)
        self.num_points = num_points

        # 为控制器准备的“环形”路径数据，方便搜索
        self.x_wrapped = np.append(self.x, self.x)
        self.y_wrapped = np.append(self.y, self.y)