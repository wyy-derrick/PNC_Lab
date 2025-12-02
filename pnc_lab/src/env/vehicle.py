import math
from ..utils.types import State
from ..utils.math_utils import normalize_angle

class Vehicle:
    def __init__(self, config):
        self.L = config['vehicle']['L']
        self.max_steer = config['vehicle']['max_steer']
        self.max_acc = config['vehicle']['max_acc']
        self.dt = config['vehicle']['dt']
        # 初始状态将由 reset 或外部设定，这里先初始化为0
        self.state = State(0.0, 0.0, 0.0, 0.0)

    def set_state(self, x, y, v, yaw):
        self.state = State(x, y, v, yaw)

    def update(self, acc, delta):
        """
        acc: 加速度
        delta: 前轮转角
        """
        # 限制输入
        acc = max(-self.max_acc, min(self.max_acc, acc))
        delta = max(-self.max_steer, min(self.max_steer, delta))
        
        # 运动学方程更新
        s = self.state
        s.x += s.v * math.cos(s.yaw) * self.dt
        s.y += s.v * math.sin(s.yaw) * self.dt
        s.yaw += s.v / self.L * math.tan(delta) * self.dt
        s.yaw = normalize_angle(s.yaw)
        s.v += acc * self.dt
        
        return self.state
