from .controller_base import ControllerBase
from ..utils.math_utils import dist, normalize_angle
import math

class PurePursuitController(ControllerBase):
    def __init__(self, config):
        self.cfg_pp = config['control']['pure_pursuit']
        self.cfg_pid = config['control']['pid']
        self.target_speed = config['control']['target_speed']
        self.L = config['vehicle']['L']
        self.dt = config['vehicle']['dt']
        self.sum_err = 0.0
        self.prev_err = 0.0

    def compute_command(self, state, path):
        if not path: return 0.0, 0.0, None
        
        # 1. 纵向 PID 控制
        err = self.target_speed - state.v
        self.sum_err += err * self.dt
        acc = (self.cfg_pid['kp'] * err + 
               self.cfg_pid['ki'] * self.sum_err + 
               self.cfg_pid['kd'] * (err - self.prev_err) / self.dt)
        self.prev_err = err

        # 2. 横向 Pure Pursuit 控制
        ld = self.cfg_pp['k'] * state.v + self.cfg_pp['L_min']
        
        # 寻找最近点索引
        dists = [dist(state, p) for p in path]
        min_idx = dists.index(min(dists))
        
        # 寻找预瞄点 (Lookahead Point)
        target = path[-1]
        for i in range(min_idx, len(path)):
            if dist(state, path[i]) >= ld:
                target = path[i]
                break
        
        alpha = normalize_angle(math.atan2(target.y - state.y, target.x - state.x) - state.yaw)
        steer = math.atan2(2.0 * self.L * math.sin(alpha), ld)
        
        return acc, steer, target
