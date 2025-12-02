import math
from .types import Point

def normalize_angle(angle):
    """将角度归一化到 [-pi, pi]"""
    return (angle + math.pi) % (2 * math.pi) - math.pi

def dist(p1: Point, p2: Point):
    """计算两点间欧几里得距离"""
    return math.hypot(p1.x - p2.x, p1.y - p2.y)
