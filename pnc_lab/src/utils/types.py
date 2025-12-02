from dataclasses import dataclass

@dataclass
class Point:
    x: float
    y: float

@dataclass
class State:
    x: float
    y: float
    v: float
    yaw: float
