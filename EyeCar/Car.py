import cfg
from typing import *
from simple_pid import PID

k_dev = cfg.MAX_ANGLE / cfg.MAX_DEVIATION
k_break = .5

speed, angle = 0, 0

pid = PID(1, 0.1, 0.05, setpoint=0)

def constrain(x, min_x, max_x):
    return max(min(x, max_x), min_x)

def calc_params(deviation: float) -> Tuple[int, int]:
    global speed, angle
    
    angle = int(pid(deviation * k_dev))
    speed = int(cfg.MAX_SPEED - abs(angle) * k_break)

    speed = constrain(speed, cfg.MIN_SPEED, cfg.MAX_SPEED)
    angle = constrain(angle, -cfg.MAX_ANGLE, cfg.MAX_ANGLE)

    return speed, angle
