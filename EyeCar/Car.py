import cfg
from typing import *
from simple_pid import PID

k_dev = cfg.MAX_ANGLE / cfg.MAX_DEVIATION
k_break = .5

angle, speed = 0, 0

pid = PID(1, 0.1, 0.05, setpoint=0)


def calc_params(deviation: float) -> Tuple[int, int]:
    global angle, speed

    angle = int(deviation * k_dev)
    speed = int(cfg.MAX_SPEED - angle * k_break)

    return angle, speed
