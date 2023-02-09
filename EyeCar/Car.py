import numpy as np
import cfg
from typing import *


k_dev = cfg.MAX_ANGLE / cfg.MAX_DEVIATION
k_break = .5

angle, speed = 0, 0


def calc_params(deviation: float) -> Tuple[int, int]:
    global angle, speed

    angle = int(deviation * k_dev)
    speed = int(cfg.MAX_SPEED - angle * k_break)

    # TODO PID smoothing
