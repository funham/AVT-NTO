import numpy as np

MAX_ANGLE: float = 45
MAX_SPEED: int = 255
MAX_DEVIATION: float = 5
TARGET_DISTANCE = np.inf

k_dev = MAX_ANGLE / MAX_DEVIATION
k_break = .5

angle, speed = 0, 0

def direct(deviation: float) -> tuple[int, int]:
    global angle, speed 
    
    angle = int(deviation * k_dev)
    speed = int(MAX_SPEED - angle * k_break)

    # TODO PID smoothing
    return angle, speed

