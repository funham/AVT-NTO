"""Config file of this project"""

import numpy as np
from include.io_client import InputType


INPUT_MODE = InputType.VIDEO_PLAYER
DEBUG = True

IMG_W = 350
IMG_H = 200
IMG_SHAPE = (IMG_W, IMG_H)
FPS = 10

CAR_MAX_ANGLE: float = 45
CAR_MAX_SPEED: int = 255
CAR_MIN_SPEED: int = 100
MAX_DEVIATION: float = 1
PIXEL_TO_CM_RATIO: float = 0.05

CROSSROAD_STOP_DIST: float = 6.5
CROSSROAD_SLOW_DOWN_DIST: float = 15
CROSSROAD_STOP_TIME: float = 1.0
CROSSROAD_SPEED_LIMIT: float = 50

PEDESTRIAN_STOP_DISTANCE: float = 10
PEDESTRIAN_SLOW_DOWN_DISTANCE: float = 20
PEDESTRIAN_SPEED_LIMIT: float = 50

CMD_BUFF_LEN = 2

UDP_HOST = "0.0.0.0"
UDP_PORT = 5000
# TCP_HOST = "192.168.0.12"
TCP_HOST = "192.168.17.46"
TCP_PORT = 5001

DEFAULT_VIDEO_SOURCE_PATH = 'EyeCar-server/res/videos'
DEFAULT_VIDEO_SOURCE_FILE = 'FTL_full.mkv'
DEFAULT_IMG_SOURCE_FOLDER_PATH = 'EyeCar-server/res/images'
DEFAULT_MODEL_FOLDER = ''

# toffset, boffset, margin, height, bwidth, twidth, wscale
# PERSPECTIVE_TRANSFORM_PARAMS = (0.13, 0.171, 0.1, 0.5, 0.4, 0.17, 2.0)
PERSPECTIVE_TRANSFORM_PARAMS =  (-0.1, -0.1,   0.1, 0.7, 0.4, 0.1, 3.5)
