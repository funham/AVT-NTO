"""Config file of this project"""

import numpy as np
from include.io_client import InputType


INPUT_MODE = InputType.EYECAR
DEBUG = False
MOVE = True

IMG_W = 350
IMG_H = 200
IMG_SHAPE = (IMG_W, IMG_H)
FPS = 60

CAR_MAX_ANGLE: float = 45
CAR_MAX_SPEED: int = 1430 * MOVE
CAR_MIN_SPEED: int = 1430 * MOVE
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

ARDUINO_PORT = '/dev/ttyUSB0'
CAMERA_ID = '/dev/video0'
UDP_HOST = "0.0.0.0"
UDP_PORT = 5000
# TCP_HOST = "192.168.0.12"
TCP_HOST = "192.168.17.46"
TCP_PORT = 5001

DEFAULT_VIDEO_SOURCE_PATH = 'res/videos'
DEFAULT_VIDEO_SOURCE_FILE = 'FTL_full.mkv'
DEFAULT_IMG_SOURCE_FOLDER_PATH = 'res/images'
DEFAULT_MODEL_FOLDER = ''

# toffset, boffset, margin, height, bwidth, twidth, wscale
# PERSPECTIVE_TRANSFORM_PARAMS =( 0.13, 0.171, 0.10, 0.50, 0.40, 0.17, 2.00)
PERSPECTIVE_TRANSFORM_PARAMS =  (-0.10, -0.10, 0.15, 0.60, 0.70, 0.40, 1.20)

# (w, h)
LANE_WINDOW_SIZE = (50, 15)
LINE_NORMAL_DEVIATION = 80
