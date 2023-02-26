import numpy as np
from io_client import InputType

INPUT_MODE = InputType.SERVER_CAMERA
DEBUG = True
MODEL_DETECTION = False

MAX_ANGLE: float = 45
MAX_SPEED: int = 255
MIN_SPEED: int = 100
MAX_DEVIATION: float = 1
PIXEL_TO_CM_RATIO: float = 0.05

TARGET_DISTANCE = np.inf
CROSSROAD_STOP_DIST: float = 6.5

CMD_BUFF_LEN = 2

UDP_HOST = "0.0.0.0"
UDP_PORT = 5000
TCP_HOST = "192.168.0.12"
TCP_PORT = 5001

IMG_SOURCE_FOLDER_PATH = r'C:\Users\fun1h\Documents\AVT-NTO\EyeCar\images'