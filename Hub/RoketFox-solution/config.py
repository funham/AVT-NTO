import os
import time
from typing import *
from functools import partial
from itertools import product
import cv2
import numpy as np
import RPi.GPIO as GPIO

TESTING = False
TARGET_MARKING = [[2, 1], [9, 6]]