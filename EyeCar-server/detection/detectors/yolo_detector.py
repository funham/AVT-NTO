"""
Implementation of Yolo model detectors
"""

import cfg
import cv2

from detection.detection import IDetector

# TODO implement correct working with YOLO model
# and return data in dict with a correct structure


class YoloV5Detector(IDetector):
    def __init__(self, path):
        cfg.CAR_MAX_ANGLE
        self.model = ...

    def forward(self, frame: cv2.Mat) -> dict:
        return {}


class YoloV8Detector(IDetector):
    def __init__(self, path):
        self.model = ...

    def forward(self, frame: cv2.Mat) -> dict:
        ...
        return {}
