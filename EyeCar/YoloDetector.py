"""
Implementation of Yolo model detectors
"""


import cfg
import cv2

from Detector import IDetector
import YOLO


class YoloV5Detector(IDetector):
    def __init__(self, path):
        self.model = ...

    def forward(self, frame: cv2.Mat) -> dict:
        # TODO implement correct working with YOLO model
        # and return data in dict with a correct structure
        detection = self.model(frame)
        ...
        return detection


class YoloV8Detector(IDetector):
    def __init__(self, path):
        self.model = ...

    def forward(self, frame: cv2.Mat) -> dict:
        detection = self.model(frame)
        ...
        return detection
