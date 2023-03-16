"""
Implementation of Yolo model detectors
"""

import cfg
import cv2
import yolopy

from detection.detection import IDetector

# TODO implement correct working with YOLO model
# and return data in dict with a correct structure

class YoloV4Detector(IDetector):
    def __init__(self, path):
        self.model = yolopy.Model(path, use_uint8=True, use_timvx=True, cls_num=1)

    def forward(self, frame: cv2.Mat) -> dict:
        ...

class ParkingDetector(YoloV4Detector):
    def __init__(self, path):
        super().__init__(path)

    def forward(self, frame: cv2.Mat) -> dict:
        classes, scores, boxes = self.model.detect(frame)

        box = boxes[0]

        dist = box.area

        return {'parking_dist': dist}




class YoloV5Detector(IDetector):
    def __init__(self, path):
        self.model = ...

    def forward(self, frame: cv2.Mat) -> dict:
        return {}


class YoloV8Detector(IDetector):
    def __init__(self, path):
        self.model = ...

    def forward(self, frame: cv2.Mat) -> dict:
        ...
        return {}
