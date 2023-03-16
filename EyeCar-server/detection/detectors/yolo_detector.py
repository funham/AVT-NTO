"""
Implementation of Yolo model detectors
"""

import cfg
import cv2
import yolopy

from detection.detection import IDetector
from include.vid_writer import VideoWriter


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

        if len(boxes) == 0:
            VideoWriter().write('parking', frame)
            return {}

        box = boxes[0]
        print(boxes)

        x, y, w, h = box

        detected = frame.copy()

        cv2.rectangle(detected, (x, y), (x+w, y+h), (244, 2, 232), 2, cv2.LINE_AA)
        VideoWriter().write('parking', detected)

        area = w * h

        dist = area * .5

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
