"""
Implementation of Yolo model detectors
"""

import cfg
import cv2
import math

from include.vid_writer import VideoWriter
from detection.detectors.yolo_detector import YoloV4Detector


class ParkingDetector(YoloV4Detector):
    def __init__(self, path):
        super().__init__(path)

    def forward(self, frame: cv2.Mat) -> dict:
        classes, scores, boxes = self.model.detect(frame)

        if len(boxes) == 0:
            VideoWriter().write('detection', frame)
            return {}

        box = boxes[0]
        print(boxes)

        x, y, w, h = box

        detected = frame.copy()

        cv2.rectangle(detected, (x, y), (x+w, y+h), (244, 2, 232), 2, cv2.LINE_AA)
        VideoWriter().write('detection', detected)

        area = w * h
        dist = math.sqrt(area) * .5

        return {'parking_dist': dist}
