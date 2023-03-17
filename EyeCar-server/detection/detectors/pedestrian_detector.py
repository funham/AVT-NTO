"""
Implementation of Yolo model detectors
"""

import cfg
import cv2
import numpy as np

from include.vid_writer import VideoWriter
from detection.detectors.yolo_detector import YoloV4Detector


class PedestrianDetector(YoloV4Detector):
    def __init__(self, path):
        super().__init__(path)

    def forward(self, frame: cv2.Mat) -> dict:
        classes, scores, boxes = self.model.detect(frame)

        if len(boxes) == 0:
            VideoWriter().write('detection', frame)
            return {}

        pedestrians = []
        for box in boxes:
            x, y, w, h = box

            detected = frame.copy()

            cv2.rectangle(detected, (x, y), (x+w, y+h), (0, 255, 0), 2, cv2.LINE_AA)
            VideoWriter().write('detection', detected)

            dist = frame.shape[0] - y - h
            offset = (x + w // 2) - frame.shape[1] // 2

            if dist < cfg.PEDESTRIAN_STOP_DISTANCE and offset > cfg.PEDESTRIAN_SIDEWALK_OFFSET:
                dist = np.inf


            pedestrians.append(dist)

        return {'pedestrians': [{'dist' : dist, 'offset': off} for dist, off in pedestrians]}
