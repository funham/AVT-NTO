"""
Detector of a Lane.

Extracts all the information about lane such as deviation of the course,
tracking travelled distance, crossroad distance etc.  
"""

import cv2
import cfg
import numpy as np

from detection.detection import IDetector

from detection.detectors.lane_helpers.brokenline_tracking import BrokenLineTracker
from detection.detectors.lane_helpers.perspective_transformation import PerspectiveTransformation
from detection.detectors.lane_helpers.thresholding import Thresholder
from detection.detectors.lane_helpers.stopline import StoplineDetector
from detection.detectors.lane_helpers.lane_lines import LaneLines


class RoadDetector(IDetector):
    def __init__(self) -> None:
        self.stopline_detector = StoplineDetector()
        self.thresholder = Thresholder()
        self.broken_line_tracker = BrokenLineTracker()
        self.lane_lines = LaneLines()

        self.perspective_transformer = PerspectiveTransformation(
            cfg.IMG_SHAPE, (320, 400), *cfg.PERSPECTIVE_TRANSFORM_PARAMS)

    def forward(self, frame: cv2.Mat) -> dict:
        flat_view = self.perspective_transformer(frame)
        layout = self.thresholder(flat_view)

        out_img = np.dstack((layout, layout, layout))

        deviation = self.lane_lines.get_deviation_only_right(layout, out_img)
        distance_travelled = self.broken_line_tracker.get_distance_travelled(layout, out_img)
        crossroad_distance = self.stopline_detector.get_stopline_distance(layout, out_img)

        cv2.imshow('Road lines', out_img)

        return {'lane_deviation': deviation,
                'distance_travelled': distance_travelled,
                'crossroad_distance': crossroad_distance}
