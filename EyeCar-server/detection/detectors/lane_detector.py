"""
Detector of a Lane.

Extracts all the information about lane such as deviation of the course,
tracking travelled distance, crossroad distance etc.  
"""

import cv2
import cfg
from detection.detection import IDetector

from detection.detectors.lane_helpers.brokenline_tracking import BrokenLineTracker
from detection.detectors.lane_helpers.perspective_transformation import PerspectiveTransformation
from detection.detectors.lane_helpers.thresholding import Thresholder
from detection.detectors.lane_helpers.stopline import StoplineDetector
from detection.detectors.lane_helpers.lane_lines import LaneLines


class LaneDetector(IDetector):
    def __init__(self) -> None:
        self.stopline_detector = StoplineDetector()
        self.thresholder = Thresholder()
        self.bline_tracker = BrokenLineTracker()
        self.curvature_computer = LaneLines()

        self.perspective_transformer = PerspectiveTransformation(cfg.IMG_SHAPE, (320, 400), *cfg.PERSPECTIVE_TRANSFORM_PARAMS)

    def get_curvature_and_deviation(self, frame: cv2.Mat) -> dict:
        flat_view = self.perspective_transformer(frame)
        layout = self.thresholder(flat_view)

        curvature, deviation = self.curvature_computer.get_curvature_and_deviation(layout)
        distance_travelled = self.bline_tracker.get_distance_travelled(layout)
        crossroad_distance = self.stopline_detector.get_stopline_distance(layout)

        return {'lane_curvature': curvature,
                'lane_deviation': deviation,
                'distance_travelled': distance_travelled,
                'crossroad_distance': crossroad_distance}
