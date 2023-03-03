"""
Detector of a Lane.

Extracts all the information about lane such as deviation of the course,
tracking travelled distance, crossroad distance etc.  
"""

import cv2
import cfg
from detection.detection import IDetector

from detection.lane_helpers.brokenline_tracking import BrokenLineTracker
from detection.lane_helpers.perspective_transformation import PerspectiveTransformation
from detection.lane_helpers.thresholding import Thresholder
from detection.lane_helpers.stopline import StoplineDetector
from detection.lane_helpers.curvature import LineCurvature


class LaneDetector(IDetector):
    def __init__(self) -> None:
        self.stopline_detector = StoplineDetector()
        self.thresholder = Thresholder()           
        self.bline_tracker = BrokenLineTracker()
        self.curvature_computer = LineCurvature()
        self.perspective_transformer = PerspectiveTransformation(in_size=cfg.IMG_SHAPE,
                                                                 out_size=(320, 320),
                                                                 toffset = 0.13,
                                                                 boffset = 0.171,
                                                                 margin = 0.1,   
                                                                 height = 0.5,   
                                                                 bwidth = 0.4,   
                                                                 twidth = 0.17,  
                                                                 wscale = 2.0)
        
    def forward(self, frame: cv2.Mat) -> dict:
        flat_view = self.perspective_transformer(frame)
        layout = self.thresholder(flat_view)

        deviation = self.curvature_computer.get_deviation(layout)
        distance_travelled = self.bline_tracker.get_distance_travelled(layout)
        crossroad_distance = self.stopline_detector.get_stopline_distance(layout)

        return {'lane_deviation': deviation, 
                'distance_travelled': distance_travelled, 
                'crossroad_distance': crossroad_distance}
