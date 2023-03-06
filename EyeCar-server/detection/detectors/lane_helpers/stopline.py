import cfg
import cv2
import numpy as np


class StoplineDetector:
    def __init__(self):
        ...

    def get_stopline_distance(self, layout: cv2.Mat) -> float:
        """Calculates the distance to the nearest crossroad."""
        assert len(layout.shape) == 2

        # TODO implement for curved lines
        h, w = layout.shape
        hist = layout.sum(axis=1) # vertical histogram of a layout
        maxi = hist.argmax() # index of a brightest row in on the layout, supposedly a crossroad.
        
        # if the brightest row is not bright enough to be a crossroad then no crossroad detected
        if hist[maxi] < 15000:
            if cfg.DEBUG:
                cv2.imshow('stopline', layout)
            return np.inf
        
        if cfg.DEBUG:
            stopline = cv2.cvtColor(layout, cv2.COLOR_GRAY2BGR)
            stopline = cv2.line(stopline, (0, maxi), (w, maxi), (255, 255, 0), 2)
            cv2.imshow('stopline', stopline)

        
        dist = (h - maxi) * cfg.PIXEL_TO_CM_RATIO
        return dist