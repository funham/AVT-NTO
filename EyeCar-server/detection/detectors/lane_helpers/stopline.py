import cfg
import cv2
import numpy as np


class StoplineDetector:
    def __init__(self):
        ...

    def get_stopline_distance(self, layout: cv2.Mat, out_img: cv2.Mat) -> float:
        """Calculates the distance to the nearest crossroad."""
        assert len(layout.shape) == 2

        h, w = layout.shape


        stopline_h, stopline_w= 25, 150
        
        y0 = int(h - h / 2)

        x1 = (w - stopline_w) // 2
        x2 = x1 + stopline_w

        hist = layout[y0:, w//2:x2].sum(axis=1) # vertical histogram of a layout
        maxi = hist.argmax() # index of a brightest row in on the layout, supposedly a crossroad.
        
        # if the brightest row is not bright enough to be a crossroad then no crossroad detected
        if hist[maxi] / 255 < 20:
            print(hist[maxi] / 255)
            return np.inf
        
        if cfg.DEBUG:
            y1 = maxi + y0 - stopline_h // 2
            y2 = y1 + stopline_h
            
            print((x1, y1), (x2, y2))

            cv2.rectangle(out_img, (x1, y1), (x2, y2), (255, 255, 0), 2)
        
        dist = (h - y0 - maxi) * cfg.PIXEL_TO_CM_RATIO
        return dist