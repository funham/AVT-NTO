import cfg
import cv2
import math
import numpy as np


class Rect:
    def __init__(self, cnt):
        self._rect = cv2.minAreaRect(cnt)
        self.w = self._rect[1][0]
        self.h = self._rect[1][1]
        self.x = self._rect[0][0]
        self.y = self._rect[0][1]
        self.angle = self._rect[2]
        
    @property
    def center(self):
        return self._rect[0]
    
    @property
    def shape(self):
        return self._rect[1]

    @property
    def area(self):
        return self.w * self.h
    
    @property
    def boxpoints(self):
        # The 4 corner points are ordered clockwise starting from the point with the highest 
        return np.int0(cv2.boxPoints(self._rect))


class LayoutSegment:
        APPROX_W = 10
        APPROX_H = 65
        
        def __init__(self, bbox: Rect):
            self.box = bbox
            self.top: int = bbox.boxpoints[0][1]
            self.bottom: int = bbox.boxpoints[2][1]
        
        @property
        def touches_bottom(self):
            return self.bottom >= cfg.IMG_H
        
        @property
        def is_correct(self):
            return self.box.area <= 1000
        

class BrokenLineTracker:
    def __init__(self):
        self.distance_travelled = 0.0
        self._prev_segments: list[LayoutSegment] = []

    def get_distance_travelled(self, layout: cv2.Mat, out_img: cv2.Mat) -> float:
        assert len(layout.shape) == 2

        h, w = layout.shape
        broken_line_image = layout[h//2:, :w//2]

        cnts, _ = cv2.findContours(broken_line_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        bboxs = map(Rect, cnts)
        bboxs = sorted(bboxs, key=lambda bbox: bbox.y, reverse=True)[:2]
        curr_segments = list(map(LayoutSegment, bboxs))

        if len(bboxs) < 2 or not all([seg.is_correct for seg in curr_segments]):
            return 0.0  # couldn't find two bounding boxes, so assume no change in distance.

        if not self._prev_segments:
            self._prev_segments = curr_segments            
            
            return 0.0  # no previous frames

        # calculating tracking points coordinate difference
        # 0th index stands for bottom segment, and 1st index stands for top segment
        if self._prev_segments[0].touches_bottom and not curr_segments[0].touches_bottom:
            inc = curr_segments[0].top - self._prev_segments[0].top
        else:
            inc = curr_segments[0].bottom - self._prev_segments[1].bottom

        if cfg.DEBUG:
            for bbox in bboxs:
                cv2.drawContours(out_img, [np.array(bbox.boxpoints) + [0, h//2]], 0, (0, 255, 0), 2)
            

        self.distance_travelled += inc * cfg.PIXEL_TO_CM_RATIO

        return self.distance_travelled
    