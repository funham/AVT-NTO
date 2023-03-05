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
        
        def is_correct(self):
            if self.touches_bottom:
                return True

            w, h = self.box.shape

            if abs(w - self.APPROX_W) > abs(h - self.APPROX_W):
                w, h = h, w

            diff = abs(w - self.APPROX_W) + abs(h - self.APPROX_H)

            if cfg.DEBUG:
                print(f'{w=:2.2f}, {h=:2.2f}, {diff=:2.2f}, correct: {diff < 70}')

            return diff < 70
        
        # TODO
        @staticmethod
        def fix(seg, img):
            if seg.is_correct():
                return seg
            
            w, h = seg.box.shape

            src = np.float32(seg.box.boxpoints)
            dst = np.float32([[0, 0], [0, w], [h, w], [h, 0]])

            M = cv2.getPerspectiveTransform(src, dst)
            M_inv = cv2.getPerspectiveTransform(dst, src)
            lines = cv2.polylines(img, [np.int32(src)],
                                  True, (255, 0, 0), 2, cv2.LINE_AA)
            # print(img.shape)
            # cv2.imshow('seg persp lines', img)


            # seg_img = cv2.warpPerspective(img, M, np.int0([h, w]), flags=cv2.INTER_LINEAR)
            # cv2.imshow('seg image', cv2.resize(seg_img, (0, 0), fx=5, fy=5))

            return seg


class BrokenLineTracker:
    def __init__(self):
        self.distance_travelled = 0.0
        self._prev_segments: list[LayoutSegment] = []

    def get_distance_travelled(self, layout: cv2.Mat) -> float:
        assert len(layout.shape) == 2

        h, w = layout.shape
        broken_line_image = layout[h//2:, :w//2]

        cnts, _ = cv2.findContours(broken_line_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        bboxs = map(Rect, cnts)
        bboxs = sorted(bboxs, key=lambda bbox: bbox.y, reverse=True)[:2]

        if len(bboxs) < 2:
            if cfg.DEBUG:
                cv2.imshow('broken line', broken_line_image)
            return 0.0  # couldn't find two bounding boxes, so assume no change in distance.
        

        curr_segments = list(map(LayoutSegment, bboxs))
        curr_segments = list(map(LayoutSegment.fix, curr_segments, broken_line_image))


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
            broken_line_boxes = cv2.cvtColor(broken_line_image, cv2.COLOR_GRAY2BGR)
            for bbox in bboxs:
                cv2.drawContours(broken_line_boxes, [bbox.boxpoints], 0, (255, 0, 255), 2)
            
            cv2.imshow('broken line', cv2.resize(broken_line_boxes, (0, 0), fx=1, fy=1))

        self.distance_travelled += inc * cfg.PIXEL_TO_CM_RATIO

        return self.distance_travelled
    