import cfg
import cv2


class BrokenLineTracker:
    class LayoutSegment:
        def __init__(self, bbox):
            x, y, w, h = bbox
            self.top: int = y
            self.bottom: int = y + h
        
        @property
        def touches_bottom(self):
            return self.bottom == cfg.IMG_H
    
    def __init__(self):
        self.distance_travelled = 0.0
        self._prev_segments: list[self.LayoutSegment] = []

    def get_distance_travelled(self, layout: cv2.Mat) -> float:
        h, w = layout.shape
        broken_line_image = layout[:, :w//2]

        cnts, _ = cv2.findContours(broken_line_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        bboxs = map(cv2.boundingRect, cnts)
        bboxs = sorted(bboxs, key=lambda bbox: bbox[1], reverse=True)[:2]
        
        if len(bboxs) < 2:
            if cfg.DEBUG:
                cv2.imshow('broken line', layout[:, :w//2])
            return 0.0  # couldn't find two bounding boxes, so assume no change in distance.
        
        curr_segments = list(map(self.LayoutSegment, bboxs))

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
                x, y, w, h = bbox
                broken_line_boxes = cv2.rectangle(broken_line_boxes, (x, y), (x + w, y + h), (255, 0, 255), 2)
            
            cv2.imshow('broken line', cv2.resize(broken_line_boxes, (0, 0), fx=1, fy=1))

        self.distance_travelled += inc * cfg.PIXEL_TO_CM_RATIO

        return self.distance_travelled
    