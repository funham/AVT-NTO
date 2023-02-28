import cv2
import numpy as np
import cfg
from Detector import IDetector


class LaneDetector(IDetector):
    def forward(self, frame: cv2.Mat) -> dict:
        layout = self._get_layout(frame)
        deviation = self._calc_deviation(layout)
        self._dist += self._calc_distance_increment(layout)
        crossroad_dist = self._get_crossroad_distance(layout)

        print(f'{deviation=:.2f}')
        print(f'distance travelled: {self._dist:.2f}cm')
        if crossroad_dist < np.inf:
            print(f'crossroad distance: {crossroad_dist:.2f}cm')
        else:
            print('no crossroad detected')

        return {'lane_deviation': deviation, 
                'distance_travelled': self._dist, 
                'crossroad_distance': crossroad_dist}
        
    def _calc_deviation(self, layout: cv2.Mat) -> float:
        """
        Calculates the deviation of the car course from the center of the road.
        returns values from `-1.0` to `1.0`, where `0.0` is the center of the road.
        """
        hist = layout.sum(axis=0)
        imid = hist.size // 2

        # relative deviation of a left and right layout lines from center of an image
        ldev = 1 - hist[:imid].argmax() / imid
        rdev = hist[imid:].argmax() / imid

        deviation = rdev - ldev  # if lines are equally distanced from center then deviation is zero 

        if cfg.DEBUG:
            h, w = layout.shape
            lx, rx = imid - int(ldev * imid), imid + int(rdev * imid)
            layout_lines = cv2.cvtColor(layout, cv2.COLOR_GRAY2BGR)
            layout_lines = cv2.line(layout_lines, (lx, 0), (lx, h), (0, 0, 255), 2)
            layout_lines = cv2.line(layout_lines, (rx, 0), (rx, h), (0, 0, 255), 2)
            layout_lines = cv2.line(layout_lines, (imid, 0), (imid, h), (255, 255, 0), 2)

            cv2.imshow('layout lines', layout_lines)

        return deviation

    def _get_layout(self, img: cv2.Mat) -> cv2.Mat:
        flat_view = self._get_flat_view(img)
        hsv = cv2.cvtColor(flat_view, cv2.COLOR_BGR2HSV)

        layout = cv2.inRange(hsv, (0, 0, 200), (180, 150, 255))

        if cfg.DEBUG:
            cv2.imshow('flat view', flat_view)

        return layout

    def _get_flat_view(self, img: cv2.Mat) -> cv2.Mat:
        """Returns a view of the image with the road straightened out."""

        toffset = 0.13    # offset of the transformation, from -1.0 to 1.0, where 0.0 is no offset
        boffset = 0.171   # offset of the transformation, from -1.0 to 1.0, where 0.0 is no offset
        margin = 0.1      # bottom margin of the transformation
        height = 0.25      # height of the perspective
        bwidth = 0.4      # width of the bottom line of the perspective, relative to the image width, 1.0 is full width
        twidth = 0.34     # width of the top line of the perspective, relative to the image width, 1.0 is full width
        wscale = 2.0      # scale of the top and the bottom width parameters

        h, w, _ = img.shape
        
        tl = (w // 2 * (1 + toffset - twidth * wscale), h * (1 - margin - height))  # top left
        bl = (w // 2 * (1 + boffset - bwidth * wscale), h * (1 - margin))           # bottom left
        tr = (tl[0] + w * twidth * wscale, tl[1])  # top right
        br = (bl[0] + w * bwidth * wscale, bl[1])  # bottom right

        out_w, out_h = 200, 200

        input_pts = np.float32([bl, tl, tr, br])
        output_pts = np.float32(
            [[0, out_h-1], [0, 0], [out_w-1, 0], [out_w-1, out_h-1]])
        M = cv2.getPerspectiveTransform(input_pts, output_pts)

        flat_view = cv2.warpPerspective(img, M, (out_w, out_h))

        if cfg.DEBUG:
            p_mid_top = np.int32(((tl[0] + tr[0]) // 2, tl[1]))
            p_mid_bottom = np.int32(((bl[0] + br[0]) // 2, bl[1]))
            lpts = input_pts.reshape((-1, 1, 2)).astype(np.int32)
            lines = cv2.polylines(img, [lpts],
                                  True, (255, 0, 0), 2, cv2.LINE_AA)
            lines = cv2.line(lines, p_mid_bottom, p_mid_top, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.imshow('lines', lines)

        return flat_view

    def _get_crossroad_distance(self, layout: cv2.Mat) -> float:
        """Calculates the distance to the nearest crossroad."""

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

    def _calc_distance_increment(self, layout: cv2.Mat) -> float:
        """
        Calculates the distance travelled by the car since last frame.
        """
        h, w = layout.shape
        broken_line_image = layout[:, :w//2]

        cnts, _ = cv2.findContours(broken_line_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        bboxs = map(cv2.boundingRect, cnts)
        bboxs = sorted(bboxs, key=lambda bbox: bbox[1], reverse=True)[:2]
        
        if len(bboxs) < 2:
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

        return inc * cfg.PIXEL_TO_CM_RATIO
    

    class LayoutSegment:
        def __init__(self, bbox):
            x, y, w, h = bbox
            self.top: int = y
            self.bottom: int = y + h
        
        @property
        def touches_bottom(self):
            return self.bottom == cfg.IMG_H
    
    _dist: float = 0.0
    _prev_segments: list[LayoutSegment] = []