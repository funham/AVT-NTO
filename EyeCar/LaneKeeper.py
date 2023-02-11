import cv2
import numpy as np
import cfg

from dataclasses import dataclass


@dataclass
class Lane:
    deviation: float
    distance_travelled: int
    crossroad_distance: int


class LaneKeeper:
    dist: float = 0.0

    def forward(self, frame: cv2.Mat) -> Lane:
        layout = self._get_layout(frame)
        deviation = self._calc_deviation(layout)
        self.dist += self._calc_distance_increment(layout)

        print(f'{deviation=:.2f}')

        return Lane(deviation=deviation,
                    distance_travelled=self.dist,
                    crossroad_distance=np.inf)

    def _calc_distance_increment(self, layout: cv2.Mat) -> float:
        """
        Calculates the distance travelled by the car since last frame.
        """
        return 0.0  # not implemented

    def _calc_deviation(self, layout: cv2.Mat) -> float:
        """
        Calculates the deviation of the car course from the center of the road.
        returns values from -1.0 to 1.0, where 0.0 is the center of the road.
        """
        hist = layout.sum(axis=0)
        mid = hist.size // 2
        ldev = 1 - hist[:mid].argmax() / mid
        rdev = hist[mid:].argmax() / mid

        if cfg.DEBUG:
            layout_lines = cv2.cvtColor(layout, cv2.COLOR_GRAY2BGR)
            layout_lines[:, mid - int(ldev * mid)] = (255, 0, 0)
            layout_lines[:, mid + int(rdev * mid)] = (255, 0, 0)
            layout_lines[:, mid] = (255, 0, 0)

            cv2.imshow('layout lines', layout_lines)

            print(f'{ldev=}, {rdev=}')

        return rdev - ldev

    def _get_layout(self, img: cv2.Mat) -> cv2.Mat:
        # perspective transformation
        flat_view = self._get_flat_view(img)

        # binarization
        hsv = cv2.cvtColor(flat_view, cv2.COLOR_BGR2HSV)
        layout = cv2.inRange(hsv, (0, 0, 200), (180, 150, 255))

        if cfg.DEBUG:
            cv2.imshow('flat view', flat_view)

        return layout

    def _get_crossroad_distance(self, layout: cv2.Mat) -> float:
        """
        Calculates the distance to the nearest crossroad.
        """

        return np.inf  # not implemented

    def _get_flat_view(self, img: cv2.Mat) -> cv2.Mat:
        toffset = 0.13    # offset of the transformation, from -1.0 to 1.0, where 0.0 is no offset
        boffset = 0.171   # offset of the transformation, from -1.0 to 1.0, where 0.0 is no offset
        margin = 0.1      # bottom margin of the transformation
        height = 0.5      # height of the perspective
        bwidth = 0.4      # width of the bottom line of the perspective, relative to the image width, 1.0 is full width
        twidth = 0.17     # width of the top line of the perspective, relative to the image width, 1.0 is full width
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
            lpts = input_pts.reshape((-1, 1, 2)).astype(np.int32)
            lines = cv2.polylines(img, [lpts],
                                  True, (255, 0, 0), 2)
            cv2.imshow('lines', lines)

        return flat_view
