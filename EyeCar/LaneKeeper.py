import cv2
import numpy as np

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

        return rdev - ldev

    def _get_layout(self, img: cv2.Mat) -> cv2.Mat:
        # perspective transformation
        w, h, _ = img.shape
        out_w, out_h = 200, 200
        bx, by = w * .2, h * .9
        tx, ty = w * .3, h * .6

        input_pts = np.float32([[bx, by], [tx, ty], [w-tx, ty], [w-tx, h-ty]])
        output_pts = np.float32(
            [[0, out_h-1], [0, 0], [out_w-1, 0], [out_w-1, out_h-1]])
        M = cv2.getPerspectiveTransform(input_pts, output_pts)
        flat_view = cv2.warpPerspective(img, M, (out_w, out_h))

        # color processing
        hsv = cv2.cvtColor(flat_view, cv2.COLOR_BGR2HSV)
        layout = cv2.inRange(flat_view, (0, 0, 100), (180, 150, 255))

        return layout
