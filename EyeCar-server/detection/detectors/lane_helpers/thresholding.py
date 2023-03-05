import cfg
import cv2


class Thresholder:
    def __init__(self):
        ...

    def __call__(self, frame: cv2.Mat) -> cv2.Mat:
        """Returns a binary image where lines are white and background is black"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        layout = cv2.inRange(hsv, (0, 0, 240), (180, 150, 255))

        if cfg.DEBUG:
            cv2.imshow('layout', layout)


        return layout