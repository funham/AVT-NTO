import cv2
import numpy as np
from itertools import combinations
from typing import *


class FaceContour:
    def __init__(self, contour: np.ndarray):
        self.cnt = contour
        self._excluded = False

    @property
    def center(self):
        x, y, w, h = cv2.boundingRect(self.cnt)
        return np.array((x + w//2, y + h//2))

    @property
    def excluded(self):
        return self._excluded

    def exclude(self):
        self._excluded = True

    @property
    def area(self):
        return cv2.boundingRect(self.cnt)[2] * cv2.boundingRect(self.cnt)[3]

    @property
    def raw(self) -> np.ndarray:
        return self.cnt


def find_all_contours(img: cv2.Mat) -> Generator:
    gaus_ksize = 7
    canny_tres1, canny_tres2 = 200, 100

    grsc = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gaus = cv2.GaussianBlur(grsc, (gaus_ksize, gaus_ksize), 0)
    edges = cv2.Canny(gaus, canny_tres1, canny_tres2)
    edges = cv2.dilate(edges, None, iterations=0)
    edges = cv2.erode(edges, None, iterations=0)

    raw_cnts, _ = cv2.findContours(
        edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = (FaceContour(c) for c in raw_cnts)

    return contours


def find_weight_contours(img: cv2.Mat) -> Generator:
    '''
    Returns the contours and coordinates (in pixels) of the objects in the image.
    '''
    contours = find_all_contours(img)

    MIN_FACE_AREA = 20000
    contours = filter(lambda c: c.area > MIN_FACE_AREA, contours)

    # for c in contours:
    #     print(c.area)
    
    contours: Iterable[FaceContour] = sorted(contours, key=lambda c: c.area)

    for c1, c2 in combinations(contours, 2):
        if np.linalg.norm(c1.center - c2.center) < 50:
            max(c1, c2, key=lambda c: c.area).exclude()

    raw_contours = (cnt.raw for cnt in contours if not cnt.excluded)

    return raw_contours
