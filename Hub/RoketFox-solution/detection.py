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
    def size(self):
        x, y, w, h = cv2.boundingRect(self.cnt)
        return w, h

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

def contast(img, x):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    h,s,v = cv2.split(hsv)

    x = 255 - x
    v[v>x] = 255
    v[v<x] = x

    hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)

def find_all_contours(img: cv2.Mat, t1 = 100, t2 = 50, d = 1, e = 1) -> Generator:
    gaus_ksize = 11
    canny_tres1, canny_tres2 = t1, t2

    c = contast(img, 70)
    gaus = cv2.GaussianBlur(img, (gaus_ksize, gaus_ksize), 0)
    grsc = cv2.cvtColor(gaus, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(grsc, canny_tres1, canny_tres2)
    edges = cv2.dilate(edges, None, iterations=d)
    edges = cv2.erode(edges, None, iterations=e)

    ed = cv2.resize(edges, (600,300))
    cv2.imshow("edges", ed)

    raw_cnts, _ = cv2.findContours(
        edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours = tuple(FaceContour(c) for c in raw_cnts)

    return contours


def find_weight_contours(img: cv2.Mat) -> Generator:
    '''
    Returns the contours and coordinates (in pixels) of the objects in the image.
    '''
    contours = find_all_contours(img)

    

    MIN_FACE_AREA = 4500
    MAX_FACE_AREA = 10000
    contours = filter(lambda c: c.area > MIN_FACE_AREA, contours)
    contours = filter(lambda c: c.area < MAX_FACE_AREA, contours)

    # for c in contours:
    #     print(c.area)
    
    contours: Iterable[FaceContour] = sorted(contours, key=lambda c: c.area)
    
    for c in contours:
        w, h = c.size
        if abs(w-h) > 10:
            c.exclude()


    for c1, c2 in combinations(contours, 2):
        if np.linalg.norm(c1.center - c2.center) < 50:
            min(c1, c2, key=lambda c: c.area).exclude()

    raw_contours = (cnt.raw for cnt in contours if not cnt.excluded)

    return raw_contours
