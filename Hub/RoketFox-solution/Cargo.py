import cv2
import numpy as np
from typing import *
from Marking import Marking
from detection import find_weight_contours


class Cargo:
    def __init__(self, img: cv2.Mat, cnt: np.ndarray):
        self.img = img
        self.cnt = cnt
        self.face_dims = (200, 200)  # dimensions of the output face
        self.marking = Marking(self.get_face())

    def __face_center(self) -> Tuple[int, int]:
        bbox = cv2.minAreaRect(self.cnt)
        center = bbox[0]
        
        return center

    @property
    def coords(self) -> Tuple:
        return self.__face_center()

    def get_face(self) -> cv2.Mat:
        width, height = self.face_dims

        rect = cv2.minAreaRect(self.cnt)
        box = np.float32(cv2.boxPoints(rect))
        corners = np.float32(
            [[0, 0], [width, 0], [width, height], [0, height]])
        matrix = cv2.getPerspectiveTransform(box, corners)
        face = cv2.warpPerspective(self.img, matrix, (width, height))

        return face

    @staticmethod
    def find_weights(img: cv2.Mat) -> Iterable:
        '''
        Find weights on the image
        '''
        contours = find_weight_contours(img)
        weights = (Cargo(img, cnt) for cnt in contours)

        return weights
