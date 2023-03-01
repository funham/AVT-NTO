import cv2
import numpy as np
from typing import *
from Marking import MatrixMarking, StripesMarking
from Coords import CargoPosition
from detection import find_weight_contours


class Cargo:
    def __init__(self, img: cv2.Mat, cnt: np.ndarray):
        self.img = img
        self.cnt = cnt
        self.face_dims = (200, 200)  # dimensions of the output face
        self.transform = CargoPosition(self.cnt)
        self.marking = StripesMarking(self.get_face_view())

    def get_face_view(self) -> cv2.Mat:
        width, height = self.face_dims

        rect = cv2.minAreaRect(self.cnt)
        box = np.float32(cv2.boxPoints(rect))
        corners = np.float32(
            [[0, 0], [width, 0], [width, height], [0, height]])
        matrix = cv2.getPerspectiveTransform(box, corners)
        face = cv2.warpPerspective(self.img, matrix, (width, height))

        return face

    @staticmethod
    def find_weights(img: cv2.Mat) -> Iterable[Self]:
        '''
        Find weights on the image
        '''
        contours = find_weight_contours(img)
        weights = (Cargo(img, cnt) for cnt in contours)

        return weights
