import cv2
import numpy as np
from functools import partial
from itertools import product
from color import Color
from typing import *
from abc import ABC, abstractmethod


class Marking(ABC):
    def __init__(self, face: cv2.Mat):
        self.__decoded = self._get_decoded(face)

    @abstractmethod
    def _get_decoded(self, face: cv2.Mat) -> tuple:
        ...

    @property
    def decoded(self) -> Tuple:
        return self.__decoded

    def __repr__(self):
        return str(self.decoded)

    def __eq__(self, mark_code: tuple) -> bool:
        return False if self.decoded == None else self.decoded == mark_code


class MatrixMarking(Marking):
    """Works with square marking 2x2"""

    def __get_cropped_marks(self, face: cv2.Mat) -> Generator:
        for x, y in product(range(2), repeat=2):
            sector = face[x*100 + 10:x*100 + 90, y*100 + 10:y*100 + 90]
            yield sector

    def __is_correct(self, marks: list) -> bool:
        if marks[0] + marks[1] == 11 and marks[0] + marks[3] != 11:
            return True
        else:
            return False

    def __get_correct_rotation(self, color_matrix: np.ndarray) -> np.ndarray:
        new_matrix = color_matrix.copy()

        for _ in range(4):
            if self.__is_correct(new_matrix):
                return tuple(new_matrix)
            else:
                new_matrix = [new_matrix[1], new_matrix[3],
                              new_matrix[0], new_matrix[2]]
        return color_matrix

    def _get_decoded(self, face: cv2.Mat) -> tuple:
        hsv_face = cv2.cvtColor(face, cv2.COLOR_BGR2HSV)

        cropped_sectors = self.__get_cropped_marks(hsv_face)

        get_avg_color = partial(np.mean, axis=(0, 1))
        avg_colors = map(get_avg_color, cropped_sectors)

        colors = list(map(Color.get_color_code, avg_colors))
        colors = self.__get_correct_rotation(colors)

        return colors


class StripesMarking(Marking):
    """Works with stripes marking 1x3"""

    def __get_cropped_marks(self, face: cv2.Mat) -> Tuple:
        h, w, _ = face.shape
        sw = (w//4) # sector width
        sh = (w//4) # sector height
        m = 20 # margins

        for x in range(4):
            left_sector = face[m:h-m, m:sw-m]
            up_sector = face[m:sh-m, m:w-m]
            if self.__is_black(left_sector) and self.__is_black(up_sector):
                _1 = face[sh+m:h-m, 1*sw+m:2*sw-m]
                _2 = face[sh+m:h-m, 2*sw+m:3*sw-m]
                _3 = face[sh+m:h-m, 3*sw+m:4*sw-m]
                return _1, _2, _3
                # for i in range(3):
                    # yield face[sh+m:h-m, i*sw+m:(i+1)*sw-m]
            else:
                face = cv2.rotate(face, cv2.ROTATE_90_CLOCKWISE)
        
        _1 = face[sh+m:h-m, 1*sw+m:2*sw-m]
        _2 = face[sh+m:h-m, 2*sw+m:3*sw-m]
        _3 = face[sh+m:h-m, 3*sw+m:4*sw-m]
        return _1, _2, _3

    def __is_black(self, sector: cv2.Mat) -> bool:
        avg_color = np.mean(sector, axis=(0, 1))
        color = Color.get_color_code(avg_color)
        return True if color == 1 else False

    def _get_decoded(self, face: cv2.Mat) -> tuple:
        hsv_face = cv2.cvtColor(face, cv2.COLOR_BGR2HSV)

        cropped_sectors = self.__get_cropped_marks(hsv_face)
        
        get_avg_color = partial(np.mean, axis=(0, 1))
        avg_colors = map(get_avg_color, cropped_sectors)

        colors = list(map(Color.get_color_code, avg_colors))

        return colors