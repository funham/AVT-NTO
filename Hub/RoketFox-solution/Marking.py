import cv2
import numpy as np
from functools import partial
from itertools import product
from Color import Color
from typing import *


class Marking:
    def __init__(self, face: cv2.Mat):
        self.face = face

    def __cropped_marks(self) -> Generator:
        img_hsv = cv2.cvtColor(self.face, cv2.COLOR_BGR2HSV)

        for x, y in product(range(2), repeat=2):
            sector = img_hsv[x*100+10:x*100+90, y*100+10:y*100+90]
            yield sector

    def __rotate_color_matrix(self, color_matrix: np.ndarray) -> np.ndarray:
        '''
        Rotate the color matrix until the rule is satisfied.

        `arr[[0,1], [2,3]]`
        The rule: `arr[0] + arr[1] == 11 and arr[0] + arr[3] != 11`.
        '''
        while color_matrix[0][0] + color_matrix[0][1] != 11 and color_matrix[0][0] + color_matrix[1][0] == 11:
            color_matrix = np.rot90(color_matrix)
        return color_matrix

    def __decoded(self) -> np.ndarray:
        cropped_sectors = self.__cropped_marks()

        get_avg_color = partial(np.mean, axis=(0, 1))
        avg_colors = map(get_avg_color, cropped_sectors)
        colors = list(map(Color.get_color_code, avg_colors))

        colors = np.reshape(colors, (2, 2))

        colors = self.__rotate_color_matrix(colors)

        return colors.flatten()

    def __repr__(self):
        return f"{self.__decoded()}"

    def __eq__(self, __o: tuple) -> bool:
        return (self.__decoded() == __o).all
