import numpy as np
import cv2
from typing import *
import config as cfg

class CargoPosition:
    def __init__(self, cnt: np.ndarray) -> None:
        self.__coords = self.__face_center(cnt)

    def __face_center(self, cnt) -> Tuple[int, int]:
        bbox = cv2.minAreaRect(cnt)
        center = tuple(map(int, bbox[0]))

        return center
 
    @property
    def position(self) -> Tuple:
        return self.__coords
    
    @staticmethod
    def pix_to_coords(pix: tuple) -> Tuple:
        '''
        Convert pixels to coordinates based on img and carriage size from config.
        '''
        x, y = pix
        coord_w, coord_h = cfg.CARRIAGE_SIZE
        img_w, img_h = cfg.IMG_SIZE

        return (x/img_w*coord_w, y/img_h*coord_h)