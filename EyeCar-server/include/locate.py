import cfg
import cv2
import numpy as np
import time

from include.io_client import IOClient, ImageFolderClient
from include.intersection_directions import Directions


class Locate:
    pos_route_entry = {
        1: [Directions.RIGHT, Directions.STRAIGHT, Directions.RIGHT],
        2: [Directions.RIGHT, Directions.RIGHT, Directions.RIGHT],
        3: [Directions.STRAIGHT, Directions.RIGHT, Directions.RIGHT]
    }

    def __init__(self, io_client: IOClient):
        self.cap = io_client
        self.img_cap = ImageFolderClient('/res/orientation/')
    
    def calculate_route(self) -> list:
        frame = self.accumulate_frame(nframes=10, delay=0.05)
        pos_index = np.argmin(self.tmp_err(frame))  # smallest error
        route = self.pos_route_entry[pos_index]

        return route

    def tmp_err(self, frame: cv2.Mat) -> list:
        tmps = [self.img_cap.read_frame() for _ in range(3)]
        errs = [(tmp ^ frame).sum() for tmp in tmps]

        return errs

    def accumulate_frame(self, nframes: int, delay: float):
        out_img = np.zeros(cfg.IMG_SHAPE)
        for _ in range(nframes):
            out_img += self.cap.read_frame().astype(np.float32) / nframes
            time.sleep(delay)

        return out_img