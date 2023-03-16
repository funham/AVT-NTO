import cfg
import cv2
import numpy as np
import time

from include.io_client import IOClient, ImageFolderClient
from include.intersection_directions import Directions


class KeyPoints:
    sift = cv2.xfeatures2d.SIFT_create()
    bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)

    def __init__(self, img: cv2.Mat) -> None:
        if len(img.shape) > 2:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            self.keypoints, self.descriptors = KeyPoints.sift.detectAndCompute(img, None)
    
    @staticmethod
    def descriptors_dist(descriptors1: np.ndarray, descriptors2: np.ndarray) -> float:
        matches = KeyPoints.bf.match(descriptors1, descriptors2)
        matches = sorted(matches, key=lambda x: x.distance)
        distances = [dmatch.distance for dmatch in matches]

        return sum(distances) / len(distances)    


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
        
        return errs

    def accumulate_frame(self, nframes: int, delay: float):
        out_img = np.zeros(cfg.IMG_SHAPE)
        for _ in range(nframes):
            out_img += self.cap.read_frame().astype(np.float32) / nframes
            time.sleep(delay)

        return out_img

