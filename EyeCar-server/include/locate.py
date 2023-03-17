import cfg
import cv2
import numpy as np
import time
import pickle
import math
import os

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
    def descriptors_dist(kp1, kp2) -> float:
        matches = KeyPoints.bf.match(kp1.descriptors, kp2.descriptors)
        matches = sorted(matches, key=lambda x: x.distance)
        distances = [dmatch.distance for dmatch in matches]

        return sum(distances) / len(distances)    

class Locate:
    pos_route_entry = {
        1: [Directions.RIGHT, Directions.STRAIGHT, Directions.RIGHT, Directions.HUB],
        2: [Directions.RIGHT, Directions.RIGHT, Directions.RIGHT, Directions.HUB],
        3: [Directions.STRAIGHT, Directions.RIGHT, Directions.RIGHT, Directions.HUB]
    }

    def __init__(self, io_client: IOClient):
        self.cap = io_client
        self.img_cap = ImageFolderClient('/res/orientation/')
        #self.start_kp = pickle.load(open('cached_keypoints.pkl', 'rb'))
        self.start_kp = {}
        for start_point in os.listdir('res/keypoints_imgs'):
            start_point = int(start_point)
            for img_name in os.listdir(f'res/keypoints_imgs/{start_point}'):
                img = cv2.imread(f'res/keypoints_imgs/{start_point}/{img_name}')
                kp = KeyPoints(img)
                self.start_kp[start_point] = self.start_kp.get(start_point, []) + [kp]
    
    def calculate_route(self) -> list:
        frame = self.cap.read_frame()
        kp_fame = KeyPoints(frame)

        min_distance = math.inf
        min_pos_index = 1
        for pos_index, kps in self.start_kp.items():
            mean_distance = 0
            for start_kp in kps:
                distance = KeyPoints.descriptors_dist(kp_fame, start_kp)
                mean_distance += distance / len(kps)
            if mean_distance < min_distance:
                min_distance = mean_distance
                min_pos_index = pos_index
        
        print(min_pos_index)
        route = self.pos_route_entry[min_pos_index]

        return route

    """def accumulate_frame(self, nframes: int, delay: float):
        out_img = np.zeros(cfg.IMG_SHAPE)
        for _ in range(nframes):
            out_img += self.cap.read_frame().astype(np.float32) / nframes
            time.sleep(delay)

        return out_img"""

