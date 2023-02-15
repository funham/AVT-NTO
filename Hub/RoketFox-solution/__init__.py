import config as cfg
import cv2
import numpy as np
import facility as fcl
from typing import *
from test_utils import ImageCapture
from Cargo import Cargo
from Color import Color


def preprocessing(img: np.ndarray) -> np.ndarray:
    '''
    Basic preprocessing of the image like crop, warp and resize.
    '''
    perspective = np.float32([[172, 104],   # left_top     | corners of the working area
                              [409, 104],   # right_top    |
                              [428, 390],   # right_bottom |
                              [159, 390]])  # left_bottom  |
    out_width, out_height = cfg.ImageDims.SIZE

    corners = np.float32(
        [[0, 0], [out_width, 0], [out_width, out_height], [0, out_height]])
    matrix = cv2.getPerspectiveTransform(perspective, corners)
    croped_img = cv2.warpPerspective(img, matrix, (out_width, out_height))
    return croped_img


def main():
    cap = ImageCapture('res') if cfg.TESTING else cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()

        if not ret:
            print('\nError: Cannot read frame')
            break

        frame = preprocessing(frame)

        if cfg.TESTING:
            cv2.imshow('frame', frame)

        weights = Cargo.find_weights(frame)

        for i, cargo in enumerate(weights):
            print("=======================")
            if cfg.TESTING:
                cv2.imshow(f'n{i}', cargo.get_face())
                print(f"{cargo.marking = }")
                print(f"{cargo.coords = }")
            if cargo.marking == cfg.TARGET_MARKING:
                fcl.send_to_drone(cargo.coords)

    print('Exiting main loop')


if __name__ == "__main__":
    main()
