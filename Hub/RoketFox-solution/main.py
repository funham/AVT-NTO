import config as cfg
import cv2
import numpy as np
from facility import send_to_drone
from typing import *
from test_utils import ImageCapture
from cargo import Cargo
from color import Color


def get_flatten_view(img: cv2.Mat) -> cv2.Mat:
    '''
    Basic preprocessing of the image like crop, warp and resize.
    '''
    h, w, _= img.shape

    if h > w:
        img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)

    h, w, _= img.shape
    k = h/w

    um = int(h * 0.3)
    dm = h - int(h * 0.3)
    lm = int(w * 0.25)
    rm = w - int(w * 0.2)

    img = img[um:dm, lm:rm]

    img = cv2.resize(img, (680, int(680*k)))

    return img

    # perspective = np.float32([[172, 104],   # left_top     | corners of the working area
                            #   [409, 104],   # right_top    |
                            #   [428, 390],   # right_bottom |
                            #   [159, 390]])  # left_bottom  |
    # out_width, out_height = cfg.IMG_SIZE

    # corners = np.float32(
        # [[0, 0], [out_width, 0], [out_width, out_height], [0, out_height]])
    # matrix = cv2.getPerspectiveTransform(perspective, corners)
    # croped_img = cv2.warpPerspective(img, matrix, (out_width, out_height))
    # return croped_img


def main():
    cap = ImageCapture(cfg.TEST_IMGS) if cfg.TESTING else cv2.VideoCapture(0)


    while True:
        ret, frame = cap.read()

        frame = get_flatten_view(frame)
        
        if not ret:
            print('\nError: Cannot read frame')
            break

        # frame = get_flatten_view(frame)

        if cfg.TESTING:
            cv2.imshow('frame', frame)

        weights: Iterable[Cargo] = Cargo.find_weights(frame)

        for i, cargo in enumerate(weights):
            if cfg.TESTING:
                print("=======================")
                print(f"{cargo.marking.decoded = }")
                print(f"{cargo.transform.position = }")
                cv2.imshow(f'n{i}', cargo.get_face_view())

            if cargo.marking == cfg.TARGET_MATRIX_MARKING:
                send_to_drone(cargo.transform.position)
                break

    print('Exiting main loop')
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
