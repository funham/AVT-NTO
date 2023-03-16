import config as cfg
import cv2
import numpy as np
from facility import send_to_drone
from typing import *
from test_utils import ImageCapture
from cargo import Cargo
from color import Color
import os


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

    img = cv2.resize(img, (800, 480))

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

def kmeans(img):
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    img = cv2.GaussianBlur(img, (5,5), 0)
    Z = img.reshape((-1,3))
    # convert to np.float32
    Z = np.float32(Z)
    # define criteria, number of clusters(K) and apply kmeans()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 5
    ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    # Now convert back into uint8, and make original image
    center = np.uint8(center)
    res = center[label.flatten()]
    res2 = res.reshape((img.shape))
    res2 = cv2.cvtColor(res2, cv2.COLOR_HSV2RGB)
    return res2

def main():
    cap = ImageCapture(cfg.TEST_IMGS) if cfg.TESTING else cv2.VideoCapture(0)


    while True:
        ret, frame = cap.read()
        
        if not ret:
            print('\nError: Cannot read frame')
            break

        # frame = get_flatten_view(frame)

        if cfg.TESTING:
            im = cv2.resize(frame, (600,300))
            cv2.imshow('frame', im)
            

        weights: Iterable[Cargo] = Cargo.find_weights(frame)

        for i, cargo in enumerate(weights):
            if cfg.TESTING:
                print("=======================")
                print(f"{cargo.marking.decoded = }")
                print(f"{cargo.transform.position = }")
                cv2.imshow(f'n{i}', kmeans(cargo.get_face_view()))

            if cargo.marking == cfg.TARGET_MATRIX_MARKING:
                send_to_drone(cargo.transform.position)
                break

    print('Exiting main loop')
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
