import config as cfg
import cv2
import numpy as np
from facility import send_to_drone
from typing import *
from test_utils import ImageCapture
from cargo import Cargo
from color import Color
from detection import find_all_contours


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

    img = cv2.resize(img, (840, int(840*k)))

    return img


cv2.namedWindow("Canny", cv2.WINDOW_NORMAL)
cv2.createTrackbar('t1', 'Canny', 0, 50, lambda x: None)
cv2.createTrackbar('t2', 'Canny', 0, 50, lambda x: None)
cv2.createTrackbar('d', 'Canny', 0, 10, lambda x: None)
cv2.createTrackbar('e', 'Canny', 0, 10, lambda x: None)

cap = ImageCapture(cfg.TEST_IMGS)

while True:
    ret, frame = cap.read()

    if not ret:
        print('\nError: Cannot read frame')
        break
    
    frame = get_flatten_view(frame)

    t1 = cv2.getTrackbarPos('t1', 'Canny')
    t2 = cv2.getTrackbarPos('t2', 'Canny')
    d = cv2.getTrackbarPos('d', 'Canny')
    e = cv2.getTrackbarPos('e', 'Canny')
    
    find_all_contours(frame, t1, t2, d, e)

print('Exiting main loop')
cv2.destroyAllWindows()