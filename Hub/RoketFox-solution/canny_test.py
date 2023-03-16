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

def contast(img, x):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    h,s,v = cv2.split(hsv)

    x = 255 - x
    v[v>x] = 255
    v[v<x] = x

    hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)

def brightnes(img, x):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    h,s,v = cv2.split(hsv)

    x = 255 - x
    s[s>x] = x
    s[s<x] = 0

    v = s

    hsv = cv2.merge((h,s,v))
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)

cv2.namedWindow("Canny", cv2.WINDOW_NORMAL)
cv2.createTrackbar('t1', 'Canny', 0, 50, lambda x: None)
cv2.createTrackbar('t2', 'Canny', 0, 50, lambda x: None)
cv2.createTrackbar('d', 'Canny', 0, 10, lambda x: None)
cv2.createTrackbar('e', 'Canny', 0, 10, lambda x: None)
cv2.createTrackbar('g', 'Canny', 0, 10, lambda x: None)
cv2.createTrackbar('c', 'Canny', 0, 255, lambda x: None)

cap = ImageCapture(cfg.TEST_IMGS)

while True:
    ret, frame = cap.read()

    if not ret:
        print('\nError: Cannot read frame')
        break
    
    frame = get_flatten_view(frame)

    t1 = cv2.getTrackbarPos('t1', 'Canny')*10
    t2 = cv2.getTrackbarPos('t2', 'Canny')*10
    d = cv2.getTrackbarPos('d', 'Canny')
    e = cv2.getTrackbarPos('e', 'Canny')
    g = 2*cv2.getTrackbarPos('g', 'Canny')+1
    c = cv2.getTrackbarPos('c', 'Canny')

    frame = contast(frame, c)

    gaus = cv2.GaussianBlur(frame, (g, g), 0)
    grsc = cv2.cvtColor(gaus, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(grsc, t1, t2)
    edges = cv2.dilate(edges, None, iterations=d)
    edges = cv2.erode(edges, None, iterations=e)
    
    raw_cnts, _ = cv2.findContours(
        edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    frame = cv2.drawContours(frame, raw_cnts, -1, (0,0,255), 1)
    
    cv2.imshow("fr",frame)

print('Exiting main loop')
cv2.destroyAllWindows()