import cv2
import numpy as np

cv2.namedWindow("hsv_pallet", cv2.WINDOW_NORMAL)
cv2.createTrackbar('H', 'hsv_pallet', 0, 180, lambda x: None)
cv2.createTrackbar('S', 'hsv_pallet', 0, 255, lambda x: None)
cv2.setTrackbarPos('S', 'hsv_pallet', 255)
cv2.createTrackbar('V', 'hsv_pallet', 0, 255, lambda x: None)
cv2.setTrackbarPos('V', 'hsv_pallet', 255)

img = np.zeros((512, 512, 3), np.uint8)

while True:
    h = cv2.getTrackbarPos('H', 'hsv_pallet')
    s = cv2.getTrackbarPos('S', 'hsv_pallet')
    v = cv2.getTrackbarPos('V', 'hsv_pallet')

    img[:] = [h, s, v]
    img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    cv2.imshow('hsv_pallet', img)
    k = cv2.waitKey(1)
    if k == 27:
        break

cv2.destroyAllWindows()
