
import numpy as np
from test_utils import ImageCapture
import config as cfg
import os
import cv2


def mean_img(images: list[cv2.Mat]):
    out = np.zeros_like(images[0].shape)
    for img in images:
        out += np.float32(img) / len(images)

    return out

# def sqr_err(img: cv2.Mat, tmp: cv2.Mat):
#     assert img.shape == tmp.shadpe
#     diff = img - tmp
#     return diff ** 2

# def find_weights(img, win_shape, win_step, conf_thresh):
    
#     ...


# images = ...
images = list(cv2.imread(i) for i in os.listdir(r"Hub\res\Cubes") if i.endswith(".png"))
avg = mean_img(images)
cv2.imwrite('/res/adfdskl.png', avg)





# from matplotlib import pyplot as plt

# cap = ImageCapture(cfg.TEST_IMGS)

# while True:
#     ret, frame = cap.read()

#     if not ret:
#         print('\nError: Cannot read frame')
#         break
    
#     img_rgb = frame
#     assert img_rgb is not None, "file could not be read, check with os.path.exists()"
#     img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)

#     for path in (i for i in os.listdir(r"Hub\res\NoMove") if i.endswith(".png")):
#         template = cv.imread(os.path.join("Hub\res\Cubes", path), cv.IMREAD_GRAYSCALE)
#         cv.imshow("t", template)
#         assert template is not None, "file could not be read, check with os.path.exists()"
#         w, h = template.shape[::-1]
#         res = cv.matchTemplate(img_gray,template,cv.TM_CCOEFF_NORMED)
#         threshold = 0.1
#         loc = np.where( res >= threshold)

#         for pt in zip(*loc[::-1]):
#             cv.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)
#     cv.imwrite('res.png',img_rgb)

# cv.destroyAllWindows()

