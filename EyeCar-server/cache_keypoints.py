import os
import cv2
import pickle
from include.locate import KeyPoints


data = {}
for start_point in os.listdir('res/keypoints_imgs'):
    start_point = int(start_point)
    for img_name in os.listdir(f'res/keypoints_imgs/{start_point}'):
        img = cv2.imread(f'res/keypoints_imgs/{start_point}/{img_name}')
        kp = KeyPoints(img)
        data[start_point] = data.get(start_point, []) + [kp]
pickle.dump(data, open("cached_keypoints.pkl", "wb"))
