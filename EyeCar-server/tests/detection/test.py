# Script has to be run by root directory - EyeCar-server
import cv2

# Making possible to import modules from root dir
import os
import sys
from pathlib import Path
root_path = os.path.join(*Path(os.path.dirname(os.path.abspath(__file__))).parts[:-2])
sys.path.append(root_path)
from detection.detectors.pedestrian_detector import PedestrianDetector


model = PedestrianDetector("ai/models/sign_detector_yolov8s.pt")
results = model.forward(cv2.imread("res/images/test_sign_detection.jpg"))
print(results)
