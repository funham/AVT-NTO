"""
Implementation of Yolo model detectors
"""

import cfg
import cv2

from detection.detection import IDetector

# YOLO import
import torch
from ultralytics import YOLO

# TODO return data in dict with a correct structure


class YoloV5Detector(IDetector):
    def __init__(self, path):
        self.model = torch.hub.load('./ai/yolov5', 'custom', path=path, source='local')
        self.model.conf = 0.2

    def forward(self, frame: cv2.Mat) -> dict:
        result = []
        preds = self.model(frame).xyxy[0]
        for x1, y1, x2, y2, conf, class_id in preds:
            class_id = int(class_id)
            x1 = int(x1)
            x2 = int(x2)
            y1 = int(y1)
            y2 = int(y2)
            result.append({'class_id': class_id, 'bbox': [x1, y1, x2, y2]})
        return {'result': result}


class YoloV8Detector(IDetector):
    def __init__(self, path):
        self.model = YOLO(path)
        self.model.conf = 0.2

    def forward(self, frame: cv2.Mat) -> dict:
        result = []
        inputs = [frame]
        outputs = self.model(inputs, stream=True)
        for output in outputs:
            boxes = output.boxes
            for class_id, box in zip(boxes.cls, boxes.xyxy):
                class_id, bbox = class_id.item(), box.tolist()
                result.append({'class_id': class_id, 'bbox': bbox})
        return {'result': result}
