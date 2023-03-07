import cv2
from .yolo_detector import YoloV8Detector


class PedestrianDetector(YoloV8Detector):
    def forward(self, frame: cv2.Mat) -> dict:
        #{'pedestrians': [{'bbox': bbox, 'distance': distance}, ...]
        out_result = {'pedestrians': []}
        result = super().forward(frame)['result']
        for output in result:
            # TODO class_id change to class id of pedestrian
            if output['class_id'] != 0:
                continue
            bbox = output['bbox']
            # TODO change distance formula
            distance = 640 - (bbox[2] - bbox[0])
            out_result['pedestrians'].append({'bbox': bbox, 'distance': distance})
        return out_result

