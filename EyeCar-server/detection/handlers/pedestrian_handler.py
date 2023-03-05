"""Handles pedestrian detections"""


import cfg
from include.car_status import CarStatus

from detection.detection_handler import DetectionHandler


class PedestrianHandler(DetectionHandler):
    class DetectionParser(DetectionHandler.DetectionParserBase):
        def __init__(self, data: dict):
            self.pedestrians: list = data['pedestrians']

    def set_control(self, detections: dict, car: CarStatus) -> None:
        try:
            det = self.DetectionParser(detections)
        except:
            return print("[PedestrianHandler]: key not found")

        distances = [p['distance'] for p in det.pedestrians]
        min_dist = min(distances)

        # stopping before a pedestrian
        if min_dist <= cfg.PEDESTRIAN_STOP_DISTANCE:
            return car.stop()

        # slowing down before a pedestrian
        elif min_dist <= cfg.PEDESTRIAN_SLOW_DOWN_DISTANCE:
            car.speed = cfg.PEDESTRIAN_SLOW_DOWN_DISTANCE
            return
