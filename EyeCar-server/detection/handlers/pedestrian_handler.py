"""Handles pedestrian detections"""


import cfg
import numpy as np

from include.car_status import CarStatus
from detection.detection_handler import DetectionHandler


class PedestrianHandler(DetectionHandler):
    class DetectionParser(DetectionHandler.DetectionParserBase):
        def __init__(self, data: dict):
            self.pedestrians: list = data['pedestrians']

    def set_control(self, detections: dict, car: CarStatus) -> None:
        try:
            det = self.DetectionParser(detections)
        except KeyError as ex:
            print("[PedestrianHandler]: key not found")
            print(ex)
            return

        if len(det.pedestrians) == 0:
            return

        distances = [p['dist'] for p in det.pedestrians]
        offsets = [p['offset'] for p in det.pedestrians]
        min_dist = np.min(distances)
        min_off = offsets[np.argmin(distances)]

        print(f'{min_dist=}, {min_off=}')

        # stopping before a pedestrian
        if min_dist <= cfg.PEDESTRIAN_STOP_DISTANCE:
            print('approaching pedestrian')
            if min_off < cfg.PEDESTRIAN_SIDEWALK_OFFSET:
                print('pedestrian is on the road!')
                return car.stop()

        # slowing down before a pedestrian
        elif min_dist <= cfg.PEDESTRIAN_SLOW_DOWN_DISTANCE:
            print('slowing down before a pedestrian')
            car.speed = cfg.PEDESTRIAN_SLOW_DOWN_DISTANCE
            return
