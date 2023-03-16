import cfg
import time
from include.car_status import CarStatus

from detection.detection_handler import DetectionHandler
from typing import Callable, Optional


class ParkingHandler(DetectionHandler):
    def set_control(self, detections: dict, car: CarStatus) -> None:
        if 'parking_distance' not in detections:
            return
        
        dist = detections['parking_distance']

        print(f'Parking distance: {dist}')

        if dist < cfg.PARKING_STOP_DISTANCE:
            car.park()
