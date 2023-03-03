"""Handler for traffic light"""


from car_status import CarStatus

from detection.detection_handler import DetectionHandler
from typing import Callable, Optional

class TrafficLightHandler(DetectionHandler):
    def set_control(self, detections: dict, car: CarStatus) -> None:
        return