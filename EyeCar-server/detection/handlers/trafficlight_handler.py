"""Handler for traffic light"""


from include.car_status import CarStatus

from detection.detection_handler import DetectionHandler
from typing import Callable, Optional


class TrafficLightHandler(DetectionHandler):
    """Stops the car when red signal is detected"""
    def set_control(self, detections: dict, car: CarStatus) -> None:
        ...


class TrafficLightStaticHandler(DetectionHandler):
    """Prints out the detected traffic light signal (for subtask)"""
    def set_control(self, detections: dict, car: CarStatus) -> None:
        car.stop()
        ...