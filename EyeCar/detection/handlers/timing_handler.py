"""Handlers for timing events"""


import time
from car_status import CarStatus

from detection.detection_handler import DetectionHandler
from typing import Callable, Optional

class TimingDistanceHandler(DetectionHandler):
    def __init__(self, target_distance, on_distance_travelled: Optional[Callable] = None, *args, **kwargs):
        """Parameters:
        target_distance: the distance to reach before triggering the event
        on_distance_travelled: a callback to call when the distance is reached (None by default)

        if on_distance_traveled is not specified then run termination is called
        """

        self.last_time = time.monotonic()
        self.distance_travelled = 0
        self.target_distance = target_distance
        self.on_distance_travelled = on_distance_travelled
        self.callback_args = args
        self.callback_kwargs = kwargs

    def set_control(self, detections: dict, car: CarStatus) -> None:
        current_time = time.monotonic()
        dt = current_time - self.last_time

        self.distance_travelled += car.speed * dt
        self.last_time = current_time

        if self.distance_travelled >= self.target_distance:
            if self.on_distance_travelled is None:
                car.terminate_ride(print, f"[DistanceTimeHandler]: Car travelled {self.distance_travelled}")
            else:
                self.on_distance_travelled(*self.callback_args, **self.callback_kwargs)


class TimingHandler(DetectionHandler):
    def __init__(self, target_time, on_time_passed: Optional[Callable] = None, *args, **kwargs):
        self.start_time = time.monotonic()
        self.target_time = target_time
        self.on_time_passed = on_time_passed
        self.callback_args = args
        self.callback_kwargs = kwargs


    def set_control(self, detections: dict, car: CarStatus) -> None:
        if time.monotonic() - self.start_time >= self.target_time:
            if self.on_time_passed is None:
                car.terminate_ride(print, f"[TimingTerminationHandler]: Car reached target time")
            else:
                self.on_time_passed(*self.callback_args, **self.callback_kwargs)
