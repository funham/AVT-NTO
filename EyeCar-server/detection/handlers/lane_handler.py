"""Keeping on lane algorithm"""

import cfg

from include.car_status import CarStatus
from detection.detection_handler import DetectionHandler
from typing import Callable, Optional

class LaneTurnHandler(DetectionHandler):
    class DetectionParser(DetectionHandler.DetectionParserBase):
        def __init__(self, data: dict):
            self.deviation: float = data['lane_deviation']
            self.sl_dist: float = data['crossroad_distance']

    # TODO add PID
    def set_control(self, detections: dict, car: CarStatus) -> None:
        try:
            det = self.DetectionParser(detections)
        except KeyError as err:
            print("[LaneTurnHandler]: key not found:")
            print(err)
            return

        if det.sl_dist < 30:
            return

        k_dev = cfg.CAR_MAX_ANGLE / cfg.MAX_DEVIATION
        
        # adjusting angle to deviation
        car.angle = det.deviation * k_dev


class LaneSpeedHandler(DetectionHandler):
    class DetectionParser(DetectionHandler.DetectionParserBase):
        def __init__(self, data: dict):
            self.deviation: float = data['lane_deviation']

    # TODO add PID
    def set_control(self, detections: dict, car: CarStatus) -> None:
        try:
            det = self.DetectionParser(detections)
        except KeyError:
            return print("key not found")

        k_dev = cfg.CAR_MAX_ANGLE / cfg.MAX_DEVIATION

        # adjusting speed to be slower on tight turns
        car.speed = cfg.CAR_MAX_SPEED - abs(det.deviation) * k_dev * 0.8


class OpticalDistanceHandler(DetectionHandler):
    class DetectionParser(DetectionHandler.DetectionParserBase):
        def __init__(self, data: dict):
            self.distance: float = data['distance_travelled']
    
    def __init__(self, target_distance, on_distance_travelled: Optional[Callable] = None, *args, **kwargs):
        """Parameters:
        target_distance: the distance to reach before triggering the event
        on_distance_travelled: a callback to call when the distance is reached (None by default)

        if on_distance_traveled is not specified then run termination is called
        """

        self.target_distance = target_distance
        self.on_distance_travelled = on_distance_travelled
        self.callback_args = args
        self.callback_kwargs = kwargs

    def set_control(self, detections: dict, car: CarStatus) -> None:
        try:
            det = self.DetectionParser(detections)
        except KeyError:
            return print("key not found")
        
        if det.distance >= self.target_distance:
            if self.on_distance_travelled is None:
                car.terminate_ride(print, f"[OpticalDistanceHandler]: Car travelled {det.distance}")
            else:
                self.on_distance_travelled(*self.callback_args, **self.callback_kwargs)
        
