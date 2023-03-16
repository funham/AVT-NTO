import cfg
import time
import numpy as np

from enum import Enum
from include.car_status import CarStatus
from detection.detection_handler import DetectionHandler
from typing import List
from include.intersection_directions import Directions


class CrossroadStopHandler(DetectionHandler):
    class DetectionParser(DetectionHandler.DetectionParserBase):
        def __init__(self, data: dict):
            self.crossroad_dist = data['crossroad_distance']
            self.traffic_light_signal = data['traffic_light']

    def set_control(self, detections: dict, car: CarStatus) -> None:
        try:
            det = self.DetectionParser(detections)
        except KeyError:
            return print("[CrossroadStopHandler]: key not found")

        # if we are at crossroad zone
        if det.crossroad_dist < cfg.CROSSROAD_STOP_DIST:
            if det.traffic_light_signal == 'red':
                car.stop()
        
        # slowing down before intersection
        elif det.crossroad_dist < cfg.CROSSROAD_SLOW_DOWN_DIST:
            car.speed = cfg.CROSSROAD_SPEED_LIMIT
            

class CrossroadTurnHandler(DetectionHandler):
    def __init__(self, route: List[Directions]) -> None:
        self.turns_iter = iter(route)
        self.turning = None
        self.prev_dist = np.inf
        self.last_turn_time = None
        self.target_turn_time = None

    def set_control(self, detections: dict, car: CarStatus) -> None:
        crossroad_dist = detections.get('crossroad_distance')

        if crossroad_dist == np.inf and self.prev_dist < 100 and not self.turning:
            try:
                self.turning = next(self.turns_iter)
            except StopIteration:
                print("[ERROR]: CrossroadTurnHandler: no turns left")
                return
            
            self.last_turn_time = time.monotonic()
            self.turn(car)

        self.prev_dist = crossroad_dist

        if not self.turning:
            return
        
        if cfg.DEBUG:
            print(f'{crossroad_dist=:.2f}')
            print(f'Turn direction: {self.turning}')

        turn_time = time.monotonic() - self.last_turn_time - cfg.CROSSROAD_DELAY

        if turn_time >= self.target_turn_time:
            self.turning = None
            car.angle = 0
            self.target_turn_time = None
        
        if turn_time > 0:
            self.turn(car)

    def turn(self, car: CarStatus) -> None:
        if self.turning is None:
            return
    
        elif self.turning == Directions.STRAIGHT:
            car.angle = 0
            self.target_turn_time = 4
            # car.dist(cfg.CAR_TURN_SPEED, 200)

        elif self.turning == Directions.LEFT:
            car.angle = -30
            self.target_turn_time = 5
            # car.dist(cfg.CAR_TURN_SPEED, 300)

        elif self.turning == Directions.RIGHT:
            car.angle = 38
            self.target_turn_time = 3
            # car.dist(cfg.CAR_TURN_SPEED, 150)
