import cfg
import time
import numpy as np

from enum import Enum
from include.car_status import CarStatus
from detection.detection_handler import DetectionHandler


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
    class Directions(Enum):
        LEFT = 1
        RIGHT = 2
        STRAIGHT = 3

    def __init__(self, route: list[Directions]) -> None:
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

        turn_time = time.monotonic() - self.last_turn_time

        if turn_time >= self.target_turn_time:
            self.turning = None
            car.angle = 0
            self.target_turn_time = None
        
        self.turn(car)

    def turn(self, car: CarStatus) -> None:
        if self.turning == self.Directions.STRAIGHT:
            car.angle = 0
            self.target_turn_time = 3

        elif self.turning == self.Directions.LEFT:
            car.angle = -30
            self.target_turn_time = 5


        elif self.turning == self.Directions.RIGHT:
            car.angle = 45
            self.target_turn_time = 2
