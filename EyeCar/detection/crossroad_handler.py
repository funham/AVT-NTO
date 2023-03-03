import cfg
from car_status import CarStatus

from detection.detection_handler import DetectionHandler


class CrossroadHandler(DetectionHandler):
    class DetectionParser(DetectionHandler.DetectionParserBase):
        def __init__(self, data: dict):
            self.crossroad_dist = data['crossroad_distance']
            self.traffic_light_signal = data['traffic_light']

    def __init__(self, route: list[CarStatus.IntersectionTurnDirections]) -> None:
        self.turns_iter = iter(route)

    def set_control(self, detections: dict, car: CarStatus) -> None:
        try:
            det = self.DetectionParser(detections)
        except KeyError:
            return print("[CrossroadHandler]: key not found")

        # if we are at crossroad zone
        if det.crossroad_dist < cfg.CROSSROAD_STOP_DIST:
            if det.traffic_light_signal == 'red':
                car.stop()
            elif not car.turning:
                car.intersection_turn(next(self.turns_iter))
        
        # slowing down before intersection
        elif det.crossroad_dist < cfg.CROSSROAD_SLOW_DOWN_DIST:
            car.speed = cfg.CROSSROAD_SPEED_LIMIT

        # TODO implement intersection turn logic
        if not car.turning:
            return
        
        elif car.turning == CarStatus.IntersectionTurnDirections.STRAIGHT:
            car.angle = 0

        elif car.turning == CarStatus.IntersectionTurnDirections.LEFT:
            car.angle = -30

        elif car.turning == CarStatus.IntersectionTurnDirections.RIGHT:
            car.angle = 45
