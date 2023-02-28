import cfg
from abc import ABC, abstractmethod
from CarStatus import CarStatus

class DetectionHandler(ABC):
    """ Changes car status based on detections data """
    class DetectionParserBase(ABC):
        pass

    @abstractmethod
    def set_control(self, detections: dict, car: CarStatus) -> None:
        pass


class CrossroadHandler(DetectionHandler):
    class DetectionParser(DetectionHandler.DetectionParserBase):
        def __init__(self, data: dict):
            self.crossroad_dist = data['crossroad_distance']
            self.traffic_light_signal = data['traffic_light']


    def set_control(self, detections: dict, car: CarStatus) -> None:
        try:
            det = self.DetectionParser(detections)
        except KeyError:
            return print("key not found")

        # if we are at crossroad zone
        if det.crossroad_dist < cfg.CROSSROAD_STOP_DIST:
            if det.traffic_light_signal == 'red':
                car.stop()

            if car.stop_duration < cfg.CROSSROAD_STOP_TIME:
                car.stop()

        elif det.crossroad_dist < cfg.CROSSROAD_SLOW_DOWN_DIST:
            car.speed = cfg.CROSSROAD_SPEED_LIMIT

class PedestrianHandler(DetectionHandler):
    class DetectionParser(DetectionHandler.DetectionParserBase):
        def __init__(self, data: dict):
            self.pedestrians: list = data['pedestrians']

    def set_control(self, detections: dict, car: CarStatus) -> None:
        try:
            det = self.DetectionParser(detections)
        except:
            return print("key not found")
        
        distances = [p['distance'] for p in det.pedestrians]
        min_dist = min(distances)
        
        if min_dist <= cfg.PEDESTRIAN_STOP_DISTANCE:
            return car.stop()
            
        elif min_dist <= cfg.PEDESTRIAN_SLOW_DOWN_DISTANCE:
            car.speed = cfg.PEDESTRIAN_SLOW_DOWN_DISTANCE
            return 


class LaneHandler(DetectionHandler):
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
        k_turn_slowdown = 0.8

        car.angle = det.deviation * k_dev  # adjusting angle to deviation
        car.speed = cfg.CAR_MAX_SPEED - abs(det.deviation) * k_dev * k_turn_slowdown  # adjusting speed to be slower on tight turns 
