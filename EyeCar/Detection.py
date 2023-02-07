import cv2
from enum import Enum

from abc import ABC
from typing import Iterable
from Model import Model, Detection


class DetectedObject(ABC):
    conf: float
    distance: float

    def __init__(self, detecton: Detection) -> None:
        ...


class TrafficLight(DetectedObject):
    class Signal(Enum):
        R = 0
        Y = 1
        G = 2
        RY = 3
    
    signal: Signal

    @property
    def stop_condition(self) -> bool:
        return self.conf > 0.8 and self.distance < 3.0 and self.signal in \
                (TrafficLight.Signal.R, TrafficLight.Signal.Y, TrafficLight.Signal.RY)

    def __init__(self, model) -> None:
        ...


class Pedestrian(DetectedObject):
    def __init__(self, detection: DetectedObject):
        ...

    @property
    def stop_condition(self) -> bool:
        ...

class Sign(DetectedObject):
    def __init__(self, detection: DetectedObject):
        ...


def match_condition(detections: Iterable, cond_checker: property) -> bool:
    return any((cond_checker.fget(detection) for detection in detections))


if __name__ == "__main__":

    model = Model()
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()

        if not ret:
            continue

        det = model.forward(frame)

        traffic_lights = (TrafficLight(detection)
                          for detection in det.TrafficLights)

        pedestrians = (Pedestrian(detection)
                       for detection in det.Pedestrians)

        if match_condition(traffic_lights, TrafficLight.stop_condition) or \
           match_condition(pedestrians, Pedestrian.stop_condition) or ...:
            'stop_car()'
