import cv2
from enum import Enum

from abc import ABC, abstractmethod, abstractproperty
from collections import namedtuple
from typing import Iterable
from itertools import chain


DetectionResults = namedtuple("Detection results", "label1 label2 label3 ...")
Detection = namedtuple("Detection", "conf bbox")


class DetectedObject(ABC):
    conf: float
    distance: float

    def __init__(self, detecton: Detection) -> None:
        ...


class TrafficLight(DetectedObject):
    Signal = Enum(
        value='Signal',
        names=('stop', 'go'))

    signal: Signal

    @property
    def stop_condition(self) -> bool:
        return self.conf > 0.8 and self.distance < 3.0 and self.signal == TrafficLight.Signal.STOP

    def __init__(self, model) -> None:
        ...


class Pedestrian(DetectedObject):
    def __init__(self, detection: DetectedObject):
        ...

    @property
    def stop_condition(self) -> bool:
        ...


def match_condition(detections: Iterable, cond_checker: property) -> bool:
    return any((cond_checker.fget(detection) for detection in detections))


if __name__ == "__main__":

    model = ...
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()

        if not ret:
            continue

        det: tuple = model.forward(frame)

        traffic_lights = (TrafficLight(detection)
                          for detection in det.traffic_light)

        pedestrians = (Pedestrian(detection)
                       for detection in det.pedestrians)

        if match_condition(traffic_lights, TrafficLight.stop_condition) or \
           match_condition(pedestrians, Pedestrian.stop_condition) or ...:
            'stop_car()'
