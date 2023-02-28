from abc import ABC, abstractmethod
import cv2


class IDetector(ABC):
    @abstractmethod
    def forward(self, frame: cv2.Mat) -> dict:
        pass


class YoloV5Detector(IDetector):
    def __init__(self, path):
        self.model = ...

    def forward(self, frame: cv2.Mat) -> dict:
        detection = self.model(frame)
        ...
        return detection


class YoloV8Detector(IDetector):
    def __init__(self, path):
        self.model = ...

    def forward(self, frame: cv2.Mat) -> dict:
        detection = self.model(frame)
        ...
        return detection


class GlobalDetectionModel:
    def __init__(self):
        self.detectors = []

    def add_detector(self, detector: IDetector) -> None:
        self.detectors.append(detector)

    def forward(self, frame: cv2.Mat) -> dict:
        detection = {}

        for detector in self.detectors:
            detection |= detector.forward(frame)

        return detection
