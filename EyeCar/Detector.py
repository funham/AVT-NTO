"""
Defines basic detector interface and a global detection model 
that unites all the basic detectors.
"""

from abc import ABC, abstractmethod
import cv2


class IDetector(ABC):
    @abstractmethod
    def forward(self, frame: cv2.Mat) -> dict:
        pass


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
