from dataclasses import dataclass
from typing import List
import cv2
import numpy as np

@dataclass(frozen=True)
class Detection:
    """
    A detection is a bounding box with a confidence score.
    """
    bbox: np.ndarray
    confidence: float


@dataclass(frozen=True)
class DetectionResults:
    """
    A class to store the results of a detection.
    """
    TrafficLights: List[Detection]
    Pedestrians: List[Detection]
    Signs: List[Detection]



class Model:
    def forward(self, frame: cv2.Mat) -> DetectionResults:
        ...