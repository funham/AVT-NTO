from abc import ABC, abstractmethod
from typing import override


class Detector(ABC):
    def __init__(self, model) -> None:
        self.model = Detector._load(model)
        
    @abstractmethod
    def forward(self, frame: cv2.Mat) -> dict:
        ...
    
    @staticmethod
    def _load():
        ...
        

class PedestrianDetector(Detector):
    def __init__(self, model) -> None:
        ...
    
    @override
    def forward(self, frame: cv2.Mat) -> dict:
        ...
        
class TrafficLightDetector(Detector):
    def __init__(self, model) -> None:
        ...
    
    @override
    def forward(self, frame: cv2.Mat) -> dict:
        ...

def LaneDetector(Detector):
    def __init__(self) -> None:
        super().__init__(...)
        
    
    @override
    def forward(self, frame: cv2.Mat) -> dict:
        ...