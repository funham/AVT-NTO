"""Defines a detection handler interface and its implementations."""

from abc import ABC, abstractmethod
from car_status import CarStatus


class DetectionHandler(ABC):
    """ Changes car status based on detections data """
    class DetectionParserBase(ABC):
        """Parses data from detection dictionary and saves it as an attribute.
        if some key is not found throws KeyError exception.
        """
        
        pass

    @abstractmethod
    def set_control(self, detections: dict, car: CarStatus) -> None:
        """Modifies car status based on detections data."""
        pass




