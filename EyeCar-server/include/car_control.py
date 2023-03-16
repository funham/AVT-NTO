"""
Car control script.

Takes responsibility of handling detection results and calculating
command to send to the car.
"""

from detection.detection_handler import DetectionHandler
from include.car_status import CarStatus
from typing import List

class CarControl:
    status = CarStatus()
    handlers: List[DetectionHandler] = []

    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(CarControl, cls).__new__(cls)
        return cls.instance

    def get_command(self, detections: dict) -> str:
        """
        Calculate command to send to the car, based on a detection results.

        Applies all the handlers to the current car status and 
        returns the command to send to the car. Then resets car's
        status to be ready for the next cycle.
        """

        for handler in self.handlers:
            handler.set_control(detections, self.status)

        if self.status.parking_requested:
            return "PARK"

        control_speed = round(self.status.speed)
        control_angle = round(self.status.angle)

        cmd = f'SPEED:{control_speed}\nANGLE:{control_angle}\n'
        self.status.reset()  # reseting on each cycle

        return cmd

    def register_handler(self, handler: DetectionHandler):
        """Registers a handler to be called each cycle of detection handling"""
        self.handlers.append(handler)
