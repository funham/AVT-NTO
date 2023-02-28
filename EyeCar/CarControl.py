from DetectionHandler import DetectionHandler
from CarStatus import CarStatus

# "friend" of Car status
# Singleton


class CarControl:
    status = CarStatus()
    handlers: list[DetectionHandler] = []

    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(CarControl, cls).__new__(cls)
        return cls.instance

    def get_command(self, detections: dict) -> str:
        """ Returns a `Command` based on `detections` data """
        for handler in self.handlers:
            handler.set_control(detections, self.status)

        control_speed = round(self.status.speed)
        control_angle = round(self.status.angle)

        cmd = f'SPEED:{control_speed}\nANGLE:{control_angle}\n'
        self.status.reset()  # reseting on each cycle

        return cmd

    def register_handler(self, handler: DetectionHandler):
        self.handlers.append(handler)
