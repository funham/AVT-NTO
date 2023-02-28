from DetectionHandler import IDetectionHandler
from CarStatus import CarStatus

# "friend" of Car status
# Singleton
class CarControl:
    status = CarStatus()
    handlers: list[IDetectionHandler]

    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(CarControl, cls).__new__(cls)
        return cls.instance

    def get_command(self, detections: dict) -> str:
        """ Returns a `Command` based on `detections` data """
        for handler in self.handlers:
            handler.set_control(self.status, detections)

        cmd = f'SPEED:{self.status.speed}\nANGLE:{self.status.angle}\n'
        self.status.reset() # reseting on each cycle

        return cmd

    def register_handler(self, handler: IDetectionHandler):
        self.handlers.append(handler)