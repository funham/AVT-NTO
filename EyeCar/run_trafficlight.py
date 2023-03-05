"""
Run 3 meters subtask
"""

import cfg
import cv2

# IO and controls imports
import io_client as io_client_manager
from car_control import CarControl

# Detector imports
from detection.detection import GlobalDetectionModel
from detection.detectors.yolo_detector import YoloV8Detector

# Handler imports
from detection.handlers.trafficlight_handler import TrafficLightStaticHandler


# Global variables
io_client = io_client_manager.create_io_client(cfg.INPUT_MODE)
detector = GlobalDetectionModel()
control = CarControl()

# Adding detectors to the global detector object.
detector.add_detector(YoloV8Detector("/Models/TrafficLightsDetector"))

# Registering handlers to detections
control.register_handler(TrafficLightStaticHandler())

def main_loop() -> None:
    frame = io_client.read_frame()

    if frame is None:
        return

    cv2.imshow('frame', frame)

    detections = detector.forward(frame)
    cmd = control.get_command(detections)

    io_client.send_msg(cmd)
    io_client.handle_keyboard_input()


if __name__ == '__main__':
    try:
        while True:
            main_loop()

    except StopIteration:
        print('Exiting main loop...')

    finally:
        print('Dont get hit by a car')
        io_client.send_msg('SPEED:0\n')