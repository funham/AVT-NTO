"""
Final run task
"""

import cfg
import cv2

# IO and controls imports
import io_client as io_client_manager
from car_control import CarControl

# Detector imports
from detection.detection import GlobalDetectionModel
from detection.yolo_detector import YoloV5Detector, YoloV8Detector
from detection.lane_detector import LaneDetector

# Handler imports
from detection.lane_handler import LaneTurnHandler
from detection.crossroad_handler import CrossroadHandler
from detection.pedestrian_handler import PedestrianHandler

# Global variables
io_client = io_client_manager.get_io_client(cfg.INPUT_MODE)
detector = GlobalDetectionModel()
control = CarControl()

# Adding detectors to the global detector object.
detector.add_detector(YoloV5Detector("/Models/TrafficLightsDetector.model"))
detector.add_detector(YoloV8Detector("/Models/SignPedestrianDetector.model"))
detector.add_detector(LaneDetector())

# Registering handlers to detections
control.register_handler(LaneTurnHandler())
control.register_handler(PedestrianHandler())
control.register_handler(CrossroadHandler([]))


def main_loop() -> None:
    frame = io_client.read()

    if frame is None:
        raise StopIteration

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
