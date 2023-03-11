"""
Final run task
"""

import cfg
import cv2

# Args
import argparse
from args import get_args_parser

# IO and controls imports
import include.io_client as io_client_manager
from include.car_control import CarControl

# Detector imports
from detection.detection import GlobalDetectionModel
from detection.detectors.yolo_detector import YoloV5Detector, YoloV8Detector
from detection.detectors.lane_detector import RoadDetector

# Handler imports
from detection.handlers.lane_handler import LaneTurnHandler
from detection.handlers.crossroad_handler import CrossroadHandler
from detection.handlers.pedestrian_handler import PedestrianHandler


parser = argparse.ArgumentParser(parents=[get_args_parser()])
args = parser.parse_args()

# Global variables
io_client = io_client_manager.create_io_client(cfg.INPUT_MODE, args)
detector = GlobalDetectionModel()
control = CarControl()

# Adding detectors to the global detector object.
detector.add_detector(YoloV5Detector("/Models/TrafficLightsDetector.model"))
detector.add_detector(YoloV8Detector("/Models/SignPedestrianDetector.model"))
detector.add_detector(RoadDetector())

# Registering handlers to detections
control.register_handler(LaneTurnHandler())
control.register_handler(PedestrianHandler())
control.register_handler(CrossroadHandler([]))


def main_loop() -> None:
    frame = io_client.read_frame()

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
