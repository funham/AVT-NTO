"""
Detect location and run to hub
"""

import os
import cfg
import cv2
import numpy as np

# Args
import argparse
from args import get_args_parser

# IO and controls imports
import include.io_client as io_client_manager
from include.car_control import CarControl
# from include.locate import Locate

# Detector imports
from detection.detection import GlobalDetectionModel
from detection.detectors.lane_detector import RoadDetector
from detection.detectors.yolo_detector import ParkingDetector

# Handler imports
from detection.handlers.lane_handler import LaneTurnHandler
from detection.handlers.parking_handler import ParkingHandler


parser = argparse.ArgumentParser(parents=[get_args_parser()])
args = parser.parse_args()

# Global variables
io_client = io_client_manager.create_io_client(cfg.INPUT_MODE, args)
detector = GlobalDetectionModel()
control = CarControl()


# Adding detectors to the global detector object.
detector.add_detector(RoadDetector())
detector.add_detector(ParkingDetector('../anything_yolov4-tiny-obj_uint8.tmfile'))

# Registering handlers to detections
control.register_handler(LaneTurnHandler())
control.register_handler(ParkingHandler())

def main_loop() -> None:
    frame = io_client.read_frame()

    if frame is None:
        return

    detections = detector.forward(frame)
    cmd = control.get_command(detections)

    io_client.send_msg(cmd)
    io_client.handle_keyboard_input()


if __name__ == '__main__':
    try:
        while True:
            main_loop()

    except StopIteration as ex:
        print(ex)
        print('Exiting main loop...')
    except KeyboardInterrupt:
        ...

    finally:
        print('Dont get hit by a car')
        io_client.send_msg('SPEED:0\n')