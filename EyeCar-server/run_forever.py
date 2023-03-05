"""
Testing algorithms related to lane detection
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

# Detector imports
from detection.detection import GlobalDetectionModel
from detection.detectors.lane_detector import LaneDetector
from detection.detectors.yolo_detector import YoloV5Detector

# Handler imports
from detection.handlers.lane_handler import LaneTurnHandler
from detection.handlers.lane_handler import OpticalDistanceHandler
from detection.handlers.timing_handler import TimingDistanceHandler
from detection.handlers.timing_handler import TimingHandler
from detection.handlers.crossroad_handler import CrossroadTurnHandler


TARGET_DISTANCE = np.inf  # No objective on travelling distance

parser = argparse.ArgumentParser(parents=[get_args_parser()])
args = parser.parse_args()

# Global variables
io_client = io_client_manager.create_io_client(cfg.INPUT_MODE, args)
detector = GlobalDetectionModel()
control = CarControl()

# Adding detectors to the global detector object.
detector.add_detector(LaneDetector())

# Registering handlers to detections
control.register_handler(LaneTurnHandler())
control.register_handler(CrossroadTurnHandler([CrossroadTurnHandler.Directions.RIGHT,
                                               CrossroadTurnHandler.Directions.RIGHT]))

# Optical distance controlling (potentially the most stable)
control.register_handler(OpticalDistanceHandler(target_distance=TARGET_DISTANCE, 
                                                on_distance_travelled=lambda:...))

# Control distance by time directly (works properly only if speed is constant)
control.register_handler(TimingHandler(target_time=TARGET_DISTANCE / cfg.CAR_MAX_SPEED, 
                                       on_time_passed=lambda:...))

# Control distance by time with variable speed
control.register_handler(TimingDistanceHandler(target_distance=TARGET_DISTANCE, 
                                               on_distance_travelled=lambda:...))


def main_loop() -> None:
    frame = io_client.read_frame()

    if frame is None:
        return

    cv2.imshow('frame', frame)
    
    if os.name == 'nt':
        os.system('cls')
    else:
        os.system('clear')

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
