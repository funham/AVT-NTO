import cfg

import cv2

from io_client import get_io_client
from CarControl import CarControl

from Detector import GlobalDetector
from Detector import TrafficLightDetector, PedestrianDetector
from LaneKeeper import LaneDetector

from typing import *

io_client = get_io_client(cfg.INPUT_MODE)
detector = GlobalDetector()

detector.add_model(TrafficLightDetector())
detector.add_model(PedestrianDetector())
detector.add_model(LaneDetector())
...


def main_loop() -> None:
    frame = io_client.read()

    if frame is None:
        return

    cv2.imshow('frame', frame)
    io_client.handle_keyboard_input()

    detections = detector.forward(frame)
    cmd = CarControl().get_command(detections)
    io_client.send_msg(cmd)


if __name__ == '__main__':
    try:
        while True:
            main_loop()

    except StopIteration:
        print('Exiting main loop...')

    finally:
        print('Dont get hit by a car')
        io_client.send_msg('SPEED:0\n')
