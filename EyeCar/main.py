"""
Entry point of the Sever (PC) side. 

Here we create IO client, setting up all the detectors, reading and 
handling frames and send commands to the car.
"""

import cfg
import cv2

from io_client import get_io_client
from CarControl import CarControl

from Detector import GlobalDetectionModel
from YoloDetector import YoloV5Detector, YoloV8Detector
from LaneDetector import LaneDetector

from DetectionHandler import LaneHandler

io_client = get_io_client(cfg.INPUT_MODE)
detector = GlobalDetectionModel()
control = CarControl()

# detector.add_detector(YoloV5Detector("/Models/TrafficLightsDetector.model"))
# detector.add_detector(YoloV8Detector("/Models/SignPedestrianDetector.model"))
detector.add_detector(LaneDetector())

control.register_handler(LaneHandler())

def main_loop() -> None:
    frame = io_client.read()

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
