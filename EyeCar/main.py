import cfg
import cv2

from io_client import get_io_client
from CarControl import CarControl

from Detector import GlobalDetectionModel, YoloV5Detector, YoloV8Detector
from LaneKeeper import LaneDetector

io_client = get_io_client(cfg.INPUT_MODE)
detector = GlobalDetectionModel()

detector.add_detector(YoloV5Detector("/Models/TrafficLightsDetector.model"))
detector.add_detector(YoloV8Detector("/Models/SignPedestrianDetector.model"))
detector.add_detector(LaneDetector())


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
