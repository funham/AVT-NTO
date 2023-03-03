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
from detection.lane_detector import LaneDetector
from detection.yolo_detector import YoloV5Detector

# Handler imports
from detection.lane_handler import LaneTurnHandler
from detection.lane_handler import OpticalDistanceHandler
from detection.timing_handler import TimingDistanceHandler
from detection.timing_handler import TimingHandler
from detection.trafficlight_handler import TrafficLightHandler


TARGET_DISTANCE = 400  # 4m

# Global variables
io_client = io_client_manager.get_io_client(cfg.INPUT_MODE)
detector = GlobalDetectionModel()
control = CarControl()

# Adding detectors to the global detector object.
detector.add_detector(LaneDetector())
detector.add_detector(YoloV5Detector("/trafficlight_detector"))


# Registering handlers to detections
control.register_handler(LaneTurnHandler())
control.register_handler(TrafficLightHandler())

# Optical distance controlling (potentially the most stable)
control.register_handler(OpticalDistanceHandler(target_distance=TARGET_DISTANCE))

# Control distance by time directly (works properly only if speed is constant)
control.register_handler(TimingHandler(target_time=TARGET_DISTANCE / cfg.CAR_MAX_SPEED))

# Control distance by time with variable speed
control.register_handler(TimingDistanceHandler(target_distance=TARGET_DISTANCE))


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
