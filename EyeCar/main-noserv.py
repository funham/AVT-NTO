import cv2
import Commander
import cfg

from enum import Enum
from typing import *

class InputMode(Enum):
    CAM = 0
    IMG = 1

INPUT_MODE = InputMode.IMG

def get_cap():
    if INPUT_MODE == InputMode.CAM:
        return cv2.VideoCapture(0)
    else:
        from Capture import ImageCapture
        return ImageCapture(r'C:\Users\fun1h\Documents\AVT-NTO\EyeCar\images')

cap = get_cap()

def main_loop() -> None:
    ret, frame = cap.read()
    cv2.imshow('frame', frame)

    if cv2.waitKey(0 if INPUT_MODE == InputMode.IMG else 1) == ord('q') or not ret:
        raise StopIteration

    cmd = Commander.calculate_command(frame)
    print(cmd)


if __name__ == '__main__':
    try:
        while True:
            main_loop()

    except StopIteration:
        print('Exiting main loop...')
    finally:
        print('Dont get hit by a car')
        print(Commander.Command.STOP)
