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
    print(ret, type(frame))

    if INPUT_MODE == InputMode.CAM:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise StopIteration

    if not ret:
        raise StopIteration

    cv2.imshow('frame', frame)
    
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
