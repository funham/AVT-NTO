import cv2
import Commander

from typing import *

cap = cv2.VideoCapture(0)


def main_loop() -> None:
    ret, frame = cap.read()
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) == ord('q') or not ret:
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
