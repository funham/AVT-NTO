import cv2
import Commander

from typing import *
from beholder2048squad.Server import Server


serv = Server(9090)


def main_loop() -> None:
    frame = serv.chat_img()
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) == ord('q') or frame is None:
        raise StopIteration

    cmd = Commander.calculate_command(frame)
    serv.chat_cmd(cmd)

    print(cmd)


if __name__ == '__main__':
    try:
        while True:
            main_loop()

    except StopIteration:
        print('Exiting main loop...')
    finally:
        print('Dont get hit by a car')
        serv.chat_cmd(Commander.Command.STOP)
