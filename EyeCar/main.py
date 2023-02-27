import cfg

import cv2
import Commander
from io_client import get_io_client

from typing import *


io_client = get_io_client(cfg.INPUT_MODE)

def main_loop() -> None:
    frame = io_client.read()

    if frame is None:
        return
    
    cv2.imshow('frame', frame)

    io_client.handle_keyboard_input()
    
    cmd = Commander.calculate_command(frame)
    io_client.send_msg(cmd)

    print("cmd:")
    print(cmd)


if __name__ == '__main__':
    try:
        while True:
            main_loop()

    except StopIteration:
        print('Exiting main loop...')

    finally:
        print('Dont get hit by a car')
        io_client.send_msg(Commander.Command.STOP)
