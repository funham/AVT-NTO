import cv2
import Commander

from typing import *
from beholder2048squad.Server import Server


serv = Server(9090)

locked = False

def main_loop() -> None:
    global locked

    frame = serv.chat_img()
    cv2.imshow('frame', frame)

    key = cv2.waitKey(1)

    if key == ord('q') or frame is None:
        raise StopIteration


    cmd = Commander.calculate_command(frame)
    
    if key == ord('s'):
        locked = True

    if key == ord('g'):
        locked = False
    
    serv.chat_cmd(cmd if not locked else Commander.Command.STOP)

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
        serv.chat_cmd(Commander.Command.STOP)
