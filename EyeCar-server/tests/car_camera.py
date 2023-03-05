"""A script to test recieving image from a camera installed on a car"""

import cv2
import cfg

from beholder2048squad.Server import Server


serv = Server(udp_host=cfg.UDP_HOST, udp_port=cfg.UDP_PORT,
              tcp_host=cfg.TCP_HOST, tcp_port=cfg.TCP_PORT)


def main_loop() -> None:
    frame = serv.recv_img()
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) == ord('q') or frame is None:
        raise StopIteration

    serv.send_msg('SPEED:0\n')


if __name__ == '__main__':
    try:
        while True:
            main_loop()

    except StopIteration:
        print('Exiting main loop...')
